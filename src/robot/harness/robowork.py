"""RoboHarness-style single-agent tool workspace adapted to Piper MuJoCo.

No layered policy, stage/replan budgets, action blacklist, fixed centimetre floor,
or independent verifier. Simulation truth never enters this workspace.
"""
import json
from pathlib import Path
import time
import uuid
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .vision import LOWER, UPPER, observation, png_url, interruptible_call, Stopped, ModelAPIError
from .policy import _camera, pixel_to_base
from .visual_tracking import RGBTracks, image_id
from .robowork_executor import LatchedExecutor


class PlanningError(ValueError):
    """Local read-only IK failure, never a runtime/transport fault."""
    def __init__(self, message, *, segment, target_pose):
        super().__init__(message)
        self.details = dict(kind='ik_not_converged', failed_segment=segment,
                            attempted_pose_base_wxyz=list(target_pose), motion_written=False,
                            recovery='Inspect current RGB and joint/FK state; revise translation or orientation and plan again.')


def vector(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length or any(isinstance(x, bool) for x in value):
        raise ValueError(f'expected {length} numeric values')
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('values must be finite')
    return result


def normalized_pixel(camera, u, v):
    uv = vector([u, v], 2)
    if np.any(uv < 0) or np.any(uv > 1000):
        raise ValueError('u/v must be normalized 0..1000, not raw pixels')
    h, w = camera['rgb_shape']
    return [float(uv[0] * (w - 1) / 1000), float(uv[1] * (h - 1) / 1000)]


# Tool schema is the single source used by the prompt and argument validator.
SCHEMAS = {
    'capture_camera': {'required': [], 'optional': []},
    'read_proprioception': {'required': [], 'optional': []},
    'activate_skill': {'required': ['name'], 'optional': []},
    'deactivate_skill': {'required': [], 'optional': []},
    'read_depth': {'required': ['camera', 'image_id', 'u', 'v'], 'optional': []},
    'track_point': {'required': ['camera', 'image_id', 'u', 'v', 'label'], 'optional': []},
    'forget_track': {'required': ['track_id'], 'optional': []},
    'plan_eef': {'required': ['frame', 'delta_m', 'rot_deg'], 'optional': []},
    'plan_pose': {'required': ['pose_base_wxyz'], 'optional': []},
    'plan_grasp_rgbd': {'required': ['camera', 'image_id', 'u', 'v'], 'optional': ['yaw_deg', 'approach_m']},
    'exec_plan_pose': {'required': ['plan_id'], 'optional': []},
    'move_joints': {'required': ['joint_rad'], 'optional': []},
    'open_gripper': {'required': [], 'optional': []},
    'close_gripper': {'required': [], 'optional': []},
    'finish': {'required': ['outcome', 'evidence'], 'optional': []},
}


class Workspace:
    def __init__(self, session, executor, kinematics, output, stop):
        self.session, self.executor, self.kinematics = session, executor, kinematics
        self.output, self.stop = Path(output), stop
        self.epoch = 0
        self.plans = {}
        self.tracks = RGBTracks()
        self.skill = None
        self.obs, self.cameras = None, {}
        self.last_stamps = None
        self.last_result = None
        self.sequence = 0
        self.preview = None
        self.previous_observation = None

    def capture(self):
        deadline = time.monotonic() + 1.5
        while not self.stop.is_set():
            local = {}
            try:
                obs, stamps = observation(self.session, self.last_stamps, local=local)
                break
            except RuntimeError as exc:
                if str(exc) != 'camera did not advance' or time.monotonic() >= deadline:
                    raise
                self.stop.wait(.02)
        else:
            raise Stopped()
        self.previous_observation = self.obs
        self.obs, self.cameras, self.last_stamps = obs, local, stamps
        self.sequence += 1
        self.tracks.update(local)
        obs['image_ids'] = {n: image_id(n, s) for n, s in stamps.items()}
        obs['tool_pose_base_wxyz'] = self.kinematics.pose(obs['joint_rad'], 'tool')
        obs['gripper_command'] = self.executor.gripper_command
        obs['joint_limits_rad'] = {'lower': list(LOWER), 'upper': list(UPPER)}
        obs['motion_epoch'] = self.epoch
        obs['tracks'] = self.tracks.summary()
        # Private workspace consists only of allowlisted public sensors/robot model.
        perception = self.output / 'perception'
        perception.mkdir(exist_ok=True)
        from PIL import Image
        for name, c in local.items():
            path = perception / name; path.mkdir(exist_ok=True)
            Image.fromarray(c['rgb']).save(path / 'rgb.png')
            if 'depth_m' in c:
                np.save(path / 'depth.npy', c['depth_m'])
        state = {k: v for k, v in obs.items() if k != 'rgb'}
        (perception / 'latest.json').write_text(json.dumps(state, ensure_ascii=False))
        return state

    def point(self, args):
        name = args['camera']
        if name not in self.cameras or args['image_id'] != self.obs['image_ids'][name]:
            raise ValueError('select a point in the latest image with its exact image_id')
        return name, normalized_pixel(self.cameras[name], args['u'], args['v'])

    def path(self, start, targets):
        q = np.asarray(start, dtype=float)
        waypoints = []
        for target in targets:
            desired = vector(target, 7)
            if abs(np.linalg.norm(desired[3:]) - 1) > 1e-5:
                raise ValueError('quaternion must be unit wxyz')
            origin = np.asarray(self.kinematics.pose(q, 'tool'))
            rotations = Rotation.from_quat(np.array([origin[[4, 5, 6, 3]], desired[[4, 5, 6, 3]]]))
            angle = (rotations[0].inv() * rotations[1]).magnitude()
            n = max(1, int(np.ceil(np.linalg.norm(desired[:3] - origin[:3]) / .01)), int(np.ceil(angle / .05)))
            if n > 1000:
                raise ValueError('pose is outside the supported robot workspace')
            interpolator = Slerp([0, 1], rotations)
            for fraction in np.linspace(0, 1, n+1)[1:]:
                if self.stop.is_set():
                    raise Stopped()
                quat = interpolator(fraction).as_quat()[[3, 0, 1, 2]]
                pose = [*(origin[:3] + fraction * (desired[:3] - origin[:3])), *quat]
                try:
                    solved = np.asarray(self.kinematics.solve(q, pose, frame='base', point='tool'))
                except RuntimeError as exc:
                    # Only the local solver call is converted. Transport errors
                    # elsewhere must still stop the session.
                    raise PlanningError(f'IK planning failed: {exc}', segment=len(waypoints)+1,
                                        target_pose=pose) from exc
                if solved.shape != (6,) or not np.isfinite(solved).all() or np.any(solved < LOWER) or np.any(solved > UPPER):
                    raise ValueError('IK violates robot joint limits')
                pe, re = self.kinematics.error(solved, pose, 'tool')
                if pe > .002 or re > .02:
                    raise ValueError(f'IK residual {pe:.4f} m / {re:.4f} rad; inspect posture and choose another approach')
                waypoints.append(dict(joint=solved.tolist(), pose=pose))
                q = solved
        return waypoints

    def save_plan(self, targets, **evidence):
        path = self.path(self.obs['joint_rad'], targets)
        identifier = 'plan_' + uuid.uuid4().hex[:12]
        summary = dict(plan_id=identifier, motion_epoch=self.epoch, target_poses_base_wxyz=targets,
            waypoints=len(path), collision_checked=False, source='public camera/robot state and local Piper FK/IK', **evidence)
        self.plans[identifier] = dict(summary=summary, path=path, seed=self.obs['joint_rad'], epoch=self.epoch)
        self.preview = self.render_plan(targets, identifier)
        if len(self.plans) > 16:
            del self.plans[next(iter(self.plans))]
        return summary

    def render_plan(self, targets, identifier):
        """Frozen RGB plus tool-origin/axis projection; never a collision certificate."""
        from PIL import Image, ImageDraw
        result = {}
        for name, camera in self.cameras.items():
            cal = _camera(self.cameras, name)
            canvas = Image.fromarray(camera['rgb'].copy()); draw = ImageDraw.Draw(canvas)
            def project(point):
                p = cal['rotation'].T @ (np.asarray(point) - cal['origin'])
                if p[2] <= .001:
                    return None
                uv = cal['intrinsics'] @ p
                xy = uv[:2]/uv[2]
                if not np.isfinite(xy).all() or np.max(np.abs(xy)) > 100000:
                    return None
                return tuple(xy.tolist())
            for i, pose in enumerate(targets):
                pose = np.asarray(pose); center = project(pose[:3])
                if center is None:
                    continue
                draw.text(center, str(i+1), fill='yellow')
                r = Rotation.from_quat(pose[[4,5,6,3]]).as_matrix()
                for axis, color in enumerate(('red','green','blue')):
                    endpoint = project(pose[:3] + .025*r[:,axis])
                    if endpoint:
                        draw.line([center, endpoint], fill=color, width=2)
            canvas.save(self.output/f'{identifier}_{name}.png')
            result[name] = dict(image_id=self.obs['image_ids'][name], image=png_url(np.asarray(canvas)))
        return result

    def grasp_plan(self, args):
        name, pixel = self.point(args)
        hit = pixel_to_base(self.cameras, name, pixel)
        point = np.asarray(hit['point_base_m'])
        # Piper adapter: sampled local RGB-D surface plus known finger geometry.
        # No object centres, meshes or evaluator state. This is intentionally not
        # named as the upstream X5 volume/path solver: its geometry is incompatible.
        cam = _camera(self.cameras, name, require_depth=True)
        x, y = map(lambda z: int(round(z)), pixel)
        depth = cam['depth_m']; h, w = depth.shape
        samples = []
        for v in range(max(0, y-4), min(h, y+5)):
            for u in range(max(0, x-4), min(w, x+5)):
                d = float(depth[v, u])
                if np.isfinite(d) and d > 0 and abs(d-hit['depth_m']) < .015:
                    samples.append(pixel_to_base(self.cameras, name, [u,v])['point_base_m'])
        if len(samples) < 6:
            raise ValueError('insufficient local RGB-D surface support; select a clearly visible physical rim')
        yaw = float(args.get('yaw_deg', 0))
        approach = float(args.get('approach_m', .06))
        if not np.isfinite(yaw) or not np.isfinite(approach) or not 0 < approach <= .3:
            raise ValueError('finite yaw and positive approach_m <=0.3 required')
        r = Rotation.from_euler('z', yaw, degrees=True) * Rotation.from_euler('y', 180, degrees=True)
        # The selected surface is a tool-origin goal, not a calibrated contact
        # centre. Inspect wrist geometry before closing; do not invent an offset.
        target = point.copy()
        safe = target - r.apply([0, 0, approach])
        quat = r.as_quat()[[3,0,1,2]].tolist()
        return self.save_plan([[*safe, *quat], [*target, *quat]],
            selected_surface=hit, surface_samples=len(samples), grasp_detected='unknown',
            planner='piper_rgbd_surface_approach', gripper_policy='explicit open/close; preserve command during motion')

    def move(self, target):
        # Joint interpolation is an actuator speed envelope, not a model action
        # size restriction. There is no 1 cm floor, 5 cm ceiling or action blacklist.
        current = np.asarray(self.executor.observe()['joint'])
        desired = vector(target, 6)
        if np.any(desired < LOWER) or np.any(desired > UPPER):
            raise ValueError('joint target outside robot limits')
        segments = max(1, int(np.ceil(np.max(np.abs(desired-current)) / .04)))
        result = None
        for alpha in np.linspace(0,1,segments+1)[1:]:
            result = self.executor.command((current + alpha*(desired-current)).tolist())
            if result.get('error'):
                break
        return result

    def dispatch(self, call):
        if not isinstance(call, dict) or set(call) - {'tool', 'arguments', 'reason'}:
            raise ValueError('return tool, arguments, reason only')
        tool, args = call.get('tool'), call.get('arguments', {})
        if tool not in SCHEMAS or not isinstance(args, dict):
            raise ValueError('unknown tool or invalid arguments')
        schema = SCHEMAS[tool]
        if set(args) - set(schema['required'] + schema['optional']) or set(schema['required']) - set(args):
            raise ValueError(f'arguments must follow {tool} schema: {schema}')
        if self.stop.is_set():
            raise Stopped()
        if tool in ('capture_camera', 'read_proprioception'):
            return self.capture()
        if tool == 'activate_skill':
            if args['name'] != 'pick-up-object':
                raise ValueError('available skill: pick-up-object')
            self.skill = args['name']; return dict(active_skill=self.skill)
        if tool == 'deactivate_skill':
            self.skill = None; return dict(active_skill=None)
        if tool in ('read_depth', 'track_point'):
            name, pixel = self.point(args)
            if tool == 'read_depth':
                return pixel_to_base(self.cameras, name, pixel)
            if not isinstance(args['label'], str) or not 0 < len(args['label']) <= 80:
                raise ValueError('label must be 1..80 characters')
            return self.tracks.register(name, args['image_id'], pixel, args['label'])
        if tool == 'forget_track':
            return self.tracks.forget(args['track_id'])
        if tool == 'plan_grasp_rgbd':
            return self.grasp_plan(args)
        if tool == 'plan_pose':
            return self.save_plan([vector(args['pose_base_wxyz'],7).tolist()])
        if tool == 'plan_eef':
            delta, rot = vector(args['delta_m'],3), np.deg2rad(vector(args['rot_deg'],3))
            pose = np.asarray(self.obs['tool_pose_base_wxyz'])
            r = Rotation.from_quat(pose[[4,5,6,3]])
            frame = args['frame']
            if frame == 'base':
                transform = np.eye(3)
            elif frame == 'tool':
                transform = r.as_matrix()
            elif frame in ('head_camera','wrist_camera'):
                transform = _camera(self.cameras,frame.split('_')[0])['rotation']
            else:
                raise ValueError('frame must be base/tool/head_camera/wrist_camera')
            target = [*(pose[:3] + transform @ delta), *(Rotation.from_rotvec(transform @ rot)*r).as_quat()[[3,0,1,2]]]
            return self.save_plan([target])
        if tool == 'exec_plan_pose':
            item = self.plans.pop(args['plan_id'], None)
            if item is None or item['epoch'] != self.epoch:
                raise ValueError('unknown/consumed plan or intervening robot command; plan from current observation')
            actual = self.executor.observe()['joint']
            if np.max(np.abs(np.asarray(actual)-item['seed'])) > .01:
                raise ValueError('robot moved after planning; create a current plan')
            self.epoch += 1; self.plans.clear(); self.preview = None
            completed = 0; result = None
            for waypoint in item['path']:
                result = self.move(waypoint['joint'])
                if result.get('error'):
                    break
                completed += 1
            return dict(completed_waypoints=completed, planned_waypoints=len(item['path']),
                execution=result, grasp_detected='unknown', error=result.get('error') if result else 'empty path')
        if tool in ('move_joints','open_gripper','close_gripper'):
            self.epoch += 1; self.plans.clear(); self.preview = None
            if tool == 'move_joints':
                return self.move(args['joint_rad'])
            return self.executor.command(self.executor.observe()['joint'], gripper=0 if tool=='close_gripper' else 1,
                timeout_s=3.5, close=tool=='close_gripper')
        if tool == 'finish':
            if args['outcome'] not in ('completed','abandoned') or not isinstance(args['evidence'],str) or not args['evidence'].strip():
                raise ValueError('finish needs completed/abandoned and concrete current visual evidence')
            return dict(agent_outcome=args['outcome'], evidence=args['evidence'], independently_verified=False)

    def messages(self, task, history):
        prompt = Path(__file__).with_name('prompts').joinpath('robowork.txt').read_text()
        if self.skill:
            prompt += '\n\n' + Path(__file__).with_name('prompts').joinpath('robowork_pick.txt').read_text()
        state = {k:v for k,v in self.obs.items() if k != 'rgb'}
        payload = dict(task=task, latest=state, active_skill=self.skill, tools=SCHEMAS, recent_tool_history=history[-16:])
        content = [{'type':'text','text':json.dumps(payload,ensure_ascii=False,allow_nan=False)}]
        if self.previous_observation is not None:
            for name in ('head','wrist'):
                prior = self.previous_observation
                content += [{'type':'text','text':f'HISTORICAL previous {name} RGB image_id={prior["image_ids"][name]}; compare motion only, never click this old image'},
                            {'type':'image_url','image_url':{'url':prior['rgb'][name]}}]
        for name in ('head','wrist'):
            content += [{'type':'text','text':f'CURRENT {name} RGB image_id={self.obs["image_ids"][name]}'},
                        {'type':'image_url','image_url':{'url':self.obs['rgb'][name]}}]
        if self.preview:
            for name, preview in self.preview.items():
                content += [{'type':'text','text':f'FROZEN PLAN PREVIEW {name}, source {preview["image_id"]}; tool axes only, not current RGB or gripper collision geometry'},
                            {'type':'image_url','image_url':{'url':preview['image']}}]
        return [{'role':'system','content':prompt},{'role':'user','content':content}]


def run_workspace(session, policy, stop, output, *, task, kinematics, max_rounds=500, interval=1., resume=None):
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    (output/'run_context.json').write_text(json.dumps(dict(version=1,architecture='robowork',runner_revision='feedback-wait-v1',task=task,max_rounds=max_rounds,
        observation_policy='public RGB-D and robot proprioception; no evaluator truth',resume=resume.get('source') if resume else None),ensure_ascii=False))
    history = []
    index = resume.get('last_round',0) if resume else 0
    start = index
    outcome, exit_reason = None, 'user_stop'
    api_failures = 0
    executor = None
    print(f'RoboHarness 工具工作区：{output.resolve()}\n单 agent / RGB-D / 显式技能 / 持续夹爪目标；无阶段和恢复次数预算。\n运行版本：feedback-wait-v1（退出来源诊断 + 短暂反馈延迟等待）',flush=True)
    try:
        with LatchedExecutor(session,stop=stop,lower=LOWER,upper=UPPER,journal=output/'control.jsonl') as executor:
            workspace = Workspace(session,executor,kinematics,output,stop)
            with (output/'turns.jsonl').open('x') as journal, (output/'model_calls.jsonl').open('x') as audit:
                while not stop.is_set() and (not max_rounds or index-start < max_rounds):
                    workspace.capture()
                    messages = workspace.messages(task,history)
                    audit.write(json.dumps(dict(round=index+1,messages=messages),ensure_ascii=False)+'\n');audit.flush()
                    fatal_api_error = False
                    try:
                        call = interruptible_call(lambda: policy(messages),stop)
                        api_failures = 0
                    except (ValueError, RuntimeError) as exc:
                        # API failures are observations for the next turn, not motion.
                        call = None; result = dict(error=str(exc),source='model_api')
                        api_failures += 1
                        if isinstance(exc, ModelAPIError):
                            result.update(http_status=exc.status, retryable=exc.retryable)
                            fatal_api_error = not exc.retryable
                    index += 1
                    if call is not None:
                        print(f'第 {index} 轮：{call.get("tool") if isinstance(call,dict) else "invalid"}；{call.get("reason", "") if isinstance(call,dict) else ""}',flush=True)
                        try:
                            result = workspace.dispatch(call)
                        except (ValueError,KeyError,TypeError) as exc:
                            result = dict(error=str(exc))
                            if isinstance(exc, PlanningError):
                                result.update(exc.details)
                    record = dict(round=index,call=call,result=result)
                    journal.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n');journal.flush()
                    history.append(record)
                    if len(history)>16:
                        history.pop(0)
                    if fatal_api_error:
                        print(f'  接口请求被拒绝：{result["error"]}；停止重复请求，请修正接口配置或请求格式。',flush=True)
                        exit_reason='model_api_rejected'
                        break
                    if result.get('error'):
                        print(f'  工具反馈：{result["error"]}；交还 agent 看新图后决定。',flush=True)
                    if result.get('agent_outcome'):
                        outcome=result;exit_reason='agent_finish';break
                    stop.wait(max(interval, min(30., 2. ** min(api_failures,5))) if api_failures else interval)
                else:
                    exit_reason='user_stop' if stop.is_set() else 'max_rounds'
    except Stopped:
        exit_reason='user_stop'
    except Exception as exc:
        exit_reason='transport_or_runtime_fault:'+type(exc).__name__
        raise
    finally:
        runtime_error = getattr(executor, 'fault', None)
        stop_source = getattr(stop, 'reason', None)
        if runtime_error:
            exit_reason = 'runtime_monitor_fault'
        elif exit_reason == 'user_stop':
            exit_reason = stop_source or 'external_stop'
        labels = {'enter_pressed': '终端收到 Enter', 'stdin_eof': '终端标准输入已关闭（EOF）',
                  'stdin_error': '读取终端输入失败', 'external_stop': '收到未标注来源的停止信号',
                  'runtime_monitor_fault': '后台运行监控故障', 'max_rounds': '达到显式设置的轮数上限',
                  'agent_finish': 'agent 主动结束', 'model_api_rejected': '模型接口拒绝请求'}
        print(f'退出原因：{labels.get(exit_reason, exit_reason)}' + (f'；{runtime_error}' if runtime_error else ''), flush=True)
        (output/'run_summary.json').write_text(json.dumps(dict(architecture='robowork',last_round=index,
            exit_reason=exit_reason,agent_outcome=outcome,task_complete=False,independently_verified=False,
            shutdown_error=getattr(executor, 'shutdown_error', None),
            stop_source=stop_source, runtime_error=runtime_error),ensure_ascii=False))
    print('harness 已退出；agent 结论与真实任务验收分开记录。',flush=True)


def load_workspace_resume(directory):
    """Continue a task, not old budgets, skills, gripper intent or executable plans."""
    directory = Path(directory).resolve()
    meta_path = directory/'run_context.json'
    if not meta_path.exists():
        meta_path = directory/'evidence_notes.json'
    if meta_path.stat().st_size > 32768:
        raise ValueError('oversize run metadata')
    meta = json.loads(meta_path.read_text())
    task = meta.get('task')
    if not isinstance(task,str) or not 0 < len(task.strip()) <=4096:
        raise ValueError('missing task in run_context.json')
    last = 0
    turns = directory/'turns.jsonl'
    if turns.stat().st_size > 32*1024*1024:
        raise ValueError('oversize historical turn log')
    with turns.open() as stream:
        for line in stream:
            row = json.loads(line)
            number = row.get('round')
            if type(number) is not int or number<=last:
                raise ValueError('historical rounds must increase')
            last=number
    return dict(task=task,last_round=last,source=str(directory))
