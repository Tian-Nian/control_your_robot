# Robot harness: verified execution and optional visual simulation

The default MuJoCo visual agent now uses a single-agent RGB-D tool workspace with latched gripper commands, without the former stage/recovery budgets. See [current startup, tools and migration boundaries](harness_robowork.md). The PositionExecutor API below remains unchanged for its independent examples; older layered-agent sections apply only with `--architecture layered`.

`PositionExecutor` adds a bounded, feedback-verified execution path alongside the existing `RobotHarness` and `RuntimeEnv`. Start with the independent Piper Sim example:

```bash
cd /Users/niantian/projects/control_your_robot
source .venv/bin/activate
python examples/piper_harness.py
# Optional separate Rerun viewer:
python examples/piper_harness.py --viewer
```

Replace the checkout path on another machine. This needs the same Pinocchio/Rerun dependencies as Piper Sim. The example starts its own temporary runtime with a unique shared-memory name and discovery directory, and cleans up only that process. There is no need to stop an existing `piper_sim` service. Every default run creates a new JSONL path under `save/`; `--out` accepts an explicit, unused path.

## What the example verifies

From READY_JOINT, axis 2 goes from 1.3 to 1.35 rad; the gripper goes to 0.5; the arm returns to READY_JOINT. Each request requires fresh feedback within the joint/gripper tolerances for a continuous observation interval. A repeated request ID returns its original result without repeating motion, even if the robot has since moved elsewhere. This is a position-feedback demonstration, not a grasping or contact experiment.

## API

```python
from robot.harness import PositionExecutor

# session is a connected Session. lower/upper are the matching robot's
# joint limits in radians, in exactly the published joint order.
with PositionExecutor(
    session,
    lower=lower,
    upper=upper,
    journal="save/unique_episode.jsonl",
) as executor:
    print(executor.capabilities())
    result = executor.execute(
        "move-001",
        joint=[0, 1.35, -1, 0, 0.5, 0],
        timeout_s=3.0,
        tolerance_rad=0.005,
        dwell_s=0.2,
    )
    if result["status"] != "succeeded":
        print(result)
```

The snippet assumes an already connected, matching Piper position-mode session. For an executable setup, use the example rather than copying undefined `session/lower/upper` variables.

| Requirement | Meaning |
| --- | --- |
| Published fields | `joint`, scalar or one-element `gripper`, `control_mode`, `servo_fault` |
| Mode | Must already be `position`; no automatic switching from torque/impedance |
| Limits | Explicit finite bounds, with the same DOF as feedback |
| Ownership | One control lease for the executor lifetime; session is not closed by executor |
| Goal | Absolute joint vector and/or gripper in [0,1]; omitted part holds the observed value |
| Freshness | Positive timestamp in this machine's wall-clock domain, age ≤ max_age_ms; future timestamps rejected |
| Verification | Both sequence and timestamp advance; stale/repeated frames cannot satisfy dwell |
| Timing | Local monotonic deadline and polling; no hard-real-time guarantee |
| History | In-memory per executor, maximum 1000 completed IDs by default; no eviction or cross-restart dedup |
| Journal | Exclusive creation; requested target, write receipts, feedback metrics and terminal outcome, flushed JSONL |

`run([request_dict, ...])` executes in order and stops on the first non-success. Invalid inputs raise before any request target write. Identical completed IDs return a deep copy; conflicting IDs, concurrent calls and full history raise errors. `cancel(request_id)` can be called from another thread; wait for execute to finish before closing.

## Results, deadlines and recovery

- `succeeded`: fresh feedback met goal tolerances for the dwell interval, then a runtime position hold was acknowledged and stable fresh position samples were observed.
- `rejected`: failure before attempting a goal write. Invalid execute arguments raise ValueError.
- `failed`: execution failed/timed out, but the subsequent position hold was verified.
- `cancelled`: cancellation was requested and the subsequent position hold was verified.
- `unknown`: execution or stopping could not be verified. Never automatically replay it.

All attempted failures block new requests until explicit `executor.recover()`. Recovery requests a new hold, verifies fresh feedback, releases the runtime latch, and clears the executor fault. It does not retry previous requests. `close()` verifies a final hold and releases its lease/log even when verification fails; it does not close the supplied Session. Cancellation must come from another thread while execute is running.

`Session.write_target(..., valid_for_s=seconds)` timestamps an expiry in the host monotonic-clock domain at submission. The device loop checks it before advancing motion. Component config `stream_timeout_s` additionally caps target lifetime, including targets without client TTL; zero disables this cap. The example uses 5 seconds, and executor goals use their timeout (default 3 seconds). On expiry/lease loss, a timed position target is discarded, any motion plan is cancelled, and current position is held. Ordinary targets cannot unlock the latch: explicit `release` and a fresh target are required. Legacy untimed streams with cap zero keep the original behavior. This is a stream-target watchdog, not a deadline for every queued command.

`Session.stop_target(component)` posts an explicit stop through the stream slot. `runtime_status()` exposes stream sequence, state and reason. Runtime `stopped` means the hold command was applied, not physical rest. Executor additionally requires advancing seq/timestamps and ≤1e-4 joint/gripper drift against a fixed sample over 0.05 seconds. This is a position-stability check, not a velocity sensor or hardware e-stop. Non-position stop is reported as `stop_unconfirmed`. A blocked device loop or driver cannot enforce a Python watchdog; real hardware needs an independent stop mechanism.

## Model-independent agent skills and persistent tasks

```bash
python examples/piper_harness.py --agent
python examples/piper_harness.py --agent --viewer
```

This offline demonstration observes feedback, moves q2 to 1.35 rad, sets gripper to 0.5, and uses a base-frame **tool-point pose** to return the arm. It creates a SQLite store beside the JSONL file. `--store PATH` reuses a store; the fixed demo task ID then returns the historical result, even across process restart. It does not move again or assert the new simulation is at the historical final pose. Use a fresh store for a fresh demonstration.

```python
from robot.harness import AgentHarness
from robot.harness.kinematics import PiperKinematics

# Inside the PositionExecutor context above; Piper default model only.
agent = AgentHarness(executor, robot_id="my-piper-sim",
                     store_path="save/requests.sqlite3", kinematics=PiperKinematics())
observation = agent.dispatch("observe", {})
schemas = agent.tools()  # JSON function schemas for a model adapter.
result = agent.dispatch("set_gripper", {"request_id": "open-001", "gripper": 0.5})
```

Supported dispatch names: `observe`, `get_status`, `cancel`, `recover`, `move_joint_absolute`, `set_gripper`, and optional `move_eef`. Schemas are also enforced locally (unknown fields, dimensions, numeric types, finite values, frame and unit quaternion). Joint limits are prechecked by AgentHarness and checked again by PositionExecutor. `recover` is explicit but not an authorization system: expose it to a model only if your application permits autonomous recovery. The existing `--deepseek` test retains its fixed-task adapter; `--agent` does not call a model API or spend tokens.

EEF accepts seven values `[x,y,z,qw,qx,qy,qz]`, metres and unit quaternion, `frame="base"`, and `point="flange"` or `"tool"`. Local IK uses fresh joint feedback as its seed; its output passes the same joint executor/gate. Final local FK error must be ≤0.001 m and ≤0.01 rad. This verifies an endpoint, not a straight Cartesian path, collision clearance or grasp success. `PiperKinematics` assumes the default Piper URDF/tool offset: do not pair it with another calibration. Observed `eef_fk` is the flange pose computed from joints; it is not camera or independent pose feedback.

`run_task(task_id, [{"skill": ..., "arguments": ...}, ...])` runs 1–32 explicit movement steps, validates all schemas before starting, and stops on first non-success. Request IDs are generated as `task:<task_id>:<zero-based-step>`. Cancellation targets the active request ID; no task-wide cancellation during gaps or automatic replanning is implemented. `get_status` reports active task/step, runtime status and unresolved stored requests/tasks.

SQLite commits a pending record before execution and stores the terminal result afterwards. A crash between them leaves `unknown`; new work is blocked until explicit recovery. Identical IDs return historical outcomes; changed arguments raise conflict. Scope is robot_id + component/limits/capabilities/model calibration fingerprint, independent of episode ID. Keep the store and identity/profile stable across restarts; changing/deleting them is not recovery. This is conservative at-most-once dispatch within that scope, not exactly-once physical execution. Use one AgentHarness owner per robot; do not bypass it with direct executor calls. Journal flush alone is not power-loss durability, and no images are recorded.

## Existing API fixes

`ActionContract` now rejects empty/non-finite vectors, non-finite control frequency and a multi-element scalar gripper. `RuntimeEnv` preserves an eef target when adapting a simultaneous scalar gripper, and rejects further take_action calls after stop/max_steps. `stop()` still means ending policy submissions, not physically stopping the robot.

## Verification

```bash
python -m pytest src/robot/harness/tests src/robot/runtime/tests/test_loops.py -q
```

The tests cover invalid input, deduplication/conflicts, concurrent calls, cancellation, stale/frozen feedback, timeouts, loss of lease, evidence output and a real Piper Sim subprocess. Optional Marvin model tests skip only when their external URDF is unavailable. They do not replace Piper tests or demonstrate true hardware safety.

See the validation section at the end for the current test scope and exclusions. Physical hardware is not validated by these simulation tests.

## Live DeepSeek smoke test

With `DEEPSEEK_API_KEY` exported in the current shell, run:

```bash
python examples/piper_harness.py --deepseek
```

The default model is `deepseek-flash`; `--model` selects another available DeepSeek model. The optional `--viewer` remains available. The example uses the official HTTPS endpoint and [DeepSeek Tool Calls](https://api-docs.deepseek.com/guides/tool_calls/), without an additional SDK. It sends only the simulated capabilities, joint state, fixed task and execution results. Credentials are read from the environment, not written to code or journal, and are removed from the simulator subprocess environment.

This is a bounded integration test, not an unrestricted agent: at most four model calls, exactly one tool per action round, with local validation against the three-step task before execution. It fails rather than silently substituting a scripted action if the model proposes an unexpected target. Normal pytest runs use offline checks and the deterministic simulation, never the live API.

This fixed text task has been checked with the live provider. It verifies tool integration for that task, not general visual planning.

## Optional MuJoCo contact and visual-agent guide

The ordinary Piper Sim remains the default. MuJoCo is loaded only when explicitly
selecting `PiperMuJoCoController`; no import or startup command installs packages
or downloads assets. The prepared scene contains one Piper, a table, three
RoboTwin bowls and head/wrist RGB/depth cameras. It is a normal `cyr` runtime device.

**Capability boundary:** a fixed physical routine has grasped, lifted and released
the orange bowl. The visual CLI now uses a planner, a controller with discrete
actions, and a separate image verifier. Local geometry and PositionExecutor
bound motion. Autonomous bowl grasping/stacking with this layered policy has not
been validated; an image verifier's judgment is evidence, not physical proof.

### 1. Install the optional environment

Run in an editable checkout (prepared assets are located from the checkout):

```bash
cd /Users/niantian/projects/control_your_robot
python3 -m venv .venv-harness
source .venv-harness/bin/activate
python -m pip install -r requirements/harness-vision.txt
python -m pip install --no-deps -e .
```

`harness-vision.txt` includes MuJoCo, Rerun and Pinocchio 4.1.0. For the physical
check alone, use `requirements/harness-mujoco.txt` instead; Pinocchio is not needed
there. The asset converter uses a separate `harness-assets` extra, only if assets
must be regenerated. The minimal environment avoids the base project's physical
camera SDK dependencies. Existing `.venv` is not modified.

### 2. Start, view, run and stop

```bash
bash examples/piper_mujoco.sh start
bash examples/piper_mujoco.sh status
bash examples/piper_mujoco.sh agent
```

The launcher uses `.venv-harness` and inherits the same discovery directory as direct `cyr` commands (default `~/.local/state/cyr`). It no longer overrides `XDG_STATE_HOME`. `start` also ensures the read-only viewer is open. `agent` ensures the scene is started, opens Rerun,
then attaches the visual loop. It preserves an already-running scene. Press
**Enter in the agent terminal** to stop the agent; the scene/viewer remain open.
Stop a prior agent before starting another: control leases are exclusive.

| Command suffix | Behavior |
| --- | --- |
| `viewer` | Open/reopen current 3D scene and both camera panels without control or reset |
| `restart` | Reload scene and restore robot/bowls to initial state |
| `stop` | Stop services in this launcher's discovery directory |
| `pick-test` | Run fixed physical grasp/lift/release check on the current scene |
| `agent --dry-run --max-rounds 2` | Local wait-only policy; no model API calls |
| `agent --max-rounds 2` | Bound a model integration check |
| `agent --task '...'` | Override the built-in orange-bowl task |

To verify physics before trying the model:

```bash
bash examples/piper_mujoco.sh restart
bash examples/piper_mujoco.sh pick-test
bash examples/piper_mujoco.sh restart
bash examples/piper_mujoco.sh agent
```

Restart after a physical check, since it moves the bowl. Joint reset does not
reset free objects. If the service has not started yet, use `start`, not `restart`.

The equivalent lower-level commands require the same environment and directory:

```bash
source .venv-harness/bin/activate
unset XDG_STATE_HOME  # 使用默认服务目录；仅旧服务迁移期间才使用旧路径
cyr start --config config/runtime/piper_mujoco.yml
python examples/piper_mujoco_agent.py --attach
cyr stop
```

`cyr start` requires `--config`. `XDG_STATE_HOME` selects discovery records/logs,
not a new shared-memory name. A service in another directory can still own
`cyr_piper_mujoco`; use the launcher rather than deleting live shared memory.
`cyr stop` stops all services in its current discovery directory.

For an isolated self-cleaning scene, use
`python examples/piper_mujoco_harness.py --pick-test --viewer` or
`python examples/piper_mujoco_agent.py --viewer`. These select unique runtime
names/directories. Do not combine `--viewer` with `--attach`.

### 3. Configure the model

The visual runner reads `~/.config/control_your_robot/vision_api.json` when present.
The current workstation uses `http://127.0.0.1:3425/v1` and
`codex/gpt-6.1-sol`; this private configuration/key is not part of the repository.
On another machine, create a private JSON file, for example:

```json
{
  "base_url": "http://127.0.0.1:3425/v1",
  "model": "codex/gpt-6.1-sol",
  "api_key_env": "CYR_VISION_API_KEY"
}
```

Export that environment variable with your key, or use an `api_key` field in the
private file (file permission `600`). Do not put credentials in runtime YAML,
source code or logs. The base URL is extended with `/chat/completions`; the
provider must accept image content and JSON action responses. There is no silent
provider/model fallback. `--api-config`, `--base-url`, `--model`, and `--api-key-env`
override configuration; a changed endpoint does not implicitly reuse the private
file's key. Default without configuration is the legacy `deepseek-flash` provider
and `DEEPSEEK_API_KEY` (or `CYR_VISION_API_KEY`). DeepSeek-only options are sent
only to its official host. The older `piper_harness.py --deepseek` remains a
separate text-only fixed-task example.

### 4. Layered roles and model inputs

The CLI defaults to layered mode. At startup the planner receives the task and
current head/wrist RGB images, and returns 1–6 ordered stages. Each stage has
`id`, `target`, `affordance`, `motion`, `description`, and a visible `completion`
condition. Motion categories are `APPROACH`, `ALIGN`, `DESCEND`, `GRASP`, `LIFT`,
`PLACE`, `RELEASE`, and `RETREAT`. The prompt groups approach/alignment/closing
into GRASP and keeps LIFT separate; this is guidance, not an enforced grasp path.
An unavailable or invalid plan falls back to one APPROACH stage whose completion
restates the task. No scripted trajectory substitutes for a planner failure.

| Role | Model-visible context | Images |
| --- | --- | --- |
| Planner | Task, explicitly loaded notes, target selection, completed stages, recent actions and replan reason; periodic review/continuation also receive current robot FK/height | Current head/wrist; previous pair when available on replanning |
| Controller | Original task, current stage, episode progress, up to eight recent actions, six joints, normalized gripper, `robot.joint_limits_rad`, optional FK/feasibility hints, static tabletop/bowl-height reference and current tool height, blocked action, last failed action, consecutive failures, latest point query, optional notes | Current head/wrist; immediately previous controller pair from the second turn |
| Verifier | Only stage ID and visible completion condition | Stage-start head/wrist and a newly acquired pair after `done` or an automatic close/upward/periodic check |

The controller gets up to eight `recent_actions` entries and the original task,
plus separate failure memory. Planner replans receive current progress and
image-verified completed stages. This is bounded episode memory, not an object tracker.
Joint-derived FK can include base-frame tool/flange poses, tool axes, height and
downward tilt; feasibility hints check six 1 cm directions and a tilt correction.
Both base and tool directions are checked independently. The controller also
receives a `wrist_tool_guide`: tool axes expressed in optical right/down/forward
coordinates, the nominal tool origin's projected pixel (when in front of the
camera), and its image timestamp. This is derived from robot FK and wrist
calibration only, without reading scene depth or detecting an object. Full
calibration matrices are not sent. Missing calibration disables the guide.
These hints check IK/joint bounds, not collisions. The verifier receives no
joint state, requested action, execution result, notes, or controller success claim.
Only a verifier response with boolean `complete=true` advances a stage; all
stages must pass before the loop reports task completion.

The built-in payload explicitly adds `scene_reference`: a static tabletop Z, rough upright-bowl height and fixed upright tool-height reference, plus current FK height above the table. It contains no live object poses or
velocities, contacts/forces, whole depth maps, calibration matrices, reward/success
oracle, XML/URDF, or fixed pick trajectory. A point query can expose one measured
base-frame point and its selected depth or two-view residual, as described below.
The full runtime frame stays local and model inputs are rebuilt through an
allowlist. User task text and explicitly loaded notes remain user-supplied text;
this boundary is not an OS sandbox for arbitrary plugins.

Robot feedback must be at most 500 ms old, images at most 1 s, image timestamps
must advance between acquisitions, and camera skew must be at most 200 ms. The
runtime publishes at 50 Hz, physics steps at 500 Hz, cameras sample at 10 simulated
Hz. The loop refreshes images after planning and for every explicit or automatic image check. If a fast callback
finishes before the next camera frame, acquisition waits up to 1 s for advancing
timestamps, interruptible by Enter. Stale/unhealthy feedback still fails
immediately; a nonadvancing stream fails after the bound. Repeated runtime
frames do not mean a newly captured image.
Role prompts live in `src/robot/harness/prompts/*.txt` and are included as package
data. Changing the task does not require editing the shared controller prompt.

### 5. Discrete actions, local geometry and compatibility

In layered mode the controller returns exactly one JSON object, with a nonempty
`reason` of at most 200 characters. Unknown fields and low-level joint/pose
commands are rejected. The current prompt asks for a short Chinese reason.

| Action | Fields and local behavior |
| --- | --- |
| `move` | `direction`: `forward`, `back`, `left`, `right`, `up`, or `down`; optional `step_cm` from 1 to 5 (default 1) along the corresponding fixed base axis, preserving orientation |
| `rotate` | `sense`: `down`, `cw`, or `ccw`; hold the tool point, rotate at most 0.04 rad toward base down, or ±0.04 rad about tool +Z (`ccw` positive, right-hand rule) |
| `tool_move` | `direction`: `x+`, `x-`, `y+`, `y-`, `z+`, `z-`; optional `step_cm` from 1 to 5 (default 1) along the **current tool axis**, preserving orientation |
| `tool_rotate` | `axis`: `x`, `y`, `z`; integer `sign`: `1` or `-1`; right-hand 0.04 rad rotation about the current tool axis, holding the tool origin |
| `set_gripper` | `gripper`: exactly 0 (closed) or 1 (open); successful position feedback does not prove a grasp |
| `point` | `camera`: `head` or `wrist`, `pixel`: `[u,v]`; query local geometry without motion; optional `second` selects a corresponding pixel in the other camera |
| `replan` | No target; immediately request a fresh recovery plan when stage premises or approach fail |
| `wait` | No target; observe again; default `--max-waits 3` triggers recovery planning in layered mode |
| `done` | No target; request an independent image check of the current stage |

Single-camera point is disabled in the default CLI. With explicit `--allow-depth-point`, a controller can request:

```json
{"action":"point","camera":"head","pixel":[160,120],"reason":"先测量可见碗沿的位置"}
```

A two-view query adds `"second":{"camera":"wrist","pixel":[u,v]}` with the
corresponding pixel. The schema accepts this optional field, although the current
controller prompt shows only the single-camera example. Coordinates must be
finite nonnegative numbers inside the image, and are rounded to the nearest
pixel. Booleans are invalid. Calibration and depth stay inside a copied snapshot
of the exact pictured frame, even if new camera frames arrive during the model
request.

The 1 s camera-age check applies only when `observation()` acquires that snapshot.
Interpreting `point` later is a read-only query of that same validated copy. Model
inference taking more than 1 s does not make it stale and does not trigger another
capture or projection onto a newer frame. There is no small query TTL. A missing
snapshot, a missing/non-integer/nonpositive timestamp, or a timestamp after the
query clock is rejected. A legitimate old-but-nonfuture timestamp remains valid.
Robot feedback freshness and PositionExecutor movement checks are unchanged.

A single-camera query requires finite axial depth `0 < Z <= 5` metres, intrinsics,
and `world_from_camera`. Depth is optical-axis Z, not distance along a normalized
ray: the local point is `[(u-cx)*Z/fx, (v-cy)*Z/fy, Z]`, then transformed into the
base frame. The simulator publishes camera pose as xyz + unit **wxyz**; the local
helper also accepts a rigid 4×4 transform. A two-view query uses RGB dimensions
and two calibrated rays without reading depth; it rejects nearly parallel rays,
intersections behind either camera, or ray disagreement over 2 cm. There is no
monocular or guessed-depth fallback.

A successful query result marks itself as `source: "captured_snapshot"`,
`evidence: "historical"`, `live_motion_target: false`, and returns
`image_timestamps_ns`, `queried_at_ns`, and `image_age_ms_at_query`. Each image in
the model input is labeled with its `captured timestamp_ns`, so the model sees
which exact frame a `point` pixel refers to and that the returned point is
historical evidence of that snapshot, not a live target. It does not imply the
world is unchanged and is never projected onto a newer frame. Returned points
are not persistent object tracks and do not automatically become motion targets.


| Base direction | Definition | 1 cm displacement |
| --- | --- | --- |
| `forward` / `back` | Along initial arm extension / opposite | `[±.01,0,0]` |
| `left` / `right` | Left / right, standing behind the base facing +X | `[0,±.01,0]` |
| `up` / `down` | Vertically up / down | `[0,0,±.01]` |

These are fixed base directions, not camera pixel offsets. Screen directions depend on
the selected camera profile; do not assume a fixed head-image/base mapping. Wrist image
axes change after rotation. The controller uses the wrist view when the target
is visible there and corrects one visible axis at a time. Static height priors
are explicitly supplied in `scene_reference`; they are not current object/contact
measurements and never replace wrist alignment or image-based completion. There
is no fixed rim-on-right closing rule.

#### Wrist-led grasping in tool coordinates

For the default Piper model the tool frame has the flange orientation, with its
origin offset 0.13503 m along flange +Z. Opposing fingers open along tool Y;
tool +Z is the approach axis and X is the other transverse axis. Tool +Z means
down only when the tool points down. `point=tool` selects an origin; choosing
`frame=tool` additionally selects moving axes. These are distinct choices.

```json
{"action":"tool_move","direction":"z+","reason":"腕图显示碗沿进入夹口附近，沿工具接近轴推进一步后复查"}
```

```json
{"action":"tool_rotate","axis":"z","sign":1,"reason":"在有间隙处绕工具轴调整夹口方向"}
```

Let the current tool pose in base coordinates be `(p,R)`. A tool increment uses
`p_new = p + R*d_tool` and `R_new = R*Exp(w_tool)`. Both increments use axes
**before** this action; rotation keeps the tool origin fixed, not the wrist
camera or fingertips. The resolved base displacement and rotation vector are
recorded alongside `requested_frame=tool`. A pictured-to-current tool drift
over 1 mm or 0.01 rad rejects layered tool commands and requests reobservation.
The existing IK, per-joint 0.08 rad bound, FK verification, cancellation and
verified recovery remain in effect. No trajectory or collision guarantee is added.

The intended feedback sequence is:

1. Acquire the bowl with the head view; open the fingers and approach with clearance.
2. While clear, use `rotate down` and FK tilt feedback to approach a downward posture.
3. Switch to wrist-led corrections: align **one rim segment** with the visible
   opposing finger pads, using tool X/Y. Do not center the entire bowl or assume
   image center equals the grasp point. `tool_origin_pixel` is robot geometry,
   not a detected fingertip or a guaranteed contact location.
4. Compare wrist images after each step. The gripper is nearly fixed in its own
   camera; the stationary bowl moves in the opposite image direction to camera
   translation. `tool_axes_optical` helps form a direction hypothesis, but
   optical Z changes scale and pixel gaps are not distances. Recheck after tilting.
5. Once the rim enters the closing region, advance along tool +Z only with
   visible clearance, then close. Avoid endless lateral polishing or alternating
   point queries and motions. `point` remains optional; single-camera queries
   read depth and should not be used for an RGB-only task.
6. GRASP completion checks visible enclosure by closed fingers, not lifting.
   LIFT performs a small base-up test with the gripper closed, checking that the
   bowl follows and separates from support. If it stays put, reopen and realign;
   closing feedback alone never proves a grasp. Occluded enclosure is unverified.

These are model instructions, not an automatic image-servo controller or grasp
detector. No bowl location or contact truth is supplied by the tool-frame guide.
Position-only closing can still time out when blocked by an object; verified
holding does not establish sustained grip force. Autonomous grasp success must
still be demonstrated on fresh images and independently checked.

The local interpreter maps move/rotate to tool-point EEF deltas, checks the
pictured and fresh joint poses, and passes validated absolute joint targets to
PositionExecutor. Each joint step is at most 0.08 rad and stays within limits.
The visual EEF validator permits 1–5 cm nonzero translation and at most 0.15 rad
rotation. Layered move/tool_move use `step_cm` (default 1), and discrete rotations
remain at most 0.04 rad. Longer requests are fully preflighted as Cartesian
waypoints at most 1 cm apart, then position-verified one by one. The 1 cm minimum
applies to model requests; internal interpolation can be finer. A failed segment
abandons all remaining segments; partial measured feedback is recorded. Failed IK or an invalid target is rejected without
scaling, changing direction, or writing a goal. Final joint-derived FK must
match within 1 mm and 0.01 rad. IK does not plan a straight Cartesian path or
check collision clearance. `--joint-only` disables FK/IK, so layered move/rotate
are rejected; wait, point queries, gripper commands and image checks remain.

`--legacy-actions` selects the old controller contract and skips planner/verifier.
It accepts `move_joint_absolute` (six bounded radians, ≤0.08 rad per joint),
`move_eef` (xyz + unit wxyz, `frame=base`, tool/flange point, nonzero translation
1–5 cm, rotation ≤0.15 rad), `move_eef_delta` (1–5 cm, or zero translation
with nonzero rotation; `frame=base|tool`, tool frame requires `point=tool`), scalar `set_gripper` in [0,1], and `wait`. In tool frame both `delta_xyz` and
`delta_rotvec` use current tool axes; in base frame both use base axes. Absolute
`move_eef` remains base only. Legacy reasons
may be up to 500 characters. Legacy mode cannot report verified task completion.
For Python callers, `run_loop` still defaults to `legacy_actions=True`; pass
`legacy_actions=False` to use the CLI's layered contract. No AgentHarness skill
or persistent `run_task` implementation is changed by this visual loop.

### 6. Replanning, evidence notes and dry run

`--stage-rounds 12` schedules a review after that many controller rounds in the
current stage. When the latest action succeeded with a measured joint/gripper
change and no cycle or repeated-failure trigger, this is a periodic review
(`kind=stage_review`), independent of `--max-replans`. The planner checks fresh
images and current robot FK/height before retaining or revising the remaining
plan; measured execution does not prove visual task progress. This avoids the
old automatic stop at 12 × (1 + 2) = 36 successful turns.

Measured cycling, two consecutive failures/rejections, or a stage interval ending
without successful measured change use the bounded stall budget
`--max-replans 2`. Waiting when this budget is exhausted may use the separate
wait-recovery budget below before pausing. Successful stage completion resets
its round counter; every new plan starts a new review interval. The default
500-round cap still applies across all plans. A false or invalid verification leaves the stage active and
does not call executor recovery. Controller/API exceptions terminate the loop
with cleanup; planner exceptions use the fallback stage, and verifier exceptions
record incomplete evidence. These roles use the same configured model/API by
default, so separate calls do not guarantee independent errors.

On exit, `evidence_notes.json` stores at most eight recent stage checks with the
round, stage ID, boolean completion and visible-evidence reason. It contains no
automatically generated executable code or motion plan. Reuse is explicit via
`--notes PATH`; the loader requires version 1, a file at most 16 KiB, at most
eight bounded evidence records, and an exact task-text match. Notes are historical
text, not current success, and must be rechecked against images. They are not
automatically searched or loaded for the next run.

Useful options forwarded by `bash examples/piper_mujoco.sh agent`:

| Option | Behavior |
| --- | --- |
| `--task '...'` | Goal and visible completion condition; default is to grasp and lift the orange bowl with new-image evidence |
| `--max-rounds N` | Default **500** controller rounds; 0 removes the cap; planning/verifier calls are additional; Enter stops early |
| `--stage-rounds 12 --max-replans 2` | Review every 12 stage turns; bound stall plans separately to two |
| `--resume save/vision_<id>` | Attach to the existing scene, load bounded historical task/target/actions and plan again from fresh input |
| `--max-waits 3 --max-wait-replans 3` | Wait trigger and separate recovery-plan budget, shared by explicit `replan`; 0 disables recovery plans |
| `--notes save/vision_<id>/evidence_notes.json` | Explicitly reuse evidence for exactly the same task |
| `--verification-interval 6` | Automatic GRASP/LIFT checks on eligible close/upward events and periodic closed observation; 0 disables automatic triggers |
| `--allow-depth-point` | Explicitly enable single-camera depth point; default CLI rejects it before resolution |
| `--legacy-actions` | Use low-level compatibility actions without task verification |
| `--dry-run --max-rounds 2` | Local wait-only policy; skip every external model callback and do not generate motion goals |

Dry run still needs a healthy simulator, fresh head/wrist RGB, an unused output
directory, and a control lease. It uses the executor's ordinary verified holds
and cleanup. It does not request planning, verification, completion, or
replanning; the audit logs contain locally generated controller waits. It needs
no API key. A non-`--joint-only` run still loads Pinocchio, and CLI configuration
and notes are still validated. Dry run checks plumbing, not policy quality or a
successful grasp. Attach to an already-running scene to preserve its placement;
starting an owned scene initializes that simulator.

### 7. Failures, recovery and Enter

A failed action can partially move. Each result includes actual measured joints/
gripper, FK tool/flange poses and EEF residual errors when available. The model
must replan from this feedback and new images, not from the requested target.

| Outcome | Visual-loop policy |
| --- | --- |
| Tracking timeout or EEF residual failure, `failed` and stop confirmed | Read fresh healthy feedback; explicitly verify hold with `executor.recover()`; reobserve and let model choose a different action |
| Same failed action proposed again | Reject until a different motion succeeds; numeric spelling changes are not a new plan |
| Unknown outcome, unconfirmed stop, stale feedback, lost lease, failed recovery | Pause without resuming motion |
| Enter / cancellation | Stop, verify hold and release control; no recovery/retry |
| Consecutive failures/rejections | Default `--max-failures 3`; pause at limit; a positive override allows more attempts |

The recovery policy is explicit and limited to this simulator application.
Generic PositionExecutor still latches attempted failures until explicit recover;
old failed IDs/results are never rewritten as success. There is no automatic
replay. Suggested adjustments are a 1 cm retreat, lateral repositioning, or
small rotation at clearance; timeout alone does not prove contact or grasp.

With `--max-rounds 0`, interactive pauses wait for Enter. Positive limits (including the default 500) exit on a pause condition or when the cap is reached. While HTTP
is pending, Enter discards late replies without waiting for the server. Requests
already sent may still incur usage. Default continuous runs consume API calls;
`--interval 1` is a pause after each turn, not a 1 Hz control loop.

### 8. Rerun, logs and troubleshooting

The read-only viewer bridge copies published joint/object poses into a display
model and runs FK without physics stepping, commands, or a control lease. It
reopens/replays geometry on `viewer`, exits after the source stops publishing,
and uses the SDK's bundled binary to avoid Conda version mismatches. Visualization
uses simulation object poses for drawing; they do not enter the policy payload.
It can stay open while an agent controls the scene.

| File under `save/vision_<id>/` | Content |
| --- | --- |
| `latest_messages.json` | Exact most recent role input, including PNG data URLs; overwritten before each planner/controller/verifier callback |
| `model_calls.jsonl` | Round, role and exact request messages, raw response objects, or exception type for each callback; requests are flushed before the call, including interrupted calls |
| `turns.jsonl` | Current stage, raw controller reply, accepted/rejected action, execution/query/wait status, measured feedback, FK/recovery decisions, or independent verification and image timestamps |
| `control_*.jsonl` | Executor goals, feedback, holds and outcomes; a new executor/journal every 128 controller rounds |
| `run_context.json` | Version, task, starting round, sensor/verification options and limits; no credentials |
| `decisions.jsonl` | Structured scheduling decision and budget counters |
| `run_summary.json` | Exit reason, round/plan counters, task completion and last image check |
| `evidence_notes.json` | Version, exact task and at most eight stage-verification evidence records; written on exit, explicitly reusable with `--notes` |

Credentials from configuration/authorization are not included in role inputs.
Transport errors in the role audit store exception types, not provider error
bodies. Raw model replies, user task/notes and PNG images are stored as received;
keep these logs private and avoid putting secrets in model-visible text. An
unsupported raw reply remains in the audit but is not echoed into the next
controller observation. In-memory history/evidence is bounded; disk audit/image
logs accumulate. These logs do not provide cross-restart motion deduplication.
Fixed physical tests produce `save/mujoco_<id>.jsonl` and `.pick.json`; these
contain independent ground-truth evidence, not model observations. Viewer logs
are in `~/.local/state/cyr/viewer.log（自定义 XDG_STATE_HOME 时随其变化）`.

| Symptom | Check / action |
| --- | --- |
| Shared-memory busy after stop | Same discovery directory? Use the wrapper; do not unlink a live owner's segment |
| Control lock held | Stop the prior agent with Enter; open `viewer` rather than another agent |
| No window / window was closed | `bash examples/piper_mujoco.sh viewer`; check viewer.log and bundled SDK version |
| Repeated wait | Recovery planner receives fresh RGB/FK/height and current feasible alternatives; only pause when its separate budget is exhausted |
| IK / step rejection | Coordinate frame, quaternion order, 1–5 cm request norm, feasible options; reduce step or adjust posture/direction |
| Timeout near rim | Check actual FK error; verified recovery permits a different action, not pushing down repeatedly |
| Cameras stale | Capture timestamp, frame advancement, runtime load and graphics context |
| Rerunning a fixed check fails | Restart to restore object placement; resetting joints is insufficient |

### 9. Validation and remaining work

Current harness verification on 2026-10-26 (rendering enabled, no model API calls):

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CYR_TEST_MUJOCO_RENDER=1 .venv-harness/bin/python -m pytest src/robot/harness/tests -q --tb=short
```

Result: **261 passed, 2 skipped** in 28.06 s. Both skips require external Marvin
assets. Coverage includes axial depth with translated/rotated xyz+wxyz and
matrix camera poses, depth-free stereo and degenerate rays, invalid pixels and
calibration, exact snapshot copying/freshness, bounded token interpretation,
role input privacy, strict action/plan/verifier schemas, prompt package data,
planner fallback, controller exception cleanup, failed-action recovery/guard,
independent stage completion, bounded replanning, notes validation and dry run.
Tool-control cases check all six tool directions, all signed local rotations,
noncommuting body/base rotations, combined increments using the original axes,
unchanged step limits, robot-only wrist references, off-image/behind-camera
origins, drift rejection, frame waiting and cancellation. Policy tests use
deterministic sessions/local FK/IK and forbid real HTTP requests.
The visual regression tests expect `robot.joint_limits_rad`, no `scene_priors`,
and a single latest history entry. A separate, isolated cyr MuJoCo scene ran
four tool translations and two signed tool-Z rotations through the actual
layered execution loop: all six succeeded, maximum FK residual 0.0000631 m and
0.000316 rad. Its robot-only wrist guide was available. The fast-planner camera
refresh race was reproduced and fixed by bounded new-frame waiting. This probe
used scripted action tokens, not a model policy or a grasp trajectory; it did
not touch the user's existing scene. No live model or hardware run is claimed.

The snapshot-query latency fix was checked separately on 2026-10-26 with
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-harness/bin/python -m pytest src/robot/harness/tests/test_policy.py src/robot/harness/tests/test_vision.py -q --tb=short`:
**175 passed** in 10.64 s. Regressions cover queries delayed beyond 1 s, 60 s and
one day, exact copied geometry despite later frame changes, source timestamps
and historical evidence in model inputs, invalid/missing/future timestamps for
both cameras, unchanged capture freshness, and no motion goals on query success
or rejection. These are offline checks; no live model or running robot service
was used for this fix.

Ordinary pytest does not call a model API. Missing optional dependencies/assets
skip their relevant checks. For broader runtime/rendering checks, use a separate
discovery directory so service tests cannot stop an unrelated service. Rendering
requires a graphics context; the following is an optional command, not a result
of the current offline verification:

```bash
python -m pip install -r requirements/harness-test.txt
XDG_STATE_HOME="$(mktemp -d)" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CYR_TEST_MUJOCO_RENDER=1 python -m pytest src/robot/harness/tests src/robot/runtime/tests/test_watchdog.py src/robot/runtime/tests/test_config.py src/robot/runtime/tests/test_reset.py src/robot/runtime/tests/test_loops.py -q
```

Earlier integration checks reported normal cyr lifecycle, camera rendering,
wrist/head transforms, a fixed physical orange-bowl grasp/lift/release (about
10.7 cm lift), local-model RGB input, 1 cm moves, rotation-only control, and
injected timeout followed by model-selected recovery motion. A previous 16-round
visual trial moved but did not grasp the bowl. These historical checks do not
validate autonomous success of the new layered policy. Head/wrist transforms
and provenance are documented in
[the asset README](../assets/sim/piper_mujoco/README.md).

Remaining limits: approximate mass/friction and convex robot collision, disabled
robot self-collision, ideal pinhole imagery (Mac OpenGL may warn about depth
precision), no collision planner, no completed STEP pipeline, and no validated
general pick/place or autonomous stacking. Position-to-zero gripper verification
can fail against a bowl; verified position hold does not preserve closing effort
for a force-maintaining grasp. The independent physical check deliberately uses
a dedicated closure/contact routine. Hardware stopping behavior is not validated.

Point-query limitation: a delayed result describes the captured scene only.
`image_age_ms_at_query` is the age at `queried_at_ns`, not a continuously updated
age when the result appears in later rounds. The controller receives source and
current image timestamps and must recheck current images and robot feedback
before choosing movement. Moving objects or cameras are not tracked or
compensated; querying an old snapshot provides no current collision clearance
or live motion target.


### Episode progress and manual baseline (2026-10-26)

See [the current architecture, prompt snapshots and Chinese debugging guide](harness_agent_review.md).
Valid planner `stages` plus an optional short `reason` now parse correctly. Invalid
responses explicitly print a fallback notice and are recorded in `plans.jsonl`.
The common prompt no longer asks planners to emit action-shaped replies.

`EpisodeProgress` keeps textual target/part selection across a grasp retry, up to
eight recent actions and measured FK effects, completed image checks and close
attempt state. Four successful measured translations in the same stage travelling
at least 3 cm but returning within 3 mm trigger bounded replanning; this is a robot
motion cycle, not a visual object-progress verdict. Replanning retains history but
starts a new cycle window. Later grasp stages may select a different object.

The controller retains original task constraints. The verifier compares stage-start
and new current RGB pairs without receiving robot/action success claims. Wrist
instructions allow the far finger to be occluded, require reidentification of the
same rim, discourage endless polishing, and explain close → inspect → small lift
trial → inspect → reopen/retry. The position gripper still does not guarantee force
or persistent squeeze after a blocked close/verified hold.

A bounded `joint_step` action (`joint_index=1..6`, integer `sign=±1`) now permits
single-joint posture changes of 0.04 rad via existing joint limits and the executor.
It does not preserve the EEF pose or guarantee collision avoidance. Use clearance
and reobserve, especially when an EEF vertical rotation fails IK.

```bash
# Stop the old agent with Enter first, leaving the service/viewer running.
bash examples/piper_mujoco.sh teleop
# Then, after leaving teleop with an empty line:
bash examples/piper_mujoco.sh agent
```

Teleop calls no model and uses the same interpreter and executor as the agent.
Tokens: `x+ x- y+ y- z+ z-` (tool, default 1 cm), `rx+ rx- ry+ ry- rz+ rz-`
(tool 0.04 rad), `forward back left right up down` (base, default 1 cm),
`j1+ j1-` through `j6+ j6-` (joint 0.04 rad), `vertical`, `open`, `close`.
An empty line stops. Logs default to `save/teleop_*`; per-action requests include
captured RGB for local inspection and are never sent to an API by teleop.

Isolated smoke `save/progress_smoke_ca36c2bce6` verified ten position actions
(open, five tilts, tool approach, close, two base-up steps), without an autonomous
grasp claim. A separate scripted pick probe `save/same_tools_pick_1719400b3c`
reached twelve successful motions then failed IK twice while tilting vertically
at the initial tool location; it never reached a grasp. This is a limitation of
that posture path, not evidence that a close or lift succeeded.

Final episode-progress/posture regression (2026-10-26): `288 passed, 2 skipped` in 30.90 s, with `CYR_TEST_MUJOCO_RENDER=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-harness/bin/python -m pytest src/robot/harness/tests -q --tb=short`. Skips require external Marvin assets. The earlier 261-test count above is historical.

Additional isolated joint-step smoke: `save/joint_step_smoke_c3944f6dcd`, joint 6 +0.04/-0.04 rad both succeeded; temporary service cleaned up.


### Nearer orange bowl and bounded descent (2026-10-26)

Orange bowl_0 now starts at `[0.32, -0.12, 0.08]` instead of
`[0.42, -0.12, 0.08]`, 10 cm toward the base. Other bowls stay in place.
The XML, converter, manifest and fixed physical pick check agree on this layout.
At nominal rim tool xy `[0.32, -0.04]` and x ±1 cm, the same Pinocchio
interpreter as the agent passed six 1 cm tool +Z steps from vertical tool
height 9.5 cm to about 3.5 cm and six -Z retreats, with physical position
feedback and normal joint-step bounds. This does not guarantee arbitrary
orientations remain reachable. The initial wrist image may clip the near bowl
at its lower edge; acquire/approach with the head view, then align with wrist.

The isolated runtime physical pick check passed bilateral contact, about
10.9 cm lift and release (`save/mujoco_516b679501.pick.json`). Its fixed poses
and continuing-close flow are not an autonomous model grasp result or an input
to the agent. Stop the old agent with Enter, run
`bash examples/piper_mujoco.sh restart`, then
`bash examples/piper_mujoco.sh agent` to load the layout. Joint reset or
restarting only the agent does not reset object placement.


### Variable step and tabletop feedback (2026-10-26)

Controller actions `move` and `tool_move` accept optional numeric `step_cm` in
**[1, 5]**, including fractions such as 2.5; omission preserves 1 cm. This is a
request limit, not a promise of reachability. Example:

```json
{"action":"tool_move","direction":"x+","step_cm":3,"reason":"Visible clearance permits a larger local approach"}
```

The full endpoint is retained. Preflight computes base-frame Cartesian waypoints
at most 1 cm apart; it checks ALL IK/FK solutions, joint limits and ≤0.08 rad
adjacent joint changes before writing. Internal interpolation can be below 1 cm,
so 2.5 cm uses three internal waypoints; this does not expose sub-centimetre model
actions. PositionExecutor verifies every waypoint. Failure/cancellation abandons
the rest and preserves partial feedback; it does not silently shrink/retry the
model request. No new RGB/model decision occurs inside a long action. This is
neither collision planning nor a straight continuous Cartesian servo guarantee.
The existing direction hints test only 1 cm. Even a larger supported request can
be rejected for a particular direction/posture.

Use 3–5 cm for visible clear approach/retreat, around 2 cm for intermediate
corrections, and 1 cm near a rim/table or uncertain clearance. Rotations and
joint_step keep their previous limits. Teleop accepts `x+ 3`, `down 2.5`, etc.
A bare direction still moves 1 cm. Restart only the agent process to load these
code/prompt changes; no runtime restart is needed if the current scene is already
loaded correctly.

Controller now receives `scene_reference`:

| Field | Meaning |
| --- | --- |
| `table_top_z_m` | Static tabletop top in base frame; bundled box is -0.025 + 0.025 = **0 m** |
| `tool_z_base_m` | Current nominal tool-origin Z from measured joints and FK |
| `tool_height_above_table_m` | tool Z minus static tabletop Z, in metres |
| `bowl_height_above_table_approx_m` / `bowl_rim_z_approx_m` | Rough bundled upright bowl height/rim, about **0.05 m** above table |
| `upright_grasp_tool_z_reference_m` | Fixed upright-grasp example's tool Z, about **0.035 m**; a reference, not a target |

The table comes from the attached component's declared local scene, not RGB or
live object truth. Unsupported/missing/rotated tables are marked unavailable;
custom scenes do not inherit the bundled bowl height. Verifier remains image-only
and receives none of these height fields. Near tool heights 3–8 cm, local contact
with the upright bowl rim is plausible; use wrist feedback and a 1 cm step. The
tool origin is not the lowest finger/robot point: tilt and other objects may cause
contact at higher tool Z. Bowl height references assume an upright bowl on the
table and do not describe a moved, tilted or stacked bowl.

Validation of variable-step/height support: **322 passed, 2 skipped** (34.51 s, rendering enabled). Isolated runtime `save/variable_step_smoke_a8622a833a` verified six base actions at 5/3/2.5 cm; two tool-Y actions were preflight-rejected by the preserved joint-step bound. Separate `save/variable_step_smoke_d11773b445` verified four tool-X/Z ±2.5 cm actions, each in three internal segments. All temporary services were cleaned up. These are scripted control/height checks, not an autonomous grasp verdict.


### Wait recovery and explicit strategy changes (2026-10-26)

Layered mode no longer pauses immediately after three waits. `--max-waits 3`
triggers a fresh recovery plan. The controller can also return
`{"action":"replan","reason":"Current lift presumes a grasp but the new images contradict it"}`
to request it immediately without waiting. Both use a separate cumulative
`--max-wait-replans 3` budget, independent of ordinary `--max-replans 2`.
The counter does not reset after reopening, a successful motion or a new stage,
so repeated bad decisions cannot cause unbounded model calls. Budget exhaustion
still pauses; 0 disables these recovery plans. Legacy/dry-run keep their wait
pause behavior. No automatic motion, gripper opening, scene reset or command
replay is inserted by the recovery supervisor.

On recovery the planner now receives fresh robot joints/gripper, FK/height,
1 cm feasible base/tool directions, joint-limit margins and predicted FK effects
of valid ±0.04 rad joint steps, as well as RGB and history. The controller sees
these alternatives after a wait or during recovery. `loop_status` tracks wait
counts and exact RGB-byte equality/joint change since the last wait; image equality
is not a target tracker or grasp verdict. `recovery_options` uses no object/contact
truth and does not check collisions. Chosen actions still pass normal fresh-state
validation and PositionExecutor verification. The verifier gets none of the new
robot/recovery fields.

Prompts distinguish an empty attempt from an uncertain held object. Current images
supporting a missed grasp on its support permit reopening, changed approach or a
small posture adjustment followed by re-grasp. Possible holding requires a
visually supported retreat/support/verification, not blind opening. Infeasible
upward or tool-withdrawal motion does not invalidate every other direction. At a
joint limit, a temporary posture relaxation with clearance may help; waiting at
the same pose cannot remove the limit or rigid-camera occlusion.

The reported run `save/vision_caa4244d0b` had 24 motion records all at tool Z about
0.2247 m, no actual descent, and no `scene_reference` input (an older process).
Its final pose had joint 5 near its upper limit: up/tool withdrawal were blocked,
while base forward/back/lateral/down and several tool moves were feasible.
A physics replay verified that joint 5 -0.04 rad restores margin and raises the
nominal tool point about 3.6 mm, with lateral motion too. This proves a robot
alternative exists, not collision clearance or an autonomous grasp.

Isolated runtime `save/wait_recovery_smoke_665c9f1b22` verified scripted close →
three waits → recovery plan → open → joint adjustment → changed approach, with
fresh FK and 12 joint candidates in the recovery planner input. It did not call
a model or establish autonomous grasp success. Stop the OLD agent with Enter,
then run `bash examples/piper_mujoco.sh agent` to load the new code/prompts.

Wait/explicit-replan recovery regression: **332 passed, 2 skipped** (38.54 s, MuJoCo rendering enabled). This includes finite recovery budgets, explicit replan, resuming through open/change-approach after repeated waits, fresh recovery-planner constraints, hidden-state boundaries, Enter during planning, unconfirmed-stop blocking and physics replay of the logged joint-limit posture. The two skips require external Marvin assets.

Default limit update (2026-10-27): CLI and Python `run_loop()` default to 500 controller rounds. Movement, gripper, point, wait and replan turns all count; internal waypoints and planner/verifier calls do not add controller rounds. Completion, failure/recovery limits and Enter can stop earlier. `--max-rounds 0` removes only the round cap. Manual teleop explicitly keeps no round cap.


### Continue after a budget stop (2026-10-27)

Keep the existing MuJoCo service running and let the old agent release its lease.
For example, the 36-turn run `vision_43f42cb2cf` ended after two successful
1 cm tool-Z approaches; its final measured tool Z was about 0.113 m. The old
12-turn/two-replan coupling stopped the controller before reaching the 500-round
cap; it was not an IK or tracking failure.

```bash
cd /Users/niantian/projects/control_your_robot
bash examples/piper_mujoco.sh resume save/vision_43f42cb2cf
```

The `resume` wrapper opens/reuses Rerun and attaches to the existing service;
it never starts or resets a scene. `agent --resume PATH` also skips automatic
service startup. Direct Python use is `python examples/piper_mujoco_agent.py
--resume PATH` with the same optional environment and XDG discovery namespace;
`--resume` implies `--attach`. If the service is absent, attachment fails: start
a scene explicitly, then treat the prior history as potentially stale.

Continuation loads the original task and at most eight sanitized recent actions,
textual target selection, close attempts and historical image checks from the
selected run. It first obtains fresh RGB/joints/FK/height and requests a new plan.
It does not replay a pending motion, restore object physics or treat a historical
grasp/lift as current success. Only allowlisted typed fields are loaded; model
message/image files, arbitrary result fields and API configuration are not read.
Older layered runs with `evidence_notes.json` and `turns.jsonl` are supported.
No automatic “latest run” selection is used.

Logs go into a new `save/vision_<id>` directory. Round numbers continue (36 → 37),
but `--max-rounds 500` means up to **500 additional controller turns** in this
invocation. Stage/stall/wait budgets start fresh after this explicit user action.
The original task is retained; omit `--task` or supply exactly matching text.
Press Enter to stop. To begin afresh instead, run `agent` without `--resume`;
that loses the prior task history but still reuses a running scene. To reset
physical state/layout, explicitly restart the service before a new task.


Continuation/review validation: **343 passed, 2 skipped** (57.61 s, MuJoCo rendering enabled; skips require external Marvin assets). Isolated physical run `save/continuation_smoke_c1d3fe8c28` released the executor after two scripted joint steps, then resumed the same simulator at rounds 3–4. Both new actions succeeded and simulation time kept increasing. No model API was called; this verifies continuation mechanics, not autonomous grasping.


### Structural review and automatic image checks (2026-10-27)

See [the consolidated review](harness_refactor_review.md) for measured failures and remaining limits. Scheduling now lives in `supervision.py`, segmented execution in `visual_execution.py`, and offline summaries in `diagnostics.py`. CLI automatically checks GRASP/LIFT images after eligible close/upward-probe events, plus periodic safe closed observation (default interval six stage turns). The latest check and up to four grasp-attempt histories survive replanning. Image checks never generate motion. Motion outcomes and visual completion stay separate, and an ambiguous check does not prove a miss. The verifier remains image-only.

The Python adapter retains compatibility defaults: `verification_interval=0`, `allow_depth_point=True`. For the new layered behavior, pass `legacy_actions=False, verification_interval=6, allow_depth_point=False`; the CLI does so by default. Controller prompts compose common + controller + tools + grasp.

The runtime YAML now selects `head_camera_profile: oblique`, a fixed overview at `[0.75,-0.65,0.65]` looking at `[0.32,0,0.04]`, 50° vertical FOV. Published calibration follows the renderer. The original RoboTwin wrist mount stays 37°. The controller constructor and underlying MJCF retain the original `robotwin` head profile. Restarting a scene applies the runtime profile and resets physical state; resuming a live service preserves its existing camera configuration. The oblique view improves visibility at a recorded problematic posture; it is not an all-pose visibility guarantee.

`bash examples/piper_mujoco.sh report --recent 8` summarizes turns, close commands, image checks, depth queries and exit reasons offline. Position-gripper sustained holding remains a separate unresolved capability; no autonomous grasp success is claimed by these changes.


## RoboHarness-inspired visual evidence and motion plans

The visual CLI now enables RGB patch tracking and read-only `plan_motion` / single-use `execute_plan` tools. Base, tool and calibrated camera frames support translation and rotation; original direct controls remain available. `--no-grounded-tools` disables the added layer. These tools do not enable depth, semantic detections, collision planning or contact truth. See the [source comparison, architecture and protocol guide](harness_roboharness_adaptation.md) for units, freshness checks and remaining grasp-control limitations.
