# Robot harness: verified execution and optional visual simulation

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
the orange bowl. The visual agent supports continuous joint/EEF control and
bounded recovery, but autonomous bowl grasping/stacking has not been validated.

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

The launcher always uses `.venv-harness` and `save/mujoco-runtime`, regardless of
the shell's active environment. `agent` ensures the scene is started, opens Rerun,
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
export XDG_STATE_HOME="$PWD/save/mujoco-runtime"
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

### 4. What the visual model receives

| Input | Source and limits |
| --- | --- |
| RGB | Current head/wrist pair; from round 2, also the immediately previous pair |
| Proprioception | Six joint angles and normalized gripper opening |
| FK | Base-frame tool/flange poses, tool axes, tool height and downward tilt from joints + Pinocchio |
| Feasibility hints | Local IK/joint checks for six 1 cm directions and a downward-tilt rotation suggestion |
| Context | User task, schemas, limits, coordinate guide and last eight actions/results |
| Scene priors | Authored tabletop/bowl-height and tool-offset references described below |

No live object poses/velocities, contacts/forces, depth maps, camera calibration
matrices, rewards/success oracle, scene XML/URDF, or fixed pick trajectory go to
the model. Feasibility hints check the robot only; they do not check collisions.
The full frame stays local and is rebuilt through an allowlist before HTTP.
This is an API boundary, not an OS sandbox for arbitrary plugins.

Robot feedback must be at most 500 ms old, images at most 1 s, image timestamps
must advance across rounds, and camera skew must be at most 200 ms. The runtime
publishes at 50 Hz, physics steps at 500 Hz, cameras sample at 10 simulated Hz.
Repeated runtime frames do not mean a newly captured image.

### 5. Actions, frames and posture

The model returns one JSON action and a short Chinese `reason` (at most 500 chars).
All commands are validated locally and run through PositionExecutor.

| Action | Fields / constraint |
| --- | --- |
| `move_joint_absolute` | `joint`: six absolute radians; within limits; each joint changes at most 0.08 rad |
| `move_eef` | `pose`: xyz metres + unit **wxyz**; `frame=base`, `point=tool` or `flange`; nonzero translation 1–2 cm, rotation at most 0.15 rad |
| `move_eef_delta` | `delta_xyz`: base-frame vector, nonzero norm exactly 1 cm; optional `delta_rotvec`: base-frame right-hand radians, norm at most 0.15 |
| `set_gripper` | `gripper`: scalar [0,1], 0 closed, 1 open; position feedback does not prove grasp |
| `wait` | Hold and observe again; after `--max-waits 3` consecutive waits, pause calls |

Pure rotation permits `delta_xyz=[0,0,0]` if rotation is nonzero. Omit
`delta_rotvec` to preserve orientation. Composition is
`R_new = Exp(delta_rotvec) * R_current`, around the selected tool/flange point.
A useful small rotation is 0.02–0.04 rad (about 1.1–2.3 degrees). Both translation
and rotation can be included, subject to IK and the same 0.08 rad joint bound.

```json
{"action":"move_eef_delta","delta_xyz":[0,0,0],"delta_rotvec":[0,0.04,0],"frame":"base","point":"tool","reason":"在较高处调整抓取姿态"}
```

This illustrates the schema, not a universally feasible or correct direction.
Prefer the current `orientation_hint` when reducing downward tilt. IK uses a
local model, never writes the simulator qpos, and does not plan a straight
Cartesian or collision-free path. After motion, joint-derived FK must match
within 1 mm and 0.01 rad. Invalid targets are rejected, never silently shrunk.

| Base direction | Definition | 1 cm displacement |
| --- | --- | --- |
| +X / -X | Forward along initial arm extension / backward | `[±.01,0,0]` |
| +Y / -Y | Left / right, standing behind the base facing +X | `[0,±.01,0]` |
| +Z / -Z | Vertically up / down | `[0,0,±.01]` |

`point=tool` selects the control point, not a tool-frame command. Base axes stay
fixed after joint1/wrist rotation. Tool axes supplied by FK rotate with the tool;
local tool +Z motion requires `.01 * tool_z_axis_base` as a base displacement.
Positive base rotations follow the right-hand rule: X turns Y toward Z, Y turns
Z toward X, Z turns X toward Y. Quaternion/rotation-vector values are not Euler
degrees. Absolute xyz and delta xyz must not be interchanged.

The fixed head view maps gripper +Y left, -Y right; both forward +X and lift +Z
may appear upward, so consult FK height. A stationary bowl stays fixed in head
pixels until touched. Wrist-camera image directions change with wrist posture;
reassess them after rotation. Pixel distances are not metres.

### 6. Grasp layout and height references

| Bowl | Initial base xyz (m) |
| --- | --- |
| bowl_0 / orange target | `[0.42,-0.12,0.08]` |
| bowl_1 / blue, parked aside | `[0.46,0.18,0.08]` |
| bowl_2 / green, parked aside | `[0.25,0.23,0.08]` |

Bowls settle onto table z≈0. Target rim is visible in both initial views. Layout
is recorded in scene XML, converter and manifest. Those object coordinates and
the fixed test trajectory are not sent to the visual agent.

The explicitly supplied **static scene priors** are: tabletop z=0, nominal rim
z≈0.05 m, tool grasp height ≈0.035 m (reference band 0.03–0.04), staging tool
height ≈0.12–0.17 m. These are baseline references, not live object observations.
Flange-to-tool offset is 0.13503 m along local +Z; heights are not interchangeable.

The prompt asks for tool local +Z toward base -Z, roughly within 10° near grasp.
Adjust posture gradually at clearance; then, when the visible rim is near the
wrist-view gripper, stop endlessly refining pixels and descend in 1 cm steps,
reobserving near the nominal grasp-height band. Rim-on-right is a user-provided
cue to try grasping, not proof. Grasp/lift must be assessed from new evidence.

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

Interactive pauses wait for Enter; bounded `--max-rounds` checks exit. While HTTP
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
| `latest_messages.json` | Exact latest model-visible messages, including PNG data URLs; overwritten per turn |
| `turns.jsonl` | Validated/rejected actions, status, errors, FK and recovery decisions |
| `control_*.jsonl` | Executor targets, feedback, holds and outcomes; rotated every 128 rounds |

Keys are not logged. Histories/images in memory are bounded; disk logs accumulate.
Fixed physical tests produce `save/mujoco_<id>.jsonl` and `.pick.json`; these
contain independent ground-truth evidence, not model observations. Viewer logs
are in `save/mujoco-runtime/cyr/viewer.log`.

| Symptom | Check / action |
| --- | --- |
| Shared-memory busy after stop | Same discovery directory? Use the wrapper; do not unlink a live owner's segment |
| Control lock held | Stop the prior agent with Enter; open `viewer` rather than another agent |
| No window / window was closed | `bash examples/piper_mujoco.sh viewer`; check viewer.log and bundled SDK version |
| Repeated wait | Read model reason and images; wait is a policy choice, not an executed movement |
| IK / step rejection | Coordinate frame, quaternion order, 1 cm norm, feasible options; adjust posture/direction |
| Timeout near rim | Check actual FK error; verified recovery permits a different action, not pushing down repeatedly |
| Cameras stale | Capture timestamp, frame advancement, runtime load and graphics context |
| Rerunning a fixed check fails | Restart to restore object placement; resetting joints is insufficient |

### 9. Validation and remaining work

Run offline checks in a separate discovery directory so service tests cannot
stop an unrelated running service. Rendering tests require a graphics context:

```bash
python -m pip install -r requirements/harness-test.txt
XDG_STATE_HOME="$(mktemp -d)" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CYR_TEST_MUJOCO_RENDER=1 python -m pytest src/robot/harness/tests src/robot/runtime/tests/test_watchdog.py src/robot/runtime/tests/test_config.py src/robot/runtime/tests/test_reset.py src/robot/runtime/tests/test_loops.py -q
```

Final verification on 2026-10-26: **134 passed, 2 skipped, 3 deselected** in
all harness tests plus runtime watchdog/config/reset/loops/node/session/transport/
safety/service checks, with rendering explicitly enabled. The two skips require
external Marvin assets; the three deselections are two HMI interaction tests and
the Linux `/dev/shm` attachment probe. A fork-in-threaded-process deprecation
warning remains in the existing zombie-process test.
Ordinary pytest does not call a model API. Missing MuJoCo/Pinocchio/Marvin assets
skip only the relevant optional checks. The original environment remains free
of MuJoCo. Wider macOS runtime checks exclude the Linux `/dev/shm` attachment
probe and HMI interaction tests; neither is claimed fixed here.

Manually checked: normal cyr lifecycle, camera rendering, wrist/head transforms,
physical orange-bowl grasp/lift/release (about 10.7 cm lift), local-model RGB
input, 1 cm moves, rotation-only control, and injected timeout followed by
model-selected recovery motion. A 16-round visual trial executed motions but did
not grasp the bowl. Head/wrist transforms and provenance are documented in
[the asset README](../assets/sim/piper_mujoco/README.md).

Remaining limits: approximate mass/friction and convex robot collision, disabled
robot self-collision, ideal pinhole imagery (Mac OpenGL may warn about depth
precision), no collision planner, no completed STEP pipeline, and no validated
general pick/place or autonomous stacking. Position-to-zero gripper verification
can fail against a bowl; verified position hold does not preserve closing effort
for a force-maintaining grasp. The independent physical check deliberately uses
a dedicated closure/contact routine. Hardware stopping behavior is not validated.
