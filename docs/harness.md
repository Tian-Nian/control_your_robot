# Verified position harness

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

## Results and interruption

- `succeeded`: the position verification condition was satisfied by fresh observations. It does not imply zero velocity or task-level success.
- `rejected`: pre-execution feedback requirements failed. Input schema errors instead raise ValueError.
- `unknown`: a write may have occurred, but verification failed, timed out, was cancelled or lost control. The executor attempts a position hold only when fresh valid feedback and its lease are available. `hold_requested` means a hold target was written, not that physical stopping was confirmed.

After an unknown result, new requests are blocked for that instance. Inspect the reason, feedback, runtime status and hold outcome, close the executor, and explicitly establish a fresh context. Do not automatically retry incremental actions or interpret creating a new executor as resolving a physical fault. `close()` attempts a final hold, releases the lease and closes the log even when hold fails; it propagates that failure. It does not close the caller's Session.

Lease acquisition clears pending stream slots in the current runtime, but an already active motion plan can continue. Use a dedicated idle teaching service for this first version, as the example does. This is not a mode-independent stop controller. The local deadline cannot enforce a stop if Python or the transport blocks/crashes; a runtime-level watchdog is a separate future capability. Flush is not fsync or power-loss durability. The journal records submitted targets, not an independently measured driver-applied target after clipping. No images are collected by this executor.

## Existing API fixes

`ActionContract` now rejects empty/non-finite vectors, non-finite control frequency and a multi-element scalar gripper. `RuntimeEnv` preserves an eef target when adapting a simultaneous scalar gripper, and rejects further take_action calls after stop/max_steps. `stop()` still means ending policy submissions, not physically stopping the robot.

## Verification

```bash
python -m pytest src/robot/harness/tests src/robot/runtime/tests/test_loops.py -q
```

The tests cover invalid input, deduplication/conflicts, concurrent calls, cancellation, stale/frozen feedback, timeouts, loss of lease, evidence output and a real Piper Sim subprocess. Optional Marvin model tests skip only when their external URDF is unavailable. They do not replace Piper tests or demonstrate true hardware safety.

Validation on this development machine: the targeted harness + runtime loop suite passed **41 tests**, with **2 optional Marvin URDF tests skipped**. A broader runtime + harness run (before the final additional preflight-cancel test) had **100 passed, 4 skipped, 2 failed**: the attacher test assumes Linux `/dev/shm`, and the HMI exit test timed out on this macOS host. These failures are not reported as passing. The isolated Piper example passed headlessly; GUI presentation was not revalidated.


## Live DeepSeek smoke test

With `DEEPSEEK_API_KEY` exported in the current shell, run:

```bash
python examples/piper_harness.py --deepseek
```

The default model is `deepseek-flash`; `--model` selects another available DeepSeek model. The optional `--viewer` remains available. The example uses the official HTTPS endpoint and [DeepSeek Tool Calls](https://api-docs.deepseek.com/guides/tool_calls/), without an additional SDK. It sends only the simulated capabilities, joint state, fixed task and execution results. Credentials are read from the environment, not written to code or journal, and are removed from the simulator subprocess environment.

This is a bounded integration test, not an unrestricted agent: at most four model calls, exactly one tool per action round, with local validation against the three-step task before execution. It fails rather than silently substituting a scripted action if the model proposes an unexpected target. Normal pytest runs use offline checks and the deterministic simulation, never the live API.

On 2026-10-23 the live test passed: three model-selected skills were feedback-verified, followed by a completion reply; 4 API calls and 4,556 total tokens reported by the API. The three requests were move to q2=1.35, set gripper=0.5, return to READY_JOINT. This verifies the model/tool/harness/Piper Sim integration for that task, not general task planning or real hardware. The viewer was not tested in this run.
