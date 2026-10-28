# Optional Piper contact scene

This asset is selected only by `config/runtime/piper_mujoco.yml`. It is not the
Pinocchio teaching scene. All units are metres, radians and kilograms.

## Provenance and reproduction

- Robot: repository `../piper/piper_single.urdf` (RoboTwin-derived).
- Three bowls: three independent instances of RoboTwin `objects/002_bowl`,
  `model_id=3`, original scale `[0.05, 0.05, 0.05]`.
- Wrist mount and meshes: RoboTwin `embodiments/piper/piper.urdf` and its
  `camera_mount`/`camera` visual meshes.
- Head pose: RoboTwin `embodiments/piper/config.yml`, converted from its world
  to this scene's base-link frame using the inverse `robot_pose` transform.
- Source assets were already present in the local RoboTwin checkout. No bulk
  archive or download is required at runtime. `manifest.json` contains SHA-256
  digests of the exact source files; the local RoboTwin code revision was
  `0a8cf79191b179a8f349cf8660fdcce2cc92dd59` (asset hashes are the identity).
- Upstream dataset: https://huggingface.co/datasets/TianxingChen/RoboTwin2.0
  (dataset card declares MIT). Upstream notice is in `ROBOTWIN_LICENSE`.

To regenerate explicitly, install the optional `harness-assets` tools
(trimesh, pycollada and fast-simplification), then from the repository root:

```bash
python examples/assets/prepare_piper_mujoco.py --robotwin-assets /path/to/RoboTwin/assets
```

The converter uses URDF fixed-axis XYZ RPY rotations (`eulerseq="XYZ"`).
It preserves source inertias and writes position actuators; commands change
actuator goals, not measured qpos. Visual textures are replaced by solid colors.
The camera visual mesh is simplified to 20,000 triangles; this does not change
its mounting frame or optical calibration. Its visuals are massless and have no
collision geometry because the source camera links have no inertials/collisions.

## Camera coordinate chain

The wrist sensor is on **URDF link `camera`**, matching RoboTwin's
`envs/robot/robot.py` link lookup and `envs/camera/camera.py::update_wrist_camera`:

| Joint | Translation in parent (m) | Fixed-axis RPY (rad) |
| --- | --- | --- |
| link6 → camera_mount | `0 0 -0.077` | `0 0 1.5708` |
| camera_mount → camera | `0.032 0.055 0.115` | `0 -1.1116926535897932 -1.5707963267948966` |

Visual origins (`camera_mount`: z=0.08; `camera`: xyz=0.0015,-0.0315,0 and
rpy=0.1,0,-pi/2) affect the meshes only, NOT the sensor pose.

SAPIEN camera axes are forward/left/up. MuJoCo uses right/up/backward;
its local axes expressed in SAPIEN are right=-Y, up=+Z, backward=-X.
Published `world_from_camera` uses OpenCV right/down/forward. In this source MJCF both camera
FOVs are the RoboTwin D435 profile's **37° vertical**, at 320×240 by default.
The head camera in this base frame is `(0, 0.032, 0.6)`, looking along
`(0.6, 0, -0.8)`, with left `(0, 1, 0)`. The scene/render appearance is not an
exact recreation of RoboTwin lighting/textures; these are ideal RGB/depth
cameras, not an emulation of D435 stereo noise.

## Physics limits

The default layout was adjusted for a single-bowl grasp exercise (2026-10-26):

| Body / color | Initial base-frame xyz (m) | Purpose |
| --- | --- | --- |
| bowl_0 / orange | `[0.32, -0.12, 0.08]` | Target moved 10 cm toward the base; acquire with head view, then align with wrist |
| bowl_1 / blue | `[0.46, 0.18, 0.08]` | Parked to the side |
| bowl_2 / green | `[0.25, 0.23, 0.08]` | Parked to the side |

The initial 0.08 m height lets the bowls settle onto the tabletop, near z=0.
The positions are also recorded in `manifest.json` and the offline converter.
The fixed physical pick test was updated to approach orange bowl_0's rim at
tool xy `[0.32, -0.04]`, and verifies bilateral contact, lifting and release.
These fixed poses and ground-truth checks are not sent to the visual agent.
Restart a running scene to load this layout; ordinary joint reset does not reset bowls.

Validation of the nearer layout on 2026-10-26: the runtime pick check lifted orange bowl_0 by approximately
0.109 m with bilateral contact, then released it onto the table. A separate
16-round RGB-only `codex/gpt-6.1-sol` trial on the preceding, farther layout executed its motions successfully but
remained in visual alignment and did not grasp/lift the bowl. The fixed physical
check establishes layout feasibility, not visual policy success.

Each bowl uses 24 separate convex collision pieces from the original GLB;
the cavity is not collapsed into a single convex hull. Bowl mass is an explicit
teaching assumption of 0.08 kg; inertia/friction are approximate. Robot collisions
use one convex hull per visual link; robot self-collision is disabled. This is
sufficient for a small contact platform test, not calibrated real hardware.
The default gripper range follows this repository's `[0, 0.04]` joint limits;
RoboTwin's original config has a different normalized gripper mapping.

Tests cover falling/settling, physical pinch/lift/release, and a separately
initialized three-bowl support fixture. The support fixture is not evidence of
a robot executing a complete three-bowl stacking trajectory.

The nearer orange layout also passed six consecutive 1 cm tool +Z descents
from a vertical tool height of 0.095 m to about 0.035 m, followed by six
-Z retreats, using the agent's Pinocchio interpreter and normal joint-step
bounds. This was checked at the nominal rim x and x ±0.01 m in physics.
It does not make all wrist orientations/redundancy branches IK-feasible.
The initial wrist view clips much of the nearer bowl at its lower edge; use
the head view to approach/reacquire before local wrist alignment. The physical
pick uses fixed poses and continued closing, not the model's position executor.
Runtime evidence: `save/mujoco_516b679501.pick.json`.


The runtime YAML selects the controller's `oblique` head profile (fixed base position `[0.75,-0.65,0.65]`, looking at `[0.32,0,0.04]`, 50° vertical FOV) to reduce arm occlusion. This overrides the head camera at setup and updates published calibration; it does not alter this source MJCF or the RoboTwin wrist mount. Set `head_camera_profile: robotwin` to retain the source head view. Changes require a new runtime instance.


### 持续夹持（2026-10-28）

场景采用椭圆摩擦锥和 `impratio=100`，降低软接触摩擦导致的缓慢滑移；NoSlip 保持关闭，生成器同步设置。PiperMuJoCoController 的 `gripper_preload_n` 默认 2 N/指，仅在显式闭合时激活，总执行器力仍受 ±10 N 限制，张开/运行时保持/停止撤去预紧力。它们不改变物体质量、摩擦系数、相机或视觉网格。固定轨迹验证须包括抬升后至少 15 秒保持及释放，不能仅验收瞬时抬升。修改场景和驱动后须重启服务，当前场景会重置。
