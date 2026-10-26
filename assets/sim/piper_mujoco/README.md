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
Published `world_from_camera` uses OpenCV right/down/forward. Both camera
FOVs are the RoboTwin D435 profile's **37° vertical**, at 320×240 by default.
The head camera in this base frame is `(0, 0.032, 0.6)`, looking along
`(0.6, 0, -0.8)`, with left `(0, 1, 0)`. The scene/render appearance is not an
exact recreation of RoboTwin lighting/textures; these are ideal RGB/depth
cameras, not an emulation of D435 stereo noise.

## Physics limits

The default layout was adjusted for a single-bowl grasp exercise (2026-10-26):

| Body / color | Initial base-frame xyz (m) | Purpose |
| --- | --- | --- |
| bowl_0 / orange | `[0.42, -0.12, 0.08]` | Target; rim visible in head and wrist at ready pose |
| bowl_1 / blue | `[0.46, 0.18, 0.08]` | Parked to the side |
| bowl_2 / green | `[0.25, 0.23, 0.08]` | Parked to the side |

The initial 0.08 m height lets the bowls settle onto the tabletop, near z=0.
The positions are also recorded in `manifest.json` and the offline converter.
The fixed physical pick test was updated to approach orange bowl_0's rim at
tool xy `[0.42, -0.04]`, and verifies bilateral contact, lifting and release.
These fixed poses and ground-truth checks are not sent to the visual agent.
Restart a running scene to load this layout; ordinary joint reset does not reset bowls.

Validation on 2026-10-26: the runtime pick check lifted orange bowl_0 by approximately
0.107 m with bilateral contact, then released it onto the table. A separate
16-round RGB-only `codex/gpt-6.1-sol` trial executed its motions successfully but
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
