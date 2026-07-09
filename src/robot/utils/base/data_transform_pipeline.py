''' Data transform pipelines for CollectAny write() '''

from robot.utils.base.data_handler import debug_print

import subprocess
import h5py
import numpy as np
import cv2
import os

from robot.utils.base.xpolicy_hdf5 import write_xpolicy_episode_metadata_to_file


def image_rgb_encode_pipeline(collection, save_path, episode_id, mapping):
    def images_encoding(imgs):
        encode_data = []
        padded_data = []
        max_len = 0
        for i in range(len(imgs)):
            success, encoded_image = cv2.imencode(".jpg", imgs[i])
            jpeg_data = encoded_image.tobytes()
            encode_data.append(jpeg_data)
            max_len = max(max_len, len(jpeg_data))
        for i in range(len(imgs)):
            padded_data.append(encode_data[i].ljust(max_len, b"\0"))
        return encode_data, max_len
    
    hdf5_path = os.path.join(save_path, f"{episode_id}.hdf5")

    with h5py.File(hdf5_path, "w") as f:
        obs = f
        for name, items in mapping.items():
            group = obs.create_group(name)
            if name in collection.condition["image"]:
                for item in items:
                    data = collection.get_item(name, item)
                    if item == "color":
                        img_rgb_enc, img_rgb_len = images_encoding(data)
                        debug_print(f"image_rgb_encode_pipeline", f"success encode rgb data for {name}", "INFO")
                        group.create_dataset("color", data=img_rgb_enc, dtype=f"S{img_rgb_len}")
                    else:
                        group.create_dataset(item, data=data)
            else:
                for item in items:
                    data = collection.get_item(name, item)
                    group.create_dataset(item, data=data)
    
    debug_print("image_rgb_encode_pipeline", f"save data success at: {hdf5_path}!", "INFO")

def general_hdf5_rdt_format_pipeline(collection, save_path, episode_id, mapping):
    def images_encoding(imgs):
        encode_data = []
        padded_data = []
        max_len = 0
        for i in range(len(imgs)):
            success, encoded_image = cv2.imencode('.jpg', imgs[i])
            jpeg_data = encoded_image.tobytes()
            encode_data.append(jpeg_data)
            max_len = max(max_len, len(jpeg_data))
        # padding
        for i in range(len(imgs)):
            padded_data.append(encode_data[i].ljust(max_len, b'\0'))
        return encode_data, max_len

    hdf5_path = os.path.join(save_path, f"{episode_id}.hdf5")
    with h5py.File(hdf5_path, "w") as f:
        left_joint, left_gripper = collection.get_item("left_arm", "joint"), collection.get_item("left_arm", "gripper")
        right_joint, right_gripper = collection.get_item("right_arm", "joint"), collection.get_item("right_arm", "gripper")
        
        qpos = np.concatenate([left_joint, left_gripper, right_joint, right_gripper], axis=1)

        actions = []
        for i in range(len(qpos) - 1):
            actions.append(qpos[i+1])
        last_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        actions.append(last_action)

        cam_head = collection.get_item("cam_head", "color")
        cam_left_wrist = collection.get_item("cam_left_wrist", "color")
        cam_right_wrist = collection.get_item("cam_right_wrist", "color")

        head_enc, head_len = images_encoding(cam_head)
        left_enc, left_len = images_encoding(cam_left_wrist)
        right_enc, right_len = images_encoding(cam_right_wrist)

        f.create_dataset('action', data=np.array(actions), dtype="float32")
        observation = f.create_group("observations")
        observation.create_dataset('qpos', data=np.array(qpos), dtype="float32")
        images = observation.create_group("images")

        images.create_dataset('cam_high', data=head_enc, dtype=f'S{head_len}')
        images.create_dataset('cam_left_wrist', data=left_enc, dtype=f'S{left_len}')
        images.create_dataset('cam_right_wrist', data=right_enc, dtype=f'S{right_len}')
    
    debug_print("general_hdf5_rdt_format_pipeline", f"save data success at: {hdf5_path}!", "INFO")


def _is_uncompressed_colors(color_data):
    if isinstance(color_data, (list, tuple)):
        if len(color_data) == 0:
            return False
        first = color_data[0]
        return isinstance(first, np.ndarray) and first.ndim == 3
    if isinstance(color_data, np.ndarray):
        return color_data.ndim == 4 and color_data.shape[-1] == 3
    return False


def _rgb_frame_to_jpeg(frame, quality=95):
    if frame.ndim != 3 or frame.shape[-1] != 3:
        raise ValueError(f"expected HxWx3 frame, got {frame.shape}")
    bgr = np.ascontiguousarray(frame[:, :, ::-1])
    ok, encoded = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("cv2.imencode failed")
    return encoded.tobytes()


def _encode_rgb_colors_to_jpeg_rows(colors, quality=95):
    colors = np.asarray(colors)
    if colors.ndim != 4:
        raise ValueError(f"expected [T,H,W,3], got {colors.shape}")
    rows = []
    max_len = 0
    for i in range(colors.shape[0]):
        jpeg = _rgb_frame_to_jpeg(colors[i], quality)
        row = np.frombuffer(jpeg, dtype=np.uint8)
        rows.append(row)
        max_len = max(max_len, row.shape[0])
    out = np.zeros((colors.shape[0], max_len), dtype=np.uint8)
    for i, row in enumerate(rows):
        out[i, : row.shape[0]] = row
    return out


def _ensure_compressed_colors(color_data, jpeg_quality=95):
    if _is_uncompressed_colors(color_data):
        colors = np.asarray(color_data) if not isinstance(color_data, np.ndarray) else color_data
        debug_print(
            "X_spark_format_pipeline",
            f"compressing uncompressed vision colors: {colors.shape}",
            "INFO",
        )
        return _encode_rgb_colors_to_jpeg_rows(colors, jpeg_quality)
    return color_data


def _compute_next_state_action(state):
    state = np.asarray(state)
    action = state.copy()
    if state.shape[0] > 1:
        action[:-1] = state[1:]
    return action


def X_spark_format_pipeline(collection, save_path, episode_id, mapping):
    output_path = save_path
    debug_print("X_spark_format_pipeline", f"save to: {output_path}/ start!", "INFO")
    left_eef = collection.get_item("left_arm", "eef")
    left_joint = collection.get_item("left_arm", "joint")
    left_gripper = collection.get_item("left_arm", "gripper")
    right_eef = collection.get_item("right_arm", "eef")
    right_joint = collection.get_item("right_arm", "joint")
    right_gripper = collection.get_item("right_arm", "gripper")

    cam_head_color = collection.get_item("cam_head", "color")
    cam_left_wrist_color = collection.get_item("cam_left_wrist", "color")
    cam_right_wrist_color = collection.get_item("cam_right_wrist", "color")

    episode_name = f"episode_{int(episode_id):07d}"
    data_dir = os.path.join(save_path, "data")
    hdf5_path = os.path.join(data_dir, f"{episode_name}.hdf5")
    preview_video_dir = os.path.join(save_path, "preview_video")
    merge_preview_path = os.path.join(preview_video_dir, f"{episode_name}.mp4")
    separate_preview_paths = {
        "cam_head": os.path.join(preview_video_dir, f"{episode_name}_cam_head.mp4"),
        "cam_left_wrist": os.path.join(preview_video_dir, f"{episode_name}_cam_left_wrist.mp4"),
        "cam_right_wrist": os.path.join(preview_video_dir, f"{episode_name}_cam_right_wrist.mp4"),
    }

    def get_cam_shape(img_bytes):
        if isinstance(img_bytes, np.ndarray) and img_bytes.ndim == 3:
            return np.asarray(img_bytes.shape)
        if isinstance(img_bytes, np.ndarray):
            jpeg_bytes = img_bytes.tobytes().rstrip(b"\0")
        else:
            jpeg_bytes = img_bytes.rstrip(b"\0")
        nparr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        shape = cv2.imdecode(nparr, 1).shape
        return shape

    def ensure_ee_joint_shape(gripper):
        gripper = np.asarray(gripper)
        if gripper.ndim == 1:
            return gripper.reshape(-1, 1)
        return gripper

    def rpy2quat(xyzrpy):
        xyzrpy = np.asarray(xyzrpy)

        if xyzrpy.ndim == 1:
            xyzrpy = xyzrpy.reshape(1, -1)

        if xyzrpy.ndim != 2 or xyzrpy.shape[1] != 6:
            raise ValueError(f"Expected input shape (N, 6), got {xyzrpy.shape}")

        xyz = xyzrpy[:, :3]
        roll = xyzrpy[:, 3]
        pitch = xyzrpy[:, 4]
        yaw = xyzrpy[:, 5]

        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        quat = np.stack([qw, qx, qy, qz], axis=1)
        return np.concatenate([xyz, quat], axis=1)

    left_gripper = ensure_ee_joint_shape(left_gripper)
    right_gripper = ensure_ee_joint_shape(right_gripper)

    cam_head_color = _ensure_compressed_colors(cam_head_color)
    cam_left_wrist_color = _ensure_compressed_colors(cam_left_wrist_color)
    cam_right_wrist_color = _ensure_compressed_colors(cam_right_wrist_color)

    os.makedirs(data_dir, exist_ok=True)
    if os.path.exists(hdf5_path):
        os.remove(hdf5_path)
    with h5py.File(hdf5_path, "w") as f:
        vision = f.create_group("vision")
        state = f.create_group("state")
        action = f.create_group("action")
        cam_head = vision.create_group("cam_head")
        cam_head.create_dataset("colors", data=cam_head_color)
        cam_head.create_dataset("shape", data=get_cam_shape(cam_head_color[0]))

        cam_left_wrist = vision.create_group("cam_left_wrist")
        cam_left_wrist.create_dataset("colors", data=cam_left_wrist_color)
        cam_left_wrist.create_dataset("shape", data=get_cam_shape(cam_left_wrist_color[0]))

        cam_right_wrist = vision.create_group("cam_right_wrist")
        cam_right_wrist.create_dataset("colors", data=cam_right_wrist_color)
        cam_right_wrist.create_dataset("shape", data=get_cam_shape(cam_right_wrist_color[0]))

        state_action_pairs = [
            ("left_arm_joint_states", left_joint),
            ("left_ee_joint_states", left_gripper),
            ("right_arm_joint_states", right_joint),
            ("right_ee_joint_states", right_gripper),
        ]
        for key, value in state_action_pairs:
            state.create_dataset(key, data=value)
            action.create_dataset(key, data=_compute_next_state_action(value))

        state.create_dataset("left_ee_poses", data=rpy2quat(left_eef))
        state.create_dataset("right_ee_poses", data=rpy2quat(right_eef))

        instruction = collection.condition.get("xpolicy_instruction")
        frequency = int(collection.condition.get("save_freq", 25))
        write_xpolicy_episode_metadata_to_file(
            f,
            instruction=str(instruction).strip() if instruction else None,
            frequency=frequency,
        )

    if os.environ.get("XSPARK_SKIP_PREVIEW", "").lower() in {"1", "true", "yes"}:
        debug_print("X_spark_format_pipeline", "skip preview video (XSPARK_SKIP_PREVIEW=1)", "INFO")
        debug_print("X_spark_format_pipeline", f"save data success at: {output_path} !", "INFO")
        return

    def decode_cam_frames(color_data):
        frames = []
        for img in color_data:
            if isinstance(img, np.ndarray) and img.ndim == 3:
                frame = np.ascontiguousarray(img)
            else:
                if isinstance(img, np.ndarray):
                    jpeg_bytes = img.tobytes().rstrip(b"\0")
                else:
                    jpeg_bytes = bytes(img).rstrip(b"\0")
                frame = cv2.imdecode(np.frombuffer(jpeg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError("JPEG decode failed while building preview video")
            frames.append(np.ascontiguousarray(frame[:, :, ::-1]))  # RGB -> BGR
        return frames

    def save_preview_video(frames, video_path, out_w=640, out_h=480, fps=25):
        if len(frames) == 0:
            raise ValueError(f"No frames for preview video: {video_path}")
        ffmpeg_cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo", "-vcodec", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{out_w}x{out_h}", "-framerate", str(fps), "-i", "-",
            "-an", "-c:v", "libx264", "-preset", "fast", "-pix_fmt", "yuv420p",
            "-crf", "30", video_path,
        ]
        proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
        for frame in frames:
            resized = cv2.resize(frame, (out_w, out_h), interpolation=cv2.INTER_AREA)
            proc.stdin.write(np.ascontiguousarray(resized).tobytes())
        proc.stdin.close()
        if proc.wait() != 0:
            raise RuntimeError(f"FFmpeg preview video encoding failed: {video_path}")

    os.makedirs(preview_video_dir, exist_ok=True)
    if os.path.exists(merge_preview_path):
        os.remove(merge_preview_path)

    head_frames = decode_cam_frames(cam_head_color)
    left_frames = decode_cam_frames(cam_left_wrist_color)
    right_frames = decode_cam_frames(cam_right_wrist_color)
    num_frames = min(len(head_frames), len(left_frames), len(right_frames))
    if num_frames == 0:
        raise ValueError("No frames for merged preview video")

    head_w, head_h = 320, 240
    wrist_w, wrist_h = 160, 120
    canvas_w, canvas_h = 320, head_h + wrist_h

    ffmpeg_cmd = [
        "ffmpeg", "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{canvas_w}x{canvas_h}", "-framerate", "25", "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "fast", "-pix_fmt", "yuv420p",
        "-crf", "30", merge_preview_path,
    ]
    proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
    for i in range(num_frames):
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        canvas[0:head_h, 0:head_w] = cv2.resize(
            head_frames[i], (head_w, head_h), interpolation=cv2.INTER_AREA
        )
        canvas[head_h:head_h + wrist_h, 0:wrist_w] = cv2.resize(
            left_frames[i], (wrist_w, wrist_h), interpolation=cv2.INTER_AREA
        )
        canvas[head_h:head_h + wrist_h, wrist_w:canvas_w] = cv2.resize(
            right_frames[i], (wrist_w, wrist_h), interpolation=cv2.INTER_AREA
        )
        proc.stdin.write(np.ascontiguousarray(canvas).tobytes())
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"FFmpeg merged preview video encoding failed: {merge_preview_path}")

    for video_path in separate_preview_paths.values():
        if os.path.exists(video_path):
            os.remove(video_path)

    save_preview_video(decode_cam_frames(cam_head_color), separate_preview_paths["cam_head"])
    save_preview_video(decode_cam_frames(cam_left_wrist_color), separate_preview_paths["cam_left_wrist"])
    save_preview_video(decode_cam_frames(cam_right_wrist_color), separate_preview_paths["cam_right_wrist"])

    debug_print("X_spark_format_pipeline", f"save data success at: {output_path} !", "INFO")
