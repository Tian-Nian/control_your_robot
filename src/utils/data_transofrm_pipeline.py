from utils.data_handler import debug_print

import h5py
import numpy as np
import cv2
import os


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


