import argparse
import glob
import cv2
import os
import h5py
import numpy as np

def hdf5_groups_to_dict(hdf5_path):
    result = {}
    
    with h5py.File(hdf5_path, 'r') as f:
        def visit_handler(name, obj):
            if isinstance(obj, h5py.Group):
                group_dict = {}
                for key in obj.keys():
                    if isinstance(obj[key], h5py.Dataset):
                        group_dict[key] = obj[key][()]
                result[name] = group_dict
                
        f.visititems(visit_handler)
    
    return result

def images_decoding(encoded_data, valid_len=None):
    imgs = []
    for data in encoded_data:
        if valid_len is not None:
            data = data[:valid_len]
        nparr = np.frombuffer(data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        imgs.append(img)
    return imgs


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

def main(args):
    hdf5_paths = args.input
    # 自动决定输出路径
    if args.encode:
        output_path = hdf5_paths + "_zip"
    else:
        output_path = hdf5_paths.replace("_zip", "")
    os.makedirs(output_path, exist_ok=True)

    hdf5_files = glob.glob(f"{hdf5_paths}/*.hdf5")

    if args.tarj_only:
        from y1_sdk import Y1SDKInterface, ControlMode
        master_arm = Y1SDKInterface(
            can_id="can0",
            urdf_path="/home/xspark-ai/project/y1_sdk_python/y1_ros/src/y1_controller/urdf/y1_with_gripper.urdf",
            arm_end_type=3,
            enable_arm=False,
        )        
        master_arm.Init()

    for hdf5_file in hdf5_files:
        dataset_path = os.path.join(output_path, os.path.basename(hdf5_file))
        print(f"processing {dataset_path}")

        ep = hdf5_groups_to_dict(hdf5_file)

        if args.encode:
            cam_head_images = ep["cam_head"]["color"]
            cam_left_wrist_images = ep["cam_left_wrist"]["color"]
            cam_right_wrist_images = ep["cam_right_wrist"]["color"]

            head_enc, head_len = images_encoding(cam_head_images)
            left_enc, left_len = images_encoding(cam_left_wrist_images)
            right_enc, right_len = images_encoding(cam_right_wrist_images)
        else:
            cameras = ["cam_high", "cam_left_wrist", "cam_right_wrist"]
            imgs = {}
            for camera in cameras:
                imgs_array = []
                for data in ep["observations"][f"{camera}"]:
                    jpeg_bytes = data.tobytes().rstrip(b"\0")
                    nparr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                    imgs_array.append(cv2.imdecode(nparr, 1))
                imgs[camera] = np.array(imgs_array)

        with h5py.File(dataset_path, "w") as root:
            if args.encode:
                if not args.tarj_only:
                    obs = root.create_group("observations")
                else:
                    obs = root
                if not args.tarj_only:
                    obs.create_dataset("cam_high", data=head_enc, dtype=f"S{head_len}")
                    obs.create_dataset("cam_left_wrist", data=left_enc, dtype=f"S{left_len}")
                    obs.create_dataset("cam_right_wrist", data=right_enc, dtype=f"S{right_len}")

                left_arm = obs.create_group("left_arm")
                right_arm = obs.create_group("right_arm")

                def joint2qpos(joints):
                    qpos = []
                    for j in joints:
                        end_pose = master_arm.EndPose(j.tolist())
                        qpos.append(end_pose)
                    # print(qpos)
                    return np.array(qpos)
                
                left_arm.create_dataset("joint", data=ep["left_arm"]["joint"][:])
                left_arm.create_dataset("gripper", data=ep["left_arm"]["gripper"][:])
                if args.tarj_only:
                    qpos = joint2qpos(ep["left_arm"]["joint"][:])
                    left_arm.create_dataset("qpos", data=qpos)
                else:
                    left_arm.create_dataset("qpos", data=ep["left_arm"]["qpos"][:])
                
                right_arm.create_dataset("joint", data=ep["right_arm"]["joint"][:])
                right_arm.create_dataset("gripper", data=ep["right_arm"]["gripper"][:])
                if args.tarj_only:
                    qpos = joint2qpos(ep["right_arm"]["joint"][:])
                    right_arm.create_dataset("qpos", data=qpos)
                else:
                    right_arm.create_dataset("qpos", data=ep["right_arm"]["qpos"][:])
            else:
                cam_head = root.create_group("cam_head")
                cam_head.create_dataset("color", data=imgs["cam_high"])

                cam_right_wrist = root.create_group("cam_right_wrist")
                cam_right_wrist.create_dataset("color", data=imgs["cam_right_wrist"])

                cam_left_wrist = root.create_group("cam_left_wrist")
                cam_left_wrist.create_dataset("color", data=imgs["cam_left_wrist"])

                left_arm = root.create_group("left_arm")
                right_arm = root.create_group("right_arm")

                left_arm.create_dataset("joint", data=ep["observations/left_arm"]["joint"][:])
                left_arm.create_dataset("gripper", data=ep["observations/left_arm"]["gripper"][:])
                left_arm.create_dataset("qpos", data=ep["observations/left_arm"]["qpos"][:])

                right_arm.create_dataset("joint", data=ep["observations/right_arm"]["joint"][:])
                right_arm.create_dataset("gripper", data=ep["observations/right_arm"]["gripper"][:])
                right_arm.create_dataset("qpos", data=ep["observations/right_arm"]["qpos"][:])

        print(f"saved {dataset_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=str, help="输入的 hdf5 文件夹路径")
    parser.add_argument("--encode", action="store_true", help="是否进行图像压缩编码")
    parser.add_argument("--tarj_only", action="store_true", help="是否只保留图像")
    args = parser.parse_args()
    main(args)
