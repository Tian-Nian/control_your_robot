import sys
sys.path.append("./")
from utils.data_handler import debug_print

import h5py
import numpy as np
import cv2

def print_hdf5_group_info(group, indent=0):
    """
    Recursively print detailed information of an HDF5 Group.
    Skip the dataset named 'instruction'.
    """
    for key in group.keys():
        if key == 'instruction':  # 跳过 'instruction'
            continue

        item = group[key]
        indent_str = ' ' * indent
        if isinstance(item, h5py.Group):
            print(f"{indent_str}Group: {key}")
            print_hdf5_group_info(item, indent + 2) 
        elif isinstance(item, h5py.Dataset):
            print(f"{indent_str}Dataset: {key}")
            print(f"{indent_str}  Shape: {item.shape}, dtype: {item.dtype}")

def save_video_from_bytes(img_bytes, video_path, fps=30, frame_size=(640, 480)):
    """
     Decode binary compressed image data and save it as a local video.

    Args:
        img_bytes (bytes): Binary compressed image data.
        video_path (str): Path to the output video file.
        fps (int): Frame rate of the video.
        frame_size (tuple): Resolution of the video.
    """
    try:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
        video_writer = cv2.VideoWriter(video_path, fourcc, fps, frame_size)

        for i,img_byte in enumerate(img_bytes):
            nparr = np.frombuffer(img_byte, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR) 
            if img is not None:
                img_resized = cv2.resize(img, frame_size)
                video_writer.write(img_resized)
            else:
                debug_print("read_rdt_data","Image decoding failed.", "ERROR")
    except Exception as e:
        debug_print("read_rdt_data", f"Error decoding image: {e}", "ERROR")
    finally:
        # release VideoWriter
        if 'video_writer' in locals():
            video_writer.release()

def display_image_from_bytes(img_bytes):
    try:
        nparr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if img is not None:
            cv2.imshow("Decoded Image", img)
            cv2.waitKey(10)
        else:
            debug_print("read_rdt_data","Image decoding failed.", "ERROR")
    except Exception as e:
        debug_print("read_rdt_data", f"Error decoding image: {e}", "ERROR")


def print_hdf5_file_info_and_display_images(file_path):
    """
    Read all data from an HDF5 file, including images, states, actions, etc., and display the images.  
    Skip the 'instruction' dataset and correctly parse data nested under 'observations'.

    Args:  
        file_path (str): Path to the HDF5 file.
    """
    with h5py.File(file_path, 'r') as f:
        print("HDF5 File Structure:")
        print_hdf5_group_info(f) 
        print("-" * 40)

        print(f.keys())
        for key in f.keys():
            if key == 'instruction':
                continue

            dataset = f[key]
            if key == 'observations':
                print("Parsing nested observations data:")
                if 'qpos' in dataset:
                    print(f"qpos shape: {dataset['qpos'].shape}")
                    print(f"Sample qpos data (first 5 elements): {dataset['qpos'][:5]}")
                if 'effort' in dataset:
                    print(f"effort shape: {dataset['effort'].shape}")
                    print(f"Sample effort data (first 5 elements): {dataset['effort'][:5]}")
                if 'qvel' in dataset:
                    print(f"qvel shape: {dataset['qvel'].shape}")
                    print(f"Sample qvel data (first 5 elements): {dataset['qvel'][:5]}")

                if 'images' in dataset:
                    print("Parsing images in observations:")
                    for image_key in dataset['images']:
                        img_data = dataset['images'][image_key][:]
                        print(f"Displaying images from {image_key}:")
                        # save video
                        # save_video_from_bytes(img_data, f"datasets/{image_key}.mp4", 10)
                        for i, img_bytes in enumerate(img_data):
                            # imshow
                            # display_image_from_bytes(img_bytes)
                            continue
                            
            else:
                print(f"Sample data (first 5 elements): {dataset[:5]}")

        cv2.destroyAllWindows()

# sample
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Show your rdt data.')
    parser.add_argument('episode_id', type=int,
                    help='your episode id, like episode_0.hdf5 id is 0')
    parser.add_argument('data_path', type=str, nargs='?',default="datasets/RDT/",
                    help="your data dir like: datasets/RDT/")
    args = parser.parse_args()
    data_path = args.data_path
    i = args.episode_id
    file_path = f"datasets/RDT/episode_{i}.hdf5"
    print_hdf5_file_info_and_display_images(file_path)
