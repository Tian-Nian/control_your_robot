import os
import json
import concurrent.futures
from collections import defaultdict
import time

def process_folder(base_dir, folder_name):
    """处理单个文件夹的数据统计"""
    folder_path = os.path.join(base_dir, folder_name)
    sync_file = os.path.join(folder_path, "sync.txt")
    
    if not os.path.exists(sync_file):
        print(f"⚠️ sync.txt not found in {folder_path}")
        return folder_name, None
    
    # 读取sync.txt获取JSON文件列表
    with open(sync_file, 'r') as f:
        json_files = [line.strip() for line in f if line.strip()]
    
    # 初始化统计数据结构
    stats = {
        "joint1": {"min": float('inf'), "max": float('-inf')},
        "joint2": {"min": float('inf'), "max": float('-inf')},
        "joint3": {"min": float('inf'), "max": float('-inf')},
        "joint4": {"min": float('inf'), "max": float('-inf')},
        "joint5": {"min": float('inf'), "max": float('-inf')},
        "joint6": {"min": float('inf'), "max": float('-inf')},
        "gripper": {"min": float('inf'), "max": float('-inf')}
    }
    
    # 处理每个JSON文件
    for json_file in json_files:
        file_path = os.path.join(folder_path, json_file)
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
                position = data.get("position", [])
                
                # 更新关节统计
                for i in range(6):
                    joint_name = f"joint{i+1}"
                    value = position[i]
                    if value < stats[joint_name]["min"]:
                        stats[joint_name]["min"] = value
                    if value > stats[joint_name]["max"]:
                        stats[joint_name]["max"] = value
                
                # 更新夹爪统计
                gripper_value = position[6] if len(position) > 6 else 0
                if gripper_value < stats["gripper"]["min"]:
                    stats["gripper"]["min"] = gripper_value
                if gripper_value > stats["gripper"]["max"]:
                    stats["gripper"]["max"] = gripper_value
                    
        except Exception as e:
            print(f"❌ Error processing {file_path}: {str(e)}")
    
    return folder_name, stats

def main():
    start_time = time.time()
    base_dir = "/home/usst/kwj/GitCode/control_your_robot/data/episode1/arm/jointState"
    folders = ["masterLeft", "masterRight", "puppetLeft", "puppetRight"]
    results = {}
    
    # 使用线程池并行处理文件夹
    with concurrent.futures.ThreadPoolExecutor() as executor:
        futures = [executor.submit(process_folder, base_dir, folder) for folder in folders]
        
        for future in concurrent.futures.as_completed(futures):
            folder_name, stats = future.result()
            if stats:
                results[folder_name] = stats
                print(f"✅ {folder_name} processed successfully")
    
    # 保存结果到JSON文件
    with open("check_data.json", "w") as f:
        json.dump(results, f, indent=4)
    
    print(f"⏱️ Total processing time: {time.time()-start_time:.2f} seconds")
    print(f"📊 Results saved to check_data.json")

if __name__ == "__main__":
    main()