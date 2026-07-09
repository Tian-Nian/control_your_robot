当前项目的简介


# example
`collect`:数据采集的范式  
`deploy`:模型部署的范式

# my_robot
`agilex_piper_single`: 松灵单臂piper示例, 包含一个head camera和一个wrist camera  
`agilex_piper_dual`: 松灵双臂piper示例, 包含一个head camera和左右各一个wrist camera  
`realman_dual_3_camera`: realman65双臂diy的机器人示例, 包含一个head camera和左右各一个wrist camera  
`mobile robot`: 暂时还没实现

# 主要实现类结构 
## controller
用于定义所有的控制器,目前只有机械臂控制器, 待添加灵巧手控制器与底盘控制器,后面可以根据不同的需求增加新的控制器基类.
### class controller:
定义整体的数据交互结构  
`get()`:获取当前时刻controller的相关信息, 依照set_collect_info中的设置, 返回一个对应的字典  
`set_collect_info(List[str])`:用于设置需要被获取的信息
`__repr__()`:输出当前controller的相关配置信息

#### class arm_controller(controller):
`get_arm_information()`:根据collet_info获取信息
`move()`: 输入为字典, 对应不同操作(如joint, eef, gripper)操作机械臂
##### 继承该类需要实现的函数
根据collect_info来决定, 不需要collect的函数是不会被调用  
一定要实现:  
`get_state()`  
选择性实现:  
"eef": `set_position() `  
"joint": `set_joint()`  
如果机械臂绑定了gripper就基于本类来获取信息, 也可以直接在继承类中绑定gripper  
"gripper":`set_gripper()`,`get_gripper()` 

#### class hand_controller(controller):
预留给灵巧手的空间, 目前支持:  
`joint`  
`action`  
`velocity`  
`force`  

#### class mobile_controller(controller):
预留给底盘的空间, 目前支持:  
`rotate`  
`move_velocity`  
`move_to`  

## sensor
用于定义所有的传感器, 如视觉传感器, 触觉传感器, 可以根据需求添加基类.  
### class sensor
`set_collect_info()`:设置获取信息类型  
`get()`: 获取对应信息  
`__repr__()`: 输出调试信息  

#### class vision_sensor(sensor):
` get_information()`:获取对应信息  
##### 继承该类需要实现的函数  
`get_image()`  

#### class touch_sensor(sensor):
` get_information()`:获取对应信息  
##### 继承该类需要实现的函数  
`get_touch()`  

## data
所有基础数据的存储都在该类下
### class CollectAny():
#### 初始化参数
##### condition
`condition`: 字典,配置数据存储相关信息
``` python
condition = {
    "save_path": "datasets/{机械臂名称(可选)}",
    "task_name": "task_example",
    "save_format": "hdf5",
    "save_interval": 10,
}
```
##### image_map
`image_map`: 字典,映射当前图像数据名称数据到对应存储数据名称
``` python
设置map, 用于将默认数据格式映射到lerobot的对应features中
多层索引使用`.`分割
多数据结合用List[str]表示
例如:
map = {
    "observation.images.cam_high": "observation.cam_head.color",
    "observation.images.cam_left_wrist": "observation.cam_left_wrist.color",
    "observation.images.cam_right_wrist": "observation.cam_right_wrist.color",
    "observation.state": ["observation.left_arm.joint","observation.left_arm.gripper","observation.right_arm.joint","observation.right_arm.gripper"],
}
```

##### start_episode
`start_episode`: 开始的episode, 默认为0

#### 函数
`collect(controller_data, sensor_data)`:  
输入的二者都是List[Dict], 会存储对应controller、sensor里面所有的获取信息

`write()`:
按照hdf5格式存储完整的episode, 然后重置episode, 准备下一条轨迹的采集存储, 并且会在整个存储的主目录下存储一次当前condition配置.

### generate_lerobot.py
里面封装了`class MyLerobotDataset`, 便于简单的转化数据格式, 支持最新版lerobot(25.04.09)需要注意的是, openpi的lerobot版本比较低, 所以需要修改部分代码, 在代码中已经注明.

## utils
### class time_scheduler
时间控制器, 用来同步指定的controller_worker, 使其操作同步(主要用于collect环节)
#### 初始化参数
##### time_locks: List[multiprocessing.Lock]
对应需要同步的进程的时间锁
##### time_freq: int
对应进程执行操作的频率

### def robot_worker
用于在multiprocessing库启动的进程函数, 使用了Lock来同步当前的进程相关信息.
需要给出对应函数:  
`robot_get_func`: 获取信息  
`robot_save_func`: 直接输入获取的信息, 进行存储  
`robot_write_func`: 根据判断, 写入当前存储的信息

### data_handler.py
提供一些处理数据的函数


