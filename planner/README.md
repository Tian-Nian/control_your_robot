### 通用的机械臂planner
由于好多厂家自带的planner很垃圾,所以我结合了RoboTwin2.0最新的planner,实现了一个通用接口.  
考虑到有些朋友还没有自己的机器人,所以顺便给出了RoboTwin中我们最喜欢的cobomagic双臂平台的操控示例啦.  
这个会慢慢做成一个仿真教程~
这边还在慢慢开发, 代码变动会很大, 如果希望提供支持, 或者有什么建议, 可以提issue或者私信我.
### 下载URDF
agliex cobomagic机械臂URDF:
通过网盘分享的文件：仿真机械臂
链接: https://pan.baidu.com/s/1Mfrs3spVTeRWUHf_pyHZjQ?pwd=yq7m  
提取码: yq7m 
更多URDF请访问RoboTwin2.0, 提供了许多精修的URDF:
https://github.com/RoboTwin-Platform/RoboTwin.git
提供的dual_piper_sim_robot.py就是里面的piper URDF.

需要修改`curobo_left.yml`和`curobo_right.yml`中`collision_spheres`和`urdf_path`,要求为绝对路径.

环境配置:
```bash
# 安装sapien基础环境
pip install - r requirements.txt

# 安装curobo
cd ../third_party
git clone https://github.com/NVlabs/curobo.git
cd curobo
pip install -e . --no-build-isolation
cd ../..
```

### 快速上手
需要下载对应的URDF文件, 然后设置代码里面的索引路径.cobomagic示例还要额外修改`.yml`文件的路径索引.
```bash
# cobomagic通用控制
python planner/cobomagic_sim_robot.py
# piper双臂通用控制
python planner/dual_piper_sim_robot.py 
```
### 已经实现
| 日期       | 更新内容                          | 状态     |
|------------|----------------------------------|----------|
| 2025.6.23 | 🤖仿真环境中的双单臂组合双臂示例 | ✅ 已发布 |
| 2025.5.22 | 🤖仿真环境中的通用IK示例(4090 0.004s/step) | ✅ 已发布 |
| 2025.5.22 | 🤖仿真环境中的通用planner示例(4090 0.15s/step)  | ✅ 已发布 |
| 2025.5.22 | 💻通用planner接入                   | ✅ 已发布 |
| 2025.5.22 | 🏙️接入D435仿真摄像头设置              | ✅ 已发布 |

### 正在路上
- [ ] 📷多种camera设置支持
- [ ] 📖URDF, sapien使用简单教学与示例

