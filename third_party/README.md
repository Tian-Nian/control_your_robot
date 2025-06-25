# 此文件夹用于放置一些机械臂配置的python文件，如无法pip install安装环境，请将配置库放在这里

1. dr:  
是大然aloha机械臂的控制底层代码, 无需编译, 使用提供的默认x64编译后文件, 如果是不同架构系统, 请与厂家联系获得支持.
2. curobo:
提供了IK / planner, 需要编译使用. 
```bash
git clone https://github.com/NVlabs/curobo.git
cd curobo
pip install -e . --no-build-isolation
```
3. oculus_reader
用于控制VR遥操设备QuestVR, 需要编译使用.
