#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 节点：读取键盘方向键输入并发布到 /key_cmd 话题
支持 Ctrl+C 安全退出
"""

import rospy
from std_msgs.msg import String
import sys, select, termios, tty

# 键盘映射
key_mapping = {
    '\x1b[A': 'UP',     # ↑
    '\x1b[B': 'DOWN',   # ↓
    '\x1b[C': 'RIGHT',  # →
    '\x1b[D': 'LEFT'    # ←
}

def get_key(settings):
    """非阻塞读取键盘按键"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(3) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def keyboard_publisher():
    rospy.init_node('keyboard_publisher', anonymous=True)
    pub = rospy.Publisher('/key_cmd', String, queue_size=10)
    rate = rospy.Rate(20)

    print("✅ 键盘控制启动（↑ ↓ ← →），按 Ctrl+C 退出。")

    # 记录原始终端设置
    settings = termios.tcgetattr(sys.stdin)

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key in key_mapping:
                cmd = key_mapping[key]
                rospy.loginfo(f"按键：{cmd}")
                pub.publish(cmd)
            rate.sleep()
    except KeyboardInterrupt:
        print("\n🛑 检测到 Ctrl+C，正在安全退出...")
    finally:
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    keyboard_publisher()
