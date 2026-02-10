#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 节点：读取键盘输入并发布到 /key_cmd
- ↑ ↓ ← → : 前后左右
- W / S   : 前 / 后
- P       : 退出
- Ctrl+C  : 安全退出
"""

import rospy
from std_msgs.msg import String
import select
import termios
import tty

# 键盘映射
key_mapping = {
    '\x1b[A': 'FORWARD',    # ↑
    '\x1b[B': 'BACKWARD',   # ↓
    '\x1b[C': 'RIGHT',      # →
    '\x1b[D': 'LEFT',       # ←
    'w': 'UP',
    's': 'DOWN',
    'n': "TELEOP",
    'm': "END",
}

EXIT_KEY = 'p'

def get_key(settings):
    """非阻塞读取键盘"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)

    key = ''
    if rlist:
        key = sys.stdin.read(1)
        # 方向键是 ESC + [ + A/B/C/D
        if key == '\x1b':
            key += sys.stdin.read(2)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def keyboard_publisher():
    rospy.init_node('keyboard_publisher', anonymous=True)
    pub = rospy.Publisher('/key_cmd', String, queue_size=10)
    rate = rospy.Rate(20)

    print(
        "\n✅ 键盘控制启动\n"
        "  ↑ / w : 前\n"
        "  ↓ / s : 后\n"
        "  ←     : 左\n"
        "  →     : 右\n"
        "  p     : 退出\n"
        "  Ctrl+C: 安全退出\n"
    )

    settings = termios.tcgetattr(sys.stdin)

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if not key:
                rate.sleep()
                continue

            if key == EXIT_KEY:
                rospy.loginfo("🛑 按下 P，退出节点")
                break

            if key in key_mapping:
                cmd = key_mapping[key]
                rospy.loginfo(f"按键指令: {cmd}")
                pub.publish(cmd)

            rate.sleep()

    except KeyboardInterrupt:
        rospy.loginfo("🛑 Ctrl+C，安全退出")

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.signal_shutdown("Keyboard exit")


if __name__ == '__main__':
    keyboard_publisher()
