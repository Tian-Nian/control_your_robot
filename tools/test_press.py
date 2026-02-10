import termios
import tty
import select
import time

def is_enter_pressed():
    return select.select([sys.stdin], [], [], 0)[0] and sys.stdin.read(1) == '\n'

def is_space_pressed():
    return select.select([sys.stdin], [], [], 0)[0] and sys.stdin.read(1) == ' '

# 示例主循环
while True:
    if is_enter_pressed():
        print("Enter pressed")

    elif is_space_pressed():
        print("Space pressed")
        break
    else:
        time.sleep(1 / 10)  
