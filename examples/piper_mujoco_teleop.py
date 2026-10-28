"""Manual wrist/tool-frame baseline using the SAME visual-agent interpreter and executor.

No model calls. Default creates an isolated scene; --attach uses the running scene.
Type one token and Enter for each bounded action; an empty line stops the loop.
The saved RGB requests and measured feedback can diagnose a camera/tool/executor
problem separately from a model-decision problem. They are not grasp labels.
"""
import argparse
from pathlib import Path
import threading
import uuid

from piper_mujoco_agent import ROOT, run_loop
from piper_mujoco_harness import scene_session
from robot.harness.kinematics import PiperKinematics


def command(token):
    words = token.split()
    if len(words) == 2:
        action = command(words[0])
        if action["action"] not in ("move", "tool_move"):
            raise ValueError("只有平移命令可附加厘米步长，例如 z+ 3 或 up 2.5")
        try:
            step = float(words[1])
        except ValueError:
            raise ValueError("平移步长必须是1到5厘米") from None
        if not 1 <= step <= 5:
            raise ValueError("平移步长必须是1到5厘米")
        return {**action, "step_cm": step}
    if token in ("x+", "x-", "y+", "y-", "z+", "z-"):
        return {"action": "tool_move", "direction": token, "reason": "人工腕部视角调试"}
    if len(token) == 3 and token[0] == "r" and token[1] in "xyz" and token[2] in "+-":
        return {"action": "tool_rotate", "axis": token[1], "sign": 1 if token[2] == "+" else -1,
                "reason": "人工工具系旋转调试"}
    if len(token) == 3 and token[0] == "j" and token[1] in "123456" and token[2] in "+-":
        return {"action": "joint_step", "joint_index": int(token[1]), "sign": 1 if token[2] == "+" else -1,
                "reason": "人工小幅关节调姿，重新检查工具位姿"}
    if token in ("forward", "back", "left", "right", "up", "down"):
        return {"action": "move", "direction": token, "reason": "人工基座系调试"}
    if token in ("open", "close"):
        return {"action": "set_gripper", "gripper": int(token == "open"), "reason": "人工夹爪尝试，不代表抓取成功"}
    if token == "vertical":
        return {"action": "rotate", "sense": "down", "reason": "人工调整到朝下姿态"}
    raise ValueError("未知命令，请使用上方列出的命令")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attach", action="store_true")
    parser.add_argument("--out", type=Path, default=ROOT / "save" / f"teleop_{uuid.uuid4().hex[:10]}")
    args = parser.parse_args()
    args.viewer, args.no_cameras = False, False
    stop = threading.Event()
    print("不调用模型。工具系平移1cm：x+ x- y+ y- z+ z-；工具系旋转0.04rad：rx+ rx- ry+ ry- rz+ rz-。")
    print("关节调姿0.04rad：j1+ j1- … j6+ j6-；基座系1cm：forward back left right up down；夹爪：open close；朝下：vertical。")
    print("平移可附加1到5厘米步长，例如 x+ 3、up 2.5；省略时1厘米。")
    print("另开只读 viewer 查看双视角。输入命令后按 Enter 执行；空行或 EOF 停止。")

    def policy(_):
        while not stop.is_set():
            try:
                token = input("动作> ").strip()
            except EOFError:
                token = ""
            if not token:
                stop.set()
                return {"action": "wait", "reason": "人工停止"}
            try:
                return command(token)
            except ValueError as exc:
                print(exc, flush=True)
        return {"action": "wait", "reason": "人工停止"}

    stage = {"id": "manual", "target": "人工选定的碗", "affordance": "人工选定的碗沿",
             "motion": "GRASP", "description": "人工使用腕部图像控制", "completion": "人工检查，不自动验收"}
    with scene_session(args) as session:
        run_loop(session, policy, stop, args.out, task="人工验证同一组抓取控制工具",
                 planner=lambda _: {"stages": [stage]}, legacy_actions=False,
                 kinematics=PiperKinematics(), stage_rounds=1000000, max_replans=0, interval=.2, max_rounds=0)


if __name__ == "__main__":
    main()
