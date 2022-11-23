#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
本脚本用于机器人的遥控
"""
import time
from typing import Optional, Deque, Tuple

import rospy
from core import *
from pyjoystick.sdl2 import Key, Joystick, run_event_loop
from collections import deque
from threading import Thread

Movement = Tuple[int, int, int]

helpInfo = """
Control robot with Joystick!
---------------------------
For Xbox controller:
        Y
    X       B
        A
"""


class Move:
    front = (1, 0, 0)
    turnLeft = (0, 0, 1)
    turnRight = (0, 0, -1)
    back = (-1, 0, 0)
    zero = (0, 0, 0)


# NOTE: The chassis is reversed.
moveButtonMappings = {
    # Y
    3: Move.back,
    # X
    2: Move.turnRight,
    # B
    1: Move.turnLeft,
    # A
    0: Move.front,
}
# Right Button (RB)
stopButton = 5

rospy.init_node('robot_teleop')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
moveQueue: Deque[Movement] = deque()
_defaultLastButton = ()
lastButton = _defaultLastButton


def matchButton(key: Key) -> Optional[Movement]:
    if key.keytype == "Button":
        if key.number in moveButtonMappings:
            if lastButton is not _defaultLastButton and lastButton != (key.keytype, key.number):
                moveQueue.clear()
                resetSpeed(pub)
                time.sleep(5)
            return moveButtonMappings[key.number]
        elif key.number == stopButton:
            resetSpeed(pub)
            return Move.zero
        return None
    else:
        return None


def onKeyPressed(key: Key):
    op = matchButton(key)
    if op is not None:
        moveQueue.append(op)


def onJoyAdded(joy: Joystick):
    print(f"{joy.name} Connected.")


def onJoyRemoved(joy: Joystick):
    print(f"{joy.name} Disconnected.")


def runJoyStickListener():
    run_event_loop(onJoyAdded, onJoyRemoved, onKeyPressed)


def rosLoopCallback(info: ControlInfo, e):
    """
    速度处理函数
    """
    if len(moveQueue) <= 0:
        return
    move = moveQueue.popleft()
    info.x, info.y, info.th = move

    # 目标速度=速度值*方向值
    target_speed_x = info.linear_speed * info.x
    target_speed_y = info.linear_speed * info.y
    target_turn = info.yaw_speed * info.th

    info.xSpeed = target_speed_x
    info.ySpeed = target_speed_y
    info.turnSpeed = target_turn
    publishSpeed(pub, info.xSpeed, info.ySpeed, info.turnSpeed)


def on_shutdown():
    """
    当节点关闭时的清理工作
    """
    resetSpeed(pub)


def main():
    # 打印首页
    print(helpInfo)
    info = ControlInfo()
    info.display()
    joyListener = Thread(target=runJoyStickListener)
    joyListener.daemon = True
    joyListener.start()
    rospy.Timer(rospy.Duration(0.1), lambda e: rosLoopCallback(info, e))
    rospy.on_shutdown(on_shutdown)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Cleaning up...')
        rospy.signal_shutdown("Normal exit")
        rospy.loginfo('Joysticks control disconnected.')


if __name__ == '__main__':
    main()
