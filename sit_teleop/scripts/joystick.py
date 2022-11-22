#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
本脚本用于机器人的遥控
"""
from typing import Optional, Deque

import rospy
from .core import *
from pyjoystick.sdl2 import Key, Joystick, run_event_loop
from collections import deque
from threading import Thread

Movement = tuple[int, int, int]

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


buttonMappings = {
    # Y
    3: Move.front,
    # X
    2: Move.turnLeft,
    # B
    1: Move.turnRight,
    # A
    0: Move.back,
}


# 速度增量
speed_delta = {
    'w': (1.1, 1),  # 增加最大线速度
    'x': (0.9, 1),  # 减小最大线速度
    'e': (1, 1.1),  # 增大最大角速度
    'c': (1, 0.9),  # 减小最大角速度
}

rospy.init_node('robot_teleop')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
moveQueue: Deque[Movement] = deque()


def matchButton(key: Key) -> Optional[Movement]:
    if key.keytype == "Button":
        if key.number in buttonMappings:
            return buttonMappings[key.number]
        return None
    else:
        return None


def onKeyPressed(key: Key):
    op = matchButton(key)
    if op is not None:
        moveQueue.append(op)


def onJoyAdded(joy: Joystick): pass


def onJoyRemoved(joy: Joystick): pass


def runJoyStickListener():
    run_event_loop(onJoyAdded, onJoyRemoved, onKeyPressed)


def rosLoopCallback(info: ControlInfo, e):
    """
    速度处理函数
    """
    if len(moveQueue) <= 0:
        return
    move = moveQueue.pop()
    info.x, info.y, info.th = move

    # 目标速度=速度值*方向值
    target_speed_x = info.linear_speed * info.x
    target_speed_y = info.linear_speed * info.y
    target_turn = info.yaw_speed * info.th

    for i, (control, target, delta) in enumerate(((info.xSpeed, target_speed_x, 0.006),
                                                  (info.ySpeed, target_speed_y, 0.006),
                                                  (info.turnSpeed, target_turn, 0.1))):
        if target > control:
            control = min(target, control + delta)
        elif target < control:
            control = max(target, control - delta)

        if i == 0:
            info.xSpeed = control
        elif i == 1:
            info.ySpeed = control
        else:
            info.turnSpeed = control
    publish_speed(pub, info.xSpeed, info.ySpeed, info.turnSpeed)


def on_shutdown():
    """
    当节点关闭时的清理工作
    """
    publish_speed(pub, 0, 0, 0)


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
