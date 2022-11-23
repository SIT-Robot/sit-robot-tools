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
from collections import deque, namedtuple
from threading import Thread

helpInfo = """
Control robot with Joystick!
---------------------------
For Xbox controller:
        Y
    X       B
        A
"""

Movement = namedtuple("Movement", ["x", "y", "turn"])


class Move:
    front = Movement(1, 0, 0)
    turnLeft = Movement(0, 0, 1)
    turnRight = Movement(0, 0, -1)
    back = Movement(-1, 0, 0)
    zero = Movement(0, 0, 0)


class Operation:
    def apply(self, info: ControlInfo):
        pass


class ButtonTurnOp(Operation):
    def __init__(self, direction):
        self.direction = direction

    @staticmethod
    def by(move: Movement) -> "ButtonTurnOp":
        return ButtonTurnOp(move.turn)

    def apply(self, info: ControlInfo):
        info.targetTurnSpeed = info.yawSpd * self.direction


class OmniMoveOp(Operation):
    def __init__(self, x: Optional[float] = None, y: Optional[float] = None):
        self.x = x
        self.y = y

    @staticmethod
    def by(move: Movement) -> "OmniMoveOp":
        return OmniMoveOp(move.x, move.y)

    def apply(self, info: ControlInfo):
        if self.x is not None:
            info.targetX = info.linearSpd * self.x
        if self.y is not None:
            info.targetY = info.linearSpd * self.y


class StopOp(Operation):

    def apply(self, info: ControlInfo):
        info.resetTargetSpd()


# NOTE: The chassis is reversed.
moveButtonMappings = {
    # A
    0: OmniMoveOp.by(Move.front),
    # B
    2: ButtonTurnOp.by(Move.turnRight),
    # X
    1: ButtonTurnOp.by(Move.turnLeft),
    # Y
    3: OmniMoveOp.by(Move.back),
    5: StopOp()
}
# Right Button (RB)
stopButton = 5
# Menu Button
# quitButton = 7

moveAxisMapping = {
    # Left
    0: (1, 0),
    # Right
    1: (0, 1)
}

rospy.init_node('robot_teleop')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
opQueue: Deque[Operation] = deque()


def matchButton(key: Key) -> Optional[Operation]:
    if key.keytype == "Button":
        if key.number in moveButtonMappings:
            return moveButtonMappings[key.number]
        else:
            return None
    elif key.keyname == "Axis":
        if key.number == 0:
            return OmniMoveOp(x=key.raw_value)
        elif key.number == 1:
            return OmniMoveOp(y=key.raw_value)
    else:
        return None


def onKeyPressed(key: Key):
    op = matchButton(key)
    if op is not None:
        opQueue.append(op)


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
    if len(opQueue) <= 0:
        return
    op = opQueue.popleft()
    op.apply(info)
    info.sendVia(pub)


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
