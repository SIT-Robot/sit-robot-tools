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

    def __eq__(self, other):
        if isinstance(other, ButtonTurnOp):
            return self.direction == other.direction
        else:
            return False


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

    def __eq__(self, other):
        if isinstance(other, OmniMoveOp):
            return self.x == other.x and self.y == other.y
        else:
            return False


class StopOp(Operation):

    def apply(self, info: ControlInfo):
        info.resetTargetSpd()

    def __eq__(self, other):
        if isinstance(other, StopOp):
            return True
        else:
            return False


class LinearSpdChangeOp(Operation):
    def __init__(self, delta: float):
        self.delta = delta

    def apply(self, info: ControlInfo):
        res = info.linearSpd + info.linearDeltaSpeed * self.delta
        info.linearSpd = clamp(info.minLinearSpd, res, info.maxLinearSpd)


class YawSpdChangeOp(Operation):
    def __init__(self, delta: float):
        self.delta = delta

    def apply(self, info: ControlInfo):
        res = info.yawSpd + info.yawDeltaSpeed * self.delta
        info.yawSpd = clamp(info.minYawSpd, res, info.maxYawSpd)


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
    # Menu Button
    7: StopOp()
}

speedChangeMappings = {
    Key.HAT_UP: LinearSpdChangeOp(1),
    Key.HAT_DOWN: LinearSpdChangeOp(-1),
    Key.HAT_LEFT: YawSpdChangeOp(1),
    Key.HAT_RIGHT: YawSpdChangeOp(-1),
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
lastOp: Optional[Operation] = None


def matchButton(key: Key) -> Optional[Operation]:
    if key.keytype == "Button":
        if key.number in moveButtonMappings:
            return moveButtonMappings[key.number]
    elif key.keytype == "Axis":
        if key.number == 0:
            return OmniMoveOp(y=key.raw_value)
        elif key.number == 1:
            return OmniMoveOp(x=key.raw_value)
    elif key.keytype == "Hat":
        if key.value in speedChangeMappings:
            return speedChangeMappings[key.value]
    return None


def onKeyPressed(key: Key):
    global lastOp
    op = matchButton(key)
    if lastOp != op:
        lastOp = op


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
    if lastOp is None:
        return
    lastOp.apply(info)
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
