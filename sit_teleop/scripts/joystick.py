#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
本脚本用于机器人的遥控
"""
import os
from typing import Optional, Deque, Callable

import rospy
from core import *
from geometry_msgs.msg import Twist
from pyjoystick.sdl2 import Key, Joystick, run_event_loop
from collections import deque, namedtuple
from threading import Thread
import json

from dashboard import Dashboard

helpInfo = """
Control robot with Joystick!
---------------------------
For Xbox controller:
        Y
    X       B
        A
"""

Movement = namedtuple("Movement", ["x", "y", "turn"])
localSavePath = "joystick_conf.json"


def _doNothing(): pass


class Kernal:
    def __init__(self):
        self.refreshMove = _doNothing


class Move:
    front = Movement(1, 0, 0)
    turnLeft = Movement(0, 0, 1)
    turnRight = Movement(0, 0, -1)
    back = Movement(-1, 0, 0)
    zero = Movement(0, 0, 0)


class Operation:
    def apply(self, kernal: Kernal, info: ControlInfo):
        pass


class MetaOperation(Operation):
    pass


class ButtonTurnOp(Operation):
    def __init__(self, direction):
        self.direction = direction

    @staticmethod
    def by(move: Movement) -> "ButtonTurnOp":
        return ButtonTurnOp(move.turn)

    def apply(self, kernal: Kernal, info: ControlInfo):
        info.targetX = 0
        info.targetY = 0
        info.targetYawSpeed = info.yawSpd * self.direction

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

    def apply(self, kernal: Kernal, info: ControlInfo):
        info.targetYawSpeed = 0
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

    def apply(self, kernal: Kernal, info: ControlInfo):
        info.resetTargetSpd()

    def __eq__(self, other):
        if isinstance(other, StopOp):
            return True
        else:
            return False


class LinearSpdChangeOp(MetaOperation):
    def __init__(self, delta: float):
        self.delta = delta

    def apply(self, kernal: Kernal, info: ControlInfo):
        res = info.linearSpd + info.linearDeltaSpeed * self.delta
        info.linearSpd = clamp(info.minLinearSpd, res, info.maxLinearSpd)
        kernal.refreshMove()


class YawSpdChangeOp(MetaOperation):
    def __init__(self, delta: float):
        self.delta = delta

    def apply(self, kernal: Kernal, info: ControlInfo):
        res = info.yawSpd + info.yawDeltaSpeed * self.delta
        info.yawSpd = clamp(info.minYawSpd, res, info.maxYawSpd)
        kernal.refreshMove()


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
    Key.HAT_LEFT: YawSpdChangeOp(-1),
    Key.HAT_RIGHT: YawSpdChangeOp(1),
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
# Only allow one movement operation at the same time
lastMoveOp: Optional[Operation] = None
metaOpQueue: Deque[MetaOperation] = deque()
joystick: Optional[Joystick] = None


def onKeyPressed(key: Key):
    global lastMoveOp
    op = None
    if key.keytype == "Button":
        if key.number in moveButtonMappings:
            op = moveButtonMappings[key.number]
    elif key.keytype == "Axis":
        if key.number == 0:
            op = OmniMoveOp(y=key.raw_value)
        elif key.number == 1:
            op = OmniMoveOp(x=key.raw_value)
    elif key.keytype == "Hat":
        if key.value in speedChangeMappings:
            op = speedChangeMappings[key.value]
            metaOpQueue.append(op)
    if lastMoveOp != op:
        lastMoveOp = op


def onJoyAdded(joy: Joystick):
    global joystick
    joystick = joy
    print(f"{joy.name} Connected.")


def onJoyRemoved(joy: Joystick):
    global joystick
    joystick = joy
    print(f"{joy.name} Disconnected.")


def runJoyStickListener():
    run_event_loop(onJoyAdded, onJoyRemoved, onKeyPressed)


def createRosLoopTask(kernal: Kernal, info: ControlInfo) -> Callable[[object], None]:
    def refreshMoveFunc():
        if lastMoveOp is not None:
            lastMoveOp.apply(kernal, info)

    kernal.refreshMove = refreshMoveFunc
    return lambda event: rosLoopCallback(kernal, info, event)


def rosLoopCallback(kernal: Kernal, info: ControlInfo, e):
    if lastMoveOp is not None:
        lastMoveOp.apply(kernal, info)
    if len(metaOpQueue) > 0:
        op = metaOpQueue.popleft()
        op.apply(kernal, info)
    info.sendVia(pub)


def on_shutdown():
    """
    当节点关闭时的清理工作
    """
    resetSpeed(pub)


# noinspection PyBroadException
def main():
    # 打印首页
    print(helpInfo)
    info = ControlInfo()
    try:
        if os.path.exists(localSavePath):
            with open(localSavePath, mode="r") as f:
                obj = json.loads(f.read())
                info.readFrom(obj)
    except:
        pass
    joyListener = Thread(target=runJoyStickListener)
    joyListener.daemon = True
    joyListener.start()

    def dashboardHeader():
        print(helpInfo)
        if joystick is None:
            print("[Controller Disconnected]")
        else:
            print(f'[Controller "{joystick.name}" Connected]')

    dashboard = Dashboard()
    dashboardDrawer = Thread(target=lambda: dashboard.startRender(info, dashboardHeader))
    dashboardDrawer.daemon = True
    dashboardDrawer.start()

    kernal: Kernal = Kernal()
    rospy.Timer(rospy.Duration(0.05), createRosLoopTask(kernal, info))
    rospy.on_shutdown(on_shutdown)

    try:
        rospy.spin()
    finally:
        rospy.loginfo('Cleaning up...')
        rospy.signal_shutdown("Normal exit")
        rospy.loginfo('Joysticks control disconnected.')
        try:
            content = json.dumps(info.saveTo())
            with open(localSavePath, mode="w") as f:
                f.write(content)
        except:
            pass


if __name__ == '__main__':
    main()
