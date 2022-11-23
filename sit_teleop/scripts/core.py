import os

from geometry_msgs.msg import Twist
from io import StringIO

speedBlockNumber = 10


class ControlInfo:
    def __init__(self):
        # 最终要控制的速度
        self.targetX = 0
        self.targetY = 0
        self.targetTurnSpeed = 0
        self.linearSpd = 0.2  # 最大线速度
        self.yawSpd = 1  # 最大角速度
        self.maxLinearSpd = 0.3
        self.maxYawSpd = 0.5
        self.minLinearSpd = 0
        self.minYawSpd = 0
        self.linearDeltaSpeed = 0.03
        self.yawDeltaSpeed = 0.03

    def resetTargetSpd(self):
        self.targetX = 0
        self.targetY = 0
        self.targetTurnSpeed = 0

    def sendVia(self, publisher):
        publishSpeed(publisher, self.targetX, self.targetY, self.targetTurnSpeed)


def publishSpeed(publisher, vx: float, vy: float, vth: float):
    """
    发布速度信息
    """
    # 创建并发布twist消息
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = vth
    publisher.publish(twist)


def resetSpeed(publisher):
    publishSpeed(publisher, 0, 0, 0)


def clamp(minvalue, value, maxvalue):
    return max(minvalue, min(value, maxvalue))


_upArrow = "↑"
_downArrow = "↓"
_rightArrow = "->"
_leftArrow = "<-"
_centerCross = "+"
"""
   ↑     |  
<- + ->  |  <- + -> 
   ↓     |
"""


def composeDashboard(info: ControlInfo) -> str:
    with StringIO() as s:
        # Line 1
        s.write("   ")
        if info.targetY > 0:
            s.write(_upArrow)
        else:
            s.write(" ")
        s.write("     |")
        s.write("\n")
        # Line 2
        # Left
        if info.targetX < 0:
            s.write(_leftArrow)
        else:
            s.write("  ")
        s.write(" ")
        s.write(_centerCross)
        s.write(" ")
        # Right
        if info.targetX > 0:
            s.write(_rightArrow)
        s.write("  |  ")
        if info.yawSpd < 0:
            s.write(_leftArrow)
        else:
            s.write("  ")
        s.write(" ")
        s.write(_centerCross)
        s.write(" ")
        if info.yawSpd > 0:
            s.write(_rightArrow)
        s.write("\n")
        # Line 3
        s.write("   ")
        if info.targetY < 0:
            s.write(_downArrow)
        s.write("     |")
        # Line 4
        return s.getvalue()


def composeSpeedBlock(percent: float) -> str:
    solid = int(percent * speedBlockNumber)
    hollow = speedBlockNumber - solid
    return "⬛" * solid + "⬜" * hollow


def displaySpeedBlock(minSpd, cur, maxSpd) -> str:
    percent = (cur - minSpd) / (maxSpd - minSpd)
    return composeSpeedBlock(percent)


def displaySpeedIndicator(info: ControlInfo):
    print(
        "Linear Speed:",
        displaySpeedBlock(info.minLinearSpd, info.linearSpd, info.maxLinearSpd),
        f"+{info.linearDeltaSpeed}"
    )
    print(
        "Yaw Speed:",
        displaySpeedBlock(info.minYawSpd, info.yawSpd, info.maxYawSpd),
        f"+{info.yawDeltaSpeed}"
    )


def drawDashboardFrame(info: ControlInfo, header: str = None, tail: str = None):
    os.system('clear')
    displayDashboard(info, header, tail)


def displayDashboard(info: ControlInfo, header: str = None, tail: str = None):
    if header is not None:
        print(header)
    print(f"Target:({info.targetX},{info.targetY})")
    print(composeDashboard(info))
    displaySpeedIndicator(info)
    if tail is not None:
        print(tail)
