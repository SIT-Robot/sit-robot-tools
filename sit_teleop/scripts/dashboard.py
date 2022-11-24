import time
from typing import Callable

from core import *
from utils import *

greenBlock = "🟩"
blackBlock = "⬛"
whiteBlock = "⬜"
speedBlockNumber = 20

_linearSpdHeader = "Linear Speed:"
_yawSpdHeader = "Yaw Speed:"
_yawSpdHeader += (len(_linearSpdHeader) - len(_yawSpdHeader)) * " "


class Dashboard:
    def __init__(self):
        self.speedBlockCount = 20
        self.board = Str2D(36, 11)

    def composeSpeedDashboard(self, info: ControlInfo):
        board = self.board
        center = (8, 5)
        # Move Left
        board.hline(center, -5, filler=greenBlock if info.targetY < 0 else blackBlock)
        # Move Up
        board.vline(center, 5, filler=greenBlock if info.targetX > 0 else blackBlock)
        # Move Right
        board.hline(center, 5, filler=greenBlock if info.targetY > 0 else blackBlock)
        # Move Down
        board.vline(center, -5, filler=greenBlock if info.targetX < 0 else blackBlock)
        board[center] = whiteBlock
        # Separator
        board.vline((16, 0), 11, filler=whiteBlock)

        turnCenter = (24, 5)
        # Turn Left
        board.hline(turnCenter, -5, filler=greenBlock if info.targetYawSpeed < 0 else blackBlock)
        # Turn Right
        board.hline(turnCenter, 5, filler=greenBlock if info.targetYawSpeed > 0 else blackBlock)
        board[turnCenter] = whiteBlock

    def startRender(self, info: ControlInfo, header: Callable[[], None] = None, tail: Callable[[], None] = None):
        while True:
            clearScreen()
            if header is not None:
                header()
            print(f"Target XY:({'%.2f' % info.targetX},{'%.2f' % info.targetY})")
            print(f"Target Yaw:{'%.2f' % info.targetYawSpeed}")
            self.composeSpeedDashboard(info)
            print(self.board.compose())
            displaySpeedIndicator(info)
            if tail is not None:
                tail()
            time.sleep(0.32)


def composeSpeedBlock(percent: float) -> str:
    solid = int(percent * speedBlockNumber)
    hollow = speedBlockNumber - solid
    return greenBlock * solid + whiteBlock * hollow


def displaySpeedBlock(minSpd, cur, maxSpd) -> str:
    percent = (cur - minSpd) / (maxSpd - minSpd)
    return composeSpeedBlock(percent)


def displaySpeedIndicator(info: ControlInfo):
    print(
        _linearSpdHeader,
        ' %.2f >>' % info.minLinearSpd,
        displaySpeedBlock(info.minLinearSpd, info.linearSpd, info.maxLinearSpd),
        '<< %.2f' % info.maxLinearSpd
    )
    print(
        _yawSpdHeader,
        ' %.2f >>' % info.minYawSpd,
        displaySpeedBlock(info.minYawSpd, info.yawSpd, info.maxYawSpd),
        '<< %.2f' % info.maxYawSpd
    )
