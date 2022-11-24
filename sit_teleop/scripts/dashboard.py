import time
from typing import Callable

from core import *
from utils import *

greenBlock = "🟩"
blackBlock = "⬛"
whiteBlock = "⬜"

_linearSpdHeader = "Linear Speed:"
_yawSpdHeader = "Yaw Speed:"
_yawSpdHeader += (len(_linearSpdHeader) - len(_yawSpdHeader)) * " "


class Dashboard:
    def __init__(self):
        self.speedBlockCount = 25
        self._board = Str2D(36, 11, filler=blackBlock)

    def composeSpeedDashboard(self, info: ControlInfo):
        board = self._board
        center = (9, 5)
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
        board.vline((18, 0), 11, filler=whiteBlock)

        turnCenter = (27, 5)
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
            print(self._board.compose())
            self.displaySpeedIndicator(info)
            if tail is not None:
                tail()
            time.sleep(0.32)

    def composeSpeedBlock(self, percent: float) -> str:
        solid = int(percent * self.speedBlockCount)
        hollow = self.speedBlockCount - solid
        return greenBlock * solid + whiteBlock * hollow

    def displaySpeedBlock(self, minSpd, cur, maxSpd) -> str:
        percent = (cur - minSpd) / (maxSpd - minSpd)
        return self.composeSpeedBlock(percent)

    def displaySpeedIndicator(self, info: ControlInfo):
        print(
            _linearSpdHeader,
            ' %.2f >>' % info.minLinearSpd,
            self.displaySpeedBlock(info.minLinearSpd, info.linearSpd, info.maxLinearSpd),
            '<< %.2f' % info.maxLinearSpd
        )
        print(
            _yawSpdHeader,
            ' %.2f >>' % info.minYawSpd,
            self.displaySpeedBlock(info.minYawSpd, info.yawSpd, info.maxYawSpd),
            '<< %.2f' % info.maxYawSpd
        )
