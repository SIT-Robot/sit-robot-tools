import os
import platform
from io import StringIO
from typing import Tuple

platformName = platform.platform()


class Str2D:
    def __init__(self, x: int, y: int, filler: str = " "):
        self.x = x
        self.y = y
        self.inner = [filler for _ in range(x * y)]

    def checkInRange(self, x, y) -> int:
        return 0 <= x < self.x and 0 <= y < self.y

    def __setitem__(self, key, value: str):
        if isinstance(key, int):
            if 0 <= key < len(self.inner):
                self.inner[key] = value
        elif isinstance(key, tuple):
            x, y = key
            if self.checkInRange(x, y):
                self.inner[x * self.y + y] = value

    def __getitem__(self, key) -> str:
        if isinstance(key, int):
            if 0 <= key < len(self.inner):
                return self.inner[key]
        elif isinstance(key, tuple):
            x, y = key
            if self.checkInRange(x, y):
                return self.inner[x * self.y + y]

    def compose(self) -> str:
        with StringIO() as s:
            for y in range(self.y):
                for x in range(self.x):
                    s.write(self[x, y])
                s.write("\n")
            return s.getvalue()

    @property
    def view2D(self):
        return self.compose()

    def vline(self, start: Tuple[int, int], length: int, filler: str = "*"):
        """
        vertical line
        """
        x, y = start
        if length > 0:
            length += 1
        targetY = y + length
        minY = min(y, targetY)
        maxY = max(y, targetY)
        for newY in range(minY, maxY):
            self[x, newY] = filler

    def hline(self, start: Tuple[int, int], length: int, filler: str = "*"):
        """
        horizontal line
        """
        x, y = start
        if length > 0:
            length += 1
        targetX = x + length
        minX = min(x, targetX)
        maxX = max(x, targetX)
        for newX in range(minX, maxX):
            self[newX, y] = filler

def clearScreen():
    if platformName == "Windows":
        os.system("cls")
    else:
        os.system("clear")
