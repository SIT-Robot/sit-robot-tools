class ControlInfo:
    """
    NOTE: DO NOT USE DIRTY MARK, the communication between two devices is unreliable
    """

    def __init__(self):
        self.targetX = 0
        self.targetY = 0
        self.targetYawSpeed = 0

        self.linearSpd = 0.2
        self.minLinearSpd = 0
        self.maxLinearSpd = 0.3
        self.linearDeltaSpeed = 0.01

        self.yawSpd = 0.5
        self.maxYawSpd = 1
        self.minYawSpd = 0
        self.yawDeltaSpeed = 0.03

    def resetTargetSpd(self):
        self.targetX = 0
        self.targetY = 0
        self.targetYawSpeed = 0

    def sendVia(self, publisher):
        from rosapi import publishSpeed
        publishSpeed(publisher, self.targetX, self.targetY, self.targetYawSpeed)

    def readFrom(self, json: dict):
        self.deserializeFrom("linearSpd", json, float)
        self.deserializeFrom("yawSpd", json, float)

    def saveTo(self) -> dict:
        return {
            "linearSpd": self.linearSpd,
            "yawSpd": self.yawSpd,
        }

    def deserializeFrom(self, name: str, json: dict, dtype: type):
        if name in json:
            v = json[name]
            if isinstance(v, dtype):
                setattr(self, name, v)


def resetSpeed(publisher):
    from rosapi import publishSpeed
    publishSpeed(publisher, 0, 0, 0)


def clamp(minvalue, value, maxvalue):
    return max(minvalue, min(value, maxvalue))
