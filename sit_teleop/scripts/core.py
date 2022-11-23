class ControlInfo:
    def __init__(self):
        self.targetX = 0
        self.targetY = 0
        self.targetTurnSpeed = 0
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
        self.targetTurnSpeed = 0

    def sendVia(self, publisher):
        from rosapi import publishSpeed
        publishSpeed(publisher, self.targetX, self.targetY, self.targetTurnSpeed)


def resetSpeed(publisher):
    from rosapi import publishSpeed
    publishSpeed(publisher, 0, 0, 0)


def clamp(minvalue, value, maxvalue):
    return max(minvalue, min(value, maxvalue))


