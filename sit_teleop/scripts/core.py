from geometry_msgs.msg import Twist


class ControlInfo:
    # 方向
    x = 0
    y = 0
    th = 0
    # 最终要控制的速度
    targetX = 0
    targetY = 0
    targetTurnSpeed = 0
    linearSpd = 0.2  # 最大线速度
    yawSpd = 1  # 最大角速度
    maxLinearSpd = 0.3
    maxYawSpd = 0.5
    minLinearSpd = 0
    minYawSpd = 0
    linearDeltaSpeed = 0.03
    yawDeltaSpeed = 0.03

    def display(self):
        """
        打印当前能达到的最大速度
        """
        print(f"currently:\t speed {self.linearSpd} \t turn {self.yawSpd}")

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