from geometry_msgs.msg import Twist


class ControlInfo:
    # 方向
    x = 0
    y = 0
    th = 0
    # 最终要控制的速度
    xSpeed = 0
    ySpeed = 0
    turnSpeed = 0
    linear_speed: float = 0.2  # 最大线速度
    yaw_speed: float = 1  # 最大角速度

    def display(self):
        """
        打印当前能达到的最大速度
        """
        print(f"currently:\t speed {self.linear_speed} \t turn {self.yaw_speed}")


def publish_speed(publisher, vx: float, vy: float, vth: float):
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
