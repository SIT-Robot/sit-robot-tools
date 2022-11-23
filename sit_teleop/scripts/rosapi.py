from geometry_msgs.msg import Twist

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
