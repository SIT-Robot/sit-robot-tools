import rospy
from geometry_msgs.msg import Pose
class RosPoseClient:
    def __init__(self):
        if rospy.is_shutdown():
            rospy.init_node('ros_pose_client')

    def query(self,name)->Pose:
