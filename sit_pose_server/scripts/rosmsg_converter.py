from geometry_msgs.msg import Pose as RosPose
from pose_server import Pose as MyPose


def my_pose_to_ros_pose(m_pose: MyPose) -> RosPose:
    ros_pose = RosPose()
    ros_pose.position.x = m_pose.pos_x
    ros_pose.position.y = m_pose.pos_y
    ros_pose.position.z = m_pose.pos_z

    ros_pose.orientation.x = m_pose.ori_x
    ros_pose.orientation.y = m_pose.ori_y
    ros_pose.orientation.z = m_pose.ori_z
    ros_pose.orientation.w = m_pose.ori_w

    return ros_pose


def ros_pose_to_my_pose(name: str,
                        ros_pose: RosPose) -> MyPose:
    return MyPose(
        name=name,
        pos_x=ros_pose.position.x,
        pos_y=ros_pose.position.y,
        pos_z=ros_pose.position.z,
        ori_x=ros_pose.orientation.x,
        ori_y=ros_pose.orientation.y,
        ori_z=ros_pose.orientation.z,
        ori_w=ros_pose.orientation.w,
    )
