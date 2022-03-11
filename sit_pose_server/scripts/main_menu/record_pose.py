import rospy
from rosmsg_converter import *

from geometry_msgs.msg import PoseWithCovarianceStamped
from pose_server import PoseServer
from display_util import display



def record_pose(pose_server:PoseServer,
                rostopic: str):
    try:

        target_pose: PoseWithCovarianceStamped = rospy.wait_for_message(rostopic, PoseWithCovarianceStamped)

        print('收到位姿话题：')
        print(target_pose)

        def check_name_format(n: str):
            if ' ' in n:
                return False
            if len(n) == 0:
                return False
            if n[0].isdigit():
                return False
            for ch in n:
                if ch == '_':
                    continue
                if ch.isalpha():
                    if not ch.isascii():
                        return False
            return True

        while True:
            name = input('请给定一个地点名称(需要纯英文单词，禁止包含空格，多个单词使用_分割，禁止以数字开头): ')
            if check_name_format(name):  # 验证名称
                if not pose_server.has_pose(name):  # 若不存在名字
                    my_pose = ros_pose_to_my_pose(name, target_pose.pose.pose)
                    pose_server.insert(my_pose)
                    display('位姿数据添加成功')
                    return
            else:
                print('命名格式有误，请重新起名')

    except rospy.ROSInterruptException:
        print('interrupt')
