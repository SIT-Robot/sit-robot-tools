import rospy
from python_console_menu import AbstractMenu, MenuItem

from display_util import display
from rosmsg_converter import *
from pose_server import PoseServer

from main_menu.record_pose import record_pose


class MainMenu(AbstractMenu):
    def __init__(self,
                 pose_server: PoseServer):
        self.pose_server = pose_server
        super().__init__("MainMenu")

    # override
    def update_menu_items(self):
        self.title = f"""
欢迎使用SIT-Robot出品的位姿服务器
作者: zzq
当前命名空间: {self.pose_server.namespace}
        """.strip()

    def initialise(self):
        print('initialize')
        id_it = iter(range(100))
        next(id_it)
        self.add_menu_item(MenuItem(next(id_it), "关闭位姿服务器").set_as_exit_option())
        self.add_menu_item(MenuItem(next(id_it), "从Rviz位姿估计中录入位姿信息", self.__rviz_record_pose_selected))

        self.add_menu_item(MenuItem(next(id_it), "从Amcl发布的位姿话题中录入位姿信息", self.__amcl_record_pose_selected))

        self.add_menu_item(MenuItem(next(id_it), "管理当前命名空间下所有位姿信息", self.__look_all_poses))
        self.add_menu_item(MenuItem(next(id_it), "在Rviz中以InteractiveMarkers显示所有导航目标", self.__show_marks_in_rviz))
        self.add_menu_item(MenuItem(next(id_it), "生成该命名空间下的导航代码", self.__generate_navigation_code))
        self.add_menu_item(MenuItem(next(id_it), "切换命名空间", self.__renew_namespace))

    def __show_marks_in_rviz(self):
        from pose_marker_rviz_server import RvizPoseServer
        rviz_pose_server = RvizPoseServer()
        for name in self.pose_server.query_all_name():
            pose = self.pose_server.query(name)
            pose = my_pose_to_ros_pose(pose)
            from visualization_msgs.msg import InteractiveMarkerFeedback
            def feedback(fd: InteractiveMarkerFeedback):
                print(fd.marker_name + ' is clicked')

            rviz_pose_server.add_pose(name, pose, feedback)

    def __renew_namespace(self):
        display("TODO")

    def __generate_navigation_code(self):
        display("TODO")

    def __rviz_record_pose_selected(self):
        print("""开始监听位姿话题/initialpose,
消息类型: geometry_msgs/PoseWithCovarianceStamped
(Rviz中使用2D Pose Estimate进行2D位姿的发布)...""")
        record_pose(self.pose_server, '/initialpose')

    def __amcl_record_pose_selected(self):
        print("""开始监听位姿话题/amcl_pose,
消息类型: geometry_msgs/PoseWithCovarianceStamped
正在监听...""")
        record_pose(self.pose_server, '/amcl_pose')

    def __look_all_poses(self):
        if len(self.pose_server.query_all_name()) == 0:
            display('位姿数据库为空，请先录入位姿信息')
        else:
            from main_menu.pose_manager import PoseMenu
            p = PoseMenu(self.pose_server)
            p.display()
