from display_util import display
from pose_server import PoseServer
from pose_server import Pose as MyPose
from python_console_menu import AbstractMenu, MenuItem


class PoseHandler(AbstractMenu):
    def __init__(self,
                 pose_name: str,
                 pose_server: PoseServer):
        self.__pose_name = pose_name
        self.__pose_server = pose_server
        self.__delete_option = MenuItem(4, '删除该位姿信息', self.__delete)

        super().__init__(f'Pose [{pose_name}] handler')

    def initialise(self):
        self.add_menu_item(MenuItem(0,'退出').set_as_exit_option())
        self.add_menu_item(MenuItem(1, '查看位姿详情', self.__look))
        self.add_menu_item(MenuItem(2, '导航到该位姿处', self.__navigate))
        self.add_menu_item(MenuItem(3, '更新该位姿信息', self.__update))

        self.add_menu_item(self.__delete_option)

    def __look(self):
        pose: MyPose = self.__pose_server.query(self.__pose_name)
        display(pose.as_str())

    def __navigate(self):
        print('导航到该点位')

    def __update(self):
        print('更新位姿')

    def __delete(self):
        enter = input('确认删除？[Y/N]')
        if enter == 'Y' or enter == 'y':
            self.__pose_server.delete(self.__pose_name)
            display('删除成功')
            self.__delete_option.set_as_exit_option()
        else:
            print('取消删除')