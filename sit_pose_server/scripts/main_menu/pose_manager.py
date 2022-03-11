from python_console_menu import AbstractMenu, MenuItem
from pose_server import PoseServer


class PoseMenu(AbstractMenu):
    def __init__(self,
                 pose_server: PoseServer):
        self.pose_server = pose_server
        super().__init__("位姿列表：")

    def pose_selected(self, name: str):
        def pose_selected_handle():
            from main_menu.pose_handle import PoseHandler
            ph = PoseHandler(name, self.pose_server)
            ph.display()

        return pose_selected_handle

    # override
    def initialise(self):
        self.update_menu_items()

    def update_menu_items(self):
        self.menu_items.clear()
        self.add_menu_item(MenuItem(0, '返回上一级').set_as_exit_option())
        for i, name in enumerate(self.pose_server.query_all_name()):
            self.add_menu_item(MenuItem(i + 1, name, self.pose_selected(name)))
