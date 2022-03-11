from PyQt5.QtWidgets import QPushButton, QDialog, QApplication, QHBoxLayout
from views import shovel
import rospy
from sit_basketball_shovel_srv.srv import ShovelControl
from geometry_msgs.msg import Twist
from sit_protocol_msgs.srv import RequestData, RequestDataRequest
from std_msgs.msg import UInt8


class ShovelDialog(QDialog):
    def __init__(self):
        super(ShovelDialog, self).__init__()
        self.ui = shovel.Ui_Form()
        self.ui.setupUi(self)

        self.ui.up_button.pressed.connect(self.up_button_pressed)
        self.ui.up_button.released.connect(self.button_released)
        self.ui.down_button.pressed.connect(self.down_button_pressed)
        self.ui.down_button.released.connect(self.button_released)
        self.req_proxy = rospy.ServiceProxy('ShovelControl', ShovelControl)

        # 正在等待服务可用
        rospy.loginfo('等待串口服务器可用')
        self.req_proxy.wait_for_service()
        rospy.loginfo('串口服务器链接成功')

        rospy.Timer(rospy.Duration.from_sec(0.1), self.ros_timer_handle)

        self.pub = rospy.Publisher('Shovel', UInt8, queue_size=10)
        self.up = False
        self.down = False
        self.stop = False

    def ros_timer_handle(self, param):
        if self.up:
            self.pub.publish(1)
            # self.req_proxy.call(1)

        if self.down:
            self.pub.publish(2)
            # self.req_proxy.call(2)

        if not (self.up or self.down):
            self.pub.publish(0)
            # self.req_proxy.call(0)

    def up_button_pressed(self):
        self.up = True

    def down_button_pressed(self):
        self.down = True

    def button_released(self):
        self.up = False
        self.down = False
