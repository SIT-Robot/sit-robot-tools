import rospy

from geometry_msgs.msg import Twist,TwistStamped
from sensor_msgs.msg import Imu

from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QWidget


from PyQt5 import QtCore,QtGui

import typing


from views import speed_watch_view
import threading

import sys

rospy.init_node('speed_watch')

def spin_cb():
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Exit...')

threading.Thread(target=spin_cb)


class SpeedWatchController(QWidget):
    def __init__(self, parent: typing.Optional['QWidget']) -> None:
        super().__init__(parent=parent)
        self.ui = speed_watch_view.Ui_Form()
        self.ui.setupUi(self)
        rospy.Subscriber('/cmd_vel',Twist,callback=self.cmd_vel_cb)
        rospy.Subscriber('/real_speed',TwistStamped,callback=self.real_speed_cb)
        rospy.Subscriber('/imu_raw',Imu,callback=self.imu_cb)

    def keyPressEvent(self, a0: QtGui.QKeyEvent) -> None:
        print('Hello')
        return super().keyPressEvent(a0)

    def cmd_vel_cb(self,cmd_vel:Twist):
        self.ui.expVxLabel.setText('%.3f' % cmd_vel.linear.x)
        self.ui.expVyLabel.setText('%.3f' %cmd_vel.linear.y)
        self.ui.expWLabel.setText('%.3f' %cmd_vel.angular.z)

    def real_speed_cb(self,speed:TwistStamped):
        self.ui.realVxLabel.setText('%.3f' %speed.twist.linear.x)
        self.ui.realVyLabel.setText('%.3f' %speed.twist.linear.y)
        self.ui.realWLabel.setText('%.3f' %speed.twist.angular.z)

    def imu_cb(self,imu_data:Imu):
        self.ui.angVelXLabel.setText('%.3f' % imu_data.angular_velocity.x)
        self.ui.angVelYLabel.setText('%.3f' % imu_data.angular_velocity.y)
        self.ui.angVelZLabel.setText('%.3f' % imu_data.angular_velocity.z)

        self.ui.eulerRowLabel.setText('%.3f' % 0)
        self.ui.eulerPitchLabel.setText('%.3f' % 0)
        self.ui.eulerYawLabel.setText('%.3f' % 0)

        self.ui.oriXLabel.setText('%.3f' % imu_data.orientation.x)
        self.ui.oriYLabel.setText('%.3f' % imu_data.orientation.y)
        self.ui.oriZLabel.setText('%.3f' % imu_data.orientation.z)
        self.ui.oriWLabel.setText('%.3f' % imu_data.orientation.w)

        self.ui.linAccXLabel.setText('%.3f' % imu_data.linear_acceleration.x)
        self.ui.linAccYLabel.setText('%.3f' % imu_data.linear_acceleration.y)
        self.ui.linAccZLabel.setText('%.3f' % imu_data.linear_acceleration.z)

app = QApplication(sys.argv)

sc = SpeedWatchController(None)
sc.show()

sys.exit(app.exec_())
