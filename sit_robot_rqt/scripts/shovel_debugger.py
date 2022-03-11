import rospy
from PyQt5.QtWidgets import QApplication
import sys
import threading
from controllers.shovel_dialog import ShovelDialog


if __name__ == '__main__':
    rospy.init_node('shovel_debugger', sys.argv)

    app = QApplication(sys.argv)
    dialog = ShovelDialog()
    dialog.show()


    def spin_cb():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo('Exit...')


    threading.Thread(target=spin_cb)
    sys.exit(app.exec())
