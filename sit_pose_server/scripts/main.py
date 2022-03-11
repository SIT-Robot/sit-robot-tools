import rospy
import sys
from pose_server import PoseServer
from main_menu import MainMenu
import threading

rospy.init_node('pose_server', sys.argv)

namespace = rospy.get_param('namespace', 'default')
pose_server = PoseServer('a.db', namespace)

main_menu = MainMenu(pose_server)


def spin_handle():
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('as')


threading.Thread(target=spin_handle)

try:
    main_menu.display()
except KeyboardInterrupt as e:
    print(e)
except EOFError as e:
    print(e)
