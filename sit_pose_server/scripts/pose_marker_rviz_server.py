import rospy
from interactive_markers.menu_handler import MenuHandler, MenuEntry
from interactive_markers import interactive_marker_server
from geometry_msgs.msg import Pose
from visualization_msgs.msg import *

class RvizPoseServer:
    def __init__(self):
        self.marker_server = interactive_marker_server.InteractiveMarkerServer('pose_marker_server')

    def add_pose(self,
                 name: str,
                 pose:Pose,
                 click_event):
        print(name)
        print(pose)
        marker = InteractiveMarker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.name = name
        marker.description = name
        marker.pose = pose

        viz_marker = Marker()
        viz_marker.type = Marker.ARROW
        viz_marker.pose = pose
        viz_marker.scale.x = 0.5
        viz_marker.scale.y = 0.5
        viz_marker.scale.z = 0.5
        viz_marker.color.r = 0.8

        viz_control = InteractiveMarkerControl()
        viz_control.always_visible = True
        viz_control.markers.append(viz_marker)

        click_control = InteractiveMarkerControl()
        click_control.name = 'click'
        click_control.interaction_mode = InteractiveMarkerControl.BUTTON
        click_control.always_visible = True
        click_control.markers.append(viz_marker)

        menu_control = InteractiveMarkerControl()
        menu_control.name = "menu"
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.always_visible = True
        menu_control.markers.append(viz_marker)

        marker.controls += [
            viz_control,
            click_control,
            menu_control,
        ]

        self.marker_server.insert(marker,
                                  feedback_cb=click_event)

        self.marker_server.applyChanges()