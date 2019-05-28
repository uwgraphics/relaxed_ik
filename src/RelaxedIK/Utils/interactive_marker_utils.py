from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarker, InteractiveMarkerControl, Marker
from scipy import random as r

class InteractiveMarkerFeedbackUtil:
    def __init__(self):
        self.feedback = InteractiveMarkerFeedback()
        self.active = False

    def feedback_handler(self, feedback):
        self.active = True
        self.feedback = feedback



class InteractiveMarkerServerUtil:
    def __init__(self):
        pass



class InteractiveMarkerUtil:
    def __init__(self, init_pos = [0,0,0], init_quat = [1,0,0,0]):
        self.feedback_util = InteractiveMarkerFeedbackUtil()
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "common_world"
        int_marker.name = "interactive_marker_" + str(r.randint(0, high=99999999))
        int_marker.pose.position.x = init_pos[0]
        int_marker.pose.position.y = init_pos[1]
        int_marker.pose.position.z = init_pos[2]
        int_marker.pose.orientation.w = init_quat[0]
        int_marker.pose.orientation.x = init_quat[1]
        int_marker.pose.orientation.y = init_quat[2]
        int_marker.pose.orientation.z = init_quat[3]
        int_marker.scale = 0.2

        self.interactive_marker = int_marker

        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.07
        box_marker.scale.y = 0.07
        box_marker.scale.z = 0.07
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 0.6

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)

        self.interactive_marker.controls.append(box_control)

    def add_6dof_controls(self):
        self.add_translation_controls()
        self.add_rotation_controls()

    def add_translation_controls(self):
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_1"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        self.interactive_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_2"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        self.interactive_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_3"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        self.interactive_marker.controls.append(control)

    def add_rotation_controls(self):
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_1"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        self.interactive_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_2"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        self.interactive_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_3"
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        self.interactive_marker.controls.append(control)

