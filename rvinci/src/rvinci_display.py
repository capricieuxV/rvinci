#!/usr/bin/env python
import rospy
import crtk
from sensor_msgs.msg import Joy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped

class DVRKInteractiveMarker:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('dvrk_interactive_marker')

        # CRTK interfaces for both MTML and MTMR
        self.arm_mtml = crtk.mtm('MTML')  # Left arm
        self.arm_mtmr = crtk.mtm('MTMR')  # Right arm

        # Subscriber to the clutch pedal
        self.clutch_sub = rospy.Subscriber("/footpedals/clutch", Joy, self.clutch_callback)

        # Subscriber to the grippers (if MTML or MTMR gripped)
        self.grip_mtml_sub = rospy.Subscriber("/MTML/gripper/closed", Joy, self.grip_callback_mtml)
        self.grip_mtmr_sub = rospy.Subscriber("/MTMR/gripper/closed", Joy, self.grip_callback_mtmr)

        # Publisher for the interactive marker
        self.marker_pub = rospy.Publisher('interactive_marker_topic', InteractiveMarker, queue_size=10)

        # State to track clutch quick-tap and gripping status
        self.clutch_pressed = False
        self.mtml_gripped = False
        self.mtmr_gripped = False

    def clutch_callback(self, joy_msg):
        """Callback for clutch pedal status."""
        # Assuming the first button is the clutch
        if joy_msg.buttons[0] == 1:  # Clutch pressed
            self.clutch_pressed = True
        elif joy_msg.buttons[0] == 2:  # Quick tap
            self.drop_marker()

    def grip_callback_mtml(self, joy_msg):
        """Callback for MTML gripper status."""
        self.mtml_gripped = joy_msg.buttons[0] == 1  # If gripped (button pressed)

    def grip_callback_mtmr(self, joy_msg):
        """Callback for MTMR gripper status."""
        self.mtmr_gripped = joy_msg.buttons[0] == 1  # If gripped (button pressed)

    def drop_marker(self):
        """Drop a marker at the position of the MTM that is gripped."""
        if self.mtml_gripped:
            # Drop marker for MTML
            self.add_interactive_marker_at_cursor(self.arm_mtml.measured_cp(), "mtml_marker", "MTML gripped marker")
        if self.mtmr_gripped:
            # Drop marker for MTMR
            self.add_interactive_marker_at_cursor(self.arm_mtmr.measured_cp(), "mtmr_marker", "MTMR gripped marker")

    def add_interactive_marker_at_cursor(self, pose_stamped, marker_name, description):
        """Add an interactive marker at the current pose."""
        # Create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = pose_stamped.header.frame_id
        int_marker.name = marker_name
        int_marker.description = description
        int_marker.scale = 0.1

        # Set the pose of the interactive marker
        int_marker.pose = pose_stamped.pose

        # Create a control for moving the marker
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.always_visible = True

        # Create a basic marker (sphere)
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add the marker to the control
        control.markers.append(marker)

        # Add the control to the interactive marker
        int_marker.controls.append(control)

        # Publish the interactive marker
        self.marker_pub.publish(int_marker)

if __name__ == "__main__":
    try:
        dvrk_interactive_marker = DVRKInteractiveMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
