#!/usr/bin/env python

import rospy
import crtk
import numpy as np
from geometry_msgs.msg import PoseStamped

class MeasureMTMLocation:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('measure_mtm_location')

        # Create CRTK interface for MTM (Master Tool Manipulator)
        self.mtm = crtk.mtm('MTML')  # Replace 'MTML' with 'MTMR' if you are using the right arm

        # ROS Publisher for the MTM pose
        self.pose_pub = rospy.Publisher('/mtm_pose', PoseStamped, queue_size=10)

        # Rate for the loop
        self.rate = rospy.Rate(10)  # 10 Hz

    def measure_location(self):
        """Measure and publish the MTM's current pose."""
        while not rospy.is_shutdown():
            # Get the current pose of the MTM
            current_pose = self.mtm.measured_cp()

            # Extract position and orientation
            position = current_pose.p
            orientation = current_pose.M.GetQuaternion()

            # Create a PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "base_link"  # Update this as per your setup

            # Set position (x, y, z)
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = position[2]

            # Set orientation (quaternion)
            pose_msg.pose.orientation.x = orientation[0]
            pose_msg.pose.orientation.y = orientation[1]
            pose_msg.pose.orientation.z = orientation[2]
            pose_msg.pose.orientation.w = orientation[3]

            # Publish the pose
            self.pose_pub.publish(pose_msg)

            # Print the position and orientation to the console
            rospy.loginfo("MTM Position: x={}, y={}, z={}".format(position[0], position[1], position[2]))
            rospy.loginfo("MTM Orientation: qx={}, qy={}, qz={}, qw={}".format(orientation[0], orientation[1], orientation[2], orientation[3]))

            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == "__main__":
    try:
        mtm_location_measurer = MeasureMTMLocation()
        mtm_location_measurer.measure_location()
    except rospy.ROSInterruptException:
        pass