import crtk
import dvrk
import numpy as np
import PyKDL
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DaVinciHandEyeCalibration:
    def __init__(self):
        # Initialize CRTK and dVRK interfaces for PSM1
        self.ral_psm = crtk.ral('hand_eye_calibration')
        self.psm = dvrk.arm(self.ral_psm, 'PSM2')
        
        # Initialize CRTK and dVRK interfaces for MTM1
        self.ral_mtm = crtk.ral('hand_eye_calibration')
        self.mtm = dvrk.arm(self.ral_mtm, 'MTMR')

        # Initialize image subscribers for stereo endoscope camera
        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber('/jhu_daVinci/stereo_processed/left/image', Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/jhu_daVinci/stereo_processed/right/image', Image, self.right_image_callback)

        # Initialize variables
        self.left_image = None
        self.right_image = None
        self.calibration_data = []

        # Check connections and initialize systems
        self.ral_psm.check_connections()
        self.ral_mtm.check_connections()
        self.ral_psm.spin()
        self.ral_mtm.spin()
        self.psm.enable()
        self.psm.home()
        self.mtm.enable()
        self.mtm.home()

    def left_image_callback(self, msg):
        # Convert ROS image message to OpenCV format for left camera
        self.left_image = self.ros_to_cv2_image(msg)
        self.show_images()

    def right_image_callback(self, msg):
        # Convert ROS image message to OpenCV format for right camera
        self.right_image = self.ros_to_cv2_image(msg)
        self.show_images()

    def ros_to_cv2_image(self, msg):
        # Convert ROS Image message to OpenCV image
        return cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), cv2.COLOR_BGR2RGB)

    def show_images(self):
        # Display the stereo images side by side
        if self.left_image is not None and self.right_image is not None:
            combined_image = np.hstack((self.left_image, self.right_image))
            cv2.imshow('Stereo Endoscope Images', combined_image)
            cv2.waitKey(1)  # Update the display window

    def capture_data(self):
        # Capture PSM and MTM data along with the stereo endoscope images
        if self.left_image is not None and self.right_image is not None:
            psm_position = self.psm.measured_cp()
            mtm_position = self.mtm.measured_cp()
            print("Capturing stereo data point... PSM Position: {}, MTM Position: {}".format(psm_position, mtm_position))

            # Store the captured data
            self.calibration_data.append((psm_position, mtm_position, self.left_image.copy(), self.right_image.copy()))
            print("Captured stereo data point.")

    def move_psm_to_position(self, position):
        # Move PSM to a specified position in joint space
        self.psm.move_jp(np.array(position)).wait()

    def move_mtm_in_cartesian(self, dx=0.0, dy=0.0, dz=0.05, rotation_deg=0.0):
        # Get the current position
        goal = self.mtm.setpoint_cp()
        
        # Move MTM in the cartesian space
        goal.p[0] += dx
        goal.p[1] += dy
        goal.p[2] += dz

        # Rotate tool tip frame if needed
        if rotation_deg != 0.0:
            goal.M.DoRotX(np.deg2rad(rotation_deg))
        
        self.mtm.move_cp(goal).wait()

    def run_calibration(self):
        # Example calibration loop
        for i in range(5):  # Adjust the number of iterations as needed
            # Move PSM to a new position
            self.move_psm_to_position([0.0, 0.0, 0.10 + i * 0.01, 0.0, 0.0, 0.0])
            
            # Move MTM in cartesian space
            self.move_mtm_in_cartesian(dz=i * 0.01, rotation_deg=10 * i)
            
            # Capture the data at this configuration
            self.capture_data()
            rospy.sleep(0.5)  # Wait a bit to ensure data is captured

        # After calibration, you could process or save the data
        self.save_calibration_data()

    def save_calibration_data(self):
        # Save the captured calibration data for analysis
        np.save('calibration_data.npy', self.calibration_data)
        print("Calibration data saved.")

    def shutdown(self):
        # Shutdown the CRTK and dVRK interfaces
        self.ral_psm.shutdown()
        self.ral_mtm.shutdown()
        print("Shutting down calibration process.")
        cv2.destroyAllWindows()  # Close OpenCV windows

if __name__ == '__main__':
    calibration = DaVinciHandEyeCalibration()
    try:
        calibration.run_calibration()
    except rospy.ROSInterruptException:
        pass
    finally:
        calibration.shutdown()
