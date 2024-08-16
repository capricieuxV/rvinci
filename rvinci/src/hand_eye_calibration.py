import crtk
import dvrk
import numpy as np
import PyKDL
import rospy

class DaVinciHandEyeCalibration:
    def __init__(self):
        # Initialize CRTK and dVRK interfaces for PSM1
        self.ral_psm = crtk.ral('hand_eye_calibration')
        self.psm = dvrk.arm(self.ral_psm, 'PSM2')
        
        # Initialize CRTK and dVRK interfaces for MTM1
        self.ral_mtm = crtk.ral('hand_eye_calibration')
        self.mtm = dvrk.arm(self.ral_mtm, 'MTMR')

        # Initialize variables to store calibration data
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

    def capture_data(self):
        # Capture PSM and MTM data
        psm_position = self.psm.measured_cp()
        mtm_position = self.mtm.measured_cp()
        rospy.loginfo("Capturing data point... PSM Position: {}, MTM Position: {}".format(psm_position, mtm_position))

        # Store the captured data
        self.calibration_data.append((psm_position, mtm_position))
        rospy.loginfo("Captured data point.")

    def move_psm_to_position(self, position):
        # Move PSM to a specified position in joint space
        rospy.loginfo("Moving PSM to position... {}".format(position))
        self.psm.move_jp(np.array(position)).wait()

    def move_mtm_in_cartesian(self, dx=0.0, dy=0.0, dz=0.05, rotation_deg=0.0):
        # Get the current position
        rospy.loginfo("Moving MTM in cartesian space... dx: {}, dy: {}, dz: {}, rotation_deg: {}".format(dx, dy, dz, rotation_deg))
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
        rospy.loginfo("Calibration data saved.")

    def shutdown(self):
        # Shutdown the CRTK and dVRK interfaces
        self.ral_psm.shutdown()
        self.ral_mtm.shutdown()
        rospy.loginfo("Shutting down calibration process.")

if __name__ == '__main__':
    calibration = DaVinciHandEyeCalibration()
    try:
        calibration.run_calibration()
    except rospy.ROSInterruptException:
        pass
    finally:
        calibration.shutdown()
