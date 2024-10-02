import crtk
import dvrk
import numpy as np
import PyKDL
import rospy
import csv
from datetime import datetime

class DaVinciHandEyeCalibration:
    def __init__(self):
        
        # Initialize CRTK and dVRK interfaces for MTMR
        self.ral_mtm = crtk.ral('hand_eye_calibration')
        self.mtm = dvrk.arm(self.ral_mtm, 'MTMR')

        # Initialize CRTK and dVRK interfaces for PSM2
        self.ral_psm = crtk.ral('hand_eye_calibration')
        self.psm = dvrk.arm(self.ral_psm, 'PSM1')

        # Initialize variables to store calibration data
        self.calibration_data_psm = []
        self.calibration_data_mtm = []
        
        # Adjust the number of iterations as needed
        self.n_iteration = 5

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
        self.calibration_data_psm.append(psm_position)
        self.calibration_data_mtm.append(mtm_position)
        rospy.loginfo("Captured data point.")

    def move_psm_in_cartesian(self, dx=0.0, dy=0.0, dz=0.05, rotation_deg=0.0):
        # Get the current position of PSM
        rospy.loginfo("Moving PSM in Cartesian space... dx: {}, dy: {}, dz: {}, rotation_deg: {}".format(dx, dy, dz, rotation_deg))
        psm_goal = self.psm.setpoint_cp()
        
        # Move PSM in Cartesian space
        psm_goal.p[0] += dx
        psm_goal.p[1] += dy
        psm_goal.p[2] += dz

        # Rotate tool tip frame if needed
        if rotation_deg != 0.0:
            psm_goal.M.DoRotX(np.deg2rad(rotation_deg))
        
        self.psm.move_cp(psm_goal).wait()

    def move_mtm_in_cartesian(self, dx=0.0, dy=0.0, dz=0.05, rotation_deg=0.0):
        # Get the current position of MTM
        rospy.loginfo("Moving MTM in Cartesian space... dx: {}, dy: {}, dz: {}, rotation_deg: {}".format(dx, dy, dz, rotation_deg))
        mtm_goal = self.mtm.setpoint_cp()
        
        # Move MTM in Cartesian space
        mtm_goal.p[0] += dx
        mtm_goal.p[1] += dy
        mtm_goal.p[2] += dz

        # Rotate tool tip frame if needed
        if rotation_deg != 0.0:
            mtm_goal.M.DoRotX(np.deg2rad(rotation_deg))
        
        self.mtm.move_cp(mtm_goal).wait()

    def run_calibration(self):

        ##### For now, manually move before calibration #####

        # # Move PSM2 to its home position first (self-defined)
        # rospy.loginfo("Moving PSM2 to home position: [2.00, 5.00, 70.00, 75.00, 0.00, 0.00]")
        # home_jp = np.array([2.00, 5.00, 70.00, 75.00, 0.00, 0.00])
        # self.psm.servo_jp(home_jp)
        # # wait time
        # rospy.sleep(5)

        for i in range(self.n_iteration):  
            # Move PSM in Cartesian space
            self.move_psm_in_cartesian(dx=0.01 * i, dz=0.01 * i, rotation_deg=3 * i)
            
            # Move MTM in Cartesian space
            self.move_mtm_in_cartesian(dx=-0.01 * i, dz=0.01 * i, rotation_deg=-3 * i)
            
            # Capture the data at this configuration
            self.capture_data()
            rospy.sleep(0.5)  # Wait a bit to ensure data is captured

        # After calibration, you could process or save the data
        self.save_calibration_data()

    # def save_calibration_data(self):
    #     # Save the captured calibration data for analysis
    #     np.save('calibration_data.npy', self.calibration_data)
    #     rospy.loginfo("Calibration data saved.")

    def save_calibration_data(self, file_type='csv'):

        current_date = datetime.now().strftime("%m%d")

        if file_type == 'csv':
            # Save data into CSV
            pair_name = 'R1' # which pair fo calibration data (eg.MTML & PSM2 is 'L2')
            filename = f"./data/{current_date}_cal_data_{pair_name}.csv"
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                # header
                writer.writerow(['PSM1/2 Position', 'MTMR/L Position'])
                # Write data (PSM first, then MTM)
                for psm_pos in self.calibration_data_psm:
                    writer.writerow([psm_pos])
                for mtm_pos in self.calibration_data_mtm:
                    writer.writerow([mtm_pos])
    
            rospy.loginfo(f"Calibration data saved as {filename}.")
        elif file_type == 'npy':
            # Save as NPY
            filename = f"{current_date}_cal_data.npy"
            np.savez(filename, psm_data=self.calibration_data_psm, mtm_data=self.calibration_data_mtm)
            rospy.loginfo(f"Calibration data saved as {filename}.")
        else:
            rospy.logwarn("Unsupported file type. Please use 'csv' or 'npy'.")

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