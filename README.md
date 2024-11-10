# dVRK Virtual Measurement Toolkit
This repository is based on RVinci from https://github.com/simonleonard/rvinci and 3D interaction cursors from https://github.com/aleeper/interaction_cursor_3d.
If you want to use our interactive mode, it is dependent on interactive marker tutorials package from https://github.com/ros-visualization/visualization_tutorials.

*After cloning the repo, modify rviz_stereo_pipeline.launch under rvinci/launch/!


# How to Install 

Firsly, create a work space and the source folder `catkin_ws`, please run following commands in the terminal:

```bash
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
```


Then, clone the repository into the source folder:

```bash
git clone --recursive https://github.com/ylz15/rvinci.git
```

Eventually, build the packages:

```bash
catkin build --summary
```

If having strange issues, try:
```bash
catkin build --summary --force-cmake
```


# How to run 
Launch the endoscope camera and camera calibration.
```bash
roslaunch rvinci rviz_stereo_pipeline.launch rig_name:=jhu_daVinci
```
If only want to use MTM:
```bash
rosrun dvrk_robot dvrk_console_json -j console-MTML-MTMR.json
```
If want to teleoperate PSM:
```bash
rosrun dvrk_robot dvrk_console_json -j console-MTMR-PSM1-MTML-PSM2-Teleop.json
```
Launch rvinci. We have 3 different modes to do so.
To launch rvinci:
```bash
roslaunch rvinci rvinci_rviz.launch
```
To launch rvinci with buttons to select the mode for the measurement tool:
```bash
roslaunch rvinci rvinci_with_button.launch
```
To launch rvinci with interactive mode:
```bash
roslaunch rvinci rvinci_interactive.launch
```

## How to calibrate and move the calibration file (.yaml) to correct directory
First, bring up the stereo camera:
```bash
roslaunch dvrk_video jhu_daVinci_video.launch
```

Run the calibration file (.py) program from ros package:
```bash
rosrun camera_calibration cameracalibrator.py --size 10x12 --square 0.0045 right:=/jhu_daVinci/right/decklink/jhu_daVinci_right/image_raw left:=/jhu_daVinci/left/decklink/jhu_daVinci_left/image_raw right_camera:=/jhu_daVinci/right/decklink left_camera:=/jhu_daVinci/left/decklink --approximate=0.050
```
 (furthre expand the details about grid size, camera name, raw image name blablabla)

If there is no camera_calibration package found in local, please visit ros.org for further installation tutorial.


Save results:

- Click 'Calibrate' to finalize the calibration
- Save: Click 'Save' and the calibration result is saved to /tmp/calibrationdata.tar.gz (you will see this instruction outputed on screen)
- Commit button to save (note camera_info_url should be correct)
- Untar calibration.tar file and there will be 'left.yaml' and 'right.yaml'
- Move the two .yaml files to /home//catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci

## PSM position adjustment
- Changed the json file for MTMR-PSM1-MTML-PSM2-Teleop
- Translational postion of PSM1 & PSM2 were +0.2 & -0.2 before, now it is +0.1440 & -0.1410 (based on how we wet up the hardware) 

## Untar calibration.tar file and there will be 'left.yaml' and 'right.yaml'
## Move the two .yaml files to /home/<user>/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-daVinci


