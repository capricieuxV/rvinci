# dVRK Virtual Measurement Toolkit
This repository is based on RVinci from https://github.com/simonleonard/rvinci and 3D interaction cursors from https://github.com/aleeper/interaction_cursor_3d.
If you want to use our interactive mode, it is dependent on interactive marker tutorials package from https://github.com/ros-visualization/visualization_tutorials.


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
