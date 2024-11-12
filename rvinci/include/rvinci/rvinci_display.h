/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVINCI_DISPLAY_H
#define RVINCI_DISPLAY_H

#include "rviz/display.h"

#include <string>
#include <iostream>
#include <cmath>
#include <string>
#include <std_msgs/String.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <QWidget>
#include <QDesktopWidget>
#include <QApplication>

#include <QObject>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgrePrerequisites.h>
#include <OgreVector3.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreQuaternion.h>
#include <OgreRectangle2D.h>
#include <OgreTexture.h>

#include <boost/bind.hpp>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>
#include <tf/transform_datatypes.h>

#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include <interaction_cursor_rviz/interaction_cursor.h>
#include <rvinci_input_msg/rvinci_input.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_listener.h>
#include <rviz/frame_manager.h>

#include <interactive_markers/interactive_marker_server.h>

namespace Ogre
{
class SceneNode;
class RenderWindow;
class Camera;
class Viewport;
}

namespace rviz
{
class BoolProperty;
class RenderWidget;
class VectorProperty;
class QuaternionProperty;
class RosTopicProperty;
}

namespace rvinci
{
class rvinciDisplay: public rviz::Display, public Ogre::RenderTargetListener
{
//! RVinci display plugin for RViz.
/*! The RVinci display class is a plugin for RViz which is designed to allow
 * a da Vinci surgical console to navigate the RViz environment and manipulate
 * virtual objects within the world. It spawns a seperate window with stereo display
 * whose cameras can be controlled with the console. It also provides outputs for
 * the interaction_cursor_3D to spawn two 3D cursors. rvinciDisplay::
 */
Q_OBJECT
public:
   //! A constructor
   /*!The rviz/Qt Render Widget is created here, and the Ogre
   * rendering window is attached to it. The Ogre camera node
   * is spawned and the ROS subscriber and publisher setup member is called.
   */
  rvinciDisplay();
  //!Destructor
  virtual ~rvinciDisplay();

//  virtual void reset();

  //!Override from Ogre::RenderTargetListener
  virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

  //!Override from Ogre::RenderTargetListener
  virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );

protected:
  //!Called after onInitialize.
  /*!Called after onInitialize or if display plugin is enabled
   * after being disabled. Calls camera setup member if cameras
   * are not initialized and makes external render window visible.
   */
  virtual void onEnable();
  //!Called when plugin is disabled (by deselecting the check box).
  virtual void onDisable();
  //!Contains primary logic for camera control.
  /*!Camera position is either manually entered, or calculated by position
   * of the da Vinci grips when the camera pedal is activated. A vector is
   * calculated between the right and left grips. The translation of the midpoint
   * of this vector is added to the camera node position, and the change in orientation
   * of this vector is added to the orientation of the camera node.
   */
  void cameraUpdate();
  //!Called after constructor
  virtual void onInitialize();
  //!Override from rviz display class.
  virtual void update( float wall_dt, float ros_dt );


protected Q_SLOTS:
  //!Resets or intializes camera and 3D cursor positions.
  virtual void cameraReset();
  //!Sets up ROS subscribers and publishers
  virtual void pubsubSetup();
  //!Toggle for DVRK Gravity Compensation state
  virtual void gravityCompensation();
  // virtual void updateCursorVisibility();
  // virtual void updateCursorAxisVisibility();

private:
  //!Creates viewports and cameras.
  void cameraSetup();
  //!Called when input message received.
  /*!Contains primary input logic. Records input position and calculates change in
   * input position. Updates cursor position then sends data to camera control and cursor publisher.
   */
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);
  void leftCallback(const sensor_msgs::ImageConstPtr& img);
  void rightCallback(const sensor_msgs::ImageConstPtr& img);
  void clutchCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void teleopCallback(const std_msgs::Bool::ConstPtr& msg);
  void cameraCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void MTMCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i);
  void PSMCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i);
  void gripCallback(const std_msgs::Bool::ConstPtr& grab, int i);
  void coagCallback(const sensor_msgs::Joy::ConstPtr& msg);
  // void monoCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void measurementCallback(const std_msgs::Bool::ConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  //!Publishes cursor position and grip state to interaction cursor 3D display type.
  void publishCursorUpdate(int grab[2]);
  void updateCursorVisibility(const interaction_cursor_msgs::InteractionCursorUpdate& msg);
  //!Logic for grip state, used in interaction cursor 3D display type.
  int getaGrip(bool, int);
  //publish wrench 0 and gravity compensation
  void publishGravity();
  void publishWrench();

  //visualization
  void toggleClearMode(); // Toggle to clear all markers or not
  int uniqueLineMarkerID(); // Generate a unique ID for each line marker
  visualization_msgs::Marker makeMarker(geometry_msgs::Pose p, int id);
  visualization_msgs::Marker makeLineMarker(geometry_msgs::Point p1, geometry_msgs::Point p2, int id);
  visualization_msgs::Marker makeTextMessage(geometry_msgs::Pose p, std::string msg, int id);
  visualization_msgs::Marker deleteAllMarkers();

  //measurement
  void toggleDualHandMode();
  double calculateDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
  void publishMeasurementMarkers();

  enum MeasurementApp {_BEGIN, _START_MEASUREMENT, _MOVING, _END_MEASUREMENT};
  enum MarkerID {_STATUS_TEXT, _START_POINT, _END_POINT, _LINE, _DISTANCE_TEXT, _DELETE};

  rvinci_input_msg::rvinci_input rvmsg_;
  // std_msgs::String text_message_;

  bool camera_mode_, clutch_mode_;
  bool dual_hand_mode_;  // Flag to toggle between single-hand and dual-hand measurement
  int  coag_mode_;
  bool prev_grab_[2];
  bool wrench_published_;
  bool gravity_published_;
  bool left_grab_, right_grab_;
  bool left_released_, right_released_;
  bool MTM_mm_;
  bool sys_init_;
  bool teleop_mode_;
  bool Mono_mode_;
  bool coag_init_;
  bool cursor_visible_;
  bool camera_quick_tap_;
  bool clutch_quick_tap_;
  bool flag_delete_marker_;
  bool show_axes_right_;
  bool show_cursor_right_;
  bool show_axes_left_;
  bool show_cursor_left_;  
  bool start_measurement_PSM_[2];
  bool PSM_initial_position_set_[2];

  bool single_psm_mode_;
  bool first_point_set_;
  

  int marker_side_;
  MeasurementApp measurement_status_MTM;
  MeasurementApp measurement_status_PSM_;
  MeasurementApp measurement_status_single_PSM_;
  double distance_measured_;

  static Ogre::uint32 const LEFT_VIEW = 1;
  static Ogre::uint32 const RIGHT_VIEW = 2;

  Ogre::Camera* camera_[2];
  Ogre::SceneNode *camera_node_;
  Ogre::SceneNode *target_node_;
  Ogre::SceneNode *image_node_;

  unsigned char* buffer_[2];
  Ogre::Image* backgroundImage_[2];
  Ogre::MaterialPtr material_[2];
  Ogre::TexturePtr texture_[2];
  Ogre::Rectangle2D* rect_[2];

  Ogre::Viewport *viewport_[2];
  Ogre::RenderWindow *window_;
  Ogre::RenderWindow *window_R_;

  Ogre::Vector3 initial_cvect_;
  Ogre::Vector3 camera_ipd_;
  Ogre::Vector3 camera_offset_;
  Ogre::Vector3 cursor_offset_[2];
  Ogre::Vector3 camera_pos_;
  Ogre::Quaternion camera_ori_;
  Ogre::Vector3 input_pos_[2];
  Ogre::Vector3 input_change_[2];

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_input_;
  ros::Subscriber subscriber_lcam_;
  ros::Subscriber subscriber_rcam_;
  ros::Subscriber subscriber_clutch_;
  ros::Subscriber subscriber_teleop_;
  ros::Subscriber subscriber_camera_;
  ros::Subscriber subscriber_coag_;
  ros::Subscriber subscriber_mono_;
  ros::Subscriber subscriber_MTML_;
  ros::Subscriber subscriber_MTMR_;
  ros::Subscriber subscriber_overlay_text_;
  ros::Subscriber subscriber_lgrip_;
  ros::Subscriber subscriber_rgrip_;
  ros::Subscriber subscriber_PSM1_;
  ros::Subscriber subscriber_PSM2_;
  ros::Subscriber subscriber_mm_;
  ros::Subscriber subscriber_camera_info_;

  ros::Publisher publisher_rhcursor_;
  ros::Publisher publisher_lhcursor_;
  ros::Publisher publisher_rhcursor_display_;
  ros::Publisher publisher_lhcursor_display_;
  ros::Publisher pub_robot_state_[2];
  ros::Publisher publisher_rvinci_;
  ros::Publisher publisher_markers;
  ros::Publisher publisher_lwrench_;
  ros::Publisher publisher_rwrench_;
  ros::Publisher publisher_lgravity_;
  ros::Publisher publisher_rgravity_;

  ros::Time clutch_press_start_time_;

  rviz::VectorProperty *prop_cam_focus_;
  rviz::QuaternionProperty *property_camrot_;
  rviz::BoolProperty *prop_manual_coords_;
  rviz::VectorProperty *prop_camera_posit_;
  rviz::VectorProperty *prop_input_scalar_;
  rviz::RosTopicProperty *prop_ros_topic_;
  rviz::BoolProperty *prop_gravity_comp_;
  rviz::BoolProperty *prop_cam_reset_;
  rviz::BoolProperty *property_show_cursor_;
  rviz::BoolProperty *property_show_cursor_axis_;

  rviz::RenderWidget *render_widget_;
  rviz::RenderWidget *render_widget_R_;

  geometry_msgs::Pose cursor_[2];
  geometry_msgs::Pose measurement_start_;
  geometry_msgs::Pose measurement_end_;
  geometry_msgs::Pose PSM_pose_start_;
  geometry_msgs::Pose PSM_pose_end_;
  geometry_msgs::Pose PSM_initial_pose_[2]; 
  geometry_msgs::Pose PSM_pose_single_;

  rviz::FrameManager frame_manager_;
  std_msgs::Header cam_header_;
  double fx_;
  double fy_;
  double tx_;
  double cx_;
  double cy_;
  double img_height_;
  double img_width_;
};

} // namespace rvinci

#endif