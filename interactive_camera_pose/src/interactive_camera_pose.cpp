/* 
 * Copyright (C) 2023, Michael Ferguson
 * Copyright (c) 2011, Vanadium Labs LLC
 * All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Michael Ferguson
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.hpp>

class InteractiveCameraPose : public rclcpp::Node
{
  using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
  using InteractiveMarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;

public:
  InteractiveCameraPose() : Node("interactive_camera_pose")
  {
    camera_link_ = this->declare_parameter<std::string>("camera_link", "head_tilt_link");
    base_link_ = this->declare_parameter<std::string>("base_link", "base_link");
    last_pose_.position.x = 0;
    last_pose_.position.y = 0;
    last_pose_.position.z = 0;
    last_pose_.orientation.x = 0;
    last_pose_.orientation.y = 0;
    last_pose_.orientation.z = 0;
    last_pose_.orientation.w = 1;

    // Create marker server
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("camera_control", this);

    visualization_msgs::msg::InteractiveMarker marker;
    marker.header.frame_id = camera_link_;
    marker.scale = 1;
    marker.name = "camera_pose";
    marker.description = "Camera Pose Controller";

    // Insert a box
    makeBoxControl(marker);
    InteractiveMarkerControl control;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.orientation.w = 1;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.orientation.w = 1;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.orientation.w = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    server_->insert(marker);
    server_->setCallback(marker.name, std::bind(&InteractiveCameraPose::update,
                                                this, std::placeholders::_1));
    server_->applyChanges();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
               std::bind(&InteractiveCameraPose::timer_callback, this));
  }

private:
  visualization_msgs::msg::Marker makeBox(visualization_msgs::msg::InteractiveMarker & msg)
  {
    visualization_msgs::msg::Marker marker;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
  }

  InteractiveMarkerControl &
  makeBoxControl(visualization_msgs::msg::InteractiveMarker & msg)
  {
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
  }

  void update(const InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    switch (feedback->event_type)
    {
      case InteractiveMarkerFeedback::MOUSE_UP:
        {
          // Print the new position
          double roll, pitch, yaw;
          tf2::getEulerYPR(feedback->pose.orientation, yaw, pitch, roll);
          RCLCPP_INFO_STREAM(rclcpp::get_logger("interactive_camera_pose"),
                             "xyz: " << feedback->pose.position.x << ", " 
                                     << feedback->pose.position.y << ", " 
                                     << feedback->pose.position.z << ", "
                                     << "rpy: " << roll << ", "
                                     << pitch << ", "
                                     << yaw << "\n");
        }
        break;

      case InteractiveMarkerFeedback::POSE_UPDATE:
        last_pose_.position.x = feedback->pose.position.x;
        last_pose_.position.y = feedback->pose.position.y;
        last_pose_.position.z = feedback->pose.position.z;
        last_pose_.orientation.x = feedback->pose.orientation.x;
        last_pose_.orientation.y = feedback->pose.orientation.y;
        last_pose_.orientation.z = feedback->pose.orientation.z;
        last_pose_.orientation.w = feedback->pose.orientation.w;
        break;
    }
    server_->applyChanges(); 
  }

  void timer_callback()
  {
    static tf2_ros::TransformBroadcaster broadcaster(this);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = base_link_;
    t.child_frame_id = camera_link_;
    t.transform.translation.x = last_pose_.position.x;
    t.transform.translation.y = last_pose_.position.y;
    t.transform.translation.z = last_pose_.position.z;
    t.transform.rotation.x = last_pose_.orientation.x;
    t.transform.rotation.y = last_pose_.orientation.y;
    t.transform.rotation.z = last_pose_.orientation.z;
    t.transform.rotation.w = last_pose_.orientation.w;
    broadcaster.sendTransform(t);
  }

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Pose last_pose_;
  std::string camera_link_;
  std::string base_link_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InteractiveCameraPose>());
  rclcpp::shutdown();
  return 0;
}
