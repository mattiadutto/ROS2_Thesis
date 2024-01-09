// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Taehun Lim (Darby), Ryan Shim

#ifndef TELEOP__TELEOP_HPP_
#define TELEOP__TELEOP_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <stdio.h>


#include "geometry_msgs/msg/pose_stamped.hpp"


class Teleop : public rclcpp::Node
{
public:
  Teleop();
  ~Teleop();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  // test for another publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;


  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  int key;
  geometry_msgs::msg::Twist cmd_vel;
  // test for another publisher
  geometry_msgs::msg::PoseStamped goal;

  // Function prototypes
  void update_callback();
  int getch();
  
};
#endif  
