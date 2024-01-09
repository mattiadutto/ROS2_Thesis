#ifndef GOAL_POSE___GOAL_POSE_HPP_
#define GOAL_POSE__GOAL_POSE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <stdio.h>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h> 



class GoalPose : public rclcpp::Node
{
public:
  GoalPose();
  ~GoalPose();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr current_pose_sub_;


  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
 
  // variables
  int key;
  geometry_msgs::msg::Twist cmd_vel;
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::PoseStamped base_link_pose_;


  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;

  // Function prototypes
  void update_callback();
  void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose) const;
  int getch();
  double get_user_coord();
  void transformCurrentPose();
  void get_baselink_tf();
  
};
#endif  
