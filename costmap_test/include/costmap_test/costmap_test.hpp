#ifndef COSTMAP_TEST___COSTMAP_TEST_HPP_
#define COSTMAP_TEST___COSTMAP_TEST_HPP_
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <termios.h>
#include <stdio.h>
#include <geometry_msgs/msg/pose_stamped.hpp> 
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/buffer.h> 
#include <tf2_ros/buffer_client.h>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_topic_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"



#include "costmap_converter_msgs/msg/obstacle_msg.hpp"
#include "costmap_converter/costmap_converter_interface.h"
#include "costmap_converter/costmap_to_polygons.h"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>


#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "nav_msgs/msg/grid_cells.hpp"





#include <cstdio>


class GoalPose : public rclcpp::Node
{
public:
  GoalPose();
  ~GoalPose();

  void publishAsMarker(const std::string &frame_id,const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles);


private:
  // ROS topic publishers/subscribers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::GridCells>::SharedPtr transformed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tf_sub_;


  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  // variables
  int key;

  geometry_msgs::msg::Twist cmd_vel;
  geometry_msgs::msg::PoseStamped goal;
  geometry_msgs::msg::PoseStamped base_link_pose_;

  nav_msgs::msg::OccupancyGrid local_costmap;
  geometry_msgs::msg::Polygon footprint_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
 // std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::unique_ptr<costmap_converter::CostmapToPolygonsDBSMCCH> costmap_converter_polygons_;

  std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> point_vect_;
  std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> point_vect_rotated_;


  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;



  std::vector<geometry_msgs::msg::Point> grid_;
 nav_msgs::msg::GridCells transformed_grid_;
 geometry_msgs::msg::TransformStamped received_tf_;




  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr tf_subscriber_;

  // Function prototypes
  void subscriber_callback(const geometry_msgs::msg::PointStamped clicked_point) const;
  void tfCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const;
  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void footprint_callback(const geometry_msgs::msg::Polygon footprint);
  void transformed_sub_callback(const nav_msgs::msg::GridCells::SharedPtr msg);
  void tf_sub_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

  
};
#endif  
