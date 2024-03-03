#ifndef NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_


#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <utility>
#include <chrono>
#include "nav2_core/controller.hpp"
#include "nav2_custom_controller/feedback_lin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_core/exceptions.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "costmap_converter_msgs/msg/obstacle_msg.hpp"
#include "costmap_converter/costmap_converter_interface.h"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

//MPC includes

#include "nav2_custom_controller/MPC_diffDrive_fblin.h"



namespace nav2_custom_controller
{


class CustomController : public nav2_core::Controller
{

    public:

    CustomController(); 
    ~CustomController() override = default;

    
    // Function is called when controller server enters on_configure state. It perform declarations of ROS parameters and initialization of controller’s member variables
    // Parameters:
    // - parent: weak pointer to parent node
    // - name: controller name
    // - tf: tf buffer pointer
    // - costmap_ros: shared pointer to costmap.
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

    // Function is called when controller server enters on_cleanup state. Ideally this method should clean up resources which are created for the controller.
    void cleanup() override;

    // Function is called when controller server enters on_activate state. It should implement operations which are neccessary before controller goes to an active state.
    void activate() override;

    // Function is called when controller server enters on_deactivate state. It should implement operations which are neccessary before controller goes to an inactive state.
    void deactivate() override;

    // Function is called when a new velocity command is demanded by the controller server in-order for the robot to follow the global path. 
    // Parameters:
    // - pose: current robot pose
    // - velocity: current velocity
    // - goal_checker: goal_checker object
    // Returns: TwistStamped msg which represents the velocity command for the robot to drive.
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override;

    // Function is called when the global plan is updated. Ideally this method should perform operations that transform the global plan and store it.
    // Parameters:
    // - path: received global plan
    void setPlan(const nav_msgs::msg::Path & path) override;

    // Function is called when it is required to limit the maximum linear speed of the robot. Speed limit could be expressed in absolute value (m/s) or in percentage from maximum robot speed. Note that typically, maximum rotational speed is being limited proportionally to the change of maximum linear speed, in order to keep current robot behavior untouched.
    // Parameters:
    // - speed_limit: desire speed limit
    // - percentage: percentage from maximum robot speed
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;   

    void timer_callback();
    void publishAsMarker(const std::string &frame_id,const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles);




    


    protected:
    
    // Member declaration
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_{rclcpp::get_logger("CustomController")};
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Node::SharedPtr intra_proc_node_;

    std::string costmap_converter_plugin_; //!< Define a plugin name of the costmap_converter package (costmap cells are converted to points/lines/polygons)

    int costmap_converter_rate_; //!< The rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate)

    std::string odom_topic_;

    nav2_costmap_2d::Costmap2D* costmap_;



    FeedbackLin feedback_lin_;

    nav_msgs::msg::Path global_plan_;
    geometry_msgs::msg::PoseStamped target_pose_;
    geometry_msgs::msg::TwistStamped cmd_vel_;

    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
    std::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter  

  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
 
  rclcpp::TimerBase::SharedPtr wall_timer_;

  // MPC part ///////////////////////////////////
  std::unique_ptr<MPC_diffDrive_fblin> MPC_;

  // MPC parameters
     int N_;
     double Ts_MPC_;
     double q_;
     double r_;
     int maxInfeasibleSolution_;

    // Feedback linearization parameters
     double p_dist_;
     double Ts_fblin_;

    // Robot parameters
     double wMax_;
     double wMin_;
     double R_;
     double d_;

     std::vector<double> lb_;
    std::vector<double> ub_;






};


} // namespace nav2_custom_controller

#endif