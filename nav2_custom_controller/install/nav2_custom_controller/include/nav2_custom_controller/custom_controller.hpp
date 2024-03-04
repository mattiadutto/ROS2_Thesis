#ifndef NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_


#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <fstream>


namespace nav2_custom_controller
{


class CustomController : public nav2_core::Controller
{

    public:

    CustomController(); // previously was set to = default;
    ~CustomController() override = default; // for every virtual func from base class we
                                            // use override except for constructor


    void configure( const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

    void cleanup() override;

  
    void activate() override;

  
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, nav2_core::GoalChecker * goal_checker) override;

    void setPlan(const nav_msgs::msg::Path & path) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;   

    


    protected:

    // Methods declaration
 

    // Member declaration
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_ {rclcpp::get_logger("CustomController")};
    rclcpp::Clock::SharedPtr clock_;

    double desired_linear_vel_;
    nav_msgs::msg::Path global_plan_;
    geometry_msgs::msg::PoseStamped pose_;
    rclcpp::Duration transform_tolerance_ {0, 0};

    // part of feedback_lin class 
    double xp_,yp_,xp_dot_,yp_dot_,kx_,ky_,epsilon_;

    double vel_,ang_vel_;

    int i;
    double print_x[500],print_y[500],print_w[500],print_vel[500],print_ang_vel[500],print_xp[500];
/*
    std::ofstream outputXFile;
    std::ofstream outputYFile;
    std::ofstream file;

	\*/

  





    // Publisher declaration

    //  return type of LifeCycleNode.create_publisher() is the same as this definition, where 
    // the type of publisher is user selectable
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher< geometry_msgs::msg::Twist>> twist_pub_; 
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

};


} // namespace nav2_custom_controller

#endif