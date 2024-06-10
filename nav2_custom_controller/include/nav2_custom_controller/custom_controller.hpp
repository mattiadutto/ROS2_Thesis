#ifndef NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__CUSTOM_CONTROLLER_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <utility>
#include <chrono>
#include "nav2_core/controller.hpp"
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
#include <fstream> // Add this line to include the <fstream> header

#include "costmap_converter_msgs/msg/obstacle_msg.hpp"
#include "costmap_converter/costmap_converter_interface.h"
#include "costmap_converter/costmap_to_polygons.h"

#include "nav2_custom_controller_msgs/msg/column_msg.hpp"
#include "nav2_custom_controller_msgs/msg/matrix_msg.hpp"


#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nav_msgs/msg/grid_cells.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "nav2_custom_controller/convex_hull.hpp"


#include "tf2_ros/transform_broadcaster.h"


#include "nav2_custom_controller/fblin_unicycle.h"
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
                  std::string name,
                  const std::shared_ptr<tf2_ros::Buffer> tf,
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

    void obstacle_algorithm();

    void publishAsMarker(const std::string &frame_id,const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles,bool print_convex_region);

    costmap_converter_msgs::msg::ObstacleArrayMsg computeCentroid(const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles);

    void polygon_filter(const costmap_converter_msgs::msg::ObstacleArrayMsg &polygon_centroids, 
                        const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles,
                        costmap_converter_msgs::msg::ObstacleArrayMsg &considered_polygons,
                        costmap_converter_msgs::msg::ObstacleArrayMsg &considered_centroid);

    void calcLineEquation(const geometry_msgs::msg::Point32 &p1,  const geometry_msgs::msg::Point32 &p2,std::vector<std::vector<float>> &A_matrix,std::vector<std::vector<float>> &b_vect);

    void inflate_constraint(const std::vector<float> &A_matrix_row,const float &b_vect,const geometry_msgs::msg::Point32 &centroid);

    bool isViolated(const geometry_msgs::msg::Point32 &point,const std::vector<float> &A_matrix_row,const float &b_vect);

    void compute_violated_constraints(const geometry_msgs::msg::Point32 &p_centroid,const std::vector<std::vector<float>> &A_matrix,const std::vector<std::vector<float>> &b_vect);

    void compute_most_violated_constraints();

    float compute_distance_between_lines(const std::vector<float> &A_matrix_row,const float &b_vect,const float &b_vect_inflated);


    float compute_distance_to_violated_constraint(const float &x_coord, const float &y_coord, const std::vector<float> &A_matrix_row,const float &b_vect);

    void execute_mpc();

    void execute_fblin();

    void pose_sub_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &amcl_pose);




    
    


    protected:
    
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_{rclcpp::get_logger("CustomController")};
    rclcpp::Clock::SharedPtr clock_;

    // costmap_converter declarations
    rclcpp::Node::SharedPtr intra_proc_node_;
    std::string costmap_converter_plugin_; //!< Define a plugin name of the costmap_converter package (costmap cells are converted to points/lines/polygons)
    pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
    std::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter  
    std::string odom_topic_;
    int costmap_converter_rate_; //!< The rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate)
    double obstacle_distance_thresh_;


    // Timers declarations

    rclcpp::TimerBase::SharedPtr obstacle_algorithm_timer;
    rclcpp::TimerBase::SharedPtr mpc_timer_;
    rclcpp::TimerBase::SharedPtr fblin_timer_;

    // Publishers declarations

    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polygon_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_cnvx_reg_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr point_marker_pub_;

    // Subscribers declaration

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;


    // Obstacle Algorithm declarations

    nav2_costmap_2d::Costmap2D* costmap_;
    std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> point_vect_;
    std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> point_vect_rotated_;
    std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> point_vect_constrained_;
   // std::vector<geometry_msgs::msg::Point32> robot_footprint_;
    std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> robot_footprint_;
    std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint>robot_footprint_rotated_;
    costmap_converter_msgs::msg::ObstacleArrayMsg considered_centroid_;
    costmap_converter_msgs::msg::ObstacleArrayMsg stored_centroid_point_;
    std::vector<std::vector<float>> b_vect_;
    std::vector<std::vector<float>> b_vect_inflated_;
    std::vector<std::vector<float>> A_obst_matrix_;
    std::vector<std::vector<float>> A_violated_matrix_,b_violated_vect_,result_pose_stored_;
    std::vector<std::vector<float>> A_most_violated_matrix_;
    std::vector<std::vector<float>> b_most_violated_vect_;
    std::vector<std::vector<float>> A_most_violated_matrix_considered_;
    std::vector<std::vector<float>> b_most_violated_vect_considered_;
    std::vector<std::vector<float>> b_convex_region_vect_;
    std::vector<std::vector<float>> A_convex_region_matrix_;
    nav2_custom_controller_msgs::msg::MatrixMsg mpc_obstacle_constraints_;
    Eigen::MatrixXd mpc_obstacle_constraints_matrix_;
    Eigen::VectorXd mpc_obstacle_constraints_vector_;

    float result_pose,result_centroid;
    geometry_msgs::msg::TransformStamped received_tf_;
    nav_msgs::msg::Path centroid_path_msg_;
    geometry_msgs::msg::PoseStamped centroid_pose_stamped_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    bool path_saved_; // Flag to indicate whether the path has been saved
    int index;
    nav_msgs::msg::Path global_plan_;

    geometry_msgs::msg::TwistStamped cmd_vel;




    // MPC part 

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

    std::vector<double> predicted_x;
    std::vector<double> predicted_y;
    std::vector<double> predicted_theta;
    nav_msgs::msg::Path pathMsg;

};
} 

#endif