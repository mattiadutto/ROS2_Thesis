#include "nav2_custom_controller/fblin_unicycle.h"
#include "nav2_custom_controller/MPC_diffDrive_fblin.h"
#include "nav2_custom_controller/feedback_lin.hpp"
#include <ctime>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>
#include <utility>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "nav_msgs/msg/grid_cells.hpp"


#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp" // Include PoseWithCovarianceStamped message
#include "geometry_msgs/msg/pose_stamped.hpp" // Include PoseStamped message
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "nav_msgs/msg/path.hpp"


#include "nav2_custom_controller_msgs/msg/column_msg.hpp"
#include "nav2_custom_controller_msgs/msg/matrix_msg.hpp"


//sim robot file

class EmptyNode : public rclcpp::Node {

private:

    void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &amcl_pose) 
    {


    }

    void pose_callback(const geometry_msgs::msg::PoseStamped &pose)
    {

        pose_received = true;

        auto start = std::chrono::high_resolution_clock::now();


        robot_pose_.pose.position.x = pose.pose.position.x;
        robot_pose_.pose.position.y = pose.pose.position.y;
        robot_pose_.pose.orientation.z = pose.pose.orientation.z;
        robot_pose_.pose.position.z = pose.pose.position.z;

        double v_act,w_act;

MPC->set_actualRobotState(Eigen::Vector3d(robot_pose_.pose.position.x, robot_pose_.pose.position.y, robot_pose_.pose.position.z)); // position.z is converted yaw



MPC->set_referenceRobotState(Eigen::Vector3d(ref_pose_.pose.position.x, ref_pose_.pose.position.y, 0));



MPC->get_actualControl(v_act, w_act);

cmd_vel_.linear.x = v_act;

cmd_vel_.angular.z = w_act;


cmd_vel_pub_->publish(cmd_vel_);


//  execute_mpc();

//  execute_fblin();




}


void reference_pose_callback(const geometry_msgs::msg::PoseStamped &ref_pose )
{

    ref_pose_ = ref_pose;

}

//void constraints_callback(const nav2_custom_controller_msgs::msg::MatrixMsg::SharedPtr constraints)
void constraints_callback(const nav2_custom_controller_msgs::msg::MatrixMsg &received) 
{

// Extract number of rows
    size_t num_rows = received.matrix_rows.size();

// Resize _obst_matrix and _obst_vector
    obst_matrix.resize(num_rows, 2);
    obst_vector.resize(num_rows);


// Copy data from the received message to Eigen objects
    for (size_t i = 0; i < num_rows; ++i) 
    {
    // Copy data from matrix rows
        obst_matrix(i, 0) = received.matrix_rows[i].col1;
        obst_matrix(i, 1) = received.matrix_rows[i].col2;

    // Copy data from vector rows
        obst_vector(i) = received.vector_rows[i].col1;
    }




}


void  mpc_timer()
{

    if(pose_received == true)
    {
    //MPC->initialize();

    //execute_mpc();

        MPC->set_obstacle_matrices(obst_matrix,obst_vector);

        MPC->executeMPCcontroller(); 
        double vPx_act, vPy_act;

        Eigen::VectorXd MPC_actControl;

        MPC->get_actualMPCControl(MPC_actControl);
        vPx_act = MPC_actControl(0);
        vPy_act = MPC_actControl(1);
    }

    pose_received = false;

}

void fblin_timer()
{

    MPC->executeLinearizationController();


}

///////////// Variable declaration /////////////////////////////

geometry_msgs::msg::PoseStamped robot_pose_;
geometry_msgs::msg::PoseStamped ref_pose_;

rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
geometry_msgs::msg::Twist cmd_vel_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
rclcpp::Subscription<nav2_custom_controller_msgs::msg::MatrixMsg>::SharedPtr constraints_sub_;


rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;


rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_;

std::vector<double> predicted_x;
std::vector<double> predicted_y;
std::vector<double> predicted_theta;

nav_msgs::msg::Path pathMsg;

bool pose_received;

std::shared_ptr<MPC_diffDrive_fblin> MPC;
FeedbackLin fblin;

rclcpp::TimerBase::SharedPtr mpc_timer_;
rclcpp::TimerBase::SharedPtr fblin_timer_;

// Initialize Eigen matrix and vector
Eigen::MatrixXd obst_matrix;
Eigen::VectorXd obst_vector;


public:

    EmptyNode() : Node("empty_node"),fblin()
    {

        MPC = std::make_shared<MPC_diffDrive_fblin>();

       // subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/scout_mini/amcl_pose",100,std::bind(&EmptyNode::topic_callback, this,std::placeholders::_1));
        cmd_vel_pub_ = this ->create_publisher<geometry_msgs::msg::Twist>("/scout_mini/cmd_vel_from_node",100);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/scout_mini/pose",100,std::bind(&EmptyNode::pose_callback, this,std::placeholders::_1));
        ref_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/scout_mini/reference_pose",100,std::bind(&EmptyNode::reference_pose_callback, this,std::placeholders::_1));
        predicted_path_ = this->create_publisher<nav_msgs::msg::Path>("/scout_mini/predicted_path", 10);
        constraints_sub_ = this->create_subscription<nav2_custom_controller_msgs::msg::MatrixMsg>("/scout_mini/obstacle_constraints",10,std::bind(&EmptyNode::constraints_callback, this,std::placeholders::_1));


        mpc_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&EmptyNode::mpc_timer, this));
        fblin_timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&EmptyNode::fblin_timer, this));


        pose_received = false;

    obst_matrix = Eigen::MatrixXd::Zero(0,0); // in callback method it will be resized dynamically
    obst_vector = Eigen::VectorXd::Zero(0); // in callback method it will be resized dynamically


    std::cout<<"MPC node started"<<std::endl;


    // MPC parameters




    


/************* Controller initialization *************/
    
// MPC parameters
     int N = 10;
     double Ts_MPC = 0.2;
    // Low Q, High R - > prioritizes minimizing control effort, possibly at the expense of tracking performance.
    // High Q, Low R - > prioritizes state tracking accuracy over control effort, leading to aggressive control actions.
     double q = 0.0001;
     double r = 1; 
     int maxInfeasibleSolution = 2;

// Feedback linearization parameters
     double p_dist = 0.5;
     double Ts_fblin = 0.2;

// Robot parameters
    // robot top speed is 3 m/s, and the wheel raidus is 0.08m, so the wMax for top speed is 37.5
     double wMax = 5; // 
     double wMin = -wMax;
     double R = 0.08;
     double d = 0.4;

// Create and initialize MPC controller



    predicted_x.resize((N+1),0);
    predicted_y.resize((N+1),0);
    predicted_theta.resize((N+1),0);



    std::vector<double> lb(2, -100.0);
    std::vector<double> ub(2, +100.0);



   // Set parameters from yaml file
    this->get_parameter("FollowPath.prediction_horizon",N);
    this->get_parameter("mpc_sampling_time",Ts_MPC);
    this->get_parameter("Q",q);
    this->get_parameter("R",r);
    this->get_parameter("max_infeasible_solutions",maxInfeasibleSolution);
    this->get_parameter("variable_upper_bound_from",ub[0]);
    this->get_parameter("variable_upper_bound_to",ub[1]);
    this->get_parameter("variable_lower_bound_from",lb[0]);
    this->get_parameter("variable_lower_bound_to",lb[1]);
    this->get_parameter("p_distance",p_dist);
    this->get_parameter("feedback_linearization_sampling_time",Ts_fblin);
    this->get_parameter("max_wheel_speeds",wMax);
    wMin = -wMax;
    this->get_parameter("wheel_radius",R);
    this->get_parameter("base_width",d);

std::cout<<"N is set to: "<<N<<std::endl;

    MPC->set_MPCparams(Ts_MPC, N, q, r, lb, ub, maxInfeasibleSolution);
    MPC->set_FBLINparams(Ts_fblin, p_dist);
    MPC->set_robotParams(wMax, wMin, R, d);

    MPC->initialize();

// Linearization controller
    try
    {
        fblin_unicycle fblin_controller(p_dist);

    }catch(const std::exception& e)
    {
        std::cerr<<"Exception raised"<<e.what()<<std::endl;
    }   

}

void execute_mpc() 
{


    MPC->get_predicted_states(predicted_x,predicted_y,predicted_theta);

    for (size_t i = 0; i < predicted_x.size(); ++i)
    {
    // Create a new PoseStamped message
        geometry_msgs::msg::PoseStamped poseStamped;

    // Set the pose values (assuming 2D poses with z-coordinate = 0)
        poseStamped.pose.position.x = predicted_x[i];
           //     std::cout<<"predicted x: "<<predicted_x[i]<<std::endl;

    poseStamped.pose.position.y = predicted_y[i];  // Assuming y-coordinate is 0 in 2D
    // Add the pose to the path message
    pathMsg.poses.push_back(poseStamped);
    predicted_path_->publish(pathMsg);
}


double v_act,w_act;

// Get actual control signal
MPC->get_actualControl(v_act, w_act);

cmd_vel_.linear.x = v_act;

cmd_vel_.angular.z = w_act;


cmd_vel_pub_->publish(cmd_vel_);


}

void execute_fblin()
{
    double epsilon_ = 0.15;

    RCLCPP_INFO(this->get_logger(), "fblin function called");


// Convert pose.pose.orientation from Quaternion to Roll,Pitch,Yaw
    double roll, pitch, yaw;
    tf2::Quaternion quat;
    tf2::fromMsg(robot_pose_.pose.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

// std::cout<<"yaw "<<robot_pose_.pose.position.z<<std::endl; // yaw comes from subscriber

    std::cout<<"Current x: "<<robot_pose_.pose.position.x<<std::endl;
    std::cout<<"Current y: "<<robot_pose_.pose.position.y<<std::endl<<std::endl;

    std::cout<<"Target x: "<<ref_pose_.pose.position.x<<std::endl;
    std::cout<<"Target y: "<<ref_pose_.pose.position.y<<std::endl<<std::endl;


// Print the execution time
// std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;

// calculate point P at a distance epsilon from the robot
    fblin.calcPointP(robot_pose_, robot_pose_.pose.position.z, epsilon_);

// Apply proportional control for trajectory tracking without feed forward term
    double xp_dot_ = (ref_pose_.pose.position.x - fblin.getPointP()[0]) * 1; // was 2.5 before
    double yp_dot_ = (ref_pose_.pose.position.y - fblin.getPointP()[1]) * 0.5; // was 1.5 before

// Apply feedback linearization
    cmd_vel_ = fblin.linearize(xp_dot_, yp_dot_);


    cmd_vel_pub_->publish(cmd_vel_);

}
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EmptyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
