#include "plugin/plugin.hpp"

// scout_mini_mpc_robot file
using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace plugin
{


// uncomment when using MPC
  
CustomController::CustomController():costmap_ros_(nullptr),MPC_(std::make_unique<MPC_diffDrive_fblin>())
{ 



 
        

   

   
}



 void CustomController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer>  tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>  costmap_ros)
  {


    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    std::cout<<"plugin name "<<plugin_name_<<std::endl;
    logger_ = node->get_logger(); 
    clock_ = node->get_clock();



    wall_timer_ = node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CustomController::timer_callback, this));
   // mpc_timer_ = node->create_wall_timer(std::chrono::milliseconds(200),std::bind(&CustomController::mpc_timer, this));
   // fblin_timer_ = node->create_wall_timer(std::chrono::milliseconds(10),std::bind(&CustomController::fblin_timer, this));

    

    // MPC PART

     // MPC parameters
    int N_ = 5;
    double Ts_MPC_ = 0.2; // need to automatically change the wall timer duration ! for now change manually

    // Low Q, High R - > prioritizes minimizing control effort, possibly at the expense of tracking performance.
    // High Q, Low R - > prioritizes state tracking accuracy over control effort, leading to aggressive control actions.
    double q_ = 4;
    double r_ = 10; 
    int maxInfeasibleSolution = 2; 
    
    // Feedback linearization parameters
    double p_dist_ = 0.2;
    double Ts_fblin_ = 0.01;
    
    // Robot parameters
    // robot top speed is 3 m/s, and the wheel radius is 0.08m, so the wMax for top speed is 37.5
    double wMax_ = 10; // 
    double wMin_ = -wMax_;
    double R_ = 0.08;
    double d_ = 0.4;

    std::vector<double> lb_(2, -100.0);
    std::vector<double> ub_(2, +100.0);

    predicted_x.resize((N_+1),0);
    predicted_y.resize((N_+1),0);
    predicted_theta.resize((N_+1),0);

    // create and initialize MPC
    MPC_->set_MPCparams(Ts_MPC_, N_, q_, r_, lb_, ub_, maxInfeasibleSolution_);
    MPC_->set_FBLINparams(Ts_fblin_, p_dist_);
    MPC_->set_robotParams(wMax_, wMin_, R_, d_);

    if(MPC_->initialize())
    {
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "MPC controller successfully initialized");

    }


    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "MPC controller successfully initialized");


    // Linearization controller
    try
    {
     fblin_unicycle fblin_controller(p_dist_);
     RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Linearization controller successfully initialized");

   }catch(const std::exception& e)
   {
    std::cerr<<"Exception raised"<<e.what()<<std::endl;
   }  



}


void CustomController::timer_callback()
{

  

}


void CustomController::cleanup()
{   
  RCLCPP_INFO(rclcpp::get_logger("CustomController"),"Cleaning Controller!");
}

void CustomController::activate()
{
  RCLCPP_INFO(rclcpp::get_logger("CustomController"),"Activating Controller!");
}

void CustomController::deactivate()
{
  RCLCPP_INFO(rclcpp::get_logger("CustomController"),"Deactivating Controller!");
}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &  , nav2_core::GoalChecker * ) 
{

  double roll, pitch, yaw;
  tf2::Quaternion quat;
  tf2::fromMsg(pose.pose.orientation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double v_act,w_act; 

  int status;
  MPC_->get_status(status);

  if (status == 1) // error solving optim problem
  {
    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Error solving the optimization problem - solver infeasible");
    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Re-initializing MPC");
    MPC_->initialize(); // re-init the MPC problem to try again

  }


  MPC_->set_actualRobotState(Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, yaw)); // position.z is converted yaw

  if (std::sqrt(std::pow((pose.pose.position.x) - global_plan_.poses[index].pose.position.x,2.0) + std::pow(pose.pose.position.y - global_plan_.poses[index].pose.position.y,2.0)) <= 0.8)
  {
    index = std::min(static_cast<int>(index + 1), static_cast<int>(global_plan_.poses.size() - 1));
  }


  // Convert global_plan_poses.pose.orientation from Quaternion to Roll,Pitch,Yaw
  double roll_glob_plan, pitch_glob_plan, yaw_glob_plan;
  tf2::Quaternion quat_glob_plan;
  tf2::fromMsg(global_plan_.poses[index].pose.orientation, quat_glob_plan);
  tf2::Matrix3x3(quat_glob_plan).getRPY(roll_glob_plan, pitch_glob_plan, yaw_glob_plan);

  MPC_->set_referenceRobotState(Eigen::Vector3d(global_plan_.poses[index].pose.position.x, global_plan_.poses[index].pose.position.y, yaw_glob_plan)); 
 
  // Static variable to store the last call time
    static auto last_call_time = std::chrono::steady_clock::now();

    fblin_timer();

    // Get the current time
    auto now = std::chrono::steady_clock::now();

    // Check if 200 ms have passed since the last call




    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_call_time).count() >= 200)
    {
     // RCLCPP_INFO(rclcpp::get_logger("CustomController"),"MPC optim started: %.2f",robot_pose_.pose.position.x);
      // Define a duration type for milliseconds
      using Milliseconds = std::chrono::milliseconds;

      // Get the current time before executing the function
      auto start_time = std::chrono::steady_clock::now();

      // Call mpc_timer
      mpc_timer();
      
      // Get the current time after executing the function
      auto end_time = std::chrono::steady_clock::now();

      // Calculate the duration of execution in milliseconds
      auto duration = std::chrono::duration_cast<Milliseconds>(end_time - start_time);

      double duration_ms = static_cast<double>(duration.count());

      // Print the duration in milliseconds
   //   RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Execution time: %.3f ms", duration_ms);

     // RCLCPP_INFO(rclcpp::get_logger("CustomController"),"MPC optim finished: %.2f",robot_pose_.pose.position.x);

      // Calculate the time between consecutive calls
      auto interval_duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_call_time);
      double interval_duration_ms = static_cast<double>(interval_duration.count());

      // Print the duration between consecutive calls
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Interval between MPC calls: %.3f ms", interval_duration_ms);

      // Reset the last call time
      last_call_time = now;
    }

  MPC_->get_actualControl(v_act, w_act);

  cmd_vel.twist.linear.x = v_act;
        
  cmd_vel.twist.angular.z = w_act;

  return cmd_vel;
}

void CustomController::mpc_timer() // every 200ms
{

  

 // if(obst_matrix.size()!=0) // if there are obstacle constraints, set them in MPC class
 // {

 //   MPC->set_obstacle_matrices(obst_matrix,obst_vector);
  //  RCLCPP_INFO(this->get_logger(), "Obstacle Constraints sent to MPC.");

 // }

  // Define a duration type for milliseconds
    using Milliseconds = std::chrono::milliseconds;

  // Get the current time before executing the function
    auto start_time = std::chrono::steady_clock::now();

  // Solve the MPC problem
    MPC_->executeMPCcontroller();

  // Get the current time after executing the function
    auto end_time = std::chrono::steady_clock::now();

  // Calculate the duration of execution in milliseconds
    auto duration = std::chrono::duration_cast<Milliseconds>(end_time - start_time);

  // Print the duration in milliseconds
  //  RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Execution time: %lld ms", duration.count());


    Eigen::VectorXd MPC_actControl;

    pose_received = false;

    Eigen::VectorXd point_p_pose(2);
    Eigen::VectorXd point_p_ref_pose(2);
    double obj_value;

    MPC_->get_actualMPCstate(point_p_pose);
    MPC_->get_referenceMPCstate(point_p_ref_pose);
    MPC_->get_objective_value(obj_value);

   // RCLCPP_INFO(rclcpp::get_logger("CustomController"), 
  //  "Current X: %.2f, Current Y: %.2f, Current Yaw: %.2f", 
  //  robot_pose_.pose.position.x, robot_pose_.pose.position.y, robot_pose_.pose.position.z);

  //  RCLCPP_INFO(rclcpp::get_logger("CustomController"), 
  //  "Current Xp: %.2f, Current Yp: %.2f, Xp ref: %.2f, Yp ref: %.2f, Objective value: %.2f", 
  //  point_p_pose(0), point_p_pose(1), point_p_ref_pose(0), point_p_ref_pose(1), obj_value);



/*
    MPC_->get_predicted_states(predicted_x,predicted_y,predicted_theta);
    pathMsg.poses.clear();
    for (size_t i = 0; i < predicted_x.size(); ++i)
    {
    // Create a new PoseStamped message
      geometry_msgs::msg::PoseStamped poseStamped;
      poseStamped.header.frame_id = "base_link";

    // Set the pose values (assuming 2D poses with z-coordinate = 0)
      poseStamped.pose.position.x = predicted_x[i];
    //     std::cout<<"predicted x: "<<predicted_x[i]<<std::endl;

      poseStamped.pose.position.y = predicted_y[i];  // Assuming y-coordinate is 0 in 2D
    // Add the pose to the path message
      pathMsg.poses.push_back(poseStamped);
      pathMsg.header.frame_id = "base_link";
      predicted_path_->publish(pathMsg);
    }    

*/


}

void CustomController::fblin_timer()
{
    
  MPC_->executeLinearizationController();
     
}


void CustomController::setPlan(const nav_msgs::msg::Path &path )
{

  global_plan_ = path;

}



void CustomController::setSpeedLimit(const double & , const bool & )
{

}

} // namespace

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(plugin::CustomController, nav2_core::Controller)
