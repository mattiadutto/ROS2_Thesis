#include "mpc/fblin_unicycle.h"
#include "mpc/MPC_diffDrive_fblin.h"
#include "mpc/feedback_lin.hpp"
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


//real robot file

class EmptyNode : public rclcpp::Node {

private:

    void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &amcl_pose) 
    {

    //  robot_pose_.pose.position.x = amcl_pose.pose.pose.position.x;
     // robot_pose_.pose.position.y = amcl_pose.pose.pose.position.y;

      //std::cout<<"Pose x: "<<robot_pose_.pose.position.x<<std::endl;

     //  MPC(); // Call the MPC function
   }

   void pose_callback(const geometry_msgs::msg::PoseStamped &pose)
   {

    auto start = std::chrono::high_resolution_clock::now();


     robot_pose_.pose.position.x = pose.pose.position.x;
     robot_pose_.pose.position.y = pose.pose.position.y;
     robot_pose_.pose.orientation.z = pose.pose.orientation.z;
     robot_pose_.pose.position.z = pose.pose.position.z;

     std::cout<<"current x: "<<robot_pose_.pose.position.x<<std::endl;
    // std::cout<<"current yaw: "<<robot_pose_.pose.position.z<<std::endl;
     execute_mpc();

   //  execute_fblin();


     auto end = std::chrono::high_resolution_clock::now();

     // Calculate the duration of the operation
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // Output the duration in microseconds
    std::cout << "Time taken of pose_callback: " << duration << " microseconds" << std::endl;

   }


   void reference_pose_callback(const geometry_msgs::msg::PoseStamped &ref_pose )
   {

    ref_pose_ = ref_pose;

   }

   geometry_msgs::msg::PoseStamped robot_pose_;
   geometry_msgs::msg::PoseStamped ref_pose_;

   rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
   geometry_msgs::msg::Twist cmd_vel_;
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_;

    std::vector<double> predicted_x;
    std::vector<double> predicted_y;
    std::vector<double> predicted_theta;

    nav_msgs::msg::Path pathMsg;





   std::shared_ptr<MPC_diffDrive_fblin> MPC;
   FeedbackLin fblin;

    public:

        EmptyNode() : Node("empty_node"),fblin()
        {

            MPC = std::make_shared<MPC_diffDrive_fblin>();

           // subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/scout_mini/amcl_pose",100,std::bind(&EmptyNode::topic_callback, this,std::placeholders::_1));
            cmd_vel_pub_ = this ->create_publisher<geometry_msgs::msg::Twist>("/scout_mini/cmd_vel_from_node",100);
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/scout_mini/pose",100,std::bind(&EmptyNode::pose_callback, this,std::placeholders::_1));
            ref_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/scout_mini/reference_pose",100,std::bind(&EmptyNode::reference_pose_callback, this,std::placeholders::_1));
            predicted_path_ = this->create_publisher<nav_msgs::msg::Path>("/scout_mini/predicted_path", 100);



        std::cout<<"MPC node started"<<std::endl;



   /************* Controller initialization *************/
    // MPC parameters
            const int N = 2;
            const double Ts_MPC = 0.2;
            // Low Q, High R - > prioritizes minimizing control effort, possibly at the expense of tracking performance.
            // High Q, Low R - > prioritizes state tracking accuracy over control effort, leading to aggressive control actions.
            const double q = 1;
            const double r = 4; 
            const int maxInfeasibleSolution = 2;

    // Feedback linearization parameters
            const double p_dist = 0.1;
            const double Ts_fblin = 0.01;

    // Robot parameters
            // robot top speed is 3 m/s, and the wheel raidus is 0.08m, so the wMax for top speed is 37.5
            const double wMax = 3; // 5 is too fast when doing angular velocities
            const double wMin = -wMax;
            const double R = 0.08;
            const double d = 0.4;

    // Create and initialize MPC controller


            predicted_x.resize((N+1),0);
    predicted_y.resize((N+1),0);
    predicted_theta.resize((N+1),0);



            std::vector<double> lb(2, -1.0);
            std::vector<double> ub(2, +1.0);
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


       // for (int i=0; i<50; i++)
      //  {
       // execute_mpc();
       // }






    }
    void execute_mpc() 
    {
        // Add your MPC logic here
        RCLCPP_INFO(this->get_logger(), "MPC function called");

     


      //  std::cout<<"Current x: "<<robot_pose_.pose.position.x<<std::endl;
       // std::cout<<"Current y: "<<robot_pose_.pose.position.y<<std::endl<<std::endl;

        //std::cout<<"Target x: "<<ref_pose_.pose.position.x<<std::endl;
        //std::cout<<"Target y: "<<ref_pose_.pose.position.y<<std::endl<<std::endl;

        double v_act,w_act;
        auto start = std::chrono::high_resolution_clock::now();
        // NB! if I use while loop for 400ms with wMax set to 5 rad/s then I am gettin error solving optim problem but with while loop to 1 sec no problems
        auto end = start + std::chrono::milliseconds(5); // End time
       //  auto end = start + std::chrono::milliseconds(400); // End time


        auto start_MPC_timer = std::chrono::high_resolution_clock::now(); // Start timer for 0.2-second interval
        auto start_fblin_timer = std::chrono::high_resolution_clock::now(); // Start timer for 0.2-second interval

            MPC->set_actualRobotState(Eigen::Vector3d(robot_pose_.pose.position.x, robot_pose_.pose.position.y, robot_pose_.pose.position.z)); // position.z is converted yaw

        //MPC->set_actualRobotState(Eigen::Vector3d(robot_pose_.pose.position.x, robot_pose_.pose.position.y, robot_pose_.pose.position.z)); // position.z is converted yaw
int calls =0;
       // while (std::chrono::high_resolution_clock::now() < end)
        //{



         
            
            auto start_time_MPC = std::chrono::high_resolution_clock::now();

            // Convert pose.pose.orientation from Quaternion to Roll,Pitch,Yaw
         //   double roll, pitch, yaw;
        //    tf2::Quaternion quat;
        //    tf2::fromMsg(robot_pose_.pose.orientation, quat);
      //      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // std::cout<<"yaw: "<<yaw<<std::endl;

           // std::cout<<"yaw: "<<robot_pose_.pose.orientation.z<<std::endl;


     //   std::cout<<"Current x: "<<robot_pose_.pose.position.x<<std::endl;
     //   std::cout<<"Current y: "<<robot_pose_.pose.position.y<<std::endl<<std::endl;

        
          //  MPC.set_actualRobotState(Eigen::Vector3d(robot_pose_.pose.position.x, robot_pose_.pose.position.y, robot_pose_.pose.position.z)); // position.z is converted yaw
        
          //  MPC.set_actualRobotState(Eigen::Vector3d(0.9, 0, 0));


            //std::cout<<"Current ref x: "<<ref_pose_.pose.position.x<<std::endl;
            //std::cout<<"Current ref y: "<<ref_pose_.pose.position.y<<std::endl;


            // Convert pose.pose.orientation from Quaternion to Roll,Pitch,Yaw
          //  double roll2, pitch2, yaw2;
           // tf2::Quaternion quat2;
          //  tf2::fromMsg(ref_pose_.pose.orientation, quat);
           // tf2::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);

         //   MPC.set_referenceRobotState(Eigen::Vector3d(2, -1, 0));


            MPC->set_referenceRobotState(Eigen::Vector3d(ref_pose_.pose.position.x, ref_pose_.pose.position.y, 0));



            // Check if 0.2 seconds have passed since the last MPC execution
         //   if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_MPC_timer).count() >= 4)
          //  {

                double vPx_act, vPy_act;

                // compute MPC control and optimisation to obtain optimal control inputs 
                // xp dot and yp dot to be used by the feedback linearisation to get v and w

                MPC->executeMPCcontroller(); //this function takes around 8ms to execute! with N=3, when the robot is rotating the execution times goes to 150ms
                Eigen::VectorXd MPC_actControl;

                // get xp dot and yp dot that are computed by the MPC
                MPC->get_actualMPCControl(MPC_actControl);
                vPx_act = MPC_actControl(0);
                vPy_act = MPC_actControl(1);


                // Reset the timer for the next 0.2-second interval
                start_MPC_timer = std::chrono::high_resolution_clock::now();

                calls++;
             //   std::cout<<"num of calls "<<calls<<std::endl;
          //  }


          //  if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_fblin_timer).count() >= 1)
          //  {


                MPC->executeLinearizationController();
                start_fblin_timer = std::chrono::high_resolution_clock::now();
          //  }

            // Get actual control signal
         //   MPC.get_actualControl(v_act, w_act);

          //  cmd_vel_.linear.x = v_act;
            
           // cmd_vel_.angular.z = w_act;


           // cmd_vel_pub_->publish(cmd_vel_);

        
        //}
            MPC->get_predicted_states(predicted_x,predicted_y,predicted_theta);

            for (size_t i = 0; i < predicted_x.size(); ++i)
    {
        // Create a new PoseStamped message
        geometry_msgs::msg::PoseStamped poseStamped;

        // Set the pose values (assuming 2D poses with z-coordinate = 0)
        poseStamped.pose.position.x = predicted_x[i];
        poseStamped.pose.position.y = predicted_y[i];  // Assuming y-coordinate is 0 in 2D
        // Add the pose to the path message
        pathMsg.poses.push_back(poseStamped);
    }

    predicted_path_->publish(pathMsg);

                
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
