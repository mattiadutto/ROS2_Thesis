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

#include "teleop/teleop.hpp"

#include <memory>


using namespace std::chrono_literals;

Teleop::Teleop()
: Node("teleop_node")
{
  /************************************************************
  ** Initialise variables
  

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose",qos);

 
  /************************************************************
  ** Initialise ROS timers
  ************************************************************/

  // every 10ms update_callback will be called
  update_timer_ = this->create_wall_timer(10ms, std::bind(&Teleop::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Teleop node started");
}

Teleop::~Teleop()
{
  RCLCPP_INFO(this->get_logger(), "Teleop node has been terminated");
}


int Teleop::getch() // function to get non-blocking keyboard input character
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void Teleop::update_callback()
{

  key = getch();

  if (key=='w')
  {
    RCLCPP_INFO(get_logger(),"Go forward");
   // cmd_vel.linear.x = 0.5;

  }
  else if (key=='s')
  {
    RCLCPP_INFO(get_logger(),"Stop");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  
   else if (key=='d')
   {  
     RCLCPP_INFO(get_logger(),"Rotate CW");
     cmd_vel.angular.z = 1;
   }
   else if (key=='a')
   {
    RCLCPP_INFO(get_logger(),"Rotate CCW");
    cmd_vel.angular.z = -1;
   }
   else if (key =='x')
   {
    RCLCPP_INFO(get_logger(),"Go backwards");
    cmd_vel.linear.x = -0.5;
   }  

   goal.header.frame_id = "map";
   goal.header.stamp = rclcpp::Time(0);
   goal.pose.position.x = 0;
   goal.pose.position.y = -1;
   goal.pose.position.z = 0;
   goal.pose.orientation.w = 0;


 goal_pub_ -> publish(goal);
 //cmd_vel_pub_->publish(cmd_vel);

    

   
  
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleop>());
  rclcpp::shutdown();

  return 0;
}
