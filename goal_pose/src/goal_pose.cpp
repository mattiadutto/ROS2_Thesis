
#include "goal_pose/goal_pose.hpp"
#include <memory>


using namespace std::chrono_literals;
using std::placeholders::_1;

GoalPose::GoalPose()
: Node("goal_pose_node")
{
  /************************************************************
  ** Initialise variables
  

  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose",qos);

  //current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",100,std::bind(&GoalPose::topic_callback, this, _1));




  // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Call get_baselink_tf function every second
   //  timer_ = this->create_wall_timer(
  // 1s, std::bind(&GoalPose::get_baselink_tf, this));



  /************************************************************
  ** Initialise ROS timers
  ************************************************************/

  // every 10ms update_callback will be called by using std::bind
  update_timer_ = create_wall_timer(10ms, std::bind(&GoalPose::update_callback, this));

  RCLCPP_INFO(get_logger(), "goal_pose node has started");
  RCLCPP_INFO(get_logger(), "Press 'w' to get current pose or 's' to set goal pose!");


}

GoalPose::~GoalPose()
{
  RCLCPP_INFO(get_logger(), "goal_pose node has been terminated");
}

double GoalPose::get_user_coord()
{
  double coord;

  while(true)
  {
    if (std::cin >> coord)
    {
      break;
    }
     else
        {
            // Input is not a double, clear the error state and ignore invalid input
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input. Please enter a valid double: ";
        }
  }

  return coord;

}

// function to get non-blocking keyboard input character
int GoalPose::getch() 
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

// subscriber callback
/*
void GoalPose::topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose) const 
{

 // RCLCPP_INFO(rclcpp::get_logger("Current X coordinate: "), std::to_string(amcl_pose.pose.pose.position.x).c_str());
 // RCLCPP_INFO(rclcpp::get_logger("Current Y coordinate: "),std::to_string(amcl_pose.pose.pose.position.y).c_str());
  //RCLCPP_INFO(rclcpp::get_logger("Current Z coordinate: "),std::to_string(amcl_pose.pose.pose.position.z).c_str());
 

  RCLCPP_INFO(get_logger(), "Current X coordinate: %.2f",amcl_pose.pose.pose.position.x);
  RCLCPP_INFO(get_logger(), "Current Y coordinate: %.2f",amcl_pose.pose.pose.position.y);
  RCLCPP_INFO(get_logger(), "Current W coordinate: %.2f",amcl_pose.pose.pose.orientation.w);

} \*/



void GoalPose::update_callback()
{


  key = getch();

  if (key=='w')
  { 
    get_baselink_tf();
  }
  else if (key=='s')
  {
    double x_,y_,w_;
    RCLCPP_INFO(get_logger(),"Please set goal x:");
    x_=get_user_coord();
    RCLCPP_INFO(get_logger(),"Please set goal y:");
    y_=get_user_coord();
    RCLCPP_INFO(get_logger(),"Please set goal w:");
    w_=get_user_coord();

    RCLCPP_INFO(get_logger(), "Goal Pose: x=%.2f, y=%.2f, w=%.2f",x_,y_,w_);

     goal.header.frame_id = "map";
     goal.header.stamp = rclcpp::Time(0);
     goal.pose.position.x = x_;
     goal.pose.position.y = y_;
     goal.pose.position.z = 0;
     goal.pose.orientation.w = w_;
     goal_pub_ -> publish(goal);

  }
  

  
  


    

   
  
}

// get tf from map to baselink and print the pose of the robot
void GoalPose::get_baselink_tf()
{

  std::string fromFrameRel = target_frame_.c_str();
  std::string toFrameRel = "map";
  geometry_msgs::msg::TransformStamped t;
  try {
          t = tf_buffer_->lookupTransform(
            "map", "base_link",
            tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "map", "base_link", ex.what());
          return;
        }

        base_link_pose_.pose.position.x = t.transform.translation.x;
        base_link_pose_.pose.position.y = t.transform.translation.y;
        base_link_pose_.pose.position.z = t.transform.translation.z;
        base_link_pose_.pose.orientation.w = t.transform.rotation.w;


        RCLCPP_INFO(get_logger(), "Current Pose: x=%.2f, y=%.2f, w=%.2f",
        base_link_pose_.pose.position.x,
        base_link_pose_.pose.position.y,
        base_link_pose_.pose.orientation.w);


}


/* void GoalPose::transformCurrentPose()
{
    try
    {
        // Get the transform from "map" to "base_link"
        geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);

        // Transform position and orientation
        tf2::doTransform(transform_stamped.transform.translation, base_link_pose_.pose.position, transform_stamped);
        tf2::doTransform(transform_stamped.transform.rotation, base_link_pose_.pose.orientation, transform_stamped);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "TransformException: %s", ex.what());
    }
}
\*/

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPose>());
  rclcpp::shutdown();

  return 0;
}
