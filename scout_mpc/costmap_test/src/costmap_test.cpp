
#include "costmap_test/costmap_test.hpp"
#include <memory>
#include<string>

using namespace std::chrono_literals;
using std::placeholders::_1;

GoalPose::GoalPose()
: Node("goal_pose_node")
{

auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

clicked_point_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("scout_mini/clicked_point",100,std::bind(&GoalPose::subscriber_callback, this, _1));

local_costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "scout_mini/local_costmap/costmap", 10, std::bind(&GoalPose::localCostmapCallback, this, std::placeholders::_1));
footprint_subscriber_ = this->create_subscription<geometry_msgs::msg::Polygon>("scout_mini/local_costmap/footprint",10,std::bind(&GoalPose::footprint_callback,this,std::placeholders::_1));
 
tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());

tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


      // MPC part //////////////

  //MPC param
  N_ = 2;
  Ts_MPC_ = 0.2;
  q_ = 2.0;
  r_ = 1.0;
  maxInfeasibleSolution_ = 2;

  // Feedback linearization parameters
  p_dist_ = 0.1;
  Ts_fblin_ = 0.01;

  // Robot parameters
  wMax_ = 10.0;
  wMin_ = -wMax_;
  R_ = 0.2;
  d_ = 0.4;

  lb_ = {2, -100.0};
  ub_ = {2, 100.0};

  MPC_->set_MPCparams(Ts_MPC_, N_, q_, r_, lb_, ub_, maxInfeasibleSolution_);
  MPC_->set_FBLINparams(Ts_fblin_, p_dist_);
  MPC_->set_robotParams(wMax_, wMin_, R_, d_);

  if(MPC_->initialize())
  {
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "MPC controller successfully initialized");

  }


}

GoalPose::~GoalPose()
{
  RCLCPP_INFO(get_logger(), "goal_pose node has been terminated");
}

void GoalPose::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

  local_costmap = *msg; // dereference and store in private variable the occupancy grid

  
}
void GoalPose::subscriber_callback(const geometry_msgs::msg::PointStamped clicked_point) const 
{

  RCLCPP_INFO(get_logger(), "Clicked X point: %.2f",clicked_point.point.x);
  RCLCPP_INFO(get_logger(), "Clicked Y point: %.2f",clicked_point.point.y);

  // create instance of Costmap2D object by passing the occupancy grid of the local costmap
  nav2_costmap_2d::Costmap2D costmap(local_costmap);
  unsigned int mx,my;

  
  costmap.worldToMap(clicked_point.point.x,clicked_point.point.y,mx,my);

  // get the index of the clicked cell
  unsigned int index = costmap.getIndex(mx,my);
  // get the cost of the cell with the given index
  unsigned char cost = costmap.getCost(index);

  RCLCPP_INFO(get_logger(), "Index: %u, Cost: %u", index, cost);

  //costmap.saveMap("local_costmap");


} 

void GoalPose::footprint_callback(const geometry_msgs::msg::Polygon footprint)
{
  footprint_ = footprint;

}


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
