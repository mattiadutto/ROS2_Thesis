
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


  transformed_sub_ = this->create_subscription<nav_msgs::msg::GridCells>("scout_mini/transformed_pub",10,std::bind(&GoalPose::transformed_sub_callback, this,std::placeholders::_1));
  tf_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("scout_mini/tf_pub",10,std::bind(&GoalPose::tf_sub_callback, this,std::placeholders::_1));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        //tf_buffer_ = std::make_shared<tf2_ros::BufferClient>(this);
  tf_listener_ =
  std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  costmap_converter_polygons_ = std::make_unique<costmap_converter::CostmapToPolygonsDBSMCCH>();


  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("polygon_marker", 10);

  point_vect_.reserve(1000);




   
}

GoalPose::~GoalPose()
{
  RCLCPP_INFO(get_logger(), "goal_pose node has been terminated");
}

void GoalPose::transformed_sub_callback(const nav_msgs::msg::GridCells::SharedPtr msg)
{
  transformed_grid_ = *msg;
}

void GoalPose::tf_sub_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
  received_tf_ = *msg;
}

void GoalPose::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

  local_costmap = *msg; // dereference and store in private variable the occupancy grid

  // create a grid that moves with the robot's base_link
  std::vector<geometry_msgs::msg::Point> grid;

  point_vect_.clear();

  double minX = received_tf_.transform.translation.x-1;
  double maxX = received_tf_.transform.translation.x+1;
  double minY = received_tf_.transform.translation.y-1;
  double maxY = received_tf_.transform.translation.y+1;
  double resolution = 0.2;



  for (double x = minX; x <= maxX; x += resolution)
  {

    for (double y = minY; y <= maxY; y += resolution)
    {

      costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint point;

      point.x = x;
      point.y = y;
      point_vect_.push_back(point);

    }
  }

  /// convex hull computation
  geometry_msgs::msg::Polygon convex_hull;

  costmap_converter_polygons_->convexHullWrapper(point_vect_, convex_hull);


  costmap_converter_msgs::msg::ObstacleArrayMsg convex_hull_array;

  // Create an ObstacleMsg to hold the convex hull
  costmap_converter_msgs::msg::ObstacleMsg obstacle_msg;

// Add the points of the convex hull to the ObstacleMsg
  for (const auto& point : convex_hull.points) {
    obstacle_msg.polygon.points.push_back(point);
  }

  convex_hull_array.obstacles.resize(1); // Make sure the vector has at least 1 element
  convex_hull_array.obstacles[0] = obstacle_msg;



  publishAsMarker("map",convex_hull_array);


  
}

void GoalPose::publishAsMarker(const std::string &frame_id,const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles)
{

   visualization_msgs::msg::Marker line_list; // creater line_list as Marker msg
   line_list.header.frame_id = frame_id;
   line_list.header.stamp = rclcpp::Clock().now();
  line_list.ns = "Polygons"; // namespace of the container
  line_list.action = visualization_msgs::msg::Marker::ADD; // add marker
  line_list.pose.orientation.w = 1.0; 
  line_list.id = 0;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST; //line list type 

  line_list.scale.x = 0.02;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;

  for (const auto &obstacle : obstacles.obstacles) // iterate over each element in obstacles.obstacles over each polygon
  {
        //iterate over each vertex of the current polygon and create line segments by joining polygon points
    for (int j = 0; j < (int)obstacle.polygon.points.size() - 1; ++j)
    {
      geometry_msgs::msg::Point line_start;
        line_start.x = obstacle.polygon.points[j].x; // for each point j assign to line_start 
        line_start.y = obstacle.polygon.points[j].y;
        line_list.points.push_back(line_start); // append each vertex points to line_list.points
        geometry_msgs::msg::Point line_end;
        line_end.x = obstacle.polygon.points[j + 1].x; // this creates a line_end point that is j+1
        line_end.y = obstacle.polygon.points[j + 1].y;
        line_list.points.push_back(line_end); //every subsequent point will represent a line
      }// close loop for current polygon
      
      // After iterating through all vertices of the polygon, the loop checks if the polygon is closed (i.e., if it has more than two vertices).
      if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2) // if true -> polygon is closed
      {
        geometry_msgs::msg::Point line_start;
        line_start.x = obstacle.polygon.points.back().x; // add last point of polygon to line_start.x
        line_start.y = obstacle.polygon.points.back().y;
        line_list.points.push_back(line_start); //append these last points to line_list.points vector

        if (line_list.points.size() % 2 != 0) // if number is odd its incomplete line segment
        {
          geometry_msgs::msg::Point line_end;
          line_end.x = obstacle.polygon.points.front().x; // assign the first points of polygon to line_end
          line_end.y = obstacle.polygon.points.front().y;
          line_list.points.push_back(line_end); // append line_end to line_list.points and line segment can be constructed
        }
      }
    }
      /// NB! from line_list.points points of a line can be taken so that equation of line can be formed
      /// NB! from line_list.points points of a vertices of the obstacle can be taken so that equation of line can be formed
//
    marker_pub_->publish(line_list);

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
