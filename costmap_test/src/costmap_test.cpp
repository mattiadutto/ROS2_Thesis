
#include "costmap_test/costmap_test.hpp"
#include <memory>
#include<string>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Define a lambda function for calculating Euclidean distance
  auto euclideanDistance = [](double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  };



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

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",100,std::bind(&GoalPose::pose_sub_callback, this,std::placeholders::_1));

  obstacles_sub_ = this->create_subscription<costmap_converter_msgs::msg::ObstacleArrayMsg>("costmap_obstacles",100,std::bind(&GoalPose::obstacles_sub_callback,this,std::placeholders::_1));
  point_vect_.reserve(1000);
  point_vect_rotated_.reserve(1000);


  point_vect_.clear();
  point_vect_rotated_.clear();

  double minX = -1;
  double maxX = 1;
  double minY = -1;
  double maxY = 1;
  double resolution = 0.05;



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

 // point_vect_.clear();
//  point_vect_rotated_.clear();
/*
  double minX = received_tf_.transform.translation.x-1;
  double maxX = received_tf_.transform.translation.x+1;
  double minY = received_tf_.transform.translation.y-1;
  double maxY = received_tf_.transform.translation.y+1;
  double resolution = 0.1;



  for (double x = minX; x <= maxX; x += resolution)
  {

    for (double y = minY; y <= maxY; y += resolution)
    {

      costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint point;

      point.x = x;
      point.y = y;
      point_vect_.push_back(point);

    }
  }*/
/*

   // Apply rotation
    double y = received_tf_.transform.rotation.y;
    double x = received_tf_.transform.rotation.x;
    double z = received_tf_.transform.rotation.z;
    double w = received_tf_.transform.rotation.w;

    for(const auto& point : point_vect_)
    {

      costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint poinT = point;

      double x_tmp = w * poinT.x + 0 - z * poinT.y;
      double y_tmp = w * poinT.y + z * poinT.x - 0;
      double z_tmp = 0 + x * poinT.y - y * poinT.x;
      double w_tmp = -x * poinT.x - y * poinT.y - 0;

      poinT.x = w_tmp * -x + x_tmp * w + y_tmp * -z - z_tmp * -y;
      poinT.y = w_tmp * -y + x_tmp * z + y_tmp * w + z_tmp * -x;

      point_vect_rotated_.push_back(poinT);



    }*/
  ///// v2 of the translation + rotation of the grid points

  point_vect_rotated_.clear();

  for (const auto& point : point_vect_)
  {

    /*costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint transformed_point;

    // Apply translation
    transformed_point.x = point.x + received_tf_.transform.translation.x;
    transformed_point.y = point.y + received_tf_.transform.translation.y;

      

    // when using the rotation part the translation gets wrong ?!? 


    // Apply rotation
    double x = received_tf_.transform.rotation.x;
    double y = received_tf_.transform.rotation.y;
    double z = received_tf_.transform.rotation.z;
    double w = received_tf_.transform.rotation.w;


    double x_tmp = w * transformed_point.x + 0 - z * transformed_point.y;
    double y_tmp = w * transformed_point.y + z * transformed_point.x - 0;
    double z_tmp = 0 + x * transformed_point.y - y * transformed_point.x;
    double w_tmp = -x * transformed_point.x - y * transformed_point.y - 0;

    transformed_point.x = w_tmp * -x + x_tmp * w + y_tmp * -z - z_tmp * -y;
    transformed_point.y = w_tmp * -y + x_tmp * z + y_tmp * w + z_tmp * -x;

    point_vect_rotated_.push_back(transformed_point);*/

  /// Convert pose.pose.orientation from Quaternion to Roll,Pitch,Yaw
  //
  double roll, pitch, yaw;
  tf2::Quaternion quat;
  tf2::fromMsg(received_tf_.transform.rotation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  
costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint transformed_point;
    // Rotate the point around the Z-axis
    double cos_theta = cos(yaw);
    double sin_theta = sin(yaw);
    transformed_point.x = cos_theta * point.x - sin_theta * point.y;
    transformed_point.y = sin_theta * point.x + cos_theta * point.y;

    // Translate the point
    transformed_point.x += received_tf_.transform.translation.x;
    transformed_point.y += received_tf_.transform.translation.y;


    // perform check for each point if it satisfies the most violated constraints

    point_vect_rotated_.push_back(transformed_point);



  }

 

//////////// code for linear constraints and considered polygons


  /// convex hull computation
  geometry_msgs::msg::Polygon convex_hull;
//
 // costmap_converter_polygons_->convexHullWrapper(point_vect_, convex_hull);

 costmap_converter::ObstacleArrayPtr obstacles_ptr;
 // = boost::make_shared<costmap_converter_msgs::msg::ObstacleArrayMsg>();
//costmap_converter_msgs::msg::ObstacleArrayMsg obstacles;

// not receiving polygons !!!!
 //costmap_converter::PolygonContainerConstPtr polygons = costmap_converter_polygons_->getPolygons();
  //    if (polygons)
   //   {
    //    for (const auto& polygon : *polygons)
     //   {
      //    obstacles_ptr->obstacles.emplace_back();
       //   obstacles_ptr->obstacles.back().polygon = polygon;
       // }
        // obstacles = *obstacles_ptr;


      //}

    //  std::cout<<"obstacles"<<obstacles.obstacles[0].polygon.points[0].x<<std::endl;
  




// instantiate considered_polygons and considered_centroid
  costmap_converter_msgs::msg::ObstacleArrayMsg considered_polygons;

  costmap_converter_msgs::msg::ObstacleArrayMsg considered_centroid;

  costmap_converter_msgs::msg::ObstacleArrayMsg centroid = computeCentroid(obstacles);


// will save in out parameter considered_polygons and considered_centroid only the polygons below a threshold
  polygon_filter(centroid,obstacles,considered_polygons,considered_centroid);



  A_violated_matrix_.clear();
  b_violated_vect_.clear();
  result_pose_stored_.clear();
  A_most_violated_matrix_.clear();
  b_most_violated_vect_.clear();
  A_obst_matrix_.clear();
  b_vect_.clear();
  int it=0;
  for (const auto &obstacle : considered_polygons.obstacles)
  {
    
     //iterate over each vertex of the current polygon (.size() - 2 to account for last vertex in the vector that is duplicate of the first vertex)
      
    for (int j = 0; j < (int)obstacle.polygon.points.size() - 2; ++j)

      /////NB!!//// HANDLE THE CASE WHEN ONLY 2 VERTICES ARE SENT WHEN WE HAVE A LINE NOT A POLYGON !!!!! 

   // for (int j = 0; j < (int)obstacle.polygon.points.size() - 1; ++j) 
    {

      calcLineEquation(obstacle.polygon.points[j],obstacle.polygon.points[j+1],robot_pose_,considered_centroid.obstacles[it].polygon.points[0],A_obst_matrix_,b_vect_);

    }

    // to prevent accessing empty vector (resulting in undefined behaviour) perform check if the
    // vector is not empty and if the vector size is not 2 (otherwise the for loop between 2 points will be enough)

    if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2)
    {
      // calculate the equation of the line between last point and first point of the polygon
      auto last_point = obstacle.polygon.points.end();  
      auto prev_point = std::prev(last_point, 2);  
      calcLineEquation(*prev_point,obstacle.polygon.points.front(),robot_pose_,considered_centroid.obstacles[it].polygon.points[0],A_obst_matrix_,b_vect_);

    }

    std::cout<<"Centroid x: "<<considered_centroid.obstacles[it].polygon.points[0].x<<" y: "<<considered_centroid.obstacles[it].polygon.points[0].y<<std::endl;

  it++;
// adding the most violated per each polygon

   float largest = std::numeric_limits<float>::lowest(); // Initialize with the smallest possible value

   int largest_index = 0;

      
    //for (const auto &result : result_pose_stored_)

      for (size_t row = 0; row < result_pose_stored_.size(); row++)
      {
        if(result_pose_stored_[row][0] > largest)
        {
          largest = result_pose_stored_[row][0];
          largest_index = row;
        }
      }
    
      // find the largest value 
      /*
      if (result > largest)
      {
        largest = result;
        largest_index = index; // keep track of the index
      }
      index++; */
    

    if(!A_violated_matrix_.empty()  && !b_violated_vect_.empty() )
    {

      A_most_violated_matrix_.push_back({A_violated_matrix_[largest_index][0],A_violated_matrix_[largest_index][1]});
      b_most_violated_vect_.push_back({b_violated_vect_[largest_index][0]});

    }

   // checkConstraint(robot_pose_,A_obst_matrix_,b_vect_,A_most_violated_matrix_,b_most_violated_vect_);
  

  }

   

  // test

  // m_vect_.clear();
  // b_vect_.clear();
   

  //size_t num_rows = line_eq_vect_.size();


// Initialize A_obst with the correct size
// A_obst_matrix_(line_eq_vect_.size(), std::vector<float>(2));


  // create slope (m) and intercept (b) vectors 
/*
  for (int i=0; i<line_eq_vect_.size(); i++)
  {
   // for (int j=0;j<line_eq_vect_[0].size();j++)

     m_vect_[i].resize(1); // Resize each row vector of m_vect_ to contain 1 element
     b_vect_[i].resize(1); //
    
     m_vect_[i][0] = line_eq_vect_[i][0];
     b_vect_[i][0] = line_eq_vect_[i][1]; 
    
  }*/

/*
   // Assign m_vect_ to the first column of A_obst
    for (size_t i = 0; i < m_vect_.size(); ++i) 

    {
        A_obst[i][0] = m_vect_[i][0];
        A_obst[i][1] = 1; // populate 1s in second col
    }

    std::cout<<std::endl;



   std::cout<<std::endl;*/


 std::cout<<"A_obst_matrix"<<std::endl;


      for (const auto& row : A_obst_matrix_) {
        for (const auto& val : row) {
          std::cout << val << " ";
        }
        std::cout << std::endl;
      }

      std::cout<<"b_vect"<<std::endl;


      for (const auto& row : b_vect_) {
        for (const auto& val : row) {
          std::cout << val << " ";
        }
        std::cout << std::endl;
      }

    //  std::cout<<"Pose x"<<robot_pose_.pose.position.x<<std::endl;
    //  std::cout<<"Pose y"<<robot_pose_.pose.position.y<<std::endl;

      
      std::cout<<"A_violated_matrix"<<std::endl;


      for (const auto& row : A_violated_matrix_) {
        for (const auto& val : row) {
          std::cout << val << " ";
        }
        std::cout << std::endl;
      }

      std::cout<<"b_violated_vect"<<std::endl;


      for (const auto& row : b_violated_vect_) {
        for (const auto& val : row) {
          std::cout << val << " ";
        }
        std::cout << std::endl;
      }


           
      std::cout<<"A_most_violated_matrix"<<std::endl;


      for (const auto& row : A_most_violated_matrix_) {
        for (const auto& val : row) {
          std::cout << val << " ";
        }
        std::cout << std::endl;
      }

      std::cout<<"b_most_violated_vect"<<std::endl;


      for (const auto& row : b_most_violated_vect_) {
        for (const auto& val : row) {
          std::cout << val << " ";
        }
        std::cout << std::endl;
      }















///// convex hull computation
  costmap_converter_polygons_->convexHullWrapper(point_vect_rotated_, convex_hull);



  costmap_converter_msgs::msg::ObstacleArrayMsg convex_hull_array;

  // Create an ObstacleMsg to hold the convex hull
  costmap_converter_msgs::msg::ObstacleMsg obstacle_msg;

// Add the points of the convex hull to the ObstacleMsg
  for (const auto& point : convex_hull.points) {
    obstacle_msg.polygon.points.push_back(point);
  }

  convex_hull_array.obstacles.resize(1); // Make sure the vector has at least 1 element
  convex_hull_array.obstacles[0] = obstacle_msg;



  //publishAsMarker("map",convex_hull_array);

  publishAsMarker("map",considered_polygons);


  
}


costmap_converter_msgs::msg::ObstacleArrayMsg GoalPose::computeCentroid(const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles)
{

  costmap_converter_msgs::msg::ObstacleArrayMsg centroid;

  // Clear centroid.obstacles at the beginning of the function
  centroid.obstacles.clear();


  // NB! obstacles msg accounts in .polygon.points.size() the last vertex as the first (duplicates)
  // instead of having 3 vertices it returns 4 vertices where the last one is identical to the first one
  for (const auto &obstacle:obstacles.obstacles)
  {
    double sum_x = 0;
    double sum_y = 0;

    int total_vertices = obstacle.polygon.points.size()-1;


    for (auto it = obstacle.polygon.points.begin(); it != std::prev(obstacle.polygon.points.end()); ++it)
    {
      const auto &point = *it;

      sum_x += point.x;
      sum_y += point.y;

    }
    

/*
    for (const auto &point:obstacle.polygon.points)
    {

     sum_x += point.x;
     sum_y += point.y;


    }*/


    geometry_msgs::msg::Point32 centroid_point;
    costmap_converter_msgs::msg::ObstacleMsg centroid_of_obstacle;

    centroid_point.x = sum_x/total_vertices;
    centroid_point.y = sum_y/total_vertices;

    std::cout<<"Centroid point x "<<centroid_point.x<<std::endl;


    

    centroid_of_obstacle.header = obstacle.header; 
    centroid_of_obstacle.id = obstacle.id;

    centroid_of_obstacle.polygon.points.push_back(centroid_point); // Add the centroid point to the centroid of obstacle's points       

    centroid.obstacles.push_back(centroid_of_obstacle); // Add the centroid of obstacle to the centroid obstacle array



  }




 

  return centroid;
}

void GoalPose::polygon_filter(const costmap_converter_msgs::msg::ObstacleArrayMsg &polygon_centroids, 
const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles,costmap_converter_msgs::msg::ObstacleArrayMsg &considered_polygons, costmap_converter_msgs::msg::ObstacleArrayMsg &considered_centroid)
{

  double thresh = 1.0;

  int index=0;

 // obstacles.obstacles[index].polygon.points.clear();

  // Iterate over each polygon in polygon_centroids.obstacles
  for (const auto &obstacle:polygon_centroids.obstacles)
  {




    // get euclidean distance between current polygon's centroid and robot's pose
    double distance = euclideanDistance(obstacle.polygon.points[0].x,obstacle.polygon.points[0].y,robot_pose_.pose.position.x,robot_pose_.pose.position.y);

    // if the distance is below a threshold "thresh" then consider it and store it in a vector
    if (distance < thresh)
    {
       // obstacles.obstacles[index].polygon.points.clear();

/*      geometry_msgs::msg::Point32 centroid_point;

      
      for (auto it = obstacles.obstacles[index].polygon.points.begin(); it != std::prev(obstacles.obstacles[index].polygon.points.end()); ++it)
      {    
        // store in considered_polygon the obstacle to which the computed centroid refers to
      //considered_polygons.obstacles.push_back(obstacles.obstacles[index]);

        auto &point = *it;

        centroid_point = point;

        obstacles.obstacles[index].polygon.points.push_back(centroid_point); // Add the centroid point to the centroid of obstacle's points       


      }*/



      //  considered_polygons.obstacles.push_back(obstacles.obstacles[index]); // Add the centroid of obstacle to the centroid obstacle array




            // save also as another array the centroids ( so that for example considered_polygon[1] and centroid_vect[1] will be for the same polygon)
            //centroid_vect.obstacles.push_back(obstacle.obstacles[index])

            considered_centroid.obstacles.push_back(obstacle);

            considered_polygons.obstacles.push_back(obstacles.obstacles[index]);


            //considered_polygons.obstacles[1] = centroid_vect.obstacles[1]


      }

    index++;

    }

  

  // Return the vector container with considered polygons below the threshold
  std::cout<<"Number of considered polygons:"<<considered_polygons.obstacles.size()<<std::endl;
//  std::cout<<"Number of vertiecs of first polygon:"<<considered_polygons.obstacles[0].polygon.points.size()<<std::endl;
 
if (obstacles.obstacles.size() != 0)
  {

 //   std::cout<<"Total vertices not considered: "<< obstacles.obstacles[0].polygon.points.size()<<std::endl;
}

  if (considered_polygons.obstacles.size() != 0)
  {

    std::cout<<"Total vertices: "<< considered_polygons.obstacles[0].polygon.points.size()<<std::endl;

    int index =0 ;
    for (const auto& vertex : considered_polygons.obstacles[0].polygon.points)
    {
      
    //  std::cout<<"Vertex "<< index<<std::endl;
    //  std::cout<< "x = "<<vertex.x<<std::endl;
    //  std::cout<< "y = "<<vertex.y<<std::endl;
      index++;
    }
  }
  
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

//NB! consider the footprint of the robot not the origin when checking if constraints violates ( need to add +0.35 and -0.35 on both size for the x and +0.25 and -0.25 on the y)
void GoalPose::calcLineEquation(const geometry_msgs::msg::Point32 &p1,  const geometry_msgs::msg::Point32 &p2,const geometry_msgs::msg::PoseStamped  &pose,const geometry_msgs::msg::Point32 &p3_centroid,std::vector<std::vector<float>> &A_matrix,std::vector<std::vector<float>> &b_vect)
{

  float slope,intercept;
  std::vector<float> rowVector;
  geometry_msgs::msg::Point32 point1 = p1;
  geometry_msgs::msg::Point32 point2 = p2;
    geometry_msgs::msg::Point32 centroid_point = p3_centroid;


  // do rotations from RH rule frame to Cartesian frame of each point
/*
  // if point1 is in 1st quadrant in RH frame
  if (point1.x > 0 && point1.y < 0)
  {
    // rotate it to be in 4th (90 deg rotation clockwise to align with cartesian frame)
    //point1.x = point1.x * -1;
    first = true;
  }
  // 2nd quadrant
  else if (point1.x > 0 && point1.y > 0)
  {
    //point1.y = point1.y * -1;
  }
  // 3rd quadrant
  else if (point1.x < 0 && point1.y > 0 )
  {
    //point1.x = point1.x * -1;
  }
  // 4th qudrant
  else if (point1.x < 0 && point1.y < 0)
  {
    //point1.y = point1.y * -1;
  }


  if (point2.x > 0 && point2.y < 0)
  {
    // rotate it to be in 4th (90 deg rotation clockwise to align with cartesian frame)
    //point2.x = point2.x * -1;
  }
  // 2nd quadrant
  else if (point2.x > 0 && point2.y > 0)
  {
    //point2.y = point2.y * -1;
  }
  // 3rd quadrant
  else if (point2.x < 0 && point2.y > 0 )
  {
    //point2.x = point2.x * -1;


  }
  // 4th qudrant
  else if (point2.x < 0 && point2.y < 0)
  {
    //point2.y = point2.y * -1;
   // fourth == true;
  }*/


  // calculate slope "m"

  slope = (point2.y - point1.y)/(point2.x - point1.x);
  //std::cout<<"Slope: "<<slope<<std::endl;


  // if the slope is -inf or inf we have horizontal line
  if (slope == std::numeric_limits<float>::infinity() || slope == -std::numeric_limits<float>::infinity())
  {
    slope = 1; // before slope had inverted sign
    intercept = -point1.x; //before intercept had the original non-inverted value of x
    rowVector = {slope, 0};
  }
  else if (slope == 0) // vertical line
  {
    intercept = point1.y;
    rowVector = {slope, 1};

  }
  else // general case
  {
   intercept = point1.y - slope * point1.x;
   rowVector = {-slope, 1};

  }

  A_matrix.push_back(rowVector);
  b_vect.push_back({intercept});


  /// check constraint function fusion

  int largest_column_index = 0;
  std::vector<float> rowVector2;
  float b,b2;
  //std::vector<std::vector<float>> result_vect;

  // make them private members for now later pass them by out param!!! NB!!!!!!!!!!!!!!!!///////////////////////////////////////
 // std::vector<std::vector<float>> A_violated_matrix,b_violated_vect;

  //float smallest = std::numeric_limits<float>::max();
  if (!A_matrix.empty() && !b_vect.empty())
  {

    bool horizontal_violation = false;

      // access last row of A_matrix with [A_matrix.size()-1] and b_vector
    float result_pose = (A_matrix[A_matrix.size()-1][0] * pose.pose.position.x + A_matrix[A_matrix.size()-1][1] * pose.pose.position.y) - b_vect[b_vect.size()-1][0];
    float result_centroid = (A_matrix[A_matrix.size()-1][0] * centroid_point.x + A_matrix[A_matrix.size()-1][1] * centroid_point.y) - b_vect[b_vect.size()-1][0];

    if (A_matrix[A_matrix.size()-1][1] == 0) // if we have a horizontal line
    {
      if (result_pose*result_centroid > 0) // since the horizontal lines are flipped in what they represent on the map, violation of centroid and pose is when their sign is with the same sign
      {
        horizontal_violation=true;
      }
    }
    if (result_pose * result_centroid < 0 || result_pose * result_centroid == 0  || horizontal_violation == true) // if their signs are different, then the current constraint is violated by the robot
                                                                                  // if their product is 0 it means that the centroid lies on the line (2 points line)
    {

       // rowVector2 = {A_matrix[row][0],A_matrix[row][1]};
       // b = b_vect[row][0];
       // A_violated_matrix.push_back(rowVector2); // will store slope of violated constraints
       // b_violated_vect.push_back({b}); // will store intercepts of violated constraints 

      A_violated_matrix_.push_back({A_matrix[A_matrix.size()-1][0],A_matrix[A_matrix.size()-1][1]});
      b_violated_vect_.push_back({b_vect[b_vect.size()-1][0]});
      result_pose_stored_.push_back({result_pose,0});
       // std::cout<<"Violated constraint "<<index<<" value"<<result<<std::endl;
      }

    }

      //index ++;
   //   horizontal_violation = false;

/////// CODE for calculating the most violated constraint and saving it in A and b matrix /////////////
    //int index = 0;
    /*
    int largest_index = 0;

      
    //for (const auto &result : result_pose_stored_)

      for (size_t row = 0; row < result_pose_stored_.size(); row++)
      {
        if(result_pose_stored_[row][0] > largest)
        {
          largest = result_pose_stored_[row][0];
          largest_index = row;
        }
      }
    
      // find the largest value 
      /*
      if (result > largest)
      {
        largest = result;
        largest_index = index; // keep track of the index
      }
      index++; 
    

    if(!A_violated_matrix_.empty()  && !b_violated_vect_.empty() )
    {

      A_most_violated_matrix_.push_back({A_violated_matrix_[largest_index][0],A_violated_matrix_[largest_index][1]});
      b_most_violated_vect_.push_back({b_violated_vect_[largest_index][0]});

    }

*/
 
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


void GoalPose::pose_sub_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &amcl_pose) 
{


  robot_pose_.pose.position.x = amcl_pose.pose.pose.position.x;
  robot_pose_.pose.position.y = amcl_pose.pose.pose.position.y;
}

void GoalPose::obstacles_sub_callback(const costmap_converter_msgs::msg::ObstacleArrayMsg msg )
{
  std::cout<<"Receiving"<<std::endl;
  obstacles = msg;
  std::cout<<obstacles.obstacles[0].polygon.points[0].x;


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
