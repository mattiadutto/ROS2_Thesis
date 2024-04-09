#include "nav2_custom_controller/custom_controller.hpp"

// scout_mini_mpc_sim file
using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_custom_controller
{

// Define a templated function 'min_by'
// This function finds the element in the range [begin, end) that has the minimum value
// according to a provided comparison value obtained through the 'getCompareVal' function thas it passed.
// The 'getCompareVal' function is a getter function that extracts the comparison value from an element.
template <typename Iter, typename Getter>
  Iter min_by(Iter begin, Iter end, Getter getCompareVal)
  {
  // Check if the range is empty. If so, return the 'end' iterator.
    if (begin == end)
    {
      return end;
    }

  // Initialize variables to store the lowest comparison value and its corresponding iterator.
    auto lowest = getCompareVal(*begin);
    Iter lowest_it = begin;
  // Iterate over the range starting from the second element.
    for (Iter it = ++begin; it != end; ++it)
    {
    // Obtain the comparison value for the current element.
      auto comp = getCompareVal(*it);
      if (comp < lowest)
      {
        lowest = comp;
        lowest_it = it;
      }
    }
  // Return the iterator pointing to the element with the minimum comparison value.
    return lowest_it;
  }

// Define a lambda function for calculating Euclidean distance
  auto euclideanDistance = [](double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  };


CustomController::CustomController():costmap_ros_(nullptr),costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),MPC_(std::make_unique<MPC_diffDrive_fblin>())
{ 

    A_obst_matrix_.push_back({0.0,0.0});
    b_vect_.push_back({0.0});
    A_most_violated_matrix_.push_back({0.0,0.0});
    b_most_violated_vect_.push_back({0.0});

    // define a safe zone around the robot's footprint
    robot_footprint_.resize(4);

    robot_footprint_[0].x = 0.4; 
    robot_footprint_[0].y = 0.3; 
    robot_footprint_[1].x = 0.4;
    robot_footprint_[1].y = -0.3;
    robot_footprint_[2].x = -0.4;
    robot_footprint_[2].y = 0.3;
    robot_footprint_[3].x = -0.4;
    robot_footprint_[3].y = -0.3;

    ub_.push_back(0.0);
    ub_.push_back(0.0);

    lb_.push_back(0.0);
    lb_.push_back(0.0);


   
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
    logger_ = node->get_logger(); 
    clock_ = node->get_clock();

    // create new instance of rclcpp:Node with name: costmap_converter
    intra_proc_node_.reset(new rclcpp::Node("costmap_converter", node->get_namespace(), rclcpp::NodeOptions()));

    // asign the costmap as a pointer costmap_
    costmap_ = costmap_ros_->getCostmap();

    ////////////////////////////////////////////////

    // Parameter declaration 

    // costmap_converter
    declare_parameter_if_not_declared(
      node, plugin_name_ + ".costmap_converter_plugin", rclcpp::ParameterValue("costmap_converter::CostmapToLinesDBSRANSAC"));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".costmap_converter_rate", rclcpp::ParameterValue(5));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".odom_topic", rclcpp::ParameterValue(""));

    // obstacle algorithm parameters

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".obstacle_distance_threshold", rclcpp::ParameterValue(1.0));


    // MPC parameters

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".prediction_horizon", rclcpp::ParameterValue(10));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".mpc_sampling_time", rclcpp::ParameterValue(0.2));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".Q", rclcpp::ParameterValue(2.0));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".R", rclcpp::ParameterValue(1.0));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_infeasible_solutions", rclcpp::ParameterValue(2));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".variable_upper_bound_from", rclcpp::ParameterValue(2.0));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".variable_upper_bound_to", rclcpp::ParameterValue(100.0));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".variable_lower_bound_from", rclcpp::ParameterValue(2.0));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".variable_lower_bound_to", rclcpp::ParameterValue(-100.0));


    // Feedback Linearization parameters

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".p_distance", rclcpp::ParameterValue(0.1));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".feedback_linearization_sampling_time", rclcpp::ParameterValue(0.01));

    // Robot parameters

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".max_wheel_speeds", rclcpp::ParameterValue(10.0));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".wheel_radius", rclcpp::ParameterValue(0.2));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".base_width", rclcpp::ParameterValue(0.4));


    // Set parameters from yaml file
    node->get_parameter(plugin_name_ + ".costmap_converter_plugin",costmap_converter_plugin_); 
    node->get_parameter(plugin_name_ + ".costmap_converter_rate",costmap_converter_rate_); 
    node->get_parameter(plugin_name_ + ".odom_topic",odom_topic_);
    node->get_parameter(plugin_name_ + ".obstacle_distance_threshold",obstacle_distance_thresh_);
    node->get_parameter(plugin_name_ + ".prediction_horizon",N_);
    node->get_parameter(plugin_name_ + ".mpc_sampling_time",Ts_MPC_);
    node->get_parameter(plugin_name_ + ".Q",q_);
    node->get_parameter(plugin_name_ + ".R",r_);
    node->get_parameter(plugin_name_ + ".max_infeasible_solutions",maxInfeasibleSolution_);
    node->get_parameter(plugin_name_ + ".variable_upper_bound_from",ub_[0]);
    node->get_parameter(plugin_name_ + ".variable_upper_bound_to",ub_[1]);
    node->get_parameter(plugin_name_ + ".variable_lower_bound_from",lb_[0]);
    node->get_parameter(plugin_name_ + ".variable_lower_bound_to",lb_[1]);
    node->get_parameter(plugin_name_ + ".p_distance",p_dist_);
    node->get_parameter(plugin_name_ + ".feedback_linearization_sampling_time",Ts_fblin_);
    node->get_parameter(plugin_name_ + ".max_wheel_speeds",wMax_);
    wMin_ = -wMax_;
    node->get_parameter(plugin_name_ + ".wheel_radius",R_);
    node->get_parameter(plugin_name_ + ".base_width",d_);

    ////////////////////////////////////////////////

    // costmap_converter plugin load
    try
    {
      // load the plugin
      costmap_converter_ = costmap_converter_loader_.createSharedInstance(costmap_converter_plugin_);
      // set odom topic
      costmap_converter_->setOdomTopic(odom_topic_);
      // initialize costmap_converter by passing nodehandle
      costmap_converter_->initialize(intra_proc_node_);
      // pass a pointer to the costmap
      costmap_converter_->setCostmap2D(costmap_);
      // set the rate of the plugin (it must not be much higher than costmap update rate)
      const auto rate = std::make_shared<rclcpp::Rate>((double)costmap_converter_rate_);
      // convert most recent costmap to polygons with startWroker() method
      // it also invoke compute() method of the loaded plugin that does the conversion to polygons/lines
      costmap_converter_->startWorker(rate, costmap_, "True");
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Costmap conversion plugin %s loaded.", costmap_converter_plugin_.c_str());
    }
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_INFO(rclcpp::get_logger("CustomController"),
        "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
      costmap_converter_.reset();
    }


    // Publishers and Subscribers

    obstacle_pub_ = node->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("costmap_obstacles", 100);

    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("costmap_polygons",1);

    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("polygon_marker", 10);

    marker_pub_cnvx_reg_ = node ->create_publisher<visualization_msgs::msg::Marker>("convex_region_marker",10);

    point_marker_pub_ = node ->create_publisher<visualization_msgs::msg::Marker>("point_marker",10);


    // Create a lifecycle wall timer with a callback function

    //call timer_callback() every 1 second 
    wall_timer_ = node->create_wall_timer(std::chrono::seconds(1), std::bind(&CustomController::timer_callback, this));

    pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",100,std::bind(&CustomController::pose_sub_callback, this,std::placeholders::_1));

    ////////////////////////////////////////////////

    //create grid of points 

    point_vect_.reserve(1000);
    point_vect_rotated_.reserve(1000);


    point_vect_.clear();
    point_vect_rotated_.clear();

    double minX = -1;
    double maxX = 1;
    double minY = -1;
    double maxY = 1;
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
    }

    ////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////

    // MPC PART

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


   MPC();





}

void CustomController::timer_callback()
{

  try
  { 

  // get tf from map to base link
   received_tf_ = tf_->lookupTransform("map","base_link",tf2::TimePointZero);

  } catch (tf2::LookupException &ex)
  {

    RCLCPP_WARN(rclcpp::get_logger("TF"),"Can't find base_link to map tf: %s", ex.what());
  }

  point_vect_rotated_.clear();

  // roto-translate the grid of points with base_link frame
  for (const auto& point : point_vect_)
  {

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

    point_vect_rotated_.push_back(transformed_point);


  }

  robot_footprint_rotated_.clear();

  // roto-translate robot_footprint points with base_link
  for (const auto& footprint_point : robot_footprint_)
  {

    double roll, pitch, yaw;
    tf2::Quaternion quat;
    tf2::fromMsg(received_tf_.transform.rotation, quat);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Point32 transformed_point;

     // Rotate the point around the Z-axis
    double cos_theta = cos(yaw);
    double sin_theta = sin(yaw);

    transformed_point.x = cos_theta * footprint_point.x - sin_theta * footprint_point.y;
    transformed_point.y = sin_theta * footprint_point.x + cos_theta * footprint_point.y;

    // Translate the point
    transformed_point.x += received_tf_.transform.translation.x;
    transformed_point.y += received_tf_.transform.translation.y;

    robot_footprint_rotated_.push_back(transformed_point);

  }

  /// get the obstacles container as a ptr of ObstacleArrayMsg from getObstacles() method
  costmap_converter::ObstacleArrayConstPtr obstacles_ptr = costmap_converter_->getObstacles();

  costmap_converter_msgs::msg::ObstacleArrayMsg obstacles = *obstacles_ptr;

  // get the global frame 
  std::string frame_id_ = costmap_ros_->getGlobalFrameID();

  // instantiate considered_polygons 
  costmap_converter_msgs::msg::ObstacleArrayMsg considered_polygons;

  // compute centroids of all obstacles
  costmap_converter_msgs::msg::ObstacleArrayMsg centroid = computeCentroid(obstacles);
  considered_centroid_.obstacles.clear();

  // will save in out parameter considered_polygons and considered_centroid only the polygons below a threshold
  polygon_filter(centroid,obstacles,considered_polygons,considered_centroid_);

  A_violated_matrix_.clear();
  b_violated_vect_.clear();
  result_pose_stored_.clear();
  A_most_violated_matrix_.clear();
  b_most_violated_vect_.clear();
  A_obst_matrix_.clear();
  b_vect_.clear();
  stored_centroid_point_.obstacles.clear();
  int it=0;

  for (const auto &obstacle : considered_polygons.obstacles)
  {

    //iterate over each vertex of the current polygon (.size() - 2 to account for last vertex in the vector that is duplicate of the first vertex)
    for (int j = 0; j < (int)obstacle.polygon.points.size() - 2; ++j)
    {

      calcLineEquation(obstacle.polygon.points[j],obstacle.polygon.points[j+1],robot_pose_,considered_centroid_.obstacles[it].polygon.points[0],A_obst_matrix_,b_vect_);
      compute_violated_constraints(robot_footprint_rotated_,considered_centroid_.obstacles[it].polygon.points[0],A_obst_matrix_,b_vect_);
    }

      // to prevent accessing empty vector (resulting in undefined behaviour) perform check if the
      // vector is not empty and if the vector size is not 2 (otherwise the for loop between 2 points will be enough)
    if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2)
    {

      // calculate the equation of the line between last point and first point of the polygon
      auto last_point = obstacle.polygon.points.end();  
      auto prev_point = std::prev(last_point, 2);  
      calcLineEquation(*prev_point,obstacle.polygon.points.front(),robot_pose_,considered_centroid_.obstacles[it].polygon.points[0],A_obst_matrix_,b_vect_);
      compute_violated_constraints(robot_footprint_rotated_,considered_centroid_.obstacles[it].polygon.points[0],A_obst_matrix_,b_vect_);
    }

    it++;
    // computing the most violated constraint per each polygon
    compute_most_violated_constraints();
  } 

  // consider only the points of the grid that are inside the region defined by the most violated constraints
  point_vect_constrained_.clear();

  for (const auto& point : point_vect_rotated_)
  {

    if (isViolated(point,A_most_violated_matrix_,b_most_violated_vect_) == false)
    {
      point_vect_constrained_.push_back(point);
    }

  }

  std::vector<costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint> convexHull;

  // compute the convex hull
  Polygon polygon(point_vect_constrained_);
  convexHull = polygon.ComputeConvexHull();

  costmap_converter_msgs::msg::ObstacleArrayMsg convex_hull_array;

  // Create an ObstacleMsg to hold the convex hull
  costmap_converter_msgs::msg::ObstacleMsg obstacle_msg;

  // Add the points of the convex hull to the ObstacleMsg
  for (const auto& point : convexHull)
  {

    geometry_msgs::msg::Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
    obstacle_msg.polygon.points.push_back(point32);
  }

  convex_hull_array.obstacles.resize(1); // Make sure the vector has at least 1 element
  convex_hull_array.obstacles[0] = obstacle_msg;

  // printing
/*

  std::cout<<"A_matrix"<<std::endl;


  for (const auto& row : A_obst_matrix_) {
    for (const auto& val : row) {
      std::cout << val << " ";
    }
    std::cout << std::endl;
  }

  std::cout<<"b_vector"<<std::endl;


  for (const auto& row : b_vect_) {
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
  
  */

  // function that creates polygons/lines and publishes them as Marker msg for visualisation

  publishAsMarker(frame_id_, obstacles,false);

  //publishAsMarker(frame_id_,considered_polygons,false);

  publishAsMarker(frame_id_,convex_hull_array,true);

  ///////////////////////////////////////////////////////////

  /// display the grid of points as Marker msg////

  visualization_msgs::msg::Marker point_marker; // creater line_list as Marker msg
  point_marker.header.frame_id = frame_id_;
  point_marker.header.stamp = rclcpp::Clock().now();
  point_marker.ns = "Points"; // namespace of the container
  point_marker.action = visualization_msgs::msg::Marker::ADD; // add marker
  point_marker.pose.orientation.w = 1.0; 
  point_marker.id = 0;
  point_marker.type = visualization_msgs::msg::Marker::POINTS; //line list type 
  point_marker.scale.x = 0.01;
  point_marker.scale.y = 0.01;
  point_marker.color.b = 1.0;
  point_marker.color.a = 1.0;

  for (const auto &point : point_vect_rotated_)
  {

    geometry_msgs::msg::Point add_point;
    add_point.x = point.x; 
    add_point.y = point.y;
    point_marker.points.push_back(add_point);

  }

   point_marker_pub_->publish(point_marker);

//////////////////////////////////////////////////

}


// will determine if a point is inside set of constraints
bool CustomController::isViolated(const costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint &point,const std::vector<std::vector<float>> &A_matrix,const std::vector<std::vector<float>> &b_vector)
{

  // perform a loop through A_matrix and check for each line constraint if the current point satisfies it
  // if all constraints are satisfied or violated return the bool

  std::vector<std::vector<float>> b_vect = b_vector;

  int num_constraints = A_matrix.size();

  int count = 0;


  for (size_t row = 0; row < A_matrix.size();row++)
  {
      bool horizontal = false;

   // std::cout<<"row: "<<row<<std::endl;
   //           std::cout<<"centroid x"<<final_stored_centroid_point_[row].x<<std::endl;

    if(A_matrix[row][0] == 1) // the equation for the horizontal line is flipped (equation describing line below robot is actually above)
                              // in order to get the right representation the intercept's sign should be flipped
    {
      b_vect[row][0] = b_vect[row][0]*-1; // positive intercept is top half-plane, negative is bottom half-plane
      horizontal = true;
    }

    float result = (A_matrix[row][0] * point.x + A_matrix[row][1] * point.y) - b_vect[row][0];
    float result_origin = (A_matrix[row][0] * robot_pose_.pose.position.x + A_matrix[row][1] * robot_pose_.pose.position.y) - b_vect[row][0];

    if (result * result_origin < 0)
    {
      // not in the same plane
    }
    else if (result * result_origin > 0)
    {
      count++;
    }


  }

  if (count == num_constraints) // all constraints are satisfied, therefore point satisfy 
  {
    return false; // return that point does not violate the constraints
  }
  else
  {
    return true; // if at least one constraint is violated, the point does not satisfy
  }

  
  

}


costmap_converter_msgs::msg::ObstacleArrayMsg CustomController::computeCentroid(const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles)
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
    
    geometry_msgs::msg::Point32 centroid_point;
    costmap_converter_msgs::msg::ObstacleMsg centroid_of_obstacle;

    centroid_point.x = sum_x/total_vertices;
    centroid_point.y = sum_y/total_vertices;

    centroid_of_obstacle.header = obstacle.header; 
    centroid_of_obstacle.id = obstacle.id;

    centroid_of_obstacle.polygon.points.push_back(centroid_point); // Add the centroid point to the centroid of obstacle's points       

    centroid.obstacles.push_back(centroid_of_obstacle); // Add the centroid of obstacle to the centroid obstacle array

  }

  centroid_path_msg_.poses.clear();


// convert ObstacleArrayMsg to Path msg of the centroid

  for (const auto &obstacle : centroid.obstacles)
  {



    // Set the header for the centroid pose stamped message
    centroid_pose_stamped_.header = obstacle.header;

    // Set the position of the centroid as the first point in its polygon
    centroid_pose_stamped_.pose.position.x = obstacle.polygon.points[0].x;
    centroid_pose_stamped_.pose.position.y = obstacle.polygon.points[0].y;
    
    // Push the centroid pose stamped message to centroid_path.poses
    centroid_path_msg_.poses.push_back(centroid_pose_stamped_);
  }

  // Find the closest centroid to the robot's pose

  auto closest_centroid_it =
  min_by(
    centroid_path_msg_.poses.begin(), centroid_path_msg_.poses.end(),
    [this](const geometry_msgs::msg::PoseStamped &ps)
    {
      return euclidean_distance(robot_pose_, ps);
    });

  // Check if centroid path has any poses before accessing to prevent segmentation fault
  if (!centroid_path_msg_.poses.empty() && closest_centroid_it != centroid_path_msg_.poses.end())
  {
   // std::cout << "Closest centroid x coordinate: " << closest_centroid_it->pose.position.x << std::endl;
   // std::cout << "Closest centroid y coordinate: " << closest_centroid_it->pose.position.y << std::endl;
   // double distance = euclideanDistance(closest_centroid_it->pose.position.x,closest_centroid_it->pose.position.y,robot_pose_.pose.position.x,robot_pose_.pose.position.y);
   // std::cout<<"Closest centroid distance:"<<distance<<std::endl;

  }

  return centroid;
}

//// NB combine polygon_filter and computeCentroid to be filterPolygons function !!!!
void CustomController::polygon_filter(const costmap_converter_msgs::msg::ObstacleArrayMsg &polygon_centroids, 
  const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles,costmap_converter_msgs::msg::ObstacleArrayMsg &considered_polygons, costmap_converter_msgs::msg::ObstacleArrayMsg &considered_centroid)
{

  int index=0;

  // Iterate over each polygon in polygon_centroids.obstacles
  for (const auto &obstacle:polygon_centroids.obstacles)
  {

    // get euclidean distance between current polygon's centroid and robot's pose
    double distance = euclideanDistance(obstacle.polygon.points[0].x,obstacle.polygon.points[0].y,robot_pose_.pose.position.x,robot_pose_.pose.position.y);
    
    // if the distance is below a threshold "thresh" then consider it and store it in a vector
    if (distance < obstacle_distance_thresh_)
    {

      considered_centroid.obstacles.push_back(obstacle);
      considered_polygons.obstacles.push_back(obstacles.obstacles[index]);

    }

    index++;

  }

  // Return the vector container with considered polygons below the threshold
  std::cout<<"Number of considered polygons:"<<considered_polygons.obstacles.size()<<std::endl;
  
}

void CustomController::calcLineEquation(const geometry_msgs::msg::Point32 &p1,  const geometry_msgs::msg::Point32 &p2,const geometry_msgs::msg::PoseStamped  &pose,const geometry_msgs::msg::Point32 &p3_centroid,std::vector<std::vector<float>> &A_matrix,std::vector<std::vector<float>> &b_vect)
{

  float slope,intercept;
  std::vector<float> rowVector;
  geometry_msgs::msg::Point32 point1 = p1;
  geometry_msgs::msg::Point32 point2 = p2;
  geometry_msgs::msg::Point32 centroid_point = p3_centroid;

  // calculate slope "m"

  slope = (point2.y - point1.y)/(point2.x - point1.x);


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
    
}

// decide if should I pass out parameters or save the results to private members
void CustomController::compute_violated_constraints(const std::vector<geometry_msgs::msg::Point32> &robot_footprint_,const geometry_msgs::msg::Point32 &p_centroid,const std::vector<std::vector<float>> &A_matrix,const std::vector<std::vector<float>> &b_vect)
{

  // when I have horizontal line with 2 vertices only it happens that the calculated result_centroid is zero!

  int count = 0;
  geometry_msgs::msg::Point32 centroid_point = p_centroid;
  static geometry_msgs::msg::Point32 old_centroid_point;
  old_centroid_point.x = 0;
  old_centroid_point.y = 0;

  std::vector<geometry_msgs::msg::Point32> robot_footprint = robot_footprint_;

  std::vector<float> result_footprint_points_stored;
  result_footprint_points_stored.clear();


  if (!A_matrix.empty() && !b_vect.empty())
  {

   // std::cout<<"current b vect"<<b_vect[b_vect.size()-1][0]<<std::endl;

    for (auto &point : robot_footprint)
    {

    //  std::cout<<"point 1 x: "<<point.x<<"point 1 y: "<<point.y<<std::endl;
    //  std::cout<<"A matrix [] [] "<<A_matrix[A_matrix.size()-1][0]<<" "<<A_matrix[A_matrix.size()-1][1]<<std::endl;
    //  std::cout<<"b vect "<<b_vect[b_vect.size()-1][0]<<std::endl;



      result_pose = (A_matrix[A_matrix.size()-1][0] * point.x + A_matrix[A_matrix.size()-1][1] * point.y) - b_vect[b_vect.size()-1][0];
      result_centroid = (A_matrix[A_matrix.size()-1][0] * centroid_point.x + A_matrix[A_matrix.size()-1][1] * centroid_point.y) - b_vect[b_vect.size()-1][0];
      
    //  std::cout<<"result pose: "<<result_pose<<std::endl;
    //  std::cout<<"result centroid: "<<result_centroid<<std::endl;  
      if (A_matrix[A_matrix.size()-1][1] == 0) // if we have a horizontal line
      {
        result_centroid = (A_matrix[A_matrix.size()-1][0] * centroid_point.x + A_matrix[A_matrix.size()-1][1] * centroid_point.y) - (-1 *b_vect[b_vect.size()-1][0]);
        result_pose = (A_matrix[A_matrix.size()-1][0] * point.x + A_matrix[A_matrix.size()-1][1] * point.y) - b_vect[b_vect.size()-1][0];

     //   std::cout<<"result pose: "<<result_pose<<std::endl;
     //   std::cout<<"result centroid: "<<result_centroid<<std::endl;

        if(result_pose*result_centroid > 0 || result_pose * result_centroid == 0 || result_pose * result_centroid < 0.001)
        {

          result_footprint_points_stored.push_back({result_pose});


        }

        else if (result_pose*result_centroid < 0) // since the horizontal lines are flipped in what they represent on the map, violation of centroid and pose is when their sign is with the same sign
        {
          result_footprint_points_stored.push_back({0});


        }

      }

      // if their signs are different, then the current constraint is violated by the robot
     // if their product is 0 it means that the centroid lies on the line (2 points line)

      else
      {
        if (result_pose * result_centroid < 0 || result_pose * result_centroid == 0 || result_pose * result_centroid < 0.001)
        {
          result_footprint_points_stored.push_back({result_pose});

        }
        else if (result_pose * result_centroid > 0)
        {
          result_footprint_points_stored.push_back({0});
        }
      }

    }

    if (!result_footprint_points_stored.empty())
    {

  //  std::cout<<"result 1: "<<result_footprint_points_stored[0]<<" result 2: "<<result_footprint_points_stored[1]<<" result 3: "<<result_footprint_points_stored[2]<<" result 4: "<<result_footprint_points_stored[3]<<std::endl;;

    // Check if all values have the same sign
      bool all_positive = true;
      bool all_negative = true;

      for (float value : result_footprint_points_stored)
      {
        if (value > 0)
        {
          all_negative = false;
        }
        else if (value < 0)
        {
          all_positive = false;
        }
        else if ( value == 0)
        {
          all_negative = false;
          all_positive = false;
        }
      }

    // If all values are either positive or negative, the product will be positive
      if (all_positive || all_negative)
      {
   //     std::cout << "All values have the same sign" << std::endl;
        count = 4;
      }
    }

    if (count == 4) // the robot's footprint violates the current constraint
    { 

      A_violated_matrix_.push_back({A_matrix[A_matrix.size()-1][0],A_matrix[A_matrix.size()-1][1]});
      b_violated_vect_.push_back({b_vect[b_vect.size()-1][0]});
      result_pose_stored_.push_back({result_pose,0});
      costmap_converter_msgs::msg::ObstacleMsg centroid_of_obstacle;

      if(old_centroid_point != centroid_point)
      {
        centroid_of_obstacle.polygon.points.push_back(centroid_point); 
        stored_centroid_point_.obstacles.push_back(centroid_of_obstacle);
      }
    }
    old_centroid_point = centroid_point;
  }
}

void CustomController::compute_most_violated_constraints()
{

  float largest = std::numeric_limits<float>::lowest(); // Initialize with the smallest possible value

  int largest_index = 0;

  for (size_t row = 0; row < result_pose_stored_.size(); row++)
  {

    // was > before
    if(result_pose_stored_[row][0] < largest)
    {
      largest = result_pose_stored_[row][0];
      largest_index = row;
    }

  }


  if (!A_violated_matrix_.empty()  && !b_violated_vect_.empty() )
  {

    A_most_violated_matrix_.push_back({A_violated_matrix_[largest_index][0],A_violated_matrix_[largest_index][1]});
    b_most_violated_vect_.push_back({b_violated_vect_[largest_index][0]});
    A_violated_matrix_.clear();
    b_violated_vect_.clear();
    result_pose_stored_.clear();
  }

}


void CustomController::publishAsMarker(const std::string &frame_id,const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles, bool print_convex_region)
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

  if (print_convex_region == false)
  {

    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
    line_list.color.b = 1.0;

  }else

  {
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;
  }

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

    // publish on 2 different topic depending on the passed parameter
     if (print_convex_region == false)
     {

      marker_pub_->publish(line_list);

    }else
    {
      marker_pub_cnvx_reg_ ->publish(line_list);
    }

      // else 
      // marker_pub_cnvx_reg_ -> publish(line_list)
}



void CustomController::pose_sub_callback(const geometry_msgs::msg::PoseWithCovarianceStamped &amcl_pose) 
{


  robot_pose_.pose.position.x = amcl_pose.pose.pose.position.x;
  robot_pose_.pose.position.y = amcl_pose.pose.pose.position.y;
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

/*

  static auto start_time_MPC = std::chrono::high_resolution_clock::now();

    // Convert to system_clock time point (for human-readable output)
  //  auto start_time_s = std::chrono::system_clock::to_time_t(start_time_MPC);
    
    // Print the end time in a human-readable format
    //std::cout << "Start time: " << std::ctime(&start_time_s) << std::endl;



  static auto start_time_lin = std::chrono::high_resolution_clock::now();

  double v_act,w_act;


    // Convert to system_clock time point (for human-readable output)
   // auto start_time_s2 = std::chrono::system_clock::to_time_t(start_time_lin);
    
    // Print the end time in a human-readable format
   // std::cout << "Start time lin: " << std::ctime(&start_time_s2) << std::endl;





  // Convert pose.pose.orientation from Quaternion to Roll,Pitch,Yaw
  double roll, pitch, yaw;
  tf2::Quaternion quat;
  tf2::fromMsg(pose.pose.orientation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  //std::cout<< "yaw =  "<<yaw<<std::endl;


  MPC_->set_actualRobotState(Eigen::Vector3d(pose.pose.position.x,pose.pose.position.y, yaw));

 // Find the closest pose on the path to the robot
  auto transformation_begin =
      min_by(
          global_plan_.poses.begin(), global_plan_.poses.end(),
          //&pose is current robot pose, ps is the other pose to go in euclidean_distance
          [&pose](const geometry_msgs::msg::PoseStamped &ps)
          {
            return euclidean_distance(pose, ps);
          });

  // From the closest point, look for the first point that's 0.4m away
  auto transformation_end = std::find_if(
      transformation_begin, end(global_plan_.poses),
      [&](const auto &global_plan_pose)
      {
        return euclidean_distance(pose, global_plan_pose) > 0.4; 
      });

  // assign each consecutive goal pose 0.4m away from the previous
  auto target_pose_ = *transformation_end;

// Convert pose.pose.orientation from Quaternion to Roll,Pitch,Yaw
  double roll2, pitch2, yaw2;
  tf2::Quaternion quat2;
  tf2::fromMsg(target_pose_.pose.orientation, quat2);
  tf2::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);

//  std::cout<< "yaw2 =  "<<yaw2<<std::endl;



//MPC_->set_referenceRobotState(Eigen::Vector3d(target_pose_.pose.position.x, target_pose_.pose.position.y,45));

MPC_->set_referenceRobotState(Eigen::Vector3d(1, 0.0,0));



if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time_MPC).count() >= 200) // execute the MPC every 0.2 seconds
 {

    // Get the current time point
    //auto end_time = std::chrono::high_resolution_clock::now();
    
    // Convert to system_clock time point (for human-readable output)
   // auto end_time_s = std::chrono::system_clock::to_time_t(end_time);
    
    // Print the end time in a human-readable format
  //  std::cout << "End time: " << std::ctime(&end_time_s) << std::endl;

  
 // std::cout<<"MPC executed after 3 seconds"<<std::endl;

  double vPx_act, vPy_act;

  // compute MPC control and optimisation to obtain optimal control inputs 
  // xp dot and yp dot to be used by the feedback linearisation to get v and w

  MPC_->executeMPCcontroller();

  Eigen::VectorXd MPC_actControl;

  // get xp dot and yp dot that are computed by the MPC
  MPC_->get_actualMPCControl(MPC_actControl);

  vPx_act = MPC_actControl(0);
  vPy_act = MPC_actControl(1);

  start_time_MPC = std::chrono::high_resolution_clock::now();
 }

if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time_lin).count() >= 10) // execute the FBLIN every 0.01 seconds
{

 // std::cout<<"Linearization executed after 0.01 seconds"<<std::endl;


    // Get the current time point
   // auto end_time = std::chrono::high_resolution_clock::now();
    
    // Convert to system_clock time point (for human-readable output)
   // auto end_time_s = std::chrono::system_clock::to_time_t(end_time);
    
    // Print the end time in a human-readable format
  //  std::cout << "End time linearization: " << std::ctime(&end_time_s) << std::endl;

  MPC_->executeLinearizationController();

  start_time_lin = std::chrono::high_resolution_clock::now();


}

// Get actual control signal
// store computed v and w from fblin class to v_act, w_act variables
 MPC_->get_actualControl(v_act, w_act);





  // Apply feedback linearization
  cmd_vel_.twist.linear.x = v_act;
  cmd_vel_.twist.angular.z = w_act;

  cmd_vel_.header.frame_id = pose.header.frame_id;
  cmd_vel_.header.stamp = clock_->now();



 std::cout<<"cmd_vel lin speed"<<cmd_vel_.twist.linear.x<<std::endl;
std::cout<<"cmd vel ang speed"<<cmd_vel_.twist.angular.z<<std::endl;
  



*/



//////// Feedback linearization only









  double epsilon_ = 0.15;

  // Convert pose.pose.orientation from Quaternion to Roll,Pitch,Yaw
  double roll, pitch, yaw;
  tf2::Quaternion quat;
  tf2::fromMsg(pose.pose.orientation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // Record the start time
  auto start_time = std::chrono::high_resolution_clock::now();

  // centroid.obstacles.begin(),centroid.obstacles.end()


  // Find the closest pose on the path to the robot
  auto transformation_begin =
      min_by(
          global_plan_.poses.begin(), global_plan_.poses.end(),
          //&pose is current robot pose, ps is the other pose to go in euclidean_distance
          [&pose](const geometry_msgs::msg::PoseStamped &ps)
          {
            return euclidean_distance(pose, ps);
          });

  // From the closest point, look for the first point that's 0.4m away
  auto transformation_end = std::find_if(
      transformation_begin, end(global_plan_.poses),
      [&](const auto &global_plan_pose)
      {
        return euclidean_distance(pose, global_plan_pose) > 0.4; 
      });

  // assign each consecutive goal pose 0.4m away from the previous
  auto target_pose_ = *transformation_end;

  // Record the end time
  auto end_time = std::chrono::high_resolution_clock::now();

  // Calculate the duration
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

  // Print the execution time
 // std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;

  // calculate point P at a distance epsilon from the robot
  feedback_lin_.calcPointP(pose, yaw, epsilon_);

  // Apply proportional control for trajectory tracking without feed forward term
  double xp_dot_ = (target_pose_.pose.position.x - feedback_lin_.getPointP()[0]) * 2.5;
  double yp_dot_ = (target_pose_.pose.position.y - feedback_lin_.getPointP()[1]) * 1.5;
 
  // Apply feedback linearization
  cmd_vel_ = feedback_lin_.linearize(xp_dot_, yp_dot_);

  cmd_vel_.header.frame_id = pose.header.frame_id;
  cmd_vel_.header.stamp = clock_->now();

 // std::cout<<"cmd_vel lin speed"<<cmd_vel_.twist.linear.x<<std::endl;
 // std::cout<<"cmd vel ang speed"<<cmd_vel_.twist.angular.z<<std::endl;



  return cmd_vel_;
}


void CustomController::setPlan(const nav_msgs::msg::Path &path )
{

  global_plan_ = path;






}

void CustomController::MPC()
{

    double stop = 10.0; // Stop time in seconds
    double step = 0.001;  // Time step in seconds

    // Do simulation
    for (double time = 0.0; time < stop; time += step) {
        // Perform simulation operations here
        // This loop will execute until the stop time is reached
        
        // For demonstration, let's print the current time
        std::cout << "Current time: " << time << " seconds" << std::endl;
    }

}

void CustomController::setSpeedLimit(const double & , const bool & )
{

}

} // namespace

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)