#include "nav2_custom_controller/custom_controller.hpp"

/// scout_mini_mpc_sim file
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
    A_convex_region_matrix_.push_back({0.0,0.0});
    b_convex_region_vect_.push_back({0.0});
    A_most_violated_matrix_.push_back({0.0,0.0});
    A_most_violated_matrix_considered_.push_back({0.0,0.0});
    b_most_violated_vect_.push_back({0.0});
    //b_most_violated_vect_considered_.push_back({0.0});
    //mpc_obstacle_constraints_.matrix_rows.push_back({0.0,0.0});
   // mpc_obstacle_constraints_.vector_rows.push_back({0.0,0.0});

    // define a safe zone around the robot's footprint 
    costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint robot_footprint_point;


    robot_footprint_point.x = 0.306; 
    robot_footprint_point.y = 0.290;
    robot_footprint_.push_back(robot_footprint_point); 

    robot_footprint_point.x = 0.306;
    robot_footprint_point.y = 0;
    robot_footprint_.push_back(robot_footprint_point); 

    robot_footprint_point.x = 0.306;
    robot_footprint_point.y = -0.290;
    robot_footprint_.push_back(robot_footprint_point); 

    robot_footprint_point.x = -0.306;
    robot_footprint_point.y = 0.290;
    robot_footprint_.push_back(robot_footprint_point); 

    robot_footprint_point.x = -0.306;
    robot_footprint_point.y = 0;
    robot_footprint_.push_back(robot_footprint_point); 

    robot_footprint_point.x = -0.306;
    robot_footprint_point.y = -0.290;
    robot_footprint_.push_back(robot_footprint_point); 

    robot_footprint_point.x = 0;
    robot_footprint_point.y = 0.290;
    robot_footprint_.push_back(robot_footprint_point); 

    robot_footprint_point.x = 0;
    robot_footprint_point.y = -0.290;
    robot_footprint_.push_back(robot_footprint_point); 

    disable_nav2_path_ = false;

 
    //// SWITCH -  switch variables based on intended use
    path_loaded_ = true; // if false, then path from nav2 will not be considered, but instead a path from csv file
    path_saved_ = false; // if false, then only the first passed path from nav2 will be saved to csv file 
    print_ = false; // if true, useful info will be printed




  //  ub_.push_back(0.0);
   // ub_.push_back(0.0);

   // lb_.push_back(0.0);
   // lb_.push_back(0.0);


  }



  void CustomController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer>  tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>  costmap_ros)
  {


    node_ = parent;

    auto node = node_.lock();

    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "CustomController plugin loaded!");


    logger_ = node->get_logger(); 
    clock_ = node->get_clock();

    ////////////////////////////////////////////////

    // RCLCPP Timers

    // #TODO make obstacle_algorithm to run faster (its 1 second now because its easier to read the log messages when debugging)
    obstacle_algorithm_timer = node->create_wall_timer(std::chrono::milliseconds(200), std::bind(&CustomController::obstacle_algorithm, this));
   // mpc_timer_ = node->create_wall_timer(std::chrono::milliseconds(200),std::bind(&CustomController::execute_mpc, this));
   // fblin_timer_ = node->create_wall_timer(std::chrono::milliseconds(10),std::bind(&CustomController::execute_fblin, this));

    // Publishers
    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("costmap_polygons",1);
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("polygon_marker", 10);
    marker_pub_cnvx_reg_ = node ->create_publisher<visualization_msgs::msg::Marker>("convex_region_marker",10);
    point_marker_pub_ = node ->create_publisher<visualization_msgs::msg::Marker>("point_marker",10);
    predicted_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("predicted_path", 10);

    // Subscribers
    pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/scout_mini/amcl_pose",100,std::bind(&CustomController::pose_sub_callback, this,std::placeholders::_1));



    ////////////////////////////////////////////////
    // Parameter declaration 

    // costmap_converter
    declare_parameter_if_not_declared(node, plugin_name_ + ".costmap_converter_plugin", rclcpp::ParameterValue("costmap_converter::CostmapToLinesDBSRANSAC"));

    declare_parameter_if_not_declared(node, plugin_name_ + ".costmap_converter_rate", rclcpp::ParameterValue(5));

    declare_parameter_if_not_declared(node, plugin_name_ + ".odom_topic", rclcpp::ParameterValue(""));

    // obstacle algorithm parameters

    declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_distance_threshold", rclcpp::ParameterValue(0.2));

    declare_parameter_if_not_declared(node, plugin_name_ + ".inflation_radius", rclcpp::ParameterValue(0.1));


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
      node, plugin_name_ + ".variable_upper_bound", rclcpp::ParameterValue(100));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".variable_lower_bound", rclcpp::ParameterValue(-100));


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
    node->get_parameter(plugin_name_ + ".inflation_radius",inflation_radius_);
    
    node->get_parameter(plugin_name_ + ".prediction_horizon",N_);
    node->get_parameter(plugin_name_ + ".mpc_sampling_time",Ts_MPC_);
    node->get_parameter(plugin_name_ + ".Q",q_);
    node->get_parameter(plugin_name_ + ".R",r_);
    node->get_parameter(plugin_name_ + ".max_infeasible_solutions",maxInfeasibleSolution_);
    int upper_bound,lower_bound;
    node->get_parameter(plugin_name_ + ".variable_upper_bound",upper_bound);
    node->get_parameter(plugin_name_ + ".variable_lower_bound",lower_bound);
    std::vector<double> lb_(2, lower_bound);
    std::vector<double> ub_(2, upper_bound);
    node->get_parameter(plugin_name_ + ".p_distance",p_dist_);
    node->get_parameter(plugin_name_ + ".feedback_linearization_sampling_time",Ts_fblin_);
    node->get_parameter(plugin_name_ + ".max_wheel_speeds",wMax_);
    wMin_ = -wMax_;
    node->get_parameter(plugin_name_ + ".wheel_radius",R_);
    node->get_parameter(plugin_name_ + ".base_width",d_);
  

    ////////////////////////////////////////////////
    // costmap_converter plugin load

    // create new instance of rclcpp:Node with name: costmap_converter
    intra_proc_node_.reset(new rclcpp::Node("costmap_converter", node->get_namespace(), rclcpp::NodeOptions()));


    // asign the costmap as a pointer costmap_
    costmap_ = costmap_ros_->getCostmap();

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
      costmap_converter_->startWorker(rate,costmap_, "True");
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Costmap conversion plugin %s loaded.", costmap_converter_plugin_.c_str());
    }
    catch(pluginlib::PluginlibException& ex)
    {
      RCLCPP_INFO(rclcpp::get_logger("CustomController"),
        "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
      costmap_converter_.reset();
    }


     ////////////////////////////////////////////////

    //create grid of points 
    // #TODO note: if the grid size is changed -> the constraints of the grid should be changed!!!

    point_vect_.reserve(1000);
    point_vect_rotated_.reserve(1000);


    point_vect_.clear();
    point_vect_rotated_.clear();

    double minX = -obstacle_distance_thresh_;
    double maxX = obstacle_distance_thresh_;
    double minY = -obstacle_distance_thresh_;
    double maxY = obstacle_distance_thresh_;
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




    index=1;

    ////////////////////////////////////////////////////////
/*
      // MPC parameters
    int N_ = 2;
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
\*/
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







  void CustomController::obstacle_algorithm()
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

    costmap_converter::CostmapToPolygonsDBSMCCH::KeyPoint transformed_point;

     // Rotate the point around the Z-axis
    double cos_theta = cos(yaw);
    double sin_theta = sin(yaw);

    transformed_point.x = cos_theta * footprint_point.x - sin_theta * footprint_point.y;
    transformed_point.y = sin_theta * footprint_point.x + cos_theta * footprint_point.y;

    // Translate the point
    transformed_point.x += received_tf_.transform.translation.x;
    transformed_point.y += received_tf_.transform.translation.y;

    robot_footprint_rotated_.push_back(transformed_point);

    //note: if the grid size is changed -> the constraints of the grid should be changed!!!

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
  A_most_violated_matrix_considered_.clear();
  b_most_violated_vect_.clear();
  b_most_violated_vect_considered_.clear();
  mpc_obstacle_constraints_.vector_rows.clear();
  mpc_obstacle_constraints_.matrix_rows.clear();
  mpc_obstacle_constraints_matrix_.resize(0, 0);
  mpc_obstacle_constraints_vector_.resize(0, 0);
  A_obst_matrix_.clear();
  b_vect_.clear();
  b_vect_inflated_.clear(); //// NEW !!!
  int it=0;

  for (const auto &obstacle : considered_polygons.obstacles)
  {

    //iterate over each vertex of the current polygon (.size() - 2 to account for last vertex in the vector that is duplicate of the first vertex)
    for (int j = 0; j < (int)obstacle.polygon.points.size() - 2; ++j)
    {

      calcLineEquation(obstacle.polygon.points[j],obstacle.polygon.points[j+1],A_obst_matrix_,b_vect_);
      compute_violated_constraints(considered_centroid_.obstacles[it].polygon.points[0],A_obst_matrix_,b_vect_);
    }

      // to prevent accessing empty vector (resulting in undefined behaviour) perform check if the
      // vector is not empty and if the vector size is not 2 (otherwise the for loop between 2 points will be enough)
    if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2)
    {

      // calculate the equation of the line between last point and first point of the polygon
      auto last_point = obstacle.polygon.points.end();  
      auto prev_point = std::prev(last_point, 2);  
      calcLineEquation(*prev_point,obstacle.polygon.points.front(),A_obst_matrix_,b_vect_);
      compute_violated_constraints(considered_centroid_.obstacles[it].polygon.points[0],A_obst_matrix_,b_vect_);
    }

    it++;

    // computing the most violated constraint per each polygon
    compute_most_violated_constraints();

  } 

  // store in Eigen matrix and vector the most violated constraints

  
  if (mpc_obstacle_constraints_.matrix_rows.size() != 0)
  {
    // Extract number of rows
    size_t num_rows = mpc_obstacle_constraints_.matrix_rows.size();
    mpc_obstacle_constraints_matrix_.resize(num_rows, 2);
    mpc_obstacle_constraints_vector_.resize(num_rows);

    for (size_t i = 0; i < num_rows; ++i)
    {
      // Copy data from matrix rows
      mpc_obstacle_constraints_matrix_(i, 0) = mpc_obstacle_constraints_.matrix_rows[i].col1;
      mpc_obstacle_constraints_matrix_(i, 1) = mpc_obstacle_constraints_.matrix_rows[i].col2;
    
      // Copy data from vector rows
      mpc_obstacle_constraints_vector_(i) = mpc_obstacle_constraints_.vector_rows[i].col1;
  }
  }

  // consider only the points of the grid that are inside the region defined by the most violated constraints
  point_vect_constrained_.clear();

  for (const auto& point : point_vect_rotated_)
  {
    geometry_msgs::msg::Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
    int count = 0;
    for (int row = 0; row < A_most_violated_matrix_.size(); row++)
    {
      // loop through all constraints and perform a check for the current point
      if (isViolated(point32,A_most_violated_matrix_[row],b_most_violated_vect_[row][0]) == false)
      {
        // if point and center of robot are in the same plane, increment
        count++;
      }
    }
    if (count == A_most_violated_matrix_.size())
    {
      // current point must be included in point_vect_contrained_
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
  int num = 0;
  // Add the points of the convex hull to the ObstacleMsg
  for (const auto& point : convexHull)
  {

    geometry_msgs::msg::Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
   // std::cout<<"current vertex: "<<num<<"x: "<<point.x<<"y: "<<point.y<<std::endl;
    obstacle_msg.polygon.points.push_back(point32);
    num++;
  }

  convex_hull_array.obstacles.resize(1); // Make sure the vector has at least 1 element
  convex_hull_array.obstacles[0] = obstacle_msg;

  if (print_ == true)
  {


    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "A_matrix");
    int row_number = 1;  
    for (const auto& row : A_obst_matrix_)
    {
      std::stringstream ss;
      ss << row_number << ": ";  
      for (const auto& val : row)
      {
        ss << val << " ";
      }
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "%s", ss.str().c_str());
      ++row_number;  // Increment the row number for the next row
    }

    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "b_vector");
     row_number = 1;  
    for (const auto& row : b_vect_)
    {
      std::stringstream ss;
      ss << row_number << ": ";  
      for (const auto& val : row) 
      {
        ss << val << " ";
      }
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "%s", ss.str().c_str());
      ++row_number;
    }

    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "A_violated_matrix_");
     row_number = 1;  

    for (const auto& row : A_violated_matrix_)
    {
      std::stringstream ss;
      ss << row_number << ": "; 
      for (const auto& val : row)
      {
        ss << val << " ";
      }
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "%s", ss.str().c_str());
      ++row_number;
    }

    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "b_violated_vect_");
     row_number = 1;  

    for (const auto& row : b_violated_vect_)
    {
      std::stringstream ss;
      ss << row_number << ": "; 
      for (const auto& val : row) 
      {
        ss << val << " ";
      }
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "%s", ss.str().c_str());
      ++row_number;
    }


    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "A_most_violated_matrix");
     row_number = 1;  

    for (const auto& row : A_most_violated_matrix_)
    {
      std::stringstream ss;
      ss << row_number << ": "; 
      for (const auto& val : row)
      {
        ss << val << " ";
      }
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "%s", ss.str().c_str());
      ++row_number;
    }

    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "b_most_violated_vect");
     row_number = 1;  

    for (const auto& row : b_most_violated_vect_)
    {
      std::stringstream ss;
      ss << row_number << ": "; 
      for (const auto& val : row)
      {
        ss << val << " ";
      }
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "%s", ss.str().c_str());
      ++row_number;
    }
    

    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "MPC A_matrix");
    row_number = 1;

    for (int i = 0; i < mpc_obstacle_constraints_.matrix_rows.size(); ++i)
    {
      std::stringstream ss;
      ss << row_number << ": ";
      ss << mpc_obstacle_constraints_.matrix_rows[i].col1 << " ";
      ss << mpc_obstacle_constraints_.matrix_rows[i].col2;
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "%s", ss.str().c_str());
      ++row_number;
    }

    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "MPC b_vector");
    row_number = 1;

    for (int i = 0; i < mpc_obstacle_constraints_.vector_rows.size(); ++i)
    { 
      std::stringstream ss;
      ss << row_number << ": ";
      ss << mpc_obstacle_constraints_.vector_rows[i].col1;
      RCLCPP_INFO(rclcpp::get_logger("CustomController"), "%s", ss.str().c_str());
      ++row_number;
    }
    
  }



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

  /// #TODO: NB combine polygon_filter and computeCentroid to be filterPolygons function !!!!
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
   // if (distance < obstacle_distance_thresh_)
    if(distance<obstacle_distance_thresh_)
    {

      considered_centroid.obstacles.push_back(obstacle);
      considered_polygons.obstacles.push_back(obstacles.obstacles[index]);

    }

    index++;

  }

  // Return the vector container with considered polygons below the threshold
 // RCLCPP_INFO(rclcpp::get_logger("CustomController"), 
 //  "Number of considered polygons: %d", considered_polygons.obstacles.size());

  }

void CustomController::calcLineEquation(const geometry_msgs::msg::Point32 &p1,  const geometry_msgs::msg::Point32 &p2,std::vector<std::vector<float>> &A_matrix,std::vector<std::vector<float>> &b_vect)
{

  float slope,intercept;
  std::vector<float> rowVector;
  geometry_msgs::msg::Point32 point1 = p1;
  geometry_msgs::msg::Point32 point2 = p2;

  // calculate slope "m"

  slope = (point2.y - point1.y)/(point2.x - point1.x);


  // if the slope is -inf or inf we have horizontal line
  if (slope == std::numeric_limits<float>::infinity() || slope == -std::numeric_limits<float>::infinity())
  {
    slope = 1; // before slope had inverted sign
    intercept = point1.x; //before intercept had the original non-inverted value of x
    rowVector = {slope, 0};

       // new approach, do this conversion before publishing to obstacle_constraints topic for all constraints that make result_pose (+)

  //  if (intercept < robot_footprint_rotated_[3].x) // if the line is behind the robot
   // {
    //  slope = -1; //invert
     // intercept = -1*intercept; // invert
   // }

  }
  else if (slope == 0) // vertical line
  {
    intercept = point1.y;
    rowVector = {slope, 1};

  }
  else // general case
  {

   intercept = point1.y - slope * point1.x;
   //intercept = -1*intercept; //invert 
   rowVector = {-slope, 1}; 

  }

  A_matrix.push_back(rowVector);
  b_vect.push_back({intercept});
    
}


void CustomController::compute_violated_constraints(const geometry_msgs::msg::Point32 &p_centroid,const std::vector<std::vector<float>> &A_matrix,const std::vector<std::vector<float>> &b_vect)
{
  if (!A_matrix.empty() && !b_vect.empty())
  {

    // inflate the latest constraint stored in A_matrix and b_vect
    inflate_constraint(A_matrix[A_matrix.size()-1],b_vect[b_vect.size()-1][0],p_centroid);
    result_pose = (A_matrix[A_matrix.size()-1][0] * robot_pose_.pose.position.x + A_matrix[A_matrix.size()-1][1] * robot_pose_.pose.position.y) - b_vect_inflated_[b_vect_inflated_.size()-1][0];

    //check if the current constraint separate in different half-planes the robot's center and centroid of obstacle
    if(isViolated(p_centroid,A_matrix[A_matrix.size()-1],b_vect_inflated_[b_vect_inflated_.size()-1][0]) == true)
    {
      A_violated_matrix_.push_back({A_matrix[A_matrix.size()-1][0],A_matrix[A_matrix.size()-1][1]});
      b_violated_vect_.push_back({b_vect_inflated_[b_vect_inflated_.size()-1][0]});
      result_pose_stored_.push_back({result_pose,0});
    }
    else if(isViolated(p_centroid,A_matrix[A_matrix.size()-1],b_vect_inflated_[b_vect_inflated_.size()-1][0]) == false)
    {
      // do not consider that constraint
    }
  }
}


void CustomController::compute_most_violated_constraints()
{


  // 13/06


  // look photo in phone for the constraints how they are sent to the mpc and think how I can leave only one line behind the robot instead of both being included


  /////
  float largest = std::numeric_limits<float>::lowest(); // Initialize with the lowest possible value
  int largest_index = 0;
  float distance = 0;
  for (size_t row = 0; row < result_pose_stored_.size(); row++)
  {
    // compute the distance from robot's center to the current violated line 
    distance = compute_distance_to_violated_constraint(robot_pose_.pose.position.x,robot_pose_.pose.position.y,A_violated_matrix_[row],b_violated_vect_[row][0]);
    
    if(distance > largest) 
    {
      // consider the violated line furthest from the robot
      largest = distance;
      largest_index = row;
    }
  }


  if (!A_violated_matrix_.empty()  && !b_violated_vect_.empty() )
  {
   
    A_most_violated_matrix_.push_back({A_violated_matrix_[largest_index][0],A_violated_matrix_[largest_index][1]});
    b_most_violated_vect_.push_back({b_violated_vect_[largest_index][0]});

    nav2_custom_controller_msgs::msg::ColumnMsg A_matrix_col;
    nav2_custom_controller_msgs::msg::ColumnMsg b_vect_col;

    A_matrix_col.col1 = A_violated_matrix_[largest_index][0];
    A_matrix_col.col2 = A_violated_matrix_[largest_index][1];
    b_vect_col.col1 = b_violated_vect_[largest_index][0];

    // for all most violated constraints that happen to be to the right of the robot and also for horizontal line behind robot
    // this inversion is needed for the correct representation of the lines inside the MPC problem formulation

    if(result_pose_stored_[largest_index][0] > 0) 
   // if(result_pose_stored_[largest_index][0] < 0) // for all most violated constraints that happen to be to the right of the robot and also for horizontal line behind robot ->  >0 is true for sim only, for real robot with 
                                                  // positive x map frame down and positive y frame to the right, it should be for results < 0 
    {
      // invert all values
      A_matrix_col.col1 = A_matrix_col.col1 * -1;
      A_matrix_col.col2 = A_matrix_col.col2 * -1;
      b_vect_col.col1 = b_vect_col.col1 * -1;

    }

      mpc_obstacle_constraints_.matrix_rows.push_back(A_matrix_col);
      mpc_obstacle_constraints_.vector_rows.push_back(b_vect_col);
  }

  A_violated_matrix_.clear();
  b_violated_vect_.clear();
  result_pose_stored_.clear();

}



void CustomController::inflate_constraint(const std::vector<float> &A_matrix_row,const float &b_vect,const geometry_msgs::msg::Point32 &centroid)
{
  // create inflated constraint (add 2 additional lines to the main constraint line)
  float b_violated_vect_inflated_1,b_violated_vect_inflated_2;
 // b_violated_vect_inflated_1.push_back({0.0});
 // b_violated_vect_inflated_2.push_back({0.0});

  b_violated_vect_inflated_1 = b_vect;
  b_violated_vect_inflated_2 = b_vect;
  float distance_between_lines = 0;

  while (distance_between_lines < inflation_radius_)
  {
    b_violated_vect_inflated_1 = b_violated_vect_inflated_1 + 0.025; // add inflation
    b_violated_vect_inflated_2 = b_violated_vect_inflated_2 - 0.025; // add inflation

    // send inflated line to function and compute distance between inflated line and original line
    distance_between_lines = compute_distance_between_lines(A_matrix_row,b_vect,b_violated_vect_inflated_1);

  }
  // once inflation is done, decide which inflated line to consider
  float point_line_distance_1,point_line_distance_2;

  point_line_distance_1 = compute_distance_to_violated_constraint(robot_pose_.pose.position.x,robot_pose_.pose.position.y,A_matrix_row,b_violated_vect_inflated_1);
  point_line_distance_2 = compute_distance_to_violated_constraint(robot_pose_.pose.position.x,robot_pose_.pose.position.y,A_matrix_row,b_violated_vect_inflated_2);

  // check for the current constraint if pose and centroid are in the same half-plane 
  if(isViolated(centroid,A_matrix_row,b_vect)==true) 
  {
    // centroid and robot's center not in the same half-plane, consider inflated line closer to robot
    if (point_line_distance_1 < point_line_distance_2 )
    {
      // push back b_vect_inflated_1 to b_vect_inflated_
      b_vect_inflated_.push_back({b_violated_vect_inflated_1});
    }
    else if (point_line_distance_2 < point_line_distance_1)
    {
      // push back b_vect_inflated_2 to b_vect_inflated_
      b_vect_inflated_.push_back({b_violated_vect_inflated_2});

    }
  }
  else if(isViolated(centroid,A_matrix_row,b_vect)==false) 
  {
    // centroid and robot's center in the same half-plane, consider inflated line furthest from robot
    if (point_line_distance_1 > point_line_distance_2 )
    {
      // push back b_vect_inflated_1 to b_vect_inflated_
      b_vect_inflated_.push_back({b_violated_vect_inflated_1});

    }
    else if (point_line_distance_2 > point_line_distance_1)
    {
      // push back b_vect_inflated_2 to b_vect_inflated_
      b_vect_inflated_.push_back({b_violated_vect_inflated_2});

    }
  }
} 

float CustomController::compute_distance_between_lines(const std::vector<float> &A_matrix_row,const float &b_vect,const float &b_vect_inflated)
{
  float A,B,C1,C2;

  A = A_matrix_row[0]; 
  B = A_matrix_row[1];
  C1 = b_vect;
  C2 = b_vect_inflated;

  return abs(C2-C1)/sqrt(pow(A,2) + pow(B,2));
}

// b_vector_row_value is the row, col value of b_vector (that value corresponds to the intercept of the line)
float CustomController::compute_distance_to_violated_constraint(const float &x_coord, const float &y_coord, const std::vector<float> &A_matrix_row,const float &b_vect)
{
  
  // (A*x+B*y+C)/sqr((A^2 + B^2)) - distance of a point to a line
  float A,B,C;
  A = A_matrix_row[0];
  B = A_matrix_row[1];
  C = b_vect * -1;

  return abs((A * x_coord + B * y_coord + C) / sqrt(pow(A,2) + pow(B,2)));;
}


 // will determine if a point and robot's center are in the same half-plane given a constraint 
bool CustomController::isViolated(const geometry_msgs::msg::Point32 &point,const std::vector<float> &A_matrix_row,const float &b_vect)
{

  float result = (A_matrix_row[0] * point.x + A_matrix_row[1] * point.y) - b_vect;
  float result_origin = (A_matrix_row[0] * robot_pose_.pose.position.x + A_matrix_row[1] * robot_pose_.pose.position.y) - b_vect;

  if (result * result_origin < 0 || result * result_origin == 0 || result * result_origin < 0.001)
  {
    // not in the same plane
    return true;
  }
  else if (result * result_origin > 0)
  {
    return false;
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



    ///////////////////////////////


  A_convex_region_matrix_.clear();
  b_convex_region_vect_.clear();


  //iterate over each vertex of the current polygon (.size() - 2 to account for last vertex in the vector that is duplicate of the first vertex)
    for (int j = 0; j < (int)line_list.points.size() - 2; ++j)
    {


    geometry_msgs::msg::Point32 point,point2;

    point.x = line_list.points[j].x;
    point.y = line_list.points[j].y;
    point2.x = line_list.points[j+1].x;
    point2.y = line_list.points[j+1].y;



     // calcLineEquation(line_list.points[j],line_list.points[j+1],A_convex_region_matrix_,b_convex_region_vect_);
      calcLineEquation(point,point2,A_convex_region_matrix_,b_convex_region_vect_);

    }

     // to prevent accessing empty vector (resulting in undefined behaviour) perform check if the
      // vector is not empty and if the vector size is not 2 (otherwise the for loop between 2 points will be enough)
    if (!line_list.points.empty() && line_list.points.size() != 2)
    {

      // calculate the equation of the line between last point and first point of the polygon
      auto last_point = line_list.points.end();  
      auto prev_point = std::prev(last_point, 2);  
      geometry_msgs::msg::Point32 point3,point4;
      //auto point5 = *prev_point;
      point3.x = (*prev_point).x;
      point3.y = (*prev_point).y;
      point4.x = line_list.points.front().x;
      point4.y = line_list.points.front().y;

    //  calcLineEquation(*prev_point,line_list.points.front(),A_convex_region_matrix_,b_convex_region_vect_);
          calcLineEquation(point3,point4,A_convex_region_matrix_,b_convex_region_vect_);

    }

    for (int row=0; row<A_convex_region_matrix_.size();row++)
    {
      for (int col=0; col<A_convex_region_matrix_[row].size();col++)
      {

      }
    }
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


  // configured in yaml file to run every 10ms or (100Hz) 
  geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &  , nav2_core::GoalChecker * ) 
  {


    robot_pose_.pose.position.x = pose.pose.position.x;
    robot_pose_.pose.position.y = pose.pose.position.y;



  //  RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Start of computeVelocityCommands");


  // Define a duration type for milliseconds
  using Milliseconds = std::chrono::milliseconds;

  // Get the current time before executing the function
  auto start_time = std::chrono::steady_clock::now();

  double roll, pitch, yaw;
  tf2::Quaternion quat;
  tf2::fromMsg(pose.pose.orientation, quat);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double v_act,w_act; 


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

  // Compute feedback_linearization
  execute_fblin();

    // Get the current time
  auto now = std::chrono::steady_clock::now();

  // Check if Ts_MPC_*1000 ms have passed since the last call
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_call_time).count() >= Ts_MPC_*1000)
  {

     int status;
  MPC_->get_status(status);
  if (status == 1) // error solving optim problem
  {
    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Error solving the optimization problem - solver infeasible");
    RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Re-initializing MPC");
    // make it wait for 0.5 seconds before intializing again !
   // rclcpp::sleep_for(std::chrono::milliseconds(500));
    MPC_->initialize(); // re-init the MPC problem to try again
    MPC_->set_actualRobotState(Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, yaw)); // position.z is converted yaw

    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.angular.z = 0;
    
  } 

    // RCLCPP_INFO(rclcpp::get_logger("CustomController"),"MPC optim started: %.2f",robot_pose_.pose.position.x);
    // Define a duration type for milliseconds
    using Milliseconds = std::chrono::milliseconds;

    // Calculate the time between consecutive calls
    auto interval_duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_call_time);
    double interval_duration_ms = static_cast<double>(interval_duration.count());

    // Print the duration between consecutive calls
    //RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Actual MPC sampling time: %.3f ms", interval_duration_ms);

    // Get the current time before executing the function
    auto start_time = std::chrono::steady_clock::now();

    // Execute the MPC
    execute_mpc();

    // Get the current time after executing the function
    auto end_time = std::chrono::steady_clock::now();

    // Calculate the duration of execution in milliseconds
    auto duration = std::chrono::duration_cast<Milliseconds>(end_time - start_time);

    double duration_ms = static_cast<double>(duration.count());

    // Print the duration in milliseconds
   // RCLCPP_INFO(rclcpp::get_logger("CustomController"), "MPC optimisation time: %.3f ms", duration_ms);

    // Reset the last call time
    last_call_time = now;
  }



  MPC_->get_actualControl(v_act, w_act);

  cmd_vel.twist.linear.x = v_act;

  cmd_vel.twist.angular.z = w_act;

  // Get the current time after executing the function
  auto end_time = std::chrono::steady_clock::now();

      // Calculate the duration of execution in milliseconds
  auto duration = std::chrono::duration_cast<Milliseconds>(end_time - start_time);

  double duration_ms = static_cast<double>(duration.count());

      // Print the duration in milliseconds
   //   RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Execution time of computeVelocityCommands: %.3f ms", duration_ms);
   //   RCLCPP_INFO(rclcpp::get_logger("CustomController"), "End of computeVelocityCommands: %.3f ms", duration_ms);
 




  return cmd_vel;
  }

  void CustomController::execute_mpc() // every Ts_MPC_ seconds
  {

   if(mpc_obstacle_constraints_matrix_.size()!=0) // if there are obstacle constraints, set them in MPC class
   {


    MPC_->set_obstacle_matrices(mpc_obstacle_constraints_matrix_,mpc_obstacle_constraints_vector_);
  //  RCLCPP_INFO(rclcpp::get_logger("CustomController"), "Obstacle Constraints sent to MPC.");

   }

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

  void CustomController::execute_fblin()
  {
    
  MPC_->executeLinearizationController();
     
  }


  void CustomController::setPlan(const nav_msgs::msg::Path &path )
  {
    if (path_loaded_ == false)
    {

      // load the path, do not use path from nav2 planner
      disable_nav2_path_ = true;
      path_saved_ = true; // if we are loading the path, we do not to save it again 

      std::string file_path = "/home/mario/scout_working/nav2_path.csv"; // Change the file path as needed
      std::ifstream csvfile(file_path);
      if (!csvfile.is_open())
      {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return;
      }

      std::string line;
      while (std::getline(csvfile, line))
      {
        std::istringstream line_stream(line);
        std::string field;
        std::vector<double> data;

        while (std::getline(line_stream, field, ','))
        {
          data.push_back(std::stod(field));
        }

        if (data.size() != 3)
        {
          std::cerr << "Incorrect data format in CSV file" << std::endl;
          continue;
        }
        double x = data[0];
        double y = data[1];
        double yaw = data[2];
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map"; // Set the frame ID appropriately
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        global_plan_.poses.push_back(pose_stamped);
      }

    csvfile.close();
    path_loaded_ = true; // setting it to true will make sure that the if statement is execute only once

    }

    if (path_saved_ == false)
    {

      std::ofstream csvfile("/home/mario/scout_working/nav2_path.csv"); // Change the file path as needed
      // csvfile << "x,y,yaw\n"; // Write header with yaw

      for (const auto& pose_stamped : path.poses)
      {
        double x = pose_stamped.pose.position.x;
        double y = pose_stamped.pose.position.y;
        // Extract yaw from the quaternion orientation
        tf2::Quaternion quaternion;
        tf2::fromMsg(pose_stamped.pose.orientation, quaternion);
        double yaw = tf2::getYaw(quaternion);
        csvfile << x << "," << y << "," << yaw << "\n"; // Write pose data with yaw
      }
      csvfile.close();
      path_saved_ = true; // Set the flag to true to indicate that the path has been saved, preventing from further saving when path recomputes
    }

    if (disable_nav2_path_ == false)
    {
      global_plan_ = path;
    }

}



  void CustomController::setSpeedLimit(const double & , const bool & )
  {

  }

} // namespace

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)
