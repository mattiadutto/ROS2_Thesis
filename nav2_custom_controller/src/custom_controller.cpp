#include "nav2_custom_controller/custom_controller.hpp"

// scout_working file
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


  CustomController::CustomController():costmap_ros_(nullptr),costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
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
    logger_ = node->get_logger(); 
    clock_ = node->get_clock();

    // create new instance of rclcpp:Node with name: costmap_converter
    intra_proc_node_.reset(new rclcpp::Node("costmap_converter", node->get_namespace(), rclcpp::NodeOptions()));

    // asign the costmap as a pointer costmap_
    costmap_ = costmap_ros_->getCostmap();

    // Parameter declaration 
    declare_parameter_if_not_declared(
      node, plugin_name_ + ".costmap_converter_plugin", rclcpp::ParameterValue("costmap_converter::CostmapToLinesDBSRANSAC"));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".costmap_converter_rate", rclcpp::ParameterValue(5));

    declare_parameter_if_not_declared(
      node, plugin_name_ + ".odom_topic", rclcpp::ParameterValue(""));

    // Set parameters from yaml file
    node->get_parameter(plugin_name_ + ".costmap_converter_plugin",costmap_converter_plugin_); 
    node->get_parameter(plugin_name_ + ".costmap_converter_rate",costmap_converter_rate_); 
    node->get_parameter(plugin_name_ + ".odom_topic",odom_topic_);



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

    obstacle_pub_ = node->create_publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>("costmap_obstacles", 1);

    polygon_pub_ = node->create_publisher<geometry_msgs::msg::PolygonStamped>("costmap_polygons",1);

    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("polygon_marker", 10);

    tf_pub_ = node->create_publisher<geometry_msgs::msg::TransformStamped>("tf_pub",10);

    // Create a lifecycle wall timer with a callback function

    //call timer_callback() every 1 second
    wall_timer_ = node->create_wall_timer(std::chrono::seconds(1), std::bind(&CustomController::timer_callback, this));

    pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",100,std::bind(&CustomController::pose_sub_callback, this,std::placeholders::_1));

    // costmap_converter_polygons_ = std::make_unique<costmap_converter::CostmapToPolygonsDBSMCCH>();

     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);



  }

void CustomController::timer_callback()
{


  /// get the obstacles container as a ptr of ObstacleArrayMsg from getObstacles() method
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();

  geometry_msgs::msg::TransformStamped received_tf;

  try
  { 

   received_tf = tf_->lookupTransform("map","base_link",tf2::TimePointZero);

  } catch (tf2::LookupException &ex)
  {

    RCLCPP_WARN(rclcpp::get_logger("TF"),"Can't find base_link to map tf: %s", ex.what());
  }

  tf_pub_->publish(received_tf);


  // get the global frame 
  std::string frame_id_ = costmap_ros_->getGlobalFrameID();

  costmap_converter_msgs::msg::ObstacleArrayMsg centroid = computeCentroid(*obstacles);

  costmap_converter_msgs::msg::ObstacleArrayMsg considered_polygons = polygon_filter(centroid,*obstacles);


  // function that creates polygons/lines and publishes them as Marker msg for visualisation

  //publishAsMarker(frame_id_, *obstacles);

  //  publishAsMarker(frame_id_,considered_polygons);

  // publishAsMarker(frame_id_,convex_hull_array);


}


costmap_converter_msgs::msg::ObstacleArrayMsg CustomController::computeCentroid(const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles)
{

  costmap_converter_msgs::msg::ObstacleArrayMsg centroid;

  // Clear centroid.obstacles at the beginning of the function
  centroid.obstacles.clear();

//int index = 0;
//int v_index = 0;

  for (const auto &obstacle:obstacles.obstacles)
  {
    double sum_x = 0;
    double sum_y = 0;

    int total_vertices = obstacle.polygon.points.size();

    for (const auto &point:obstacle.polygon.points)
    {

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

void CustomController::publishAsMarker(const std::string &frame_id,const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles)
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

//// NB combine polygon_filter and computeCentroid to be filterPolygons function !!!!

costmap_converter_msgs::msg::ObstacleArrayMsg CustomController::polygon_filter(const costmap_converter_msgs::msg::ObstacleArrayMsg &polygon_centroids, 
const costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles)
{

  costmap_converter_msgs::msg::ObstacleArrayMsg considered_polygons;
  double thresh = 1.0;

  int index=0;


  // Iterate over each polygon in polygon_centroids.obstacles
  for (const auto obstacle:polygon_centroids.obstacles)
  {
    

    // get euclidean distance between current polygon's centroid and robot's pose
    double distance = euclideanDistance(obstacle.polygon.points[0].x,obstacle.polygon.points[0].y,robot_pose_.pose.position.x,robot_pose_.pose.position.y);

    // if the distance is below a threshold "thresh" then consider it and store it in a vector
    if (distance < thresh)
    {

      // store in considered_polygon the obstacle to which the computed centroid refers to
      considered_polygons.obstacles.push_back(obstacles.obstacles[index]);

    }

    index++;
  }
  // Return the vector container with considered polygons below the threshold
  std::cout<<"Number of considered polygons:"<<considered_polygons.obstacles.size()<<std::endl;
  return considered_polygons;
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

  return cmd_vel_;
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
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController, nav2_core::Controller)