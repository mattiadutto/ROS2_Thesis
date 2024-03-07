#include "nav2_custom_controller/custom_controller.hpp"

// scout_working file
//
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

      std::vector<std::vector<float>> A_obst_matrix_;
      std::vector<std::vector<float>> b_vect_;



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

// print matrix of slope and intercept of each constraint

 //for(const auto& row : line_vect)
 // {
  //  std::cout<<row[0]<<row[1]<<std::endl;
 // }



  /// get the obstacles container as a ptr of ObstacleArrayMsg from getObstacles() method
  costmap_converter::ObstacleArrayConstPtr obstacles_ptr = costmap_converter_->getObstacles();

  costmap_converter_msgs::msg::ObstacleArrayMsg obstacles = *obstacles_ptr;

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

  costmap_converter_msgs::msg::ObstacleArrayMsg centroid = computeCentroid(obstacles);

  costmap_converter_msgs::msg::ObstacleArrayMsg considered_polygons = polygon_filter(centroid,obstacles);


  //line_eq_vect_.clear();

  geometry_msgs::msg::Point32 p1,p2,p3,p4,p31,p13;


  // look the point in which quadrant is and according to that quadrant apply -x or -y to the coordinates
  // to account for cartesian frame otherwise the equation of the line will be calculated in cartesian frame
  // and in this robot's frame it will represent a wrong line !!! 

  // for vertical lines apply [0 -1] for horizontal lines apply [1 0] to get it right 
  p1.x = -0.875;
  p1.y = 0.475;

  p2.x = -0.325;
  p2.y = 0.475;

  p3.x = -0.325;
  p3.y = 0.775;

 // p31.x = 0.325;
 // p31.y = 0.775;

 // p13.x = 0.875;
 // p13.y = 0.475;

 // p4.x = -0.875;
 // p4.y = 0.475;

 //std::cout<<"Slope"<<(0.475-0.775)/(-0.875-(-0.325))<<std::endl;
// std::cout<<"Slope"<<(p3.y-p1.y)/(p3.x-p1.x)<<std::endl;

 //float slope = (p3.y-p1.y)/(p3.x-p1.x);
// std::cout<<"Slope"<<slope<<std::endl;
  A_obst_matrix_.clear();
  b_vect_.clear();

  calcLineEquation(p1,p2,A_obst_matrix_,b_vect_);

  calcLineEquation(p2,p3,A_obst_matrix_,b_vect_); // this is the case when x are equal so division by zero gives inf
                                            // handle this case! if x are equal then we have horizontal line!
  calcLineEquation(p3,p1,A_obst_matrix_,b_vect_);

     // calcLineEquation(p31,p13,A_obst_matrix_,b_vect_);
      //  calcLineEquation(p4,p1,line_eq_vect_);


//NB! in the frame of the robot and map x is the vertical whyle y is horizontal
  // fix the code to account that vertical lines in the eq must be horizontal
  // and horizontal lines should be vertical!

 /* for (const auto &obstacle : considered_polygons.obstacles)
  {

     //iterate over each vertex of the current polygon (.size() - 2 to account for last vertex in the vector that is duplicate of the first vertex)
    for (int j = 0; j < (int)obstacle.polygon.points.size() - 2; ++j)
    {

      calcLineEquation(obstacle.polygon.points[j],obstacle.polygon.points[j+1],A_obst_matrix_,b_vect_);

    }

    // to prevent accessing empty vector (resulting in undefined behaviour) perform check if the
    // vector is not empty and if the vector size is not 2 (otherwise the for loop between 2 points will be enough)

    if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2)
    {
      // calculate the equation of the line between last point and first point of the polygon
      auto last_point = obstacle.polygon.points.end();  // Iterator to the end
      auto prev_point = std::prev(last_point, 2);  
      calcLineEquation(*prev_point,obstacle.polygon.points.front(),A_obst_matrix_,b_vect_);

    }

  }*/

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





  // function that creates polygons/lines and publishes them as Marker msg for visualisation

  //publishAsMarker(frame_id_, *obstacles);

    publishAsMarker(frame_id_,considered_polygons);

  // publishAsMarker(frame_id_,convex_hull_array);


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

void CustomController::calcLineEquation(const geometry_msgs::msg::Point32 &p1,  const geometry_msgs::msg::Point32 &p2,std::vector<std::vector<float>> &A_matrix,std::vector<std::vector<float>> &b_vect)
{

  float slope,intercept;
  std::vector<float> rowVector;
  bool horizontal = false;
  bool vertical = false;
  geometry_msgs::msg::Point32 point1 = p1;
  geometry_msgs::msg::Point32 point2 =p2;

  // do rotations from RH rule frame to Cartesian frame of each point

  // if point1 is in 1st quadrant in RH frame
  if (point1.x > 0 && point1.y < 0)
  {
    // rotate it to be in 4th (90 deg rotation clockwise to align with cartesian frame)
    point1.x = point1.x * -1;
  }
  // 2nd quadrant
  else if (point1.x > 0 && point1.y > 0)
  {
    point1.y = point1.y * -1;
  }
  // 3rd quadrant
  else if (point1.x < 0 && point1.y > 0 )
  {
    point1.x = point1.x * -1;
  }
  // 4th qudrant
  else if (point1.x < 0 && point1.y < 0)
  {
    point1.y = point1.y * -1;
  }


  if (point2.x > 0 && point2.y < 0)
  {
    // rotate it to be in 4th (90 deg rotation clockwise to align with cartesian frame)
    point2.x = point2.x * -1;
  }
  // 2nd quadrant
  else if (point2.x > 0 && point2.y > 0)
  {
    point2.y = point2.y * -1;
  }
  // 3rd quadrant
  else if (point2.x < 0 && point2.y > 0 )
  {
    point2.x = point2.x * -1;


  }
  // 4th qudrant
  else if (point2.x < 0 && point2.y < 0)
  {
    point2.y = point2.y * -1;
  }



  slope = (point2.y - point1.y)/(point2.x - point1.x);
  std::cout<<"Slope: "<<slope<<std::endl;
  
  // handle horizontal line case (in RH convention its horizontal line as y is positive left of the origin and x is positive above the origin)
  if (slope == std::numeric_limits<float>::infinity() || slope == -std::numeric_limits<float>::infinity())
  {
    slope = 1;
    intercept = point1.x;
    horizontal = true;

    // second col of A matrix must be 0 as y = 0 !
  }
  else if (slope == 0)
  {
    vertical = true;
    intercept = point1.y;
  }
  else 
  {
   intercept = point1.y - slope * point1.x;
  }

  if (horizontal == true) 
  {
    rowVector = {slope, 0};
    intercept = intercept * -1;
  }
  else if (vertical == true) // for vertical lines (y must be -y)
  {
    rowVector = {slope, -1};
    intercept = intercept * -1;

  }
  else
  {
    rowVector = {slope, 1};
  }

  A_matrix.push_back(rowVector);
  b_vect.push_back({intercept});

}

void checkConstraint(const geometry_msgs::msg::Point32 &p1)
{
  // multiply Amatrix * [p1.x ; p1.y] - b_vect = ? if negative constraint is satisfied if positive constraint is not satisfied
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
 costmap_converter_msgs::msg::ObstacleArrayMsg &obstacles)
{

  costmap_converter_msgs::msg::ObstacleArrayMsg considered_polygons;
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

            considered_polygons.obstacles.push_back(obstacles.obstacles[index]);


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