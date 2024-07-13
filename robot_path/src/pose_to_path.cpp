#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PoseToPath : public rclcpp::Node
{
public:
  PoseToPath()
  : Node("pose_to_path")
  {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("robot_path", 10);
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "robot_pose", 10, std::bind(&PoseToPath::poseCallback, this, std::placeholders::_1));

    path_.header.frame_id = "map"; // Set the frame ID to "map"
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Create a new PoseStamped message for the path
    geometry_msgs::msg::PoseStamped pose_stamped;
   // pose_stamped.header.stamp = this->now(); // Use current time for the pose stamp
    pose_stamped.header.frame_id = "map";   // Override the frame ID to "map"
    pose_stamped.pose = msg->pose;          // Copy the pose from the received message

    // Update the path header timestamp
    //path_.header.stamp = this->now();

    // Add the new pose_stamped to the path
    path_.poses.push_back(pose_stamped);

    // Publish the updated path
    path_pub_->publish(path_);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseToPath>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
