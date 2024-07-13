#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

class NavigateToPoseClient : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigateToPoseClient() : Node("navigate_to_pose_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

          // Create a timer to periodically check the send_goal_flag_ and send the goal if needed
  

   // pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",100,std::bind(&CustomController::pose_sub_callback, this,std::placeholders::_1));

    bool_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("bool_topic",100,std::bind(&NavigateToPoseClient::bool_callback, this,std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("final_pose",100,std::bind(&NavigateToPoseClient::pose_callback, this,std::placeholders::_1));

       timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&NavigateToPoseClient::timer_callback, this));


    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr bool_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;


        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::PoseStamped final_pose;


        int check;

    void send_goal()
    {
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = final_pose.pose.position.x;
        goal_msg.pose.pose.position.y = final_pose.pose.position.y;
        goal_msg.pose.pose.orientation.w = final_pose.pose.orientation.w;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&NavigateToPoseClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&NavigateToPoseClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&NavigateToPoseClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void timer_callback()
{

    if (check = 1)
    {

                this->send_goal();

    }
    
    
}

void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
         final_pose.pose.position.x = msg->pose.position.x;
         final_pose.pose.position.y = msg->pose.position.y;
         final_pose.pose.orientation.w = msg->pose.orientation.w;
}

void bool_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // Accessing the position.x field
     check = msg->pose.position.x;

    
}

    void goal_response_callback(std::shared_ptr<GoalHandleNavigateToPose> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: current pose (%.2f, %.2f)",
                    feedback->current_pose.pose.position.x,
                    feedback->current_pose.pose.position.y);
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }

        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateToPoseClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
