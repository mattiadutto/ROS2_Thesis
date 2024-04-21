#include "nav2_custom_controller/feedback_lin.hpp"

void FeedbackLin::calcPointP(const geometry_msgs::msg::PoseStamped &pose, const double &yaw, const double &epsilon)
{
    epsilon_ = epsilon;
    yaw_ = yaw;
    pose_ = pose;
    // Get x and y of point P at a distance epsilon
    xp_ = pose.pose.position.x + epsilon_ * cos(yaw_);
    yp_ = pose.pose.position.y + epsilon_ * sin(yaw_);
    // Store in array
    pointP_[0] = xp_;
    pointP_[1] = yp_;
}



const geometry_msgs::msg::TwistStamped &FeedbackLin::linearize(const double xp_dot, const double yp_dot)
{
    xp_dot_ = xp_dot;
    yp_dot_ = yp_dot;

    vel_.twist.linear.x = xp_dot_ * cos(yaw_) + yp_dot_ * sin(yaw_);

	vel_.twist.angular.z = (yp_dot_ * cos(yaw_) - xp_dot_ * sin(yaw_)) / epsilon_;

    return vel_;

}

const std::array<double,2> &FeedbackLin::getPointP() const
{
    return pointP_;
}

const geometry_msgs::msg::PoseStamped &FeedbackLin::getXY() const
{
    return pose_;
}



