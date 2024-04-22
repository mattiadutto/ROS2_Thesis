#ifndef NAV2_CUSTOM_CONTROLLER__FEEDBACK_LIN_
#define NAV2_CUSTOM_CONTROLLER__FEEDBACK_LIN_
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Feedback Linearization class
//
// Implements Feedback Linearization of a non-holonomic differential drive robot
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <cmath>
#include <array>
#include<vector>
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav2_core/controller.hpp"


class FeedbackLin
{

    public:

    FeedbackLin() = default; 
    ~FeedbackLin() = default;

  

    // Function that calculates position of point P at a distance epsilon from robot 
    // Parameters:
    //   - pose: current pose of the robot
    //   - yaw: current theta (yaw) angle of the robot
    //   - epsilon: distance in front of the robot

    void calcPointP(const geometry_msgs::msg::PoseStamped &pose, const double &yaw, const double &epsilon);

    // Function that performs feedback linearization
    // Parameters:
    //   - xp_dot: x-coordinate of point P
    //   - yp_dot: y-coordinate of point P
    // Returns: const reference of TwistStamped message 

    const geometry_msgs::msg::Twist &linearize(const double xp_dot, const double yp_dot);

    // Function that returns position of point P 
    // Returns: xp-coordinate in [0] and yp-coordinate in [1]
    const std::array<double,2> &getPointP() const;

    const geometry_msgs::msg::PoseStamped &getXY() const;

    private:

    // Variables Declaration

    double xp_,yp_,xp_dot_,yp_dot_,epsilon_,yaw_;
    geometry_msgs::msg::Twist vel_;
    geometry_msgs::msg::PoseStamped pose_;
    std::array<double,2> pointP_;

};

#endif