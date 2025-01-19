#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode(float lookahead_distance, float goal_tolerance, float linear_speed);

    // utility functions
    void controlLoop();
    std::pair<float, float> findLookaheadPoint();
    void computeVelocity(std::pair<float, float> lookahead_point, float distanceToGoal, float intialDistance);
    float computeDistance(std::pair<float, float> p1, std::pair<float, float> p2);
    void extractYaw();

    // subscription functions
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // constants
    float lookahead_distance;
    float goal_tolerance;
    float linear_speed_;
    float yaw;
    int last_index;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::pair<float, float> position;
    std::tuple<float, float, float, float> angle;
    std::vector<geometry_msgs::msg::PoseStamped> path;

  private:
    robot::ControlCore control_;
};

#endif
