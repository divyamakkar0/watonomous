#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void readMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void readGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void readOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishPath();
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_readings;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped::SharedPtr>::SharedPtr goal_readings; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_readings;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::pair<float,float> goal_point;
    std::pair<float,float> pos;
    

  private:
    robot::PlannerCore planner_;
};

#endif 
