#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void readCostmap(const nav_msgs::msg::OccupancyGrid msg):
    void readOdom(const nav_msgs::msg::Odometry msg);
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_readings;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_readings;

  private:
    robot::MapMemoryCore map_memory_;
};

#endif 
