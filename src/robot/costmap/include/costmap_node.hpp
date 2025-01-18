#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"
#include <utility>

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode(float size, float res);

    // Place callback function here
    void readData(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void readPosition(const nav_msgs::msg::Odometry::SharedPtr msg);
    void inflatePosition(const int i, const int j);
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_;
    std::pair<float,float> pos_;
    std::tuple<float,float,float,float> angle_;
    float inflate_distance = 1.5;


  private:

};

#endif