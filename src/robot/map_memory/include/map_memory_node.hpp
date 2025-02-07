#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode(float dt);

    void readCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void publishMap(); 
    void readOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_readings;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_readings;
    std::pair<float,float> pos; 
    std::pair<float,float> last_pos;
    float dt;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPub_;
    rclcpp::TimerBase::SharedPtr timer_;


  private:
    robot::MapMemoryCore map_memory_;
};

#endif 
