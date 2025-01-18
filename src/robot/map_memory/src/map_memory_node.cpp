#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_readings = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::readCostmap, this, _1));
  odom_readings = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::readOdom, this, _1));
}

//need to change hardcoding of the size
void MapMemoryNode::readCostmap(const nav_msgs::msg::OccupancyGrid msg){
  std::vector<int>(961) occupancyGrid = msg->occupancy_grid;
  std::vector<std::vector<int>> reshapedGrid(31, std::vector<int>(31));

  for (int i = 0; i < 31; ++i) {
        for (int j = 0; j < 31; ++j) {
            reshapedGrid[i][j] = occupancyGrid[i * 31 + j];
        }
  }
}

void MapMemoryNode::readOdom(const nav_msgs::msg::Odometry msg){
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  double distance = std::swrt
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
