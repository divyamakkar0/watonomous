#include "map_memory_node.hpp"
using std::placeholders::_1;


MapMemoryNode::MapMemoryNode(float dt) : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger(), 0.1, 30)){
  costmap_readings = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::readCostmap, this, _1));
  odom_readings = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::readOdom, this, _1));
  this->last_pos = std::make_pair(0, 0);
  this->dt = dt; 
  mapPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&MapMemoryNode::publishMap, this));
}

//need to change hardcoding of the size
void MapMemoryNode::readCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  const int length = this->map_memory_.getLength();
  float x = this->pos.first;
  float y = this->pos.second; 
  float last_x = this->last_pos.first;
  float last_y = this->last_pos.second; 
  float dt = this->dt;

  std::vector<signed char> occupancyGrid = msg->data;
  
  float distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));

  if(distance <= dt){
    return; 
  }

  for (int i = 0; i < length; i++) {
        for (int j = 0; j < length; j++) {
            this->map_memory_.UpdatePos(j, i, occupancyGrid[i * 31 + j]);
        }
  }

}

void MapMemoryNode::publishMap(){
    nav_msgs::msg::OccupancyGrid message;
    message.header.frame_id = "robot/costmap";
    message.header.stamp = this->get_clock()->now();
    message.info.resolution = this->map_memory_.resolution;
    const int length = this->map_memory_.getLength();
    message.info.width = length;
    message.info.height = length;

    std::vector<signed char> tmp(length * length);
    for(int i = 0; i < length; i++){
      for(int k = 0; k < length; k++){
        tmp[i * length + k] = this->map_memory_.OccupancyGrid[i][k];
      }
    }
    message.data = tmp;
    mapPub_->publish(message);

}

void MapMemoryNode::readOdom(const nav_msgs::msg::Odometry::SharedPtr msg){

  this->last_pos = std::make_pair(this->pos.first, this->pos.second); 
  this->pos = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

int main(int argc, char ** argv)
{
  float dt = 1.5;
  const float resolution = 0.1;
  const int size = 30;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>(dt));
  rclcpp::shutdown();
  return 0;
}
