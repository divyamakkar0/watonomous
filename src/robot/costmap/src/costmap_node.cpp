#include <chrono>
#include <memory>
#include "costmap_node.hpp"

#define PI  3.141590118408203
using std::placeholders::_1;

CostmapNode::CostmapNode(float size, float res) : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger(), res, size)){
  // Initialize the constructs and their parameters
  og_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::readData, this, _1));
  position_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&CostmapNode::readPosition, this, _1));
}


void CostmapNode::inflatePosition(const int xpos, const int ypos){

  // setup constants
  const int length = this->costmap_.getLength();
  const float res  = this->costmap_.resolution;
  const int radius = (int) ( this->inflate_distance / res);

  // get all the possible pairs in square grid
  std::vector<std::pair<int,int>> pairs;
  for(int i = 0; i < radius; i++){
    for(int k = 0; k < radius; k++){
      int x_1 = xpos + i;
      int x_2 = xpos - i;
      int y_1 = ypos + k;
      int y_2 = ypos - k;

      pairs.push_back(std::make_pair(x_1, y_1));
      pairs.push_back(std::make_pair(x_1, y_2));
      pairs.push_back(std::make_pair(x_2, y_1));
      pairs.push_back(std::make_pair(x_2, y_2));
    }
  }

  // inflate the costmap
  for(auto &i: pairs){
    int x = i.first;
    int y = i.second;
    if(x >= 0 && y >= 0 && x < length && y < length){
      float distance = sqrt(pow(xpos - x, 2) + pow(ypos - y,2));
      float val = 100 * (1 - (distance/radius));
      if(val < 0){
        val = 0;
      }
      this->costmap_.UpdatePos(x,y,val);
    }
  }

}

void CostmapNode::readData(const sensor_msgs::msg::LaserScan::SharedPtr msg)  {

  // Initialize the costmap
  this->costmap_.Initalize();

  // robot angle in quaternion
  float q_x = std::get<0>(this->angle_);
  float q_y = std::get<1>(this->angle_);
  float q_z = std::get<2>(this->angle_);
  float q_w = std::get<3>(this->angle_);

  // robot angle in euler
  float r_ang = atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y * q_y + q_z * q_z));

  const int length = this->costmap_.getLength();
  const int l2 = length / 2;
  const float res = this->costmap_.resolution;

  const int x = this->pos_.first;
  const int y = this->pos_.second;

  for (int i = 0;i < (int) msg->ranges.size(); i++){
    float angle = msg->angle_min + i * msg->angle_increment;
    float range = msg->ranges[i];
    if (range > msg->range_min && range < msg->range_max){

      range /= res;
      float new_angle = angle + r_ang;
      int x_grid = (int) (0.5 + x + range * cos(angle + r_ang));
      int y_grid = (int) (0.5 + y + range * sin(angle + r_ang)) ;

      int xpos = x_grid + l2;
      int ypos = y_grid + l2;

      // RCLCPP_INFO(this->get_logger(), "\n info %f %f %f", r_ang, new_angle, l2);
      // RCLCPP_INFO(this->get_logger(), "angle and range %f %f ", angle, range);
      // RCLCPP_INFO(this->get_logger(), " xpos %f %f %d", x_grid, x, xpos);
      // RCLCPP_INFO(this->get_logger(), "ypos %f %f %d \n", y_grid, y, ypos);

      // numerial errors
      if(xpos > l2 * 2){
        xpos = l2 * 2;
        RCLCPP_INFO(this->get_logger(), "xpos greater: %d", xpos);
      }
      if(xpos < 0){
        xpos = 0;
        RCLCPP_INFO(this->get_logger(), "xpos less: %d", xpos);
      }
      if(ypos > l2 * 2){
        RCLCPP_INFO(this->get_logger(), "ypos greeater: %d", ypos);
        ypos = l2 * 2;
      }
      if(ypos < 0){
        RCLCPP_INFO(this->get_logger(), "ypos less %d", ypos);
        ypos = 0;
      }

      this->costmap_.UpdatePos(xpos, ypos, 100);
      this->inflatePosition(xpos, ypos);
    }

 }


  nav_msgs::msg::OccupancyGrid message;
  message.header = msg->header;
  message.header.frame_id = "sim_world";
  message.info.resolution = this->costmap_.Info().first;
  message.info.width = length;
  message.info.height = length;
  message.info.origin.position.x = -15;
  message.info.origin.position.y = -15;

  std::vector<signed char> tmp(length * length);

  for(int i = 0; i < length; i++){
      for(int k = 0; k < length; k++){
        tmp[i * length + k] = this->costmap_.OccupancyGrid[i][k];
      }
  }

  message.data = tmp;
  og_->publish(message);

}

void CostmapNode::readPosition(const nav_msgs::msg::Odometry::SharedPtr msg){
  float x = msg->pose.pose.position.x / this->costmap_.resolution;
  float y = msg->pose.pose.position.y / this->costmap_.resolution;

  float q_x = msg->pose.pose.orientation.x;
  float q_y = msg->pose.pose.orientation.y;
  float q_z = msg->pose.pose.orientation.z;
  float q_w = msg->pose.pose.orientation.w;

  this->pos_ = std::make_pair(x,y);
  this->angle_ = std::make_tuple(q_x, q_y, q_z, q_w);

}

int main(int argc, char ** argv)
{
  float size = 30;
  float res = 0.1;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>(size, res));
  rclcpp::shutdown();
  return 0;
}