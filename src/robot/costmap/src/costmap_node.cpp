#include <chrono>
#include <memory>
#include "costmap_node.hpp"

#define PI  3.141590118408203
using std::placeholders::_1;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger(), 0.1, 30)){
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
  const int radius = (int) (this->inflate_distance / res);

  // get all the possible pairs in square grid
  std::vector<std::pair<int,int>> pairs;
  for(int i = 0; i < radius; i++){
    for(int k = 0; k < radius; k++){
      int x_1 = xpos + i;
      int x_2 = xpos - i;
      int y_1 = ypos + k;
      int y_2 = ypos - k;

      pairs.push_back(std::make_pair(x_1, y_1));
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
  const float z = this->angle_.first;
  const float w = this->angle_.second;
  float r_ang = 2 * asin(z);
  if(z == 0){
    r_ang = 0;
  }
  if (z < 0){
    r_ang *= -1;
  }

  const int length = this->costmap_.getLength();
  const float res = this->costmap_.resolution;

  for (int i = 0;i < (int) msg->ranges.size(); i++){
    if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min){

      float angle = (msg->angle_min + i * msg->angle_increment);
      float new_angle = angle + r_ang;

      if (new_angle < -PI){
        new_angle += 2*PI;
      } else if (new_angle > PI){
        new_angle -= 2 * PI;
      }

      float xi = (msg->ranges[i] * cos(new_angle))/res;
      float yi = (msg->ranges[i] * sin(new_angle))/res;

      float l2 = length / 2;
      int xpos = (int) (l2 + xi + this->pos_.first + 0.5);
      int ypos = (int) (l2 - yi - this->pos_.second + 0.5);

      // RCLCPP_INFO(this->get_logger(), "\n info %f %f %f", r_ang, new_angle, l2);
      // RCLCPP_INFO(this->get_logger(), "xi yi cal %f %f ", angle, msg->ranges[i]);
      // RCLCPP_INFO(this->get_logger(), "Publishing xpos %f %f %d", xi, this->pos_.first, xpos);
      // RCLCPP_INFO(this->get_logger(), "ypos %f %f %d \n", yi, this->pos_.second, ypos);

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
  message.header.frame_id = "robot/costmap";
  message.header.stamp = msg->header.stamp;
  message.info.resolution = this->costmap_.Info().first;
  message.info.width = this->costmap_.Info().second;
  message.info.height = this->costmap_.Info().second;

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
  float z = msg->pose.pose.orientation.z;
  float w = msg->pose.pose.orientation.w;

  this->pos_ = std::make_pair(x,y);
  this->angle_=std::make_pair(z,w);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}