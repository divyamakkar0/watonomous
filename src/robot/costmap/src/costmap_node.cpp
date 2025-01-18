#include <chrono>
#include <memory>
#include "costmap_node.hpp"

#define PI  3.141590118408203
using std::placeholders::_1;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger(), 1, 31)){
  // Initialize the constructs and their parameters
  og_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::readData, this, _1));
  position_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&CostmapNode::readPosition, this, _1));
}


void CostmapNode::inflatePosition(const int xpos, const int ypos){

    std::vector<std::pair<int,int>> v = {
      std::make_pair(xpos-1,ypos),
      std::make_pair(xpos+1,ypos),
      std::make_pair(xpos,ypos+1),
      std::make_pair(xpos,ypos-1),
      std::make_pair(xpos-1,ypos+1),
      std::make_pair(xpos-1,ypos-1),
      std::make_pair(xpos+1,ypos+1),
      std::make_pair(xpos+1,ypos-1)
    };

    for(auto i: v){
      int x = i.first; 
      int y = i.second; 
      int l = this->costmap_.getLength();
      if(x >= 0 && y >= 0 && x < l && y < l){
        float distance = sqrt(pow(this->pos_.first - x, 2) + pow(this->pos_.second - y, 2));
        float val = 100 * (1 - (distance/1.5));
        this->costmap_.UpdatePos(x,y,val); 
      }

    }
 
}

void CostmapNode::readData(const sensor_msgs::msg::LaserScan::SharedPtr msg)  {
  this->costmap_.Initalize();
  for (int i = 0;i < (int) msg->ranges.size(); i++){
    if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min){
      float angle = (msg->angle_min + i * msg->angle_increment);
 
      float z = this->angle_.first; 
      float w = this->angle_.second;
      float r_ang = 2 * asin(z);
      if(z == 0){
        r_ang = 0; 
      }

      if (z < 0){
        r_ang *= -1;
      }

      float new_angle = angle + r_ang;

      if (new_angle < -PI){
        new_angle += 2*PI;
      } else if (new_angle > PI){
        new_angle -= 2 * PI;
      }
      
      float xi = msg->ranges[i] * cos(new_angle);
      float yi = msg->ranges[i] * sin(new_angle);
      float l2 = this->costmap_.getLength() / 2; 
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

  int l = (int) (this->costmap_.size_ / this->costmap_.resolution);
  std::vector<signed char> a(l*l);

 for(int i = 0; i < l; i++){
    for(int k = 0; k < l; k++){
      a[i * l + k] = this->costmap_.OccupancyGrid[i][k]; 
    }
  }

  message.data = a; 
  og_->publish(message);



}

void CostmapNode::readPosition(const nav_msgs::msg::Odometry::SharedPtr msg){
  float x = msg->pose.pose.position.x / this->costmap_.resolution; 
  float y = msg->pose.pose.position.y / this->costmap_.resolution;
  float z = msg->pose.pose.orientation.z;
  float w = msg->pose.pose.orientation.w;
   RCLCPP_INFO(this->get_logger(), "POS x:%f y:%f", x,y);

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