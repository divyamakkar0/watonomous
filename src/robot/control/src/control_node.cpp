#include "control_node.hpp"

#define PI 3.141590118408203
using std::placeholders::_1;

ControlNode::ControlNode(float lookahead_distance, float goal_tolerance, float linear_speed) : Node("control"), control_(robot::ControlCore(this->get_logger())) {

  // setup constants
  this->lookahead_distance = lookahead_distance;
  this->goal_tolerance = goal_tolerance;
  this->linear_speed_ = linear_speed;

  this->last_index = 0;

  // setup publishers and subscribers this->path_ = this->create_subscription<nav_msgs::msg::Path>("path", 10, std::bind(&ControlNode::pathCallback, this, _1));
  this->odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, _1));
  this->path_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, _1));
  this->cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  this->control_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ControlNode::controlLoop, this));
  this->position = std::make_pair(-1,-1);
}


void ControlNode::controlLoop() {

  if (this->path.empty()){

    RCLCPP_INFO(this->get_logger(), "No path received");
    return;
  }
  if(this->position.first == -1 || this->position.second == -1){
    RCLCPP_INFO(this->get_logger(), "No position received");
    return;
  }

  std::pair<float, float> goal = std::make_pair(this->path.back().pose.position.x, this->path.back().pose.position.y);
  std::pair<float, float> intial = std::make_pair(this->path.front().pose.position.x, this->path.front().pose.position.y);
  float distanceToGoal = this->computeDistance(this->position, goal);
  float intialDistance = this->computeDistance(intial, goal);

  std::pair<float, float> lookahead_point = this->findLookaheadPoint();

  if(lookahead_point.first == -1 && lookahead_point.second == -1){

    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Lookahead point: %f %f", lookahead_point.first, lookahead_point.second);
  this->computeVelocity(lookahead_point, distanceToGoal, intialDistance );

}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  this->path = msg->poses;
}

std::pair<float, float> ControlNode::findLookaheadPoint(){

  float currentX = this->position.first;
  float currentY = this->position.second;

  std::pair<float, float> lookahead_point = std::make_pair(-1, -1);
  bool intersection = false;
  int index = 0;

  std::vector<std::pair<float, float>> points;

  for(auto i: this->path){
    points.push_back(std::make_pair(i.pose.position.x, i.pose.position.y));
  }


  for(int i = 0; i < (int) points.size() - 1; i++){

    if (computeDistance(points[i], this->position) < this->lookahead_distance){
      index = i;
    }

  }
  lookahead_point = points[index];
  return lookahead_point;
}

void ControlNode::computeVelocity(std::pair<float, float> lookahead_point, float distanceToGoal, float intialDistance) {


  this->extractYaw();
  float abs_angle = atan2(lookahead_point.second - this->position.second, lookahead_point.first - this->position.first);
  float diff_angle = abs_angle - this->yaw;

  if (diff_angle > PI){
    diff_angle -= 2 * PI;
  }
  else if (diff_angle < -PI){
    diff_angle += 2 * PI;
  }

  geometry_msgs::msg::Twist msg;

  msg.angular.z = 0.2 * diff_angle;
  msg.linear.x = 0.2 *( PI - abs(diff_angle));

  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;

  if(distanceToGoal < this->goal_tolerance){
    msg.linear.x = 0;
    msg.angular.z = 0;
  }

  this->cmd_vel_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Linear x: %f, Linear y: %f, Angular z: %f", msg.linear.x, msg.linear.y, msg.angular.z);

}

float ControlNode::computeDistance(std::pair<float, float> p1, std::pair<float, float> p2) {
  return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;

  float qx = msg->pose.pose.orientation.x;
  float qy = msg->pose.pose.orientation.y;
  float qz = msg->pose.pose.orientation.z;
  float qw = msg->pose.pose.orientation.w;

  this->position = std::make_pair(x, y);
  this->angle = std::make_tuple(qx, qy, qz, qw);
}

void ControlNode::extractYaw(){
  float x = std::get<0>(this->angle);
  float y = std::get<1>(this->angle);
  float z = std::get<2>(this->angle);
  float w = std::get<3>(this->angle);

  float yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  this->yaw = yaw;
}

int main(int argc, char **argv)
{

  float lookahead_distance = 0.5;
  float goal_tolerance = 0.5;
  float linear_speed = 0.5;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(lookahead_distance, goal_tolerance, linear_speed));
  rclcpp::shutdown();
  return 0;
}
