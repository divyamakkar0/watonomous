#include "control_node.hpp"

#define PI 3.141590118408203
using std::placeholders::_1;

ControlNode::ControlNode(float lookhead_distance, float goal_tolerance, float linear_speed) : Node("control"), control_(robot::ControlCore(this->get_logger())) {

  // setup constants
  this->lookahead_distance = lookahead_distance;
  this->goal_tolerance = goal_tolerance;
  this->linear_speed_ = linear_speed;

  this->lookahead_point = std::nullopt;
  this->last_index = 0;

  // setup publishers and subscribers this->path_ = this->create_subscription<nav_msgs::msg::Path>("path", 10, std::bind(&ControlNode::pathCallback, this, _1));
  this->odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&ControlNode::odomCallback, this, _1));
  this->cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  this->control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}


void ControlNode::controlLoop() {

  if (!((int) this->path.size() > 0)) {
    RCLCPP_INFO(this->get_logger(), "No path received");
    return;
  }
  if(!this->position.has_value()){
    RCLCPP_INFO(this->get_logger(), "No position received");
    return;
  }

  std::pair<float, float> lookahead_point = this->findLookaheadPoint();

  std::pair<float, float> lookhead_point = findLookaheadPoint();

  if (!lookhead_point.has_value()) {
    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found");
    return;
  }

  this->computeVelocity(lookahead_point);

}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  this->path = msg;
}

std::pair<float, float> ControlNode::findLookaheadPoint(){

  // ref: https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit

  float currentX = this->position.first;
  float currentY = this->position.second;

  std::pair<float, float> lookahead_point = std::nullopt;
  bool intersection = false;
  int starting_index = this->last_index;

  std::vector<std::pair<float, float>> points;

  for(auto i: this->path){
    points.push_back(std::make_pair(i.pose.position.x, i.pose.position.y));
  }
  for(int i = starting_index; i < (int) points.size() - 1; i++){

    float x1 = points[i].first - currentX;
    float y1 = points[i].second - currentY;
    float x2 = points[i + 1].first - currentX;
    float y2 = points[i + 1].second - currentY;
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dr = sqrt(pow(dx, 2) + pow(dy, 2));
    float D = x1 * y2 - x2 * y1;
    float discriminant = pow(this->lookahead_distance, 2) * pow(dr, 2) - pow(D, 2);

    if (discriminant >= 0){
      float sol_x1 = (D * dy + (dy < 0 ? -1 : 1) * dx * sqrt(discriminant)) / pow(dr, 2);
      float sol_x2 = (D * dy - (dy < 0 ? -1 : 1) * dx * sqrt(discriminant)) / pow(dr, 2);
      float sol_y1 = (-D * dx + abs(dy) * sqrt(discriminant)) / pow(dr, 2);
      float sol_y2 = (-D * dx - abs(dy) * sqrt(discriminant)) / pow(dr, 2);

      std::pair<float, float> sol1 = std::make_pair(sol_x1 + currentX, sol_y1 + currentY);
      std::pair<float, float> sol2 = std::make_pair(sol_x2 + currentX, sol_y2 + currentY);

      float minX = std::min(x1, x2);
      float maxX = std::max(x1, x2);
      float minY = std::min(y1, y2);
      float maxY = std::max(y1, y2);

      if ( (sol1.first >= minX && sol1.first <= maxX && sol1.second >= minY && sol1.second <= maxY) ||
           (sol2.first >= minX && sol2.first <= maxX && sol2.second >= minY && sol2.second <= maxY) ){
        intersection = true;

        if((sol1.first >= minX && sol1.first <= maxX && sol1.second >= minY && sol1.second <= maxY)
            && (sol2.first >= minX && sol2.first <= maxX && sol2.second >= minY && sol2.second <= maxY)){
          if (computeDistance(points[i+1], sol1) < computeDistance(points[i] + 1, sol2)){
            lookahead_point= sol1;
          } else {
            lookahead_point = sol2;
          }
        }
        else {
          if (sol1.first >= minX && sol1.first <= maxX && sol1.second >= minY && sol1.second <= maxY){
            lookahead_point = sol1;
          } else {
            lookahead_point = sol2;
          }
        }

      }

      if(computeDistance(points[i+1], this->lookahead_point) < computeDistance(this->position, points[i+1])){
        this->last_index = i;
        break;
      }
      else{
        this->last_index = i + 1;
      }
    }
    else{
      intersection = false;
      lookahead_pointt = points[this->last_index];
    }
  }

  return lookahead_point;

}

void ControlNode::computeVelocity(std::pair<float, float> lookahead_point) {

  this->extractYaw();
  float abs_angle = atan2(lookahead_point.second - this->position.second, lookahead_point.first - this->position.first);
  float diff_angle = abs_angle - this->yaw;

  if (diff_angle > M_PI){
    diff_angle -= 2 * PI;
  }
  else if (diff_angle < -M_PI){
    diff_angle += 2 * PI;
  }

  nav_msgs::msg::Twist msg;

  msg.linear.x = this->linear_speed_ * cos(diff_angle);
  msg.linear.y = this->linear_speed_ * sin(diff_angle);
  msg.angular.z = 0.1 * diff_angle;

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
  float goal_tolerance = 0.1;
  float linear_speed = 0.5;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(lookahead_distance, goal_tolerance, linear_speed));
  rclcpp::shutdown();
  return 0;
}
