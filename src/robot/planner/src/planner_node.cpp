#include "planner_node.hpp"
using std::placeholders::_1;

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_readings = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::readMap, this, _1));
  goal_readings = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::readGoal, this, _1));
  odom_readings = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::readOdom, this, _1));
  pathPub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  //TO-DO: bind the timer to function - temp binded to publishPath
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&MapMemoryNode::publishPath, this));
}

//updarte cmake
void PlannerNode::readMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  std::vector<signed char> map = msg->data;

}

void PlannerNode::readGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  this->goal_point = std::make_pair(msg->point.x, msg->point.y);

}

void PlannerNode::readOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
  this->pos = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
}

void

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
