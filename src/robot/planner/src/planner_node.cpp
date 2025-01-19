#include "planner_node.hpp"
using std::placeholders::_1;

struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_readings = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::readMap, this, _1));

  goal_readings = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::readGoal, this, _1));

  odom_readings = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::readOdom, this, _1));

  pathPub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&MapMemoryNode::publishPath, this));
}



void PlannerNode::publishPath(){

  // use a* to find path
  std::vector<CellIndex> path = aStar();
  CellIndex start = CellIndex((int)pos.first, (int)pos.second);
  CellIndex end = CellIndex((int)goal_point.first, (int)goal_point.second);


}


void PlannerNode::readMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  std::vector<signed char> map = msg->data;
  int length = msg->info.width;

  for(int i = 0; i < length; i++){
    for(int k = 0; k < length; k++){
      this->map[i][k] = map[i*length + k];
    }
  }

}

void PlannerNode::readGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  this->goal_point = std::make_pair(msg->point.x, msg->point.y);
}

void PlannerNode::readOdom(const nav_msgs::msg::Odometry::SharedPtr msg){
  this->pos = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
