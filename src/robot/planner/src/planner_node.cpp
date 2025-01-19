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
  bool operator()(const AStarNode a, const AStarNode b)
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
    std::chrono::milliseconds(500), std::bind(&PlannerNode::publishPath, this)
    );

  this->r = -1;
  this->l = -1;
  this->goal_point = std::make_pair(-1,-1);

}

void PlannerNode::publishPath(){

  if(this->r == -1 || this->l == -1 ){
    return;
  }
  if(this->goal_point.first == -1 || this->goal_point.second == -1){
    return;
  }

  int length = this->l;
  float res = this->r;
  float l2 = length/2;


  int start_x = (int) l2 + (pos.first / res);
  int start_y = (int) l2 + (pos.second / res);
  if(start_x < 0){
    start_x = 0;
  } else if(start_x >= length){
    start_x = length - 1;
  }
  if(start_y < 0){
    start_y = 0;
  } else if(start_y >= length){
    start_y = length - 1;
  }

  int end_x = (int) l2 + (goal_point.first / res);
  int end_y = (int) l2 + (goal_point.second / res);
  if(end_x < 0){
    end_x = 0;
  } else if(end_x >= length){
    end_x = length - 1;
  }
  if(end_y < 0){
    end_y = 0;
  } else if(end_y >= length){
    end_y = length - 1;
  }



  std::vector<CellIndex> path;
  std::vector<std::vector<bool>> visited(length, std::vector<bool>(length, false));
  std::vector<std::vector<CellIndex>> parent(length, std::vector<CellIndex>(length, CellIndex(-1, -1)));
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  CellIndex start = CellIndex(start_x, start_y);
  CellIndex end = CellIndex(end_x, end_y);
  std::vector<std::vector<int>> scores(length, std::vector<int>(length, -1));

  scores[start_y][start_x] = this->map[start_y][start_x];
  open_set.push(AStarNode(start, scores[start_y][start_x]));

  RCLCPP_INFO(this->get_logger(), "Start: %f %f %d, %d", this->pos.first, this->pos.second, start_x, start_y);
  while(!open_set.empty()){

    AStarNode current = open_set.top();
    open_set.pop();
    visited[current.index.y][current.index.x] = true;

    if(current.index == end){
      CellIndex current_index = current.index;
      while(current_index != start){
        path.push_back(current_index);
        current_index = parent[current_index.y][current_index.x];
      }
      break;
    }

    std::vector<CellIndex> coords = {
      CellIndex(current.index.x + 1, current.index.y),
      CellIndex(current.index.x - 1, current.index.y),
      CellIndex(current.index.x, current.index.y + 1),
      CellIndex(current.index.x, current.index.y - 1)
    };

    for(auto c: coords){
      if(c.x < 0 || c.x >= length || c.y < 0 || c.y >= length) continue;
      if(visited[c.y][c.x]) continue;

      double cost = scores[current.index.y][current.index.x] + sqrt(pow(c.x - current.index.x, 2) + pow(c.y - current.index.y, 2));

      if(scores[c.y][c.x] == -1 || cost < scores[c.y][c.x]){
        parent[c.y][c.x] = current.index;
        scores[c.y][c.x] = cost + this->map[c.y][c.x];
        double f = cost + sqrt(pow(c.x - end_x, 2) + pow(c.y - end_y, 2));
        open_set.push(AStarNode(c, f));
      }
    }
  }


  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "sim_world";
  path_msg.header.stamp = this->now();

  reverse(path.begin(), path.end());
  for(CellIndex node: path){
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "sim_world";
    p.pose.position.x = (node.x - l2) * res;
    p.pose.position.y = (node.y - l2) * res;

    path_msg.poses.push_back(p);
  }
  pathPub_->publish(path_msg);
}


void PlannerNode::readMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  std::vector<signed char> map2 = msg->data;
  this->l = msg->info.width;
  this->r = msg->info.resolution;
  int length = this->l;
  std::vector<std::vector<signed char>> tmp(length, std::vector<signed char>(length, 0));


  for(int i = 0; i < length; i++){
    for(int k = 0; k < length; k++){
      tmp[i][k]= map2[i*length + k];
    }
  }
  this->map = tmp;

}

void PlannerNode::readGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg){

  float x = msg->point.x;
  float y = msg->point.y;

  this->goal_point = std::make_pair(x, y);
}

void PlannerNode::readOdom(const nav_msgs::msg::Odometry::SharedPtr msg){


  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;

  this->pos = std::make_pair(x, y);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
