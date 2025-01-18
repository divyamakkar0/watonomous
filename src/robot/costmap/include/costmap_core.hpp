#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "vector"
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger, const float resolution, const int size);
    
    int getLength();
    void Initalize();
    void UpdatePos(const int i, const int j, const int val);
    std::pair<float,int> Info();
    std::vector<std::vector<int>> OccupancyGrid;
    float resolution;
    int size_;

  private:
    rclcpp::Logger logger_;
    

};

}  

#endif  