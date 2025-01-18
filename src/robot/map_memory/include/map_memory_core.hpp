#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger, const float resolution, const int size);
    
    int getLength();
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
