#include "map_memory_core.hpp"

namespace robot
{

int MapMemoryCore::getLength(){
    int length_ = (int) this->size_ / this->resolution;
    length_++; // make odd for (0,0)
    return length_;
}

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger, const float resolution, const int size_) 
  : logger_(logger) {
    this->logger_ = logger;
    this->resolution = resolution;
    this->size_ = size_;
    int length_ = this->getLength();
    std::vector<std::vector<int>> OG(length_, std::vector<int>(length_, 0));
    this->OccupancyGrid = OG;
  }


void MapMemoryCore::UpdatePos(const int i, const int j, const int val){
    this->OccupancyGrid[j][i] = std::max(this->OccupancyGrid[j][i], val);
}

std::pair<float,int> MapMemoryCore::Info(){
    std::pair<float,int> info = std::make_pair(this->resolution,this->size_);
    return info;
}

} 
