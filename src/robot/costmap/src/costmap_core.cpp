#include "costmap_core.hpp"

namespace robot
{

int CostmapCore::getLength(){
    int length_ = (int) this->size_ / this->resolution;
    return length_;
}

CostmapCore::CostmapCore(const rclcpp::Logger& logger, const float resolution, const int size_) : logger_(logger) {
    this->logger_ = logger;
    this->resolution = resolution;
    this->size_ = size_;
    int length_ = this->getLength(); 
    std::vector<std::vector<int>> OG(length_, std::vector<int>(length_, 0));
    this->OccupancyGrid = OG;  
}

void CostmapCore::Initalize(){
    for(auto &i: this->OccupancyGrid){
        for(int &k: i){
            k = 0; 
        }
    }
}
    
void CostmapCore::UpdatePos(const int i, const int j, const int val){
    this->OccupancyGrid[j][i] = std::max(this->OccupancyGrid[j][i], val);
}

std::pair<float,int> CostmapCore::Info(){
    std::pair<float,int> info = std::make_pair(this->resolution,this->size_);
    return info;
}



}