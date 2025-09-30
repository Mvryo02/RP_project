#pragma once
#include <vector> 
#include <nav_msgs/OccupancyGrid.h>;

class Costmap{
public:
        Costmap();
        void fromOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
	void computeDistanceCosts();
        int getWidth() const {return width_;}
        int getHeight() const {return height_;}
        float getResolution() const {return resolution_;}
        int getCost(int x, int y) const;
        void printDebug() const;
private:
        int height_, width_;
        float resolution_;
        std::vector<int> data_;
        int getIndex(int x, int y) const {return y*width_+x;}

};
 
