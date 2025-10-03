#include "simple_planner/costmap.hpp"
#include <iostream>
#include <queue> 
#include <cmath>


Costmap::Costmap() {
 width_= 0;
 height_=0;
 resolution_=0.0; 
}

void  Costmap::fromOccupancyGrid(const nav_msgs::OccupancyGrid& map){
	width_ = map.info.width;
	height_ = map.info.height;
	resolution_ = map.info.resolution;
	data_.resize(width_*height_);

	for(int y = 0; y<height_; y++){
		for(int x=0; x<width_; x++){
			int idx = y*width_+x;
			int occ = map.data[idx];

			if(occ == 0){
				data_[idx] = 0;
			}
			else if(occ = 100){
				data_[idx] = 255;
			}
			else{
				data_[idx] = 255;
			}
		}
	}

}
void Costmap::computeDistanceCosts() {
    std::queue<std::pair<int,int>> q;
    std::vector<int> dist(width_ * height_, -1);

   
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            int idx= width_*y+x;
            if (data_[idx] == 255) { // obstacle or unknown
                dist[idx] = 0;
                q.push({x, y});
            }
        }
    }

   
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();
        int cidx = width_*cy+cx;

        for (int k = 0; k < 4; k++) {
            int nx = cx + dx[k];
            int ny = cy + dy[k];
            if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;
            int nidx = width_*ny+nx;
            if (dist[nidx] == -1 && data_[nidx] != 255) {
                dist[nidx] = dist[cidx] + 1;
                q.push({nx, ny});
            }
        }
    }

   
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            int idx = width_*y+x;
            if (data_[idx] == 255) continue; 
            if (data_[idx] == 128) continue; 
           if (dist[idx] >= 0) {
                int cost = std::max(0, 255 - dist[idx] * 10);
                data_[idx] = cost;
            }
        }
    }
}

int Costmap::getCost(int x, int y) const {
	if(x<0||x>=width_||y<0||y>=height_)
		return 255;
	return data_[y*width_+x];
}

void Costmap::printDebug() const {
	for(int x = 0; x<width_; x++)
		for(int y = 0; y<height_; y++){
			std::cout<<(getCost(x,y));
		}
		std::cout<< std::endl;
}


