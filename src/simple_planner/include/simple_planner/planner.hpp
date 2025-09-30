#pragma once
#include "costmap.hpp"
#include <vector>
#include <utility>

class Planner{
public:
	Planner(const Costmap& costmap);

	std::vector<std::pair<int,int>> makePlan(int start_x, int start_y, int goal_x, int goal_y);

private:

	const Costmap& costmap_;

};
