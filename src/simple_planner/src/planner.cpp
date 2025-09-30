#include "simple_planner/planner.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>


struct Node{
	int x, y;
	double  g, h;
	Node* parent;
};

Planner::Planner(const Costmap& costmap): costmap_(costmap) {}

std::vector<std::pair<int, int>> Planner::makePlan(int start_x, int start_y, int goal_x, int goal_y)
{
	auto cmp = [](Node* a, Node* b){return (a->g+a->h)>(b->g+b->h);};
	std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

	std::unordered_map<int, bool> closed;

	Node* start = new Node{start_x, start_y, 0, hypot(goal_x-start_x, goal_y-start_y), nullptr};

	open.push(start);
	
	int dx[4] = {1,-1,0,0};
	int dy[4] = {0, 0, 1, -1};
	Node* goal_node = nullptr;
	
	while(!open.empty()){
		Node* current = open.top();
		open.pop();
		int idx = current->y*costmap_.getWidth()+current->x;
		if(closed[idx]) continue;
		closed[idx] = true;

	

		if(current->x == goal_x && current->y == goal_y){
			goal_node = current;
			break;
		}

		for(int k = 0; k<4;k++){
			int nx = current->x +dx[k];
			int ny = current->y+dy[k];
			if(nx<0||nx>costmap_.getWidth()||ny<0||ny>costmap_.getHeight()) continue;
			int cost = costmap_.getCost(nx, ny);

			if(cost>=255) continue;
			double g_new = current->g +1.0 + cost/255.0;
			double h_new = hypot(goal_x-nx, goal_y-ny);
			Node* neighbor = new Node{nx, ny, g_new, h_new, current};
			open.push(neighbor);
		}
	}
	std::vector<std::pair<int,int>> path;
	if(goal_node){
		Node* n= goal_node;
		while(n){
			path.push_back({n->x, n->y});
			n = n->parent;
		}
		std::reverse(path.begin(), path.end());
	}
	return path;
}


