#include "Cache.h"

#include <iostream>
#include <unordered_map>
#include <vector>

#include "../Point/Point.h"

Cache::Cache(Point start, Point finish,
             std::unordered_map<Point, Point, Point::Hash>& came_from,
             std::unordered_map<Point, double, Point::Hash>& g_values) {
  Point node = finish;
  int dynamics_points = 0;

  way_cost_ = g_values[node];
  while (node == start || came_from.count(node)) {
    if (node.in_dynamic_edge) {
      dynamics_points++;
      if (dynamics_points == 2) {
        break;
      }
    }
    way_.push_back(node);
    way_length_++;

    if (node == start) {
      break;
    }
    node = came_from[node];
  }

  std::reverse(way_.begin(), way_.end());
}

size_t Cache::GetLength() const { return way_length_; }

double Cache::GetCost() const { return way_cost_; }

const std::vector<Point>& Cache::GetWay() const { return way_; }

Point Cache::GetStartPoint() const { return way_length_ > 0 ? way_[0] : Point(0, 0); }

Point Cache::GetEndPoint() const { return way_length_ > 0 ? way_[way_length_ - 1] : Point(0, 0); }
