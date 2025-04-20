#include <unordered_map>
#include <vector>

#include "../Point/Point.h"

#ifndef CACHE
#define CACHE

class Cache {
 public:
  Cache(Point start, Point finish,
        std::unordered_map<Point, Point, Point::Hash>& came_from,
        std::unordered_map<Point, double, Point::Hash>& g_values);

  size_t GetLength() const;

  double GetCost() const;

  const std::vector<Point>& GetWay() const;

  Point GetStartPoint() const;

  Point GetEndPoint() const;

  struct Hash {
    size_t operator()(const Cache& ch) const {
      return std::hash<int>()(ch.way_length_) ^ (std::hash<int>()(ch.way_cost_) << 1);
    }
  };

 private:
  std::vector<Point> way_;
  int way_length_ = 0;
  double way_cost_ = 0;
};

#endif CACHE