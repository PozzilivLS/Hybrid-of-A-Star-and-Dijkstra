#include <functional>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../Cache/Cache.h"
#include "../Point/Point.h"

#ifndef GRAPH
#define GRAPH

class Graph {
 public:
  void AddEdge(Point a, Point b, double weight, bool is_dynamic = false);

  const std::unordered_map<Point, double, Point::Hash>& GetNeighbors(
      const Point& node);

  void UpdateEdge(Point from, Point to, double new_weight);

  bool IsDynamicEdge(const Point& from, const Point& to) const;

  bool VertexInDynamicEdge(const Point& point) const;

  void AddCache(const Cache& cache);

  const std::vector<Cache>& GetCaches(const Point& node);

  bool IsCached(const Point& node);
 private:
  // Хранение рёбер: {from: {to: weight}}
  std::unordered_map<Point, std::unordered_map<Point, double, Point::Hash>,
                     Point::Hash>
      edges_;
  std::unordered_set<Point, Point::Hash> dynamic_vertices_;

  // Хеш-функция для пары точек
  struct PairHash {
    size_t operator()(const std::pair<Point, Point>& p) const {
      return Point::Hash()(p.first) ^ Point::Hash()(p.second);
    }
  };

  std::unordered_set<std::pair<Point, Point>, PairHash> dynamic_edges_;
  std::unordered_map<Point, std::vector<Cache>, Point::Hash> cached_ways_;
  std::unordered_set<Point, Point::Hash> cached_points_;

  std::vector<Cache> caches_;
};
#endif  // !GRAPH