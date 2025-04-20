#include "Graph.h"

#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "../Point/Point.h"

void Graph::AddEdge(Point a, Point b, double weight, bool is_dynamic) {
  if (is_dynamic) {
    dynamic_edges_.emplace(a, b);
    dynamic_edges_.emplace(b, a);

    dynamic_vertices_.emplace(a);
    dynamic_vertices_.emplace(b);
  }
  
  edges_[a][b] = weight;
  edges_[b][a] = weight;  // Для неориентированного графа
}

const std::unordered_map<Point, double, Point::Hash>& Graph::GetNeighbors(
    const Point& node) {
  const std::unordered_map<Point, double, Point::Hash> empty;
  auto it = edges_.find(node);
  return it != edges_.end() ? it->second : empty;
}

void Graph::UpdateEdge(Point from, Point to, double new_weight) {
  if (dynamic_edges_.count(std::make_pair(from, to))) {
    edges_[from][to] = new_weight;
    edges_[to][from] = new_weight;
  }
}

bool Graph::IsDynamicEdge(const Point& from, const Point& to) const {
  return dynamic_edges_.count({from, to}) > 0;
}

bool Graph::VertexInDynamicEdge(const Point& point) const {
  return dynamic_vertices_.find(point) != dynamic_vertices_.end();
}

void Graph::AddCache(const Cache& cache) {
  cached_ways_[cache.GetStartPoint()].emplace_back(cache);

  auto way = cache.GetWay();

  for (const auto& point : way) {
    cached_points_.emplace(point);
  }

  caches_.emplace_back(cache);
}

const std::vector<Cache>& Graph::GetCaches(const Point& node) {
  return cached_ways_[node];
}

bool Graph::IsCached(const Point& node) {
  return cached_points_.count(node) > 0;
}
