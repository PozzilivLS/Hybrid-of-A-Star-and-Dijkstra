#include "Graph.h"

#include <unordered_map>
#include <unordered_set>

#include "../Point/Point.h"

void Graph::AddEdge(Point a, Point b, bool is_dynamic) {
  double weight = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));

  if (is_dynamic) {
    dynamic_edges_.emplace(a, b);
    dynamic_edges_.emplace(b, a);
  }

  edges_[a][b] = weight;
  edges_[b][a] = weight;  // Для неориентированного графа
}

const std::unordered_map<Point, double, Point::Hash>& Graph::GetNeighbors(const Point& node) {
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
