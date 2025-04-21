#include "Dijkstra.h"

#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

#include "Graph/Graph.h"
#include "Point/Point.h"

std::vector<Point> Dijkstra(Graph& gr, const Point& start, const Point& end) {
  std::unordered_map<Point, double, Point::Hash> dist;
  std::unordered_map<Point, Point, Point::Hash> came_from;
  std::priority_queue<std::pair<double, Point>,
                      std::vector<std::pair<double, Point>>,
                      std::greater<std::pair<double, Point>>>
      dist_queue;

  dist[start] = 0;
  dist_queue.emplace(0, start);

  while (!dist_queue.empty()) {
    auto f = dist_queue.top();
    Point choose_vertex = f.second;
    dist_queue.pop();

    if (f.first != dist[choose_vertex]) {
      continue;
    }

    for (const auto& e : gr.GetNeighbors(choose_vertex)) {
      Point to = e.first;
      double d = dist[choose_vertex] + e.second;

      if (d < dist[to] || (dist[to] == 0 && to != start)) {
        dist[to] = d;
        dist_queue.emplace(d, to);

        came_from[to] = choose_vertex;
      }
    }
  }

  std::vector<Point> path;
  Point node = end;

  while (node == start || came_from.count(node)) {
    path.emplace_back(node);
    if (node == start) {
      break;
    }
    node = came_from[node];
  }
  std::reverse(path.begin(), path.end());
  return path;
}