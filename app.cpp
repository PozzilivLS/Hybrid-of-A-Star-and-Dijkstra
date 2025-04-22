#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>
#include <iomanip>

#include <time.h>

#include "Cache/Cache.h"
#include "Graph/Graph.h"
#include "Point/Point.h"
#include "Dijkstra.h"

// Эвристическая функция (манхэттенское расстояние)
double Heuristic(const Point& a, const Point& b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

std::vector<Point> HybridAStarDijkstra(Graph& graph, const Point& start,
                                       const Point& goal,
                                       bool with_cached = false) {
  // Приоритетная очередь: пары (f_value, точка)
  std::priority_queue<std::pair<double, Point>, std::vector<std::pair<double, Point>>,
                      std::greater<std::pair<double, Point>>>
      open_set;
  open_set.push({0, start});

  std::unordered_map<Point, Point, Point::Hash> came_from;
  std::unordered_map<Point, double, Point::Hash> g_values;
  g_values[start] = 0;

  while (!open_set.empty()) {
    double value = open_set.top().first;
    auto current = open_set.top().second;
    open_set.pop();

    current.in_dynamic_edge = graph.VertexInDynamicEdge(current);

    if (current == goal) {
      // Кэширование пути до goal
      //Cache new_cache{start, current, came_from, g_values};
      //graph.AddCache(new_cache);

      // Восстановление пути
      std::vector<Point> path;
      Point node = goal;
      while (node != start || came_from.count(node)) {
        path.push_back(node);
        node = came_from[node];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());

      std::cout << "(Длина пути - " << g_values[goal] << ") ";
      return path;
    }

    if (with_cached) {
      for (const auto& neighbor_caches : graph.GetCaches(current)) {
        double new_g = g_values[current] + neighbor_caches.GetCost();
        if (!g_values.count(neighbor_caches.GetEndPoint()) ||
            new_g < g_values[neighbor_caches.GetEndPoint()]) {
          g_values[neighbor_caches.GetEndPoint()] = new_g;

          // Конструируем путь
          auto way = neighbor_caches.GetWay();
          for (int i = 1; i < way.size(); i++) {
            came_from[way[i]] = way[i - 1];
          }
        }
        double f_value = new_g + Heuristic(neighbor_caches.GetEndPoint(), goal);

        open_set.push({f_value, neighbor_caches.GetEndPoint()});
      }
    }

    for (const auto& neighbor_pair : graph.GetNeighbors(current)) {
      Point neighbor = neighbor_pair.first;
      double edge_cost = neighbor_pair.second;
      double new_g = g_values[current] + edge_cost;

      neighbor.in_dynamic_edge = graph.VertexInDynamicEdge(neighbor);

      /*if (!graph.IsDynamicEdge(current, neighbor) && graph.IsCached(current) &&
          graph.IsCached(neighbor) && with_cached) {
        continue;
      }*/
      if (!g_values.count(neighbor) || new_g < g_values[neighbor]) {
        came_from[neighbor] = current;
        g_values[neighbor] = new_g;

        double f_value;
        if (graph.IsDynamicEdge(current, neighbor)) {
          // Режим Дейкстры (h=0)
          f_value = new_g;

          // Кэшируем путь до ребра
          //Cache new_cache{start, current, came_from, g_values};
          //graph.AddCache(new_cache);

        } else {
          // Режим A* с эвристикой
          f_value = new_g + Heuristic(neighbor, goal);
        }

        open_set.push({f_value, neighbor});
      }
    }

    if (open_set.empty() && with_cached) {
      return HybridAStarDijkstra(graph, start, goal, false);
    }
  }
  return {};  // Путь не найден
}

int main() {
  setlocale(LC_ALL, "Russian");
  srand(time(NULL));
  std::cout << std::fixed << std::setprecision(5);

  Graph graph;

  std::vector<std::pair<Point, Point>> dynamic_points;
  for (int i = 0; i < 10000000; i++) {
    int x1 = std::rand() % 300;
    int y1 = std::rand() % 300;
    int x2 = std::rand() % 300;
    int y2 = std::rand() % 300;
    bool is_dynamic = (std::rand() % 100) > 95;

    if (is_dynamic) {
      dynamic_points.emplace_back(Point(x1, y1), Point(x2, y2));
    }

    graph.AddEdge(Point(x1, y1), Point(x2, y2), is_dynamic);
  }

  // Ищем путь из (x1, y1) в (x2, y2)
  int x1 = std::rand() % 300;
  int y1 = std::rand() % 300;
  int x2 = std::rand() % 300;
  int y2 = std::rand() % 300;
  Point start(x1, y1);
  Point goal(x2, y2);

  clock_t tStart = clock();

  auto path = HybridAStarDijkstra(graph, start, goal);

  clock_t tEnd = clock();

  std::cout << "Найденный путь гибридным алгоритмом: ";
  for (const auto& p : path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }
  std::cout << std::endl;
  std::cout << "Время работы: " << (int)(tEnd - tStart) << std::endl;

  tStart = clock();
  path = Dijkstra(graph, start, goal);
  tEnd = clock();
  std::cout << "Найденный путь Дейкстрой: ";
  for (const auto& p : path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }
  std::cout << std::endl;
  std::cout << "Время работы: " << (int)(tEnd - tStart) << std::endl;

  // Меняем вес динамических ребер
  for (const auto& edge : dynamic_points) {
    graph.UpdateEdge(edge.first, edge.second, std::rand() % 40);
  }

  // Ищем путь снова
  tStart = clock();
  auto new_path = HybridAStarDijkstra(graph, start, goal);
  tEnd = clock();

  std::cout << "Новый путь после изменения графа гибридным алгоритмом: ";
  for (const auto& p : new_path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }
  std::cout << std::endl;
  std::cout << "Время работы: " << (int)(tEnd - tStart) << std::endl;

  tStart = clock();
  new_path = Dijkstra(graph, start, goal);
  tEnd = clock();
  std::cout << "Найденный путь Дейкстрой: ";
  for (const auto& p : new_path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }
  std::cout << std::endl;
  std::cout << "Время работы: " << (int)(tEnd - tStart) << std::endl;

  return 0;
}