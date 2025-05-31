#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>
#include <iomanip>

#include <time.h>

#include "Graph/Graph.h"
#include "Point/Point.h"
#include "Dijkstra.h"

// ������������� �������
double Heuristic(const Point& a, const Point& b) {
  double dx = abs(a.x - b.x);
  double dy = abs(a.y - b.y);
  return std::max(dx, dy);
}

std::vector<Point> HybridAStarDijkstra(Graph& graph, const Point& start,
                                       const Point& goal) {
  // ������������ �������: ���� (f_value, �����)
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

    if (current == goal) {
      // �������������� ����
      std::vector<Point> path;
      Point node = goal;
      while (node != start) {
        path.push_back(node);
        node = came_from[node];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      std::cout << "(����� ���� - " << g_values[goal] << ") ";
      return path;
    }

    for (const auto& neighbor_pair : graph.GetNeighbors(current)) {
      Point neighbor = neighbor_pair.first;
      double edge_cost = neighbor_pair.second;
      double new_g = g_values[current] + edge_cost;

      if (!g_values.count(neighbor) || new_g < g_values[neighbor]) {
        came_from[neighbor] = current;
        g_values[neighbor] = new_g;

        double f_value;
        if (graph.IsDynamicEdge(current, neighbor)) {
          // ����� �������� (h=0)
          f_value = new_g;
        } else {
          // ����� A* � ����������
          f_value = new_g + Heuristic(neighbor, goal);
        }

        open_set.push({f_value, neighbor});
      }
    }
  }
  return {};  // ���� �� ������
}

int main() {
  setlocale(LC_ALL, "Russian");
  srand(time(NULL));
  std::cout << std::fixed << std::setprecision(5);

  Graph graph;

  std::vector<std::pair<Point, Point>> dynamic_points;
  for (int i = 0; i < 10000000; i++) {
    int x1 = std::rand() % 1000;
    int y1 = std::rand() % 1000;
    int x2 = std::rand() % 1000;
    int y2 = std::rand() % 1000;
    bool is_dynamic = (std::rand() % 100) > 95;

    if (is_dynamic) {
      dynamic_points.emplace_back(Point(x1, y1), Point(x2, y2));
    }

    graph.AddEdge(Point(x1, y1), Point(x2, y2), is_dynamic);
  }

  // ���� ���� �� (x1, y1) � (x2, y2)
  int x1 = std::rand() % 1000;
  int y1 = std::rand() % 1000;
  int x2 = std::rand() % 1000;
  int y2 = std::rand() % 1000;
  Point start(x1, y1);
  Point goal(x2, y2);

  std::cout << "��������� ��������:\t";

  clock_t tStart = clock();

  auto path = HybridAStarDijkstra(graph, start, goal);

  clock_t tEnd = clock();


  std::cout << "��������� ���� ��������� ����������: ";
  for (const auto& p : path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }
  std::cout << "����� ������: " << (int)(tEnd - tStart) << std::endl;

  std::cout << "��������:\t";
  tStart = clock();
  path = Dijkstra(graph, start, goal);
  tEnd = clock();
  std::cout << "��������� ���� ���������: ";
  for (const auto& p : path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }
  std::cout << "����� ������: " << (int)(tEnd - tStart) << std::endl;

  
  std::cout << "���������� ����...\n";
  // ������ ��� ������������ �����
  for (const auto& edge : dynamic_points) {
    graph.UpdateEdge(edge.first, edge.second, std::rand() % 40);
  }

  std::cout << "��������� ��������:\t";
  // ���� ���� �����
  tStart = clock();
  auto new_path = HybridAStarDijkstra(graph, start, goal);
  tEnd = clock();

  //std::cout << "����� ���� ����� ��������� ����� ��������� ����������: ";
  /*for (const auto& p : new_path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }*/
  std::cout << "����� ������: " << (int)(tEnd - tStart) << std::endl;

  std::cout << "��������:\t";
  tStart = clock();
  new_path = Dijkstra(graph, start, goal);
  tEnd = clock();
  //std::cout << "��������� ���� ���������: ";
  /*for (const auto& p : new_path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }*/
  std::cout << "����� ������: " << (int)(tEnd - tStart) << std::endl;

  std::cout << "���������� ����...\n";
  // ������ ��� ������������ �����
  for (const auto& edge : dynamic_points) {
    graph.UpdateEdge(edge.first, edge.second, std::rand() % 40);
  }

  std::cout << "��������� ��������:\t";
  // ���� ���� �����
  tStart = clock();
  new_path = HybridAStarDijkstra(graph, start, goal);
  tEnd = clock();

  //std::cout << "����� ���� ����� ��������� ����� ��������� ����������: ";
  /*for (const auto& p : new_path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }*/
  std::cout << "����� ������: " << (int)(tEnd - tStart) << std::endl;

  std::cout << "��������:\t";
  tStart = clock();
  new_path = Dijkstra(graph, start, goal);
  tEnd = clock();
  //std::cout << "��������� ���� ���������: ";
  /*for (const auto& p : new_path) {
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }*/
  std::cout << "����� ������: " << (int)(tEnd - tStart) << std::endl;

  return 0;
}