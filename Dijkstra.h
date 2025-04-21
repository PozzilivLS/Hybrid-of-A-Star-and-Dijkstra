#pragma once

#include <vector>

#include "Graph/Graph.h"
#include "Point/Point.h"
std::vector<Point> Dijkstra(Graph& gr, const Point& start, const Point& end);