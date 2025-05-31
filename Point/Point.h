#include <functional>

#ifndef POINT
#define POINT

class Point {
 public:
  int x = 0;
  int y = 0;

  Point() = default;
  Point(int x, int y) : x(x), y(y) {};

  bool operator==(const Point& other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const Point& other) const {
    return x != other.x || y != other.y;
  }

  bool operator<(const Point& other) const {
    return x < other.x && y < other.y;
  }
  
  // Для использования Point в unordered_map
  struct Hash {
    size_t operator()(const Point& p) const {
      return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
  };
};

#endif  // !POINT