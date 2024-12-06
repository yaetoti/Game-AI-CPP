#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <array>
#include <cassert>

template <typename T>
struct Vec2 final {
  T x;
  T y;

  explicit Vec2(T x = T(0), T y = T(0))
  : x(x)
  , y(y)
  {
  }

  Vec2(const Vec2& other) = default;
  Vec2(Vec2&& other) = default;
  Vec2& operator=(const Vec2& other) = default;
  Vec2& operator=(Vec2&& other) = default;

  bool operator==(const Vec2& other) const = default;

  template <typename U>
  explicit Vec2(const Vec2<U>& other)
  : x(T(other.x))
  , y(T(other.y))
  {
  }

  template <typename U>
  Vec2& operator=(const Vec2<U>& other) {
    if (this == &other) {
      return *this;
    }

    this.x = T(other.x);
    this.y = T(other.y);
    return *this;
  }

  Vec2 operator+(const Vec2& rhs) {
    return Vec2 { x + rhs.x, y + rhs.y };
  }

  Vec2 operator-(const Vec2& rhs) {
    return Vec2 { x - rhs.x, y - rhs.y };
  }

  Vec2 operator*(const Vec2& rhs) {
    return Vec2 { x * rhs.x, y * rhs.y };
  }

  Vec2 operator/(const Vec2& rhs) {
    return Vec2 { x / rhs.x, y / rhs.y };
  }
};

using Vec2f = Vec2<float>;
using Vec2i = Vec2<int>;


const char* kMap = "map.txt";

enum class Block {
  AIR,
  WALL,
  COUNT
};

struct NeighboursSet final {
  std::array<Vec2i, 4> neighbours;
  uint32_t count = 0;
};

struct Map final {
  size_t width = 0;
  size_t height = 0;
  std::vector<Block> blocks;

  Block At(Vec2i coords) const {
    assert(IsValid(coords));
    return blocks[coords.x + coords.y * width];
  }

  [[nodiscard]] bool IsValid(Vec2i coords) const {
    return coords.x >= 0 && coords.x < width && coords.y >= 0 && coords.y < height;
  }

  [[nodiscard]] bool IsPassable(Vec2i coords) const {
    return IsValid(coords) && At(coords) == Block::AIR;
  }

  [[nodiscard]] NeighboursSet GetNeighbours(Vec2i cell) const {
    assert(IsValid(cell));

    NeighboursSet result;
    Vec2i coords;

    coords = cell + Vec2i(1, 0);
    if (IsValid(coords)) {
      result.neighbours[result.count++] = coords;
    }
    coords = cell + Vec2i(-1, 0);
    if (IsValid(coords)) {
      result.neighbours[result.count++] = coords;
    }
    coords = cell + Vec2i(0, 1);
    if (IsValid(coords)) {
      result.neighbours[result.count++] = coords;
    }
    coords = cell + Vec2i(0, -1);
    if (IsValid(coords)) {
      result.neighbours[result.count++] = coords;
    }

    return result;
  }

  [[nodiscard]] NeighboursSet GetPassableNeighbours(Vec2i cell) const {
    assert(IsValid(cell));

    NeighboursSet result;
    Vec2i coords;

    coords = cell + Vec2i(1, 0);
    if (IsPassable(coords)) {
      result.neighbours[result.count++] = coords;
    }
    coords = cell + Vec2i(-1, 0);
    if (IsPassable(coords)) {
      result.neighbours[result.count++] = coords;
    }
    coords = cell + Vec2i(0, 1);
    if (IsPassable(coords)) {
      result.neighbours[result.count++] = coords;
    }
    coords = cell + Vec2i(0, -1);
    if (IsPassable(coords)) {
      result.neighbours[result.count++] = coords;
    }

    return result;
  }
};

std::unique_ptr<Map> ParseMap(const std::string& path) {
  std::ifstream in(path, std::ios::in);
  if (!in) {
    std::cout << "Can't open file\n";
    return nullptr;
  }

  std::vector<std::string> lines;

  // Input
  {
    std::string line;
    size_t mapWidth = -1;

    while (std::getline(in, line)) {
      // Check size
      if (mapWidth == -1) {
        mapWidth = line.size();
      }
      else if (mapWidth != line.size()) {
        std::cout << "Invalid size\n";
        return nullptr;
      }

      // Validation
      for (char c : line) {
        if (c != ' ' && c != '#') {
          std::cout << "Invalid symbol\n";
          return nullptr;
        }
      }

      lines.emplace_back(line);
    }

    if (!in.eof()) {
      std::cout << "Stream failed\n";
      return nullptr;
    }
  }

  // Parsing
  std::unique_ptr<Map> map = std::make_unique<Map>();
  map->width = lines[0].size();
  map->height = lines.size();
  map->blocks.resize(map->width * map->height);
  size_t block = 0;
  for (const std::string& line : lines) {
    for (char c : line) {
      switch (c) {
        case ' ': {
          map->blocks[block++] = Block::AIR;
          break;
        };
        case '#': {
          map->blocks[block++] = Block::WALL;
          break;
        }
        default: {
          std::cout << "Invalid symbol WTF\n";
          return nullptr;
        }
      }
    }
  }

  return map;
}

struct AStarNode;

struct AStarNode final {
  Vec2i coords;
  int gCost;
  int hCost;
  int fCost;
  Vec2i parent;

  AStarNode() = default;
  AStarNode(Vec2i coords, int gCost, int hCost, Vec2i parent)
  : coords(coords)
  , gCost(gCost)
  , hCost(hCost)
  , fCost(gCost + hCost)
  , parent(parent) {
  }
};

int ManhattanDistance(Vec2i v1, Vec2i v2) {
  return std::abs(v2.x - v1.x) + std::abs(v2.y - v1.y);
}

std::vector<Vec2i> FindPath(const Map& map, Vec2i start, Vec2i finish) {
  // open nodes
  // closed nodes

  // node:
  // coordinate
  // parent (for backtracking) (or many parents)
  // g-cost (go), h-cost (heuristics), f-cost (full)

  // vector of 'touched' nodes. Store indexes or index by coordinate

  // algo^
  // Pre-loop:
  // push start node to open

  // Loop:
  // get open node with lowest f-cost
  // if it's a finish - then backtrack a path
  // otherwise - get all neighbours.
  // For each: if already closed - don't touch
  // For each: if in open and calculated f-cost or g-cost is lower - update cost and parent
  // For each: else - calculate cost and put into open

  // Post-loop
  // Create a vector for path coordinates
  // Go from finish to start using parent pointers, while adding nodes into a vector
  // Reverse a vector (or insert to 0 from the beginning)

  // Operations:
  // Get node in open with lowest f-cost. If many equivalent - get the first with lowest g-cost
  // Determine if the node is in closed


  // xdd actions:
  // - check if start == finish. You shouldn't do that because you already do that for every open node

  // Notes:
  // Use vector. Implement sort and search by yourself to improve your understanding of what you need


  std::vector<AStarNode> open;
  std::vector<AStarNode> closed;
  std::vector<Vec2i> path;

  open.emplace_back(start, 0, ManhattanDistance(start, finish), start);

  while (!open.empty()) {
    std::ranges::sort(open, [](const AStarNode& left, const AStarNode& right) {
      if (left.fCost == right.fCost) {
        return left.gCost < right.gCost;
      }

      return left.fCost < right.fCost;
    });

    AStarNode current = open.front();
    open.erase(open.begin());
    closed.push_back(current);

    if (current.coords == finish) {
      // Backtrace path
      path.emplace(path.begin(), current.coords);

      AStarNode* tracedNode = &current;
      Vec2i currentCoords = tracedNode->coords;
      Vec2i parentCoords = tracedNode->parent;

      while (currentCoords != parentCoords) {
        // Find parent in closed by coords
        auto it = std::ranges::find_if(closed, [parentCoords](const AStarNode& node) {
          return node.coords == parentCoords;
        });
        assert(it != closed.end());
        tracedNode = &(*it);
        // Save coords and add parent node
        currentCoords = tracedNode->coords;
        parentCoords = tracedNode->parent;
        path.emplace(path.begin(), tracedNode->coords);
      }

      return path;
    }

    auto neighbours = map.GetPassableNeighbours(current.coords);
    for (uint32_t i = 0; i < neighbours.count; ++i) {
      Vec2i currentNeighbour = neighbours.neighbours[i];
      // In closed -  Don't touch
      {
        auto it = std::ranges::find_if(closed, [currentNeighbour](const AStarNode& node) {
          return node.coords == currentNeighbour;
        });

        if (it != closed.end()) {
          continue;
        }
      }

      // Calculate gCost and hCost
      int gCost = current.gCost + 1;
      int hCost = ManhattanDistance(currentNeighbour, finish);
      int fCost = gCost + hCost;

      // In open and lesser
      {
        auto it = std::ranges::find_if(open, [currentNeighbour](const AStarNode& node) {
          return node.coords == currentNeighbour;
        });
        if (it != open.end()) {
          AStarNode& neighborOpenNode = *it;
          if (neighborOpenNode.fCost > fCost || (neighborOpenNode.fCost == fCost && neighborOpenNode.gCost > gCost)) {
            neighborOpenNode.gCost = gCost;
            neighborOpenNode.fCost = fCost;
            neighborOpenNode.parent = current.coords;
            continue;
          }
        }
      }

      // Else - add to open nodes
      open.emplace_back(currentNeighbour, gCost, hCost, current.coords);
    }
  }

  return path;

  // no path found


  // Problems:
  // Touched actually needed if I need to sort afterwards, because parent is pointing to an address
  // But minheap also changes addresses, no?
}

int main() {
  auto map = ParseMap("map.txt");
  auto path = FindPath(*map, Vec2i{ 1, 1 }, Vec2i{10, 10});

  return 0;
}
