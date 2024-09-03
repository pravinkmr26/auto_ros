#include <a_star_planner/a_star.hpp>

#include <algorithm>
#include <time.h>
#include <stdio.h>

using namespace std::placeholders;

bool AStar::Point2D::operator==(const Point2D &coordinates_) {
  return (x == coordinates_.x && y == coordinates_.y);
}

bool AStar::Point2D::operator!=(const Point2D &coordinates_) {
  return !(x == coordinates_.x && y == coordinates_.y);
}

AStar::Point2D operator+(const AStar::Point2D &left_, const AStar::Point2D &right_) {
  return {left_.x + right_.x, left_.y + right_.y};
}

AStar::Node::Node(Point2D coordinates_, Node *parent_) {
  parent = parent_;
  coordinates = coordinates_;
  G = H = 0;
}

AStar::uint AStar::Node::getScore() { return G + H; }

AStar::Generator::Generator() {
  setDiagonalMovement(false);
  setHeuristic(&Heuristic::manhattan);
  direction = {{0, 1},   {1, 0}, {0, -1}, {-1, 0},
               {-1, -1}, {1, 1}, {-1, 1}, {1, -1}};
}

void AStar::Generator::setWorldSize(Point2D worldSize_) {
  worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_) {
  directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_) {
  heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2);
}

bool AStar::Generator::detectCollision(Point2D coordinates_) {
  if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
      coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
      std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
    return true;
  }
  return false;
}

void AStar::Generator::addCollision(Point2D coordinates_) {
  if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
      coordinates_.y < 0 || coordinates_.y >= worldSize.y)
    return;

  if (!detectCollision(coordinates_))
    walls.push_back(coordinates_);
}

void AStar::Generator::addCollision(Point2D coordinates_, int size) {
  if (size == 0) {
    addCollision(coordinates_);
    return;
  }

  for (int i = 0; i < 4; i++) {
    Point2D new_coord = coordinates_ + direction[i];
    addCollision(new_coord, size - 1);
  }
}

void AStar::Generator::removeCollision(Point2D coordinates_) {
  auto it = std::find(walls.begin(), walls.end(), coordinates_);
  if (it != walls.end()) {
    walls.erase(it);
  }
}

void AStar::Generator::clearCollisions() { walls.clear(); }

AStar::CoordinateList AStar::Generator::findPath(Point2D source_, Point2D target_) {
  Node *current = nullptr;
  NodeSet openSet, closedSet;
  openSet.reserve(100);
  closedSet.reserve(100);
  openSet.push_back(new Node(source_));

  struct timespec start, end;
  //clock_gettime(CLOCK_MONOTONIC, &start);
  clock_gettime(CLOCK_MONOTONIC, &start);
  while (!openSet.empty()) {
    auto current_it = openSet.begin();
    current = *current_it;

    for (auto it = openSet.begin(); it != openSet.end(); it++) {
      auto node = *it;
      if (node->getScore() <= current->getScore()) {
        current = node;
        current_it = it;
      }
    }

    clock_gettime(CLOCK_MONOTONIC, &end);
    double time_result =
        end.tv_sec - start.tv_sec + (end.tv_nsec - start.tv_nsec) * 1e-9;

    if (time_result > 1.0) { // time failure limit
      printf("time: %lf\n", time_result);
      break;
    }

    if (current->coordinates == target_) {
      break;
    }

    closedSet.push_back(current);
    openSet.erase(current_it);

    for (uint i = 0; i < directions; ++i) {
      Point2D newCoordinates(current->coordinates + direction[i]);
      if (detectCollision(newCoordinates) ||
          findNodeOnList(closedSet, newCoordinates)) {
        continue;
      }

      uint totalCost = current->G + ((i < 4) ? 10 : 14);

      Node *successor = findNodeOnList(openSet, newCoordinates);
      if (successor == nullptr) {
        successor = new Node(newCoordinates, current);
        successor->G = totalCost;
        successor->H = heuristic(successor->coordinates, target_);
        openSet.push_back(successor);
      } else if (totalCost < successor->G) {
        successor->parent = current;
        successor->G = totalCost;
      }
    }
  }

  CoordinateList path;

  if (current->coordinates == target_) {
    while (current != nullptr) {
      path.push_back(current->coordinates);
      current = current->parent;
    }
  }

  releaseNodes(openSet);
  releaseNodes(closedSet);

  return path;
}

AStar::Node *AStar::Generator::findNodeOnList(NodeSet &nodes_,
                                              Point2D coordinates_) {
  for (auto node : nodes_) {
    if (node->coordinates == coordinates_) {
      return node;
    }
  }
  return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet &nodes_) {
  for (auto it = nodes_.begin(); it != nodes_.end();) {
    delete *it;
    it = nodes_.erase(it);
  }
}





AStar::Point2D AStar::Heuristic::getDelta(Point2D source_, Point2D target_) {
  return {abs(source_.x - target_.x), abs(source_.y - target_.y)};
}

AStar::uint AStar::Heuristic::manhattan(Point2D source_, Point2D target_) {
  auto delta = std::move(getDelta(source_, target_));
  return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Point2D source_, Point2D target_) {
  auto delta = std::move(getDelta(source_, target_));
  return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Point2D source_, Point2D target_) {
  auto delta = std::move(getDelta(source_, target_));
  return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}