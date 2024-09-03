/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <vector>
#include <functional>
#include <set>
#include <math.h>

namespace AStar
{
    struct Point2D
    {
        int x, y;

        bool operator == (const Point2D& coordinates_);
        bool operator != (const Point2D& coordinates_);
    };

    using uint = unsigned int;
    using HeuristicFunction = std::function<uint(Point2D, Point2D)>;
    using CoordinateList = std::vector<Point2D>;

    struct Node
    {
        uint G, H;
        Point2D coordinates;
        Node *parent;

        Node(Point2D coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::vector<Node*>;

    class Generator
    {
        bool detectCollision(Point2D coordinates_);
        Node* findNodeOnList(NodeSet& nodes_, Point2D coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(Point2D worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        CoordinateList findPath(Point2D source_, Point2D target_);
        void addCollision(Point2D coordinates_);
        void addCollision(Point2D coordinates_, int size);
        void removeCollision(Point2D coordinates_);
        void clearCollisions();

    private:
        HeuristicFunction heuristic;
        CoordinateList direction, walls;
        Point2D worldSize;
        uint directions;
    };

    class Heuristic
    {
        static Point2D getDelta(Point2D source_, Point2D target_);

    public:
        static uint manhattan(Point2D source_, Point2D target_);
        static uint euclidean(Point2D source_, Point2D target_);
        static uint octagonal(Point2D source_, Point2D target_);
    };
}

#endif // ASTAR_HPP