#pragma once

#include <stdio.h>
#include <queue>
#include <vector>
#include <set>


#include "PositionVector.h"
#include "GridMap.h"
#

using uint = unsigned int;

/* -------------------------------------------------
    NODE STRUCT

    This struct is used to represent a node in the
    A* algorithm. It contains the position of the node
    in the grid, the parent node, and the f, g, and h
    values of the node.
    -------------------------------------------------
*/
struct Node
{
    Node() : pos(0, 0, 0), 
             parent(-1, -1, -1), 
             f(0), g(0), h(0) {}
    Node(const PositionVector& pos, float f) : 
        pos(pos), parent(-1, 1, -1), f(f), g(0), h(0) {}
    
    Node(const PositionVector& pos, const PositionVector& parent, 
        float f, float g, float h) : pos(pos), parent(parent), f(f), g(g), h(h) {}

    PositionVector pos;
    PositionVector parent;
    float f;
    float g;
    float h;
    
};
 
// Comparison operator overloading for the priority queue of the nodes
// the node with the lowest f value has the highest priority
struct CompareNodeCost 
{
    bool operator()(const Node& node_a, const Node& node_b) const
    {
        return node_a.f > node_b.f;
    }
};

/* -------------------------------------------------
    ASTAR CLASS
    
    Astar class is used to find the shortest path
    between two points in a 3D grid. It uses the A*
    algorithm to find the path.
    -------------------------------------------------
*/
class Astar
{
	
    public:
        // Constructor
        Astar();

        // Finds the shortest path between two points in a 3D grid using the A* algorithm
        std::vector<PositionVector> findPath(PositionVector start, PositionVector goal);

    private:

        /// @brief Calculates the heuristic value of a node
        std::priority_queue<Node, std::vector<Node>, CompareNodeCost> open_set;

        // This is the list of position nodes that have already been visited
        // using set for quick search time of O(1)
        std::set< std::vector<int> > closed_set;


        // Have a method called isValid that checks if the node is valid
        // by calling out graph and checking if inside and not in obstacle
        // or not
        bool isValid(const PositionVector& pos);

        float computeTotalCost();

        float computeHeuristic();







};


#ifdef ENABLE_DOCTEST_IN_LIBRARY
#include "doctest/doctest.h"
TEST_CASE("we can have tests in headers if we want")
{
    Dummy d;
    CHECK(d.doSomething() == true);
}
#endif
