#pragma once

#include <stdio.h>
#include <queue>
#include <vector>
#include <set>
#include <unordered_map>
#include <algorithm>

#include "PositionVector.h"
#include "GridMap.h"

/* -------------------------------------------------
    NODE STRUCT

    This struct is used to represent a node in the
    A* algorithm. It contains the position of the node
    in the grid, the parent node, and the f, g, and h
    values of the node.
    -------------------------------------------------
*/

// using namespace utils;

struct Node
{
    Node() : pos(0, 0, 0), 
             parent(-1, -1, -1), 
             f(0), g(0), h(0) {}
    
    // Node(const PositionVector& pos, float f) : 
    //     pos(pos), parent(-1, 1, -1), f(f), g(0), h(0) {}
    
    Node(const PositionVector& pos, const PositionVector& parent, 
        float f, float g, float h) : 
        pos(pos), parent(parent), f(f), g(g), h(h) {}

    PositionVector pos;
    PositionVector parent;
    float f;
    float g;
    float h;
    int index;

    bool operator==(const Node* OtherNode)
    {
        if (this->pos == OtherNode->pos) return true;
        else return false;
    }

    // Hash function for the Node struct for use in the closed set 
    // https://stackoverflow.com/questions/17016175/c-unordered-map-using-a-custom-class-type-as-the-key
    struct NodeHasher
    {
        size_t operator()(const Node* node) const
        {
            size_t xHash = std::hash<int>()(node->pos.x) << 1;
            size_t yHash = std::hash<int>()(node->pos.y) << 1;
            size_t zHash = std::hash<int>()(node->pos.z) << 1;
            return xHash ^ yHash ^ zHash;
        }
    };

};

/* -------------------------------------------------
    FWNODE STRUCT

    This struct is used to represent a node in the
    SA*S algorithm. It contains the position of the node
    as well as the psi and theta in dgs for the aircraft
    in the grid, the parent node, and the f, g, and h
    values of the node.
    -------------------------------------------------
*/

struct FWNode
{
    FWNode() : pos(0, 0, 0), 
             parent(-1, -1, -1), 
             f(0), g(0), h(0),
             direction_vec(0,0,0),
             theta_dg(0), psi_dg(0) {}
    
    FWNode(const PositionVector& pos, const PositionVector& parent, 
        float f, float g, float h): 
        pos(pos), parent(parent), f(f), g(g), h(h),
        direction_vec(0,0,0), theta_dg(0), psi_dg(0){}

    PositionVector pos;
    PositionVector parent;
    float f;
    float g;
    float h;

    PositionVector direction_vec;
    float theta_dg;
    float psi_dg;
    int index;

    void setDirectionVec();

    bool operator==(const FWNode* OtherNode)
    {
        if (this->pos == OtherNode->pos) return true;
        else return false;
    }

    // Hash function for the Node struct for use in the closed set 
    // https://stackoverflow.com/questions/17016175/c-unordered-map-using-a-custom-class-type-as-the-key
    struct NodeHasher
    {
        size_t operator()(const FWNode* node) const
        {
            size_t xHash = std::hash<int>()(node->pos.x) << 1;
            size_t yHash = std::hash<int>()(node->pos.y) << 1;
            size_t zHash = std::hash<int>()(node->pos.z) << 1;
            return xHash ^ yHash ^ zHash;
        }
    };

};

// Comparison operator overloading for the priority queue of the nodes
// the node with the lowest f value has the highest priority
struct CompareNodeCost 
{
    bool operator()(const Node* node_a, const Node* node_b)
    {
        return node_a->f > node_b->f;
    }

    bool operator()(const FWNode* node_a, const FWNode* node_b)
    {
        return node_a->f > node_b->f;
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
        // Constructor for the Astar class
        // Must have a GridMap and an Agent to begin
        Astar(GridMap& grid_map, Agent& agent);

        // returns the open set
        std::priority_queue<Node*, std::vector<Node*>, CompareNodeCost> 
            getOpenSet() { return open_set; }

        // //this is for sas

        // returns the closed set        

        // getLegalNeighbors() returns a vector of all the 
        // legal neighbors of a position this is done by 
        // calling isValidPosition() on all the neighbors
        std::vector<PositionVector> getLegalNeighbors(
            const PositionVector& pos);

        // Finds the shortest path between the agent's start and 
        // goal positions
        std::vector<PositionVector> searchPath();

        // Returns the path found by the A* algorithm by 
        // reversing the parent nodes
        std::vector<PositionVector> getPath(const Node* final_node);

        // Computes the 3D position to 1D index
        int positionToIndex1D(const PositionVector& pos);

        // Computes the 1D index to 3D position
        PositionVector index1DToPosition(int index);

    private:
        GridMap* grid_map_ = NULL;
        Agent* agent_ = NULL;
        

        /// @brief Calculates the heuristic value of a node
        std::priority_queue<Node*, std::vector<Node*>, CompareNodeCost> open_set;

        // This is the list of position nodes that have already been visited
        // using set for quick search time of O(1)
        std::unordered_map<int, Node*> closed_set;

        // initializeNodes() sets up the start and goal nodes 
        // based on the agent configuration 
        void initializeNodes();

        // isValidPosition() checks to see if the position 
        // is not outside the range of the grid map as well 
        // as checks if the position is not an obstacle
        bool isValidPosition(const PositionVector& pos);

        float computeHeuristic(const PositionVector& from_pos, 
            const PositionVector& to_pos);

};

/* -------------------------------------------------
    SparseAstar CLASS
    
    SparseAstar class is used to find the shortest path
    between two points in a 3D grid based on non-holonimic
    vehicle constraints
    -------------------------------------------------
*/

class SparseAstar
{
	
    public:
        // Constructor for the Astar class
        // Must have a GridMap and an Agent to begin
        // Refactor this class to use fw agent
        SparseAstar(GridMap& grid_map, FWAgent& agent);

        // returns the open set
        std::priority_queue<FWNode*, std::vector<FWNode*>, CompareNodeCost> 
            getOpenSet() { return open_set; }

        // returns the closed set        

        // getLegalNeighbors() returns a vector of all the 
        // legal neighbors of a position this is done by 
        // calling isValidPosition() on all the neighbors
        std::vector<PositionVector> getLegalNeighbors(
            const PositionVector& pos, float psi_dg);

        // Finds the shortest path between the agent's start and 
        // goal positions
        std::vector<PositionVector> searchPath();

        // Returns the path found by the SA*S algorithm by 
        // reversing the parent nodes
        std::vector<PositionVector> getPath(const FWNode* final_node);

        // Computes the 3D position to 1D index
        int positionToIndex1D(const PositionVector& pos);

        // Computes the 1D index to 3D position
        PositionVector index1DToPosition(int index);

    private:
        GridMap* grid_map_ = NULL;
        FWAgent* agent_ = NULL; // FW agent
        
        /// @brief Calculates the heuristic value of a node
        std::priority_queue<FWNode*, std::vector<FWNode*>, CompareNodeCost> 
            open_set;

        // This is the list of position nodes that have already been visited
        // using set for quick search time of O(1)
        std::unordered_map<int, FWNode*> closed_set;

        // initializeNodes() sets up the start and goal nodes 
        // based on the agent configuration 
        void initializeNodes();

        // isValidPosition() checks to see if the position 
        // is not outside the range of the grid map as well 
        // as checks if the position is not an obstacle
        bool isValidPosition(const PositionVector& pos);

        float computeHeuristic(const PositionVector& from_pos, 
            const PositionVector& to_pos);

};


#ifdef ENABLE_DOCTEST_IN_LIBRARY
#include "doctest/doctest.h"
TEST_CASE("we can have tests in headers if we want")
{
    Dummy d;
    CHECK(d.doSomething() == true);
}
#endif
