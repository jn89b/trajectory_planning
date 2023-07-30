#include "Astar.h"

// Constructor class
// --------------------------------------------------------------
Astar::Astar(GridMap& grid_map, Agent& agent)
{
    this->grid_map_ = &grid_map;
    this->agent_ = &agent;
}

// --------------------------------------------------------------

void Astar::initializeNodes()
{
    // should have checks to make sure graph is set up
    // and agent is set up
    
    // Reset the open and closed sets just in case
    open_set = std::priority_queue<Node*, std::vector<Node*>, CompareNodeCost>();
    closed_set = std::set<std::vector<int>>();
    
    // Create the start node
    Node* start_node = new Node;
    start_node->pos = agent_->getPosition();
    start_node->parent = PositionVector(-1, -1, -1);
    start_node->g = 0;
    start_node->h = 0;
    start_node->f = 0;

    open_set.push(start_node);

    // Create the goal node
    Node* goal_node = new Node;
    goal_node->pos = agent_->getGoalPosition();
    goal_node->parent = PositionVector(-1, -1, -1);
    goal_node->g = 0;
    goal_node->h = 0;
    goal_node->f = 0;
    
}

// --------------------------------------------------------------

bool Astar::isValidPosition(const PositionVector& pos)
{
    // Check if the position is out of bounds
    if (grid_map_->isOutBounds(pos))
    {
        return false;
    }
    if (grid_map_->isInsideObstacles(pos, agent_->getRadius()))
    {
        return false;
    }

    return true;
}

// --------------------------------------------------------------

std::vector<PositionVector> Astar::getLegalNeighbors(
    const PositionVector & pos)
{
    std::vector<PositionVector> neighbors;
    
    std::vector <PositionVector> moves = agent_->getMoves3D(pos);
    for (auto move : moves)
    {
        if (isValidPosition(move))
            neighbors.push_back(move);
    } 

    return neighbors;
}

// --------------------------------------------------------------

#ifdef ENABLE_DOCTEST_IN_LIBRARY
#include "doctest/doctest.h"
TEST_CASE("we can have tests written here, to test impl. details")
{
    CHECK(true);
}
#endif
