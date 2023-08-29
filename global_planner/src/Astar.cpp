#include "Astar.h"

// Need to refactor this into  a utility function
namespace utils 
{
    float rad2deg(float rad)
    {
        return rad * 180 / M_PI;
    }

    float deg2rad(float deg)
    {
        return deg * M_PI / 180;
    }
}; 


void FWNode::setDirectionVec()
{
    if (parent.x == -1 && parent.y == -1 && parent.z == -1)
    {
        direction_vec = PositionVector(pos.x,pos.y,pos.y);
        float mag_direction = sqrt((direction_vec.x * direction_vec.x) + 
            (direction_vec.y * direction_vec.y));
        

        // float theta_rad = atan2(direction_vec.z, mag_direction); // should do this with eigen
        // theta_dg = utils::rad2deg(theta_rad);
        
        // float psi_rad = atan2(direction_vec.y, direction_vec.x);
        // psi_dg = utils::rad2deg(psi_rad);

    }
    else
    {
        // printf("Parent is %0.1f %0.1f %0.1f \n", parent.x, parent.y, parent.z);
        direction_vec = PositionVector(pos.x - parent.x, 
            pos.y - parent.y, pos.z - parent.z);
        
        float mag_direction = sqrt((direction_vec.x * direction_vec.x) + 
            (direction_vec.y * direction_vec.y));

        float theta_rad = atan2(direction_vec.z, mag_direction);
        theta_dg = utils::rad2deg(theta_rad);
        
        float psi_rad = atan2(direction_vec.y, direction_vec.x);
        psi_dg = utils::rad2deg(psi_rad);
    }

}

// Constructor class
// --------------------------------------------------------------

Astar::Astar(GridMap& grid_map, Agent& agent)
{
    this->grid_map_ = &grid_map;
    this->agent_ = &agent;
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

std::vector<PositionVector> Astar::searchPath()
{
    std::vector<PositionVector> path;
    
    // Need to also check if agent goal is not out of bounds 
    // or in an obstacle 

    // Initialize the nodes
    initializeNodes();

    // need to have this as a parameter 
    int max_iter = 10;
    int iter = 0;   

    // Loop until the open set is empty
    while (!open_set.empty() || iter < max_iter)
    {
        if (iter > max_iter)
        {
            printf("Max iterations reached\n");
            break;
        }

        // Get the node with the lowest f value
        Node* current_node = new Node;
        current_node = open_set.top();
        open_set.pop();

        // Get the position of the current nodes
        // push this into the current node index
        closed_set[current_node->index] = current_node;

        // check if the current node is at the goal
        if (current_node->pos == agent_->getGoalPosition())
        {
            printf("Goal found iterations %d \n", iter);
            path = getPath(current_node);
            break;
        }

        // Get all neighbors of the current node and begin costing them
        std::vector<PositionVector> neighbors_pos = getLegalNeighbors(
            current_node->pos);

        // Insert to open set now 
        for (auto neigh_pos:neighbors_pos)
        {   
            Node* neigh_node = new Node;
            neigh_node->pos = neigh_pos;
            neigh_node->parent = current_node->pos;
            neigh_node->g = current_node->g + computeHeuristic(current_node->pos, neigh_node->pos);
            neigh_node->h = computeHeuristic(neigh_node->pos, 
                agent_->getGoalPosition());
            neigh_node->f = neigh_node->g + neigh_node->h; 
            neigh_node->index = positionToIndex1D(neigh_node->pos);
            open_set.push(neigh_node);       
        }

        if (open_set.empty())
        {
            printf("No path found\n");
            break;
        }

        iter++;
    } 

    return path;
}

// --------------------------------------------------------------

std::vector<PositionVector> Astar::getPath(const Node* final_node)
{
    std::vector<PositionVector> path;
    auto current_node = final_node;
    path.push_back(current_node->pos);

    PositionVector current_pos = current_node->parent;

    printf("\n Getting Path \n ");

    while (!(current_pos == PositionVector(-1, -1, -1)))
    {
        path.push_back(current_node->parent);
        current_node = closed_set[positionToIndex1D(current_node->parent)];
        current_pos = current_node->parent;
    }

    // reverse the path
    std::reverse(path.begin(), path.end());
    for (auto pos:path)
    {
        printf("Path is at %0.1f %0.1f %0.1f \n", pos.x, 
            pos.y, pos.z);
    }

    return path;
}

// --------------------------------------------------------------
void Astar::initializeNodes()
{
    // should have checks to make sure graph is set up
    // and agent is set up
    printf("\n setting up nodes \n");
    // Reset the open and closed sets just in case
    open_set = std::priority_queue<Node*, std::vector<Node*>, CompareNodeCost>(); 
    closed_set = std::unordered_map<int, Node*>();
    
    // Create the start node
    Node* start_node = new Node;
    start_node->pos = agent_->getPosition();
    start_node->parent = PositionVector(-1, -1, -1);
    start_node->g = 0;
    start_node->h = 0;
    start_node->f = 0;
    start_node->index = positionToIndex1D(start_node->pos);

    printf("Starting pos is at %0.1f %0.1f %0.1f \n", start_node->pos.x, 
        start_node->pos.y, start_node->pos.z);

    open_set.push(start_node);

    // Create the goal node
    Node* goal_node = new Node;
    goal_node->pos = agent_->getGoalPosition();
    goal_node->parent = PositionVector(-1, -1, -1);
    goal_node->g = 0;
    goal_node->h = 0;
    goal_node->f = 0;

    printf("Goal pos is at %0.1f %0.1f %0.1f \n", goal_node->pos.x, 
        goal_node->pos.y, goal_node->pos.z);

    return ; 
    
}

// --------------------------------------------------------------

bool Astar::isValidPosition(const PositionVector& pos)
{
    // Check if the position is out of bounds
    if (grid_map_->isOutBounds(pos))
        return false;

    // Check if the position is inside an obstacle
    if (grid_map_->isInsideObstacles(pos, agent_->getRadius()))
        return false;

    // check if in closed set 
    // PositionVector pos_vec = pos;
    int pos_index = positionToIndex1D(pos);

    if (closed_set.find(pos_index) != closed_set.end())
        return false;

    return true;
}

// ----------------------------------------------------------------

float Astar::computeHeuristic(const PositionVector& from_pos, 
    const PositionVector& to_pos)
{   
    
    float elevation_penalty = 1;
    float distance_penalty = 1;
    

    float distance_from_goal = sqrt(pow(from_pos.x - to_pos.x, 2) + 
        pow(from_pos.y - to_pos.y, 2) + 
        pow(from_pos.z - to_pos.z, 2));

    if (distance_from_goal > 50)
    {
        distance_penalty = 1.5;

        if (from_pos.z != to_pos.z)
            elevation_penalty = 1.5;
    }

    return sqrt(pow(from_pos.x - to_pos.x, 2) + 
        pow(from_pos.y - to_pos.y, 2) + 
        pow(from_pos.z - to_pos.z, 2)) * elevation_penalty * distance_penalty;


}

// ----------------------------------------------------------------

int Astar::positionToIndex1D(const PositionVector& pos)
{
    // If the z is 0 then we will treat this as a plane
    if (pos.z == 0)
    {   
        //gridsize x is the width
        return pos.x + pos.y * grid_map_->getGridSizeX();
    }
    else
    {   
        // 
        return pos.x + (pos.y * grid_map_->getGridSizeX()) + 
            (pos.z * grid_map_->getGridSizeX() * 
                grid_map_->getGridSizeY());
    }
}

// ----------------------------------------------------------------

PositionVector Astar::index1DToPosition(int index)
{
    int x = index % grid_map_->getGridSizeX();
    int y = (index / grid_map_->getGridSizeX()) % grid_map_->getGridSizeY();
    int z = index / (grid_map_->getGridSizeX() * grid_map_->getGridSizeY());

    return PositionVector(x, y, z);
    
}


// This is low key trash I got a lot of duplicate code here
// Figure out better architecture to combine Astar with SAS

// Constructor class
// --------------------------------------------------------------

SparseAstar::SparseAstar(GridMap& grid_map, FWAgent& agent)
{
    this->grid_map_ = &grid_map;
    this->agent_ = &agent;
}

// --------------------------------------------------------------

std::vector<PositionVector> SparseAstar::getLegalNeighbors(
    const PositionVector & pos, float psi_dg)
{
    std::vector<PositionVector> neighbors;
    
    std::vector <PositionVector> moves = agent_->getMoves3D(pos, psi_dg);
    for (auto move : moves)
    {
        if (isValidPosition(move))
            neighbors.push_back(move);
    } 

    return neighbors;
}

// ----------------------------------------------------------------

std::vector<PositionVector> SparseAstar::searchPath()
{
    std::vector<PositionVector> path;
    
    // Initialize the nodes
    initializeNodes();

    // need to have this as a parameter 
    int max_iter = 5000;
    int iter = 0;   
    
    // need time limit too 

    // Loop until the open set is empty
    while (!open_set.empty() && iter < max_iter)
    {
        iter++;

        // Get the node with the lowest f value
        // FWNode* current_node = new FWNode;
        FWNode* current_node = open_set.top();
        open_set.pop();

        // Get the position of the current nodes
        // push this into the current node index
        closed_set[current_node->index] = current_node;

        if (iter >= max_iter)
        {
            printf("Max iterations reached\n");
            path = getPath(current_node);
            break;
        }

        // check if the current node is at the goal
        if (current_node->pos == agent_->getGoalPosition())
        {
            printf("Goal found iterations %d \n", iter);
            path = getPath(current_node);
            break;
        }

        float dist_goal = computeHeuristic(
            current_node->pos, agent_->getGoalPosition());
        
        if (dist_goal <= agent_->getLegSegmentm())
        {
            printf("Closing in location%d \n", iter);
            printf("Distance to goal is %0.1f \n", dist_goal);
            path = getPath(current_node);
            break;
        }

        // Check time here 

        // Get all neighbors of the current node and begin costing them
        std::vector<PositionVector> neighbors_pos = getLegalNeighbors(
            current_node->pos, current_node->psi_dg);

        // Insert to open set now 
        for (auto neigh_pos:neighbors_pos)
        {   
            FWNode* neigh_node = new FWNode;
            neigh_node->pos = neigh_pos;
            neigh_node->parent = current_node->pos;
            neigh_node->g = current_node->g + 1; //computeHeuristic(current_node->pos, neigh_node->pos);
            neigh_node->h = computeHeuristic(neigh_node->pos, 
                agent_->getGoalPosition());
            neigh_node->f = neigh_node->g + neigh_node->h; 
            neigh_node->index = positionToIndex1D(neigh_node->pos);
            neigh_node->setDirectionVec();
            open_set.push(neigh_node);       
        }

        if (open_set.empty())
        {
            printf("Open set is empty\n");
            path = getPath(current_node);
            break;
        }

    } 

    return path;
}

// --------------------------------------------------------------

std::vector<PositionVector> SparseAstar::getPath(
    const FWNode* final_node)
{
    std::vector<PositionVector> path;
    auto current_node = final_node;
    path.push_back(current_node->pos);

    PositionVector current_pos = current_node->parent;

    printf("\n Getting Path \n ");

    while (!(current_pos == PositionVector(-1, -1, -1)))
    {
        path.push_back(current_node->parent);
        current_node = closed_set[positionToIndex1D(
            current_node->parent)];
        current_pos = current_node->parent;
    }

    // reverse the path
    std::reverse(path.begin(), path.end());
    // for (auto pos:path)
    // {
    //     printf("Path is at %0.1f %0.1f %0.1f \n", pos.x, 
    //         pos.y, pos.z);
    // }

    return path;
}

// -------------------------s-------------------------------------

void SparseAstar::initializeNodes()
{
    // should have checks to make sure graph is set up
    // and agent is set up
    printf("\n setting up nodes \n");
    // Reset the open and closed sets just in case
    open_set = std::priority_queue<FWNode*, std::vector<FWNode*>, 
        CompareNodeCost>(); 
    closed_set = std::unordered_map<int, FWNode*>();
    
    // Create the start node
    FWNode* start_node = new FWNode;
    start_node->pos = agent_->getPosition();
    start_node->parent = PositionVector(-1, -1, -1);
    start_node->g = 0;
    start_node->h = 0;
    start_node->f = 0;
    start_node->theta_dg = agent_->getTheta();
    start_node->psi_dg = agent_->getPsi();
    start_node->index = positionToIndex1D(start_node->pos);
    start_node->setDirectionVec();

    printf("Starting pos is at %0.1f %0.1f %0.1f \n", start_node->pos.x, 
        start_node->pos.y, start_node->pos.z);

    open_set.push(start_node);

    // Create the goal node
    FWNode* goal_node = new FWNode;
    goal_node->pos = agent_->getGoalPosition();
    goal_node->parent = PositionVector(-1, -1, -1);
    goal_node->g = 0;
    goal_node->h = 0;
    goal_node->f = 0;
    goal_node->setDirectionVec();

    printf("Goal pos is at %0.1f %0.1f %0.1f \n", goal_node->pos.x, 
        goal_node->pos.y, goal_node->pos.z);

    return ; 
    
}

// --------------------------------------------------------------

bool SparseAstar::isValidPosition(const PositionVector& pos)
{
    // Check if the position is out of bounds
    if (grid_map_->isOutBounds(pos))
        return false;

    // Check if the position is inside an obstacle
    if (grid_map_->isInsideObstacles(pos, agent_->getRadius()))
        return false;

    // check if in closed set 
    // PositionVector pos_vec = pos;
    int pos_index = positionToIndex1D(pos);

    if (closed_set.find(pos_index) != closed_set.end())
        return false;

    return true;
}

// ----------------------------------------------------------------

float SparseAstar::computeHeuristic(const PositionVector& from_pos, 
    const PositionVector& to_pos)
{       
    return sqrt((from_pos.x - to_pos.x) * (from_pos.x - to_pos.x) + 
        (from_pos.y - to_pos.y) * (from_pos.y - to_pos.y) + 
        (from_pos.z - to_pos.z) * (from_pos.z - to_pos.z));

}

// ----------------------------------------------------------------

int SparseAstar::positionToIndex1D(const PositionVector& pos)
{
    // If the z is 0 then we will treat this as a plane
    if (pos.z == 0)
    {   
        //gridsize x is the width
        return pos.x + pos.y * grid_map_->getGridSizeX();
    }
    else
    {   
        // 
        return pos.x + (pos.y * grid_map_->getGridSizeX()) + 
            (pos.z * grid_map_->getGridSizeX() * 
                grid_map_->getGridSizeY());
    }
}

// ----------------------------------------------------------------

PositionVector SparseAstar::index1DToPosition(int index)
{
    int z = index / (grid_map_->getGridSizeX() * grid_map_->getGridSizeY());
    int idx =- (z * grid_map_->getGridSizeX() * grid_map_->getGridSizeY());
    int y = idx / grid_map_->getGridSizeX();
    int x = idx % grid_map_->getGridSizeX();

    // int x = index % grid_map_->getGridSizeX();
    // int y = (index / grid_map_->getGridSizeX()) % grid_map_->getGridSizeY();
    // int z = (index / (grid_map_->getGridSizeX()) * grid_map_->getGridSizeY());

    return PositionVector(x, y, z);
    
}

// ----------------------------------------------------------------


#ifdef ENABLE_DOCTEST_IN_LIBRARY
#include "doctest/doctest.h"
TEST_CASE("we can have tests written here, to test impl. details")
{
    CHECK(true);
}
#endif
