#include <iostream>
#include <unordered_set>

#include "Astar.h"
#include "GridMap.h"
#include "PositionVector.h"

int main()
{
    printf("Hello World\n");
    
    // testing agent get moves 
    Agent agent1(2,0,0,10,10,10);
    PositionVector pos1 = agent1.getPosition();
    printf("agent pos %f %f %f\n", pos1.x, pos1.y, pos1.z);

    std::vector<PositionVector> moves = agent1.getMoves2D(pos1);
    // check if the moves2d is working correctly
    for (int i=0; i < moves.size(); i++)
    {
        printf("moves2d %d %f %f \n", i, moves[i].x, moves[i].y);
    }

    moves = agent1.getMoves3D(agent1.getPosition());
    // check if the moves2d is working correctly
    for (int i=0; i < moves.size(); i++)
    {
        printf("moves2d %d %f %f %f\n", i, moves[i].x, moves[i].y, moves[i].z);
    }

    GridMap gridmap;

    printf("Gridmap initialized\n");
    printf("grid sizes x y z %d %d %d\n", 
    gridmap.getGridSizeX(), 
    gridmap.getGridSizeY(), 
    gridmap.getGridSizeZ());

    gridmap.setGridSize(0, 0, 0, 30, 30, 10);

    printf("grid sizes of x y z %d %d %d\n", 
    gridmap.getGridSizeX(), 
    gridmap.getGridSizeY(), 
    gridmap.getGridSizeZ());

    //insert obstacle 
    Obstacle obstacle(5, 5, 1, 1, 1);
    int num_obs = gridmap.insertObstacle(obstacle);
    printf("Number of obstacles: %d\n", num_obs);
    
    Obstacle* obs = gridmap.getObstacle(0);

    printf("obs x y z %f %f %f\n", 
    obs->getX(), 
    obs->getY(), 
    obs->getZ());

    // This is a check for obstacle detection    
    if (obs->isInside2D(PositionVector(3, 5, 7)))
        printf("Inside\n");
    else
        printf("Outside\n");  
    
    // This is a check to make sure I can detect if a 
    // position is out of bounds 
    PositionVector pos= PositionVector(100, 100, 7);
    if (gridmap.isOutBounds(pos))
        printf("Out of bounds\n");
    else
        printf("Inside bounds\n");

    // This is how I will push nodes inside the open set 
    Astar astar(gridmap, agent1);
    Node* node1 = new Node; 
    node1->pos = PositionVector(0, 0, 0);
    node1->g = 0;
    node1->h = 0;
    node1->f = 10;

    Node* node2 = new Node;
    node2->pos = PositionVector(1, 1, 1);
    node2->g = 0;
    node2->h = 0;
    node2->f = 5;

    // Making sure my comparison operator works
    CompareNodeCost compare;
    if (compare(node1, node2))
        printf("node1 has lower cost\n");
    else
        printf("node2 has lower cost\n");

    // Making sure I pop out the minimum cost node out of the open set
    std::priority_queue<Node*, std::vector<Node*>, CompareNodeCost> open_set;
    open_set.push(node1);
    open_set.push(node2);

    Node* node3 = new Node; 
    
    // should be node2
    node3 = open_set.top();
    open_set.pop();
    printf("node3 f %f\n", node3->f);
    
    // should be node3
    node3 = open_set.top();
    printf("node3 f %f\n", node3->f);
    open_set.pop();

    // this is how I will check if I have visited a node '  
    // just convert the position vector as int and do a check if visited
    // std::unordered_set<PositionVector, PositionHash> closed_set;
    std::set< std::vector<int> > closed_set;
    closed_set.insert({1, 2, 3});
    closed_set.insert({1, 2, 3});
    printf("closed set size %d\n", int(closed_set.size()));

    closed_set.find({1, 2, 3});

    if (closed_set.find({1, 2, 3}) != closed_set.end())
        printf("found\n");
    else
        printf("not found\n");

    // Just want to check if this will not return anything
    closed_set.find({5, 5, 5});
    if (closed_set.find({5, 5, 5}) != closed_set.end())
        printf("found\n");
    else
        printf("not found\n");

    // // test astar isvalid
    agent1.setPosition(3, 5, 3);
    agent1.setGoalPosition(20, 30, 20);
    std::vector<PositionVector> neighbors = astar.searchPath();

    return 0;
}
