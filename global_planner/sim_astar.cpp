#include <iostream>
#include <unordered_set>
#include <unordered_map>

#include "Astar.h"
#include "GridMap.h"
#include "PositionVector.h"
#include <matplot/matplot.h>

// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;
using namespace matplot;
int main()
{
    printf("Astar finding path\n");
    
    // testing agent get moves 
    Agent agent1(2,0,0,10,10,10);
    GridMap gridmap;
    gridmap.setGridSize(0, 0, 0, 1000, 1000, 10);

    //insert obstacle 
    Obstacle obstacle(500, 500, 1, 50.0f, 1);
    int num_obs = gridmap.insertObstacle(obstacle);

    // Construct astar object with gridmap and agent 
    Astar astar(gridmap, agent1);

    // astar imlementation
    agent1.setPosition(0, 0, 3);
    agent1.setGoalPosition(650, 650, 10);
    std::vector<PositionVector> path = astar.searchPath();
    
    std::vector<int> x_pos;
    std::vector<int> y_pos;
    std::vector<int> z_pos;

    for (int i=0; i < path.size(); i++)
    {
        x_pos.push_back(path[i].x);
        y_pos.push_back(path[i].y);
        z_pos.push_back(path[i].z);
    }

    plot3(x_pos, y_pos, z_pos, "-o");
    // plot(x_pos, y_pos, "-o");
    show();
    // plt::plot(x_pos, y_pos, "-o");
    // plt::xlabel("x");
    // plt::ylabel("y");
    // plt::legend();
    // plt::show();


    return 0;
}
