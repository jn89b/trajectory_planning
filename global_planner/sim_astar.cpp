#include <iostream>
#include <unordered_set>
#include <unordered_map>

#include "Astar.h"
#include "GridMap.h"
#include "PositionVector.h"
#include <matplot/matplot.h>

#include <fstream>
#include <iterator>
#include <string>
#include <vector>

// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;
using namespace matplot;
int main()
{
    printf("Astar finding path\n");
    
    // testing agent get moves 
    Agent agent1(2,0,0,10,10,5);
    GridMap gridmap;
    gridmap.setGridSize(0, 0, 0, 1000, 1000, 10);
    agent1.setPosition(0, 0, 5);
    agent1.setGoalPosition(750, 750, 10);


    //set seed for random number generator
    srand(1);
    // insert 20 random obstacles
    for (int i = 0; i< 20; i++)
    {
        int x = rand(100,700);
        int y = rand(100,700);
        int z = rand(1,10);

        // make sure obstacle is not too close to agent
        if (sqrt(pow(x - agent1.getPosition().x, 2) + 
            pow(y - agent1.getPosition().y, 2) + 
            pow(z - agent1.getPosition().z, 10)) < 50)
            continue;

        if (sqrt(pow(x - agent1.getGoalPosition().x, 2) + 
            pow(y - agent1.getGoalPosition().y, 2) + 
            pow(z - agent1.getGoalPosition().z, 10)) < 50)
            continue;   

        Obstacle *obstacle = new Obstacle(x, y, z, 30.0f, 1);
        
        int num_obs = gridmap.insertObstacle(obstacle);
    }

    // Obstacle obstacle(500, 500, 1, 50.0f, 1);
    // int num_obs = gridmap.insertObstacle(obstacle);
    printf("Number of obstacles: %d\n", gridmap.getNumObstacles());

    // Construct astar object with gridmap and agent 
    Astar astar(gridmap, agent1);

    //time the search

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<PositionVector> path = astar.searchPath();
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    printf("Elapsed time: %f\n", elapsed.count());
    
    std::vector<int> x_pos;
    std::vector<int> y_pos;
    std::vector<int> z_pos;

    for (int i=0; i < path.size(); i++)
    {
        x_pos.push_back(path[i].x);
        y_pos.push_back(path[i].y);
        z_pos.push_back(path[i].z);
    }

    const std::vector<Obstacle*> obstacles = gridmap.getObstacles();

    auto p = plot3(x_pos, y_pos, z_pos, "-o");
    hold(on);

    std::vector<int> x_obs;
    std::vector<int> y_obs;
    std::vector<int> z_obs;


    for (int i=0; i < obstacles.size(); i++)
    {
        Obstacle* obstacle = gridmap.getObstacle(i);
        int x = obstacle->getX();
        int y = obstacle->getY();
        int z = obstacle->getZ();

        printf("Obstacle %d: %d, %d, %d\n", i, x, y, z);

        x_obs.push_back(x);
        y_obs.push_back(y);
        z_obs.push_back(z);
    }
    
    scatter3(x_obs, y_obs, z_obs, "filled");

    show();

    // write path to a text file
    std::ofstream myfile;
    myfile.open ("path.txt");
    for (int i=0; i < path.size(); i++)
    {
        myfile << path[i].x << " " << path[i].y << " " 
            << path[i].z << "\n";
    }

    myfile.close();
    printf("Done\n");

    std::ofstream obstacle_text;
    obstacle_text.open ("obstacles.txt");
    for (int i=0; i < obstacles.size(); i++)
    {
        Obstacle* obstacle = gridmap.getObstacle(i);
        int x = obstacle->getX();
        int y = obstacle->getY();
        int z = obstacle->getZ();
        int radius = obstacle->getRadius();

        obstacle_text << x << " " << y << " " << 
            z << " " << radius<< "\n";
    }
    obstacle_text.close();


    return 0;
}
