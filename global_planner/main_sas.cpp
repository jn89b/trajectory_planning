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
    printf("SparseAstar finding path\n");

    PositionVector fw_pos(-250, -150, 0);
    PositionVector goal_pos(750,750,25);
    float radius_m = 5.0f;
    float theta_dg = 0.0f;
    float psi_dg = 180.0f;
    float max_psi_turn_dg = 45.0f;
    float max_leg_segment_m = 25;
    FWAgent fw_agent(fw_pos, goal_pos, 
        radius_m, theta_dg, 
        psi_dg, 
        max_psi_turn_dg, max_leg_segment_m);

    fw_agent.setPsi(psi_dg);
    GridMap gridmap;
    gridmap.setGridSize(-500, -500, 0, 1000, 1000, 50);
    fw_agent.setGoalPosition(goal_pos.x, goal_pos.y, goal_pos.z);

    //set seed for random number generator
    srand(2);
    int n_obstacles = 250;
    // insert 20 random obstacles
    for (int i = 0; i< n_obstacles; i++)
    {
        int x = rand(gridmap.getGRID_MIN_X()+ 150,
            gridmap.getGRID_MAX_X() - 150);
        
        int y = rand(gridmap.getGRID_MIN_Y() + 150,
            gridmap.getGRID_MAX_Y() - 150);

        int z = rand(1,10);
        int rand_radius = rand(10, 50);

        // make sure obstacle is not too close to agent
        if (sqrt(pow(x - fw_agent.getPosition().x, 2) + 
            pow(y - fw_agent.getPosition().y, 2) + 
            pow(z - fw_agent.getPosition().z, 10)) < 100)
            continue;

        if (sqrt(pow(x - fw_agent.getGoalPosition().x, 2) + 
            pow(y - fw_agent.getGoalPosition().y, 2) + 
            pow(z - fw_agent.getGoalPosition().z, 10)) < 100)
            continue;   

        

        Obstacle *obstacle = new Obstacle(x, y, z, 
            float(rand_radius), 1);
        
        int num_obs = gridmap.insertObstacle(obstacle);
    }

    SparseAstar sparse_astar(gridmap, fw_agent);

    //time the search
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<PositionVector> path = sparse_astar.searchPath();
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

    // auto p = plot3(x_pos, y_pos, z_pos, "-o");
    // hold(on);

    auto s = plot(x_pos, y_pos, "-o");

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

        // printf("Obstacle %d: %d, %d, %d\n", i, x, y, z);

        x_obs.push_back(x);
        y_obs.push_back(y);
        z_obs.push_back(z);
    }
    
    scatter3(x_obs, y_obs, z_obs, "filled");

    show();

    show();

    // std::vector<PositionVector> moves;
    // moves = fw_agent.getMoves3D(fw_node.pos, psi_dg, 5);

    // for (int i=0; i<moves.size(); i++)
    // {
    //     printf("Move %d: (%f, %f, %f)\n", i, moves[i].x, moves[i].y, moves[i].z);
    // }

    // // Sanity check to plot moves
    // std::vector<double> x;
    // std::vector<double> y;
    // std::vector<double> z;

    // for (int i=0; i<moves.size(); i++)
    // {
    //     x.push_back(moves[i].x);
    //     y.push_back(moves[i].y);
    //     z.push_back(moves[i].z);
    // }

    // x.push_back(fw_pos.x);
    // y.push_back(fw_pos.y);
    // z.push_back(fw_pos.z);

    // scatter3(x, y, z);
    // hold(on);
    // // plot3(double(fw_pos.x), double(fw_pos.y), double(fw_pos.z));

    // show();
    

    return 0;
}
