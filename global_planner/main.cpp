#include <iostream>
#include "Astar.h"
#include "GridMap.h"

int main()
{
    printf("Hello World\n");
    
    GridMap gridmap;

    printf("Gridmap initialized\n");
    printf("grid sizes x y z %d %d %d\n", 
    gridmap.getGridSizeX(), 
    gridmap.getGridSizeY(), 
    gridmap.getGridSizeZ());

    gridmap.setGridSize(10, 10, 10, 20, 20, 20);

    printf("grid sizes of x y z %d %d %d\n", 
    gridmap.getGridSizeX(), 
    gridmap.getGridSizeY(), 
    gridmap.getGridSizeZ());

    //insert obstacle 
    Obstacle obstacle(1, 1, 1, 1, 1);
    int num_obs = gridmap.insertObstacle(obstacle);
    printf("Number of obstacles: %d\n", num_obs);
    
    Obstacle* obs = gridmap.getObstacle(0);

    printf("obs x y z %f %f %f\n", 
    obs->getX(), 
    obs->getY(), 
    obs->getZ());

    
    if (obs->isInside2D(PositionVector(3, 5, 7)))
        printf("Inside\n");
    else
        printf("Outside\n");  
    
    // should be outside 
    PositionVector pos= PositionVector(100, 100, 7);
    if (gridmap.isOutBounds(pos))
        printf("Out of bounds\n");
    else
        printf("Inside bounds\n");



    return 0;
}
