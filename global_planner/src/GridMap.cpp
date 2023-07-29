#include "GridMap.h"
#include <math.h>

/* -------------------------------------------------
    OBSTACLE CLASS
    -------------------------------------------------
*/

// -------------------------------------------------
Obstacle::Obstacle(double x, double y, double z, double radius, double height)        
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->radius = radius;
    this->height = height;
    
}

// -------------------------------------------------

bool Obstacle::isInside2D(PositionVector position)
{
    double dist = sqrt(pow(position.x - this->x, 2) 
    + pow(position.y - this->y, 2));
    printf("dist: %f\n", dist);

    if (dist <= this->radius)
        return true;
    else
        return false;
}

// -------------------------------------------------

Obstacle* GridMap::getObstacle(int index)
{
    Obstacle* obs = obstacle_list[index] ;
    return obs;  
}

// -------------------------------------------------

bool Obstacle::isInside3D(PositionVector position)
{
    double dist = sqrt(pow(position.x - this->x, 2) + 
        pow(position.y - this->y, 2) + 
        pow(position.z - this->z, 2));

    if (dist <= this->radius)
        return true;
    else
        return false;

}

/* -------------------------------------------------
    GRIDMAP CLASS
    -------------------------------------------------
*/

GridMap::GridMap() 
{
    //initialize the grid size map
    this->x_min = 0;
    this->y_min = 0;
    this->z_min = 0;

    this->x_max = 25;
    this->y_max = 25;
    this->z_max = 10;
}

// -------------------------------------------------

bool GridMap::setGridSize(int x_min, int y_min, int z_min, 
        int x_max, int y_max, int z_max)
{   
    this->x_min = x_min;
    this->y_min = y_min;
    this->z_min = z_min;
    this->x_max = x_max;
    this->y_max = y_max;
    this->z_max = z_max;

    return true;
}

// -------------------------------------------------

int GridMap::insertObstacle(Obstacle& obstacle)
{
    obstacle_list.push_back(&obstacle);

    return int(obstacle_list.size());
}

// -------------------------------------------------

bool GridMap::isOutBounds(PositionVector position)
{
    if (position.x < this->x_min || position.x > this->x_max)
        return true;
    if (position.y < this->y_min || position.y > this->y_max)
        return true;
    if (position.z < this->z_min || position.z > this->z_max)
        return true;
    
    return false;
}

// -------------------------------------------------