#pragma once
#include <iostream>
#include <stdio.h>
#include <vector>
#include <PositionVector.h>

/* -------------------------------------------------
    OBSTACLE CLASS
    -------------------------------------------------
*/
class Obstacle
{
    //--------------PUBLIC METHODS ----------/
    private:
    double x;
    double y;
    double z;
    double radius;
    double height;
    
    public:
    //--------------PRIVATE METHODS ----------/
    
    // Constructor
    Obstacle(double x, double y, double z, double radius, double height);        
    
    double getX() {return x;}
    double getY() {return y;}
    double getZ() {return z;}
    double getRadius() {return radius;}
    double getHeight() {return height;}
    bool isInside2D(PositionVector position);
    bool isInside3D(PositionVector position);

};

/* -------------------------------------------------
    GRIDMAP CLASS
    -------------------------------------------------
*/
class GridMap
{
    // ---- PUBLIC METHODS-------/
    public:

        // Constructor
        GridMap(); // Sets the grid size based on the user input 
        // if value is negative, return false
        bool setGridSize(int x_min, int y_min, int z_min, 
                        int x_max, int y_max, int z_max);

        // return the size of x 
        int getGridSizeX() {return x_max - x_min;}
        // return the size of y
        int getGridSizeY() {return y_max - y_min;}
        // return the size of z
        int getGridSizeZ() {return z_max - z_min;}
        
        // return min and max values of x, y, z
        int getGRID_MIN_X() {return x_min;}
        int getGRID_MIN_Y() {return y_min;}
        int getGRID_MIN_Z() {return z_min;}

        int getGRID_MAX_X() {return x_max;}
        int getGRID_MAX_Y() {return y_max;}
        int getGRID_MAX_Z() {return z_max;}

        // Inserts an obstacle into the obstacle list 
        // and returns the number of obstacles in the list
        int insertObstacle(Obstacle& obstacle);
        // returns the number of obstacles in the list
        int getNumObstacles() {return int(obstacle_list.size());}

        // Checks if the position is outside the grid 
        // returns true if it is outside the grid
        bool isOutBounds(PositionVector position);
        
        // returns the pointer obstacle based on the index
        Obstacle* getObstacle(int index);

        ~GridMap() {printf("GridMap destructor called\n");}

    // ---- PRIVATE METHODS-------/
    private:
        int x_min;
        int y_min;
        int z_min;
        int x_max;
        int y_max;
        int z_max;
    
        std::vector<Obstacle*> obstacle_list;


};


#ifdef ENABLE_DOCTEST_IN_LIBRARY
#include "doctest/doctest.h"
TEST_CASE("we can have tests in headers if we want")
{
    Dummy d;
    CHECK(d.doSomething() == true);
}
#endif
