#include "GridMap.h"
#include <math.h>


/* -------------------------------------------------
    Agent CLASS
    -------------------------------------------------
*/
Agent::Agent(int x, int y, int z, 
            int goal_x, int goal_y, int goal_z, 
            double agent_radius)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->position = PositionVector(this->x, this->y, this->z);
    this->goal_position = PositionVector(goal_x, goal_y, goal_z);
    this->agent_radius = agent_radius;
}

// -------------------------------------------------

std::vector<PositionVector> Agent::getMoves3D(PositionVector current_pos)
{   
    // loop through i,j,k and add to moves vector
    std::vector<PositionVector> moves;
    int min = -1;
    int max = 2;
    for (int i = min; i < max; i++)
    {
        for (int j = min; j< max; j++)
        {
            for (int k = min; k < max; k++)
            {
                if (i == current_pos.x && 
                    j == current_pos.y && 
                    z == current_pos.z )
                    continue;

                if (i == current_pos.x && 
                    j == current_pos.y)
                    continue;
                else
                {
                    double x = current_pos.x + i;
                    double y = current_pos.y + j;
                    double z = current_pos.z + k;
                    PositionVector move(x,y,z);
                    moves.push_back(move);
                }
            }
        }
    }

    return moves;
}

// -------------------------------------------------

// return a list of possible moves in 2D
std::vector<PositionVector> Agent::getMoves2D(PositionVector current_pos)
{
    std::vector<PositionVector> moves;
    int min = -1;
    int max = 2;
    
    for (int i=min; i<max; i++)
    {
        for (int j=min; j<max; j++)
        {
            if (i == current_pos.x && j == current_pos.y)
                continue;
            else
            {
                double x = current_pos.x + i;
                double y = current_pos.y + j;
                PositionVector move(x,y,0);
                moves.push_back(move);
            }
        }
    }

    return moves;
}

// -------------------------------------------------

/* -------------------------------------------------
    OBSTACLE CLASS
    -------------------------------------------------
*/

// -------------------------------------------------
Obstacle::Obstacle(int x, int y, int z, double radius, int height)        
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->radius = radius;
    this->height = height;

}

// -------------------------------------------------

bool Obstacle::isInside2D(PositionVector position, double robot_radius)
{
    double total_radius = this->radius + robot_radius;

    double dist = sqrt(pow(position.x - this->x, 2) 
    + pow(position.y - this->y, 2)) ;

    if (dist <= total_radius)
        return true;
    else
        return false;
}

// -------------------------------------------------

bool Obstacle::isInside3D(PositionVector position, double rob)
{
    double total_radius = this->radius + rob;

    double dist = sqrt(pow(position.x - this->x, 2) + 
        pow(position.y - this->y, 2) + 
        pow(position.z - this->z, 2));

    if (dist <= total_radius)
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
    //initialize the grid si    ze map
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

int GridMap::insertObstacle(Obstacle* obstacle)
{
    this->obstacle_list.push_back(obstacle);
    printf("Inserting Obstacle %d, %d, %d\n",  
    obstacle->getX(), obstacle->getY(), obstacle->getZ());



    return int(this->obstacle_list.size());
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

bool GridMap::isInsideObstacles(PositionVector position, float radius)
{
    for (int i=0; i<obstacle_list.size(); i++)
    {
        if (obstacle_list[i]->isInside2D(position))
            return true;
    }

    return false;
}

// -------------------------------------------------

Obstacle* GridMap::getObstacle(int index)
{
    Obstacle* obs = obstacle_list[index] ;
    return obs;  
}

