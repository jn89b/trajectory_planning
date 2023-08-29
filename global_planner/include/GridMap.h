#pragma once
#include <iostream>
#include <stdio.h>
#include <vector>
#include <PositionVector.h>

/* -------------------------------------------------
    Agent CLASS
    -------------------------------------------------
*/
class Agent 
{
    private:
        int x;
        int y;
        int z;
        
        int goal_x;
        int goal_y;
        int goal_z;

        double agent_radius = 0.0f; // default value
        PositionVector position = PositionVector(
                                    x, y, z);
        PositionVector goal_position = PositionVector(
                                    goal_x, goal_y, goal_z);

    public:
        // Constructor if you want set as position vectors
        Agent(PositionVector pos, 
            PositionVector goal_position, 
            double agent_radius=0.0f) 
            {
                this->x = pos.x;
                this->y = pos.y;
                this->z = pos.z;
                this->position = pos;
                this->goal_position = goal_position;
                this->agent_radius = agent_radius;
            }
        
        // Constructor if you want set as individual doubles
        Agent(int x, int y, int z, 
            int goal_x, int goal_y, int goal_z, 
            double agent_radius=0.0f);
        
        double getX() {return x;}
        double getY() {return y;}
        double getZ() {return z;}
        
        //setters and getters for radius of agent in meters
        void setRadius(double radius) {agent_radius = radius;}
        double getRadius() {return agent_radius;}
        
        //setter getters and update for position
        PositionVector getPosition() {return position;}
        void setPosition(PositionVector pos) {position = pos;}
        void setPosition(double x, double y, double z) 
            {position = PositionVector(x, y, z);}
        void updatePosition(PositionVector new_pos) {position = new_pos;}
        void updatePosition(double x, double y, double z) 
            {position = PositionVector(x, y, z);}

        //setter getters and update for goal position
        void setGoalPosition(PositionVector goal_pos) 
            {goal_position = goal_pos;}
        void setGoalPosition(int x, int y, int z) 
            {goal_position = PositionVector(x, y, z);}
        PositionVector getGoalPosition() {return goal_position;}
        void updateGoalPosition(PositionVector new_pos) 
            {goal_position = new_pos;}
        void updateGoalPosition(int x, int y, int z)
            {goal_position = PositionVector(x, y, z);}


        // return a vector of possible moves in 3D
        std::vector<PositionVector> getMoves3D(PositionVector current_pos);

        // return a list of possible moves in 2D
        std::vector<PositionVector> getMoves2D(PositionVector current_pos);

};


class FWAgent 
{
    private:
        int x;
        int y;
        int z;
        
        int goal_x;
        int goal_y;
        int goal_z;

        double agent_radius = 0.0f; // default value
        PositionVector position = PositionVector(
                                    x, y, z);
        PositionVector goal_position = PositionVector(
                                    goal_x, goal_y, goal_z);

        float theta_dg;
        float psi_dg;

        float max_psi_turn_dg;
        float leg_segment_m;

    public:
        // Constructor if you want set as position vectors
        FWAgent(PositionVector pos, 
            PositionVector goal_position, 
            double agent_radius=0.0f, 
            float theta_dg=0.0f, 
            float psi_dg=0.0f, 
            float max_psi_turn_dg=45.0f,
            float leg_segment_m=10.0f) 
            {
                this->x = pos.x;
                this->y = pos.y;
                this->z = pos.z;
                this->position = pos;
                this->goal_position = goal_position;
                this->agent_radius = agent_radius;
                this->theta_dg = theta_dg;
                this->psi_dg = psi_dg;
                this->max_psi_turn_dg = max_psi_turn_dg;
                this->leg_segment_m = leg_segment_m;
            }
        
        // Constructor if you want set as individual doubles
        FWAgent(int x, int y, int z, 
            int goal_x, int goal_y, int goal_z, 
            double agent_radius=0.0f, 
            float theta_dg=0.0f,
            float psi_dg=0.0f, 
            float max_psi_turn_dg=45.0f,
            float leg_segment_m=5.0f);
        
        void setMaxPsiTurnDg(float max_psi_turn_dg) 
            {this->max_psi_turn_dg = max_psi_turn_dg;}
        float getMaxPsiTurn(){return max_psi_turn_dg;}

        void setLegSegmentm(float leg_segment_m) 
            {this->leg_segment_m = leg_segment_m;}
        float getLegSegmentm(){return leg_segment_m;}
        
        double getX() {return x;}
        double getY() {return y;}
        double getZ() {return z;}
        
        //setters and getters for radius of agent in meters
        void setRadius(double radius) {agent_radius = radius;}
        double getRadius() {return agent_radius;}

        void setTheta(float theta) {theta_dg = theta;}
        float getTheta() {return theta_dg;}

        void setPsi(float psi) {psi_dg = psi;}
        float getPsi() {return psi_dg;}
        
        //setter getters and update for position
        PositionVector getPosition() {return position;}
        void setPosition(PositionVector pos) {position = pos;}
        void setPosition(double x, double y, double z) 
            {position = PositionVector(x, y, z);}
        void updatePosition(PositionVector new_pos) {position = new_pos;}
        void updatePosition(double x, double y, double z) 
            {position = PositionVector(x, y, z);}

        //setter getters and update for goal position
        void setGoalPosition(PositionVector goal_pos) 
            {goal_position = goal_pos;}
        void setGoalPosition(int x, int y, int z) 
            {goal_position = PositionVector(x, y, z);}
        PositionVector getGoalPosition() {return goal_position;}
        void updateGoalPosition(PositionVector new_pos) 
            {goal_position = new_pos;}
        void updateGoalPosition(int x, int y, int z)
            {goal_position = PositionVector(x, y, z);}

        // return a vector of possible moves in 3D
        std::vector<PositionVector> getMoves3D(
            PositionVector current_pos, float curr_psi_dg,
            int step_psi_dg=5);

};

/* -------------------------------------------------
    OBSTACLE CLASS
    -------------------------------------------------
*/
class Obstacle
{
    //--------------PUBLIC ----------/
    public:        
        // Constructor
        Obstacle(int x, int y, int z, double radius, int height);        
        
        int getX() const {return x;} 
        int getY() const {return y;} 
        int getZ() const {return z;} 
        int getRadius() const {return radius;} 
        int getHeight() const {return height;} 
        bool isInside2D(PositionVector position, double robot_radius=0.0f);
        bool isInside3D(PositionVector position, double robot_radius=0.0f);

    
    //--------------PUBLIC METHODS ----------/
    private:
        double x;
        double y;
        double z;
        double radius;
        double height;
    
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
        int insertObstacle(Obstacle* obstacle);
        // returns the number of obstacles in the list
        int getNumObstacles() {return int(obstacle_list.size());}
        // returns the list of obstacles
        const std::vector<Obstacle*> getObstacles() {return obstacle_list;}

        // Checks if the position is outside the grid 
        // returns true if it is outside the grid
        bool isOutBounds(PositionVector position);

        // Checks the position is inside the obstacle list 
        // returns true if it is inside the obstacle list
        // false if it is not inside the obstacle list
        bool isInsideObstacles(PositionVector position, float radius=0.0f);

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
