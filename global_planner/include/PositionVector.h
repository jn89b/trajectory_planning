#pragma once
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <vector>

class PositionVector
{
    public:
        PositionVector(double x, double y, double z);
        const PositionVector* getPostionVector() {return this;}
        const std::vector<double> getAsVectorDouble() {return {x, y, z};}
        const std::vector<int> getAsVectorInt() {return {int(x), int(y), int(z)};}

        // // comparison to check if two vectors are equal
		inline bool operator== (const PositionVector& v) 
            const { return (x == v.x) && (y == v.y) && (z == v.z); }
		
        inline const PositionVector operator+ (const PositionVector& v) 
            const { return PositionVector(x + v.x, y + v.y, z+v.z); }

        //returns the change 
		static PositionVector getDelta(const PositionVector& v1, const PositionVector& v2) 
        { return PositionVector(abs(v1.x - v2.x), abs(v1.y - v2.y), abs(v1.z - v2.z));}

        double x;
        double y;
        double z;
        // std::vector vector <double>;

};
