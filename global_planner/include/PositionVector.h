#pragma once

class PositionVector
{
    public:
        PositionVector(double x, double y, double z);
        PositionVector* getPostionVector() {return this;}
        double x;
        double y;
        double z;

};