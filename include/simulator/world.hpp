#ifndef WORLD_H
#define WORLD_H

#include <opencv2/opencv.hpp>
#include <iostream>

class World {
private:
    int X, Y;

public:
    World();

    void test();
    int getX() const { return X; } // Get private variable
    int getY() const { return Y; }
};

#endif
