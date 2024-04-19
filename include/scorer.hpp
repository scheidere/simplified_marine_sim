#ifndef SCORER_H
#define SCORER_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>


class Scorer {
private:
    std::string S;

public:
    Scorer();

    void test();

    std::string getS() const { return S; }

};

#endif
