#pragma once

#define MAX_ERROR 0.001

#include <cmath>
#include <iostream>

float func(float x)
{
    return x*x*x - x*x + 2;
}

float derivative(float x)
{
    return 3*x*x - 2*x;
}

void newtonRaphson(float x)
{
    float r = func(x) / derivative(x);

    while (fabs(r) >= MAX_ERROR)
    {
        r = func(x) / derivative(x);

        x = x - r;
        std::cout << "Root value: " << x << std::endl;
    }

    std::cout << "Root value: " << x << std::endl;
}
