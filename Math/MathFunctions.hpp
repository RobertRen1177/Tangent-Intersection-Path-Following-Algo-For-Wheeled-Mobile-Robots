#pragma once
#include <iostream>
#include <cmath>
#include <vector>

namespace CMath{
    inline constexpr double PI = 3.1415926535; 
    double degToRad(double n);
    double radToDeg(double n);
    double rescale180(double n, bool inRad = false);
    double rescale90(double n, bool inRad = false);
    double rescale360(double n, bool inRad = false);
    double angleFlip(double n, bool inRad = false);
    double sgn(double n);
    bool inRange(double num1, double num2, double threshold);
    double ceiling(double val, double ceiling);
    double floor(double val, double floor);
    double angleError(float angle1, float angle2, bool radians = false);
    double angleErrorDirection(float angle1, float angle2, bool counterClockWise, bool radians = false);
    float avg(std::vector<float> values);
    std::vector<float> solveQuadratic(float a, float b, float c);
    float slew(float target, float current, float maxChange);
    float regularCos(float a, float x);
    float strongerMinimizingCos(float a, float x);
    float mediumMinimzingCos(float a, float x);
}