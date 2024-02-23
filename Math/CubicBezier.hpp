#pragma once
#include <utility>
#include <cmath>
#include "Math/MathFunctions.hpp"
#include <iostream>
#include <algorithm>
#include <vector>
#include <stdexcept>
class VecPoint {
public:
    float x;
    float y;
    VecPoint() {
        this->x = 0;
        this->y = 0;
    }
    VecPoint(float x, float y) {
        this->x = x;
        this->y = y;
    }
    float distance(VecPoint p) {
        return sqrt((this->x - p.x) * (this->x - p.x) + (this->y - p.y) * (this->y - p.y));
    }
    VecPoint operator*(float scalar) const {
        return { x * scalar, y * scalar };
    }

    float operator*(VecPoint p) const {
        return (this->x * p.x + this->y * p.y);
    }

    float cross(VecPoint p) {
        return this->x * p.y - this->y * p.x;
    }

    VecPoint operator+(const VecPoint& other) const {
        return { x + other.x, y + other.y };
    }

    VecPoint operator-(const VecPoint& other) const {
        return { x - other.x, y - other.y };
    }
};

class Line {
public:
    float slope;
    float yIntercept;

    Line(float slope, float yIntercept) {
        this->slope = slope;
        this->yIntercept = yIntercept;
    }
    Line(VecPoint p1, VecPoint p2) {
        this->slope = (p1.y - p2.y) / (p1.x - p2.x);
        this->yIntercept = p1.y - this->slope * p1.x;
    }
    VecPoint intersection(Line other) {
        // Check for parallel lines
        if (this->slope == other.slope) {
            if (this->yIntercept == other.yIntercept) {
                // Lines are on top of each other
                throw std::runtime_error("Lines are coincident");
            }
            else {
                // Lines are parallel
                throw std::runtime_error("Lines are parallel and do not intersect");
            }
        }
        // Calculate the intersection point
        double x = (other.yIntercept - this->yIntercept) / (this->slope - other.slope);
        double y = this->slope * x + this->yIntercept;

        return VecPoint(x, y);
    }

    float getValue(float x) {
        return this->slope * x + this->yIntercept;
    }
};

class CubicBezier {
public:
    // Constructor
    CubicBezier(VecPoint p1, VecPoint p2, VecPoint p3, VecPoint p4, float numIterations = 500) {
        this->P0 = p1;
        this->P1 = p2;
        this->P2 = p3;
        this->P3 = p4;
        this->numIterations = numIterations;
        //this->findExtrema();
    }

    // Find a point on the curve
    VecPoint getPoint(float t) {
        float u = 1 - t;
        float tt = t * t;
        float uu = u * u;
        float uuu = uu * u;
        float ttt = tt * t;

        VecPoint point = this->P0 * uuu; // First term
        point = point + P1 * (3 * uu * t); // Second term
        point = point + P2 * (3 * u * tt); // Third term
        point = point + P3 * ttt; // Fourth term

        return point;
    }

    // Find the derivative at point t
    VecPoint getDerivative(float t) {
        float u = 1 - t;

        VecPoint point = P1 - P0;
        point = point * (3 * u * u);

        point = point + (P2 - P1) * (6 * u * t);
        point = point + (P3 - P2) * (3 * t * t);

        return point;
    }

    // Find the second derivative at point t
    VecPoint getSecondDerivative(float t) {
        return (P0 - P1 * 2.0f + P2) * (6.0f * (1 - t)) + (P1 - P2 * 2.0f + P3) * (6.0f * t);
    }

    //get distance derivative
    float distanceDerivative(float t, VecPoint Q) {
        VecPoint Bt = this->getPoint(t);
        VecPoint Bt_prime = this->getDerivative(t);
        return ((Bt - Q) * Bt_prime) * 2;
    }

    //distance second derivative
    float distanceSecondDerivative(float t, VecPoint Q) {
        VecPoint Bt = this->getPoint(t);
        VecPoint Bt_prime = this->getDerivative(t);
        VecPoint Bt_double_prime = this->getSecondDerivative(t);
        return 2 * ((Bt_prime * Bt_prime) + (Bt - Q) * Bt_double_prime);
    }

    // Return the equation of the tangent line at t
    Line getTangentLine(float t) {
        VecPoint tangent = this->getDerivative(t);
        VecPoint pointOnCurve = this->getPoint(t);

        double slope = tangent.y / tangent.x;
        double yIntercept = pointOnCurve.y - slope * pointOnCurve.x;
        return Line(slope, yIntercept);
    }



    void findExtrema() {
        //find the zeros where B_x(t) = 0, and B_y(t) = 0
        VecPoint a = this->P3 * 3 - this->P2 * 9 + this->P1 * 9 - this->P0 * 3;
        VecPoint b = this->P0 * 6 - this->P1 * 12 + this->P2 * 6;
        VecPoint c = this->P0 * -3 + this->P1 * 3;

        std::vector<float> solutions = CMath::solveQuadratic(a.x, b.x, c.x);
        std::vector<float> ySolutions = CMath::solveQuadratic(a.y, b.y, c.y);
        for (int i = 0; i < ySolutions.size(); i++) {
            solutions.push_back(ySolutions[i]);
        }
        if (solutions.empty()) {
            return;
        }

        std::sort(solutions.begin(), solutions.end());
        for (int i = 0; i < solutions.size(); i++) {
            if (solutions[i] < 0.0 || solutions[i] > 1.0) {
                solutions.erase(solutions.begin() + i);
                i--;
            }
            else {
                std::cout << solutions[i] << '\n';
            }
        }

        //then split the curves at the t values where this happens, and store them in the class variable
        std::vector<VecPoint> controlPoints{ this->P0, this->P1, this->P2, this->P3 };
        for (int i = 0; i < solutions.size(); i++) {
            float t = solutions[i];
            VecPoint p0 = controlPoints[0];
            VecPoint p1 = controlPoints[1];
            VecPoint p2 = controlPoints[2];
            VecPoint p3 = controlPoints[3];
            VecPoint P01 = p0 * (1 - t) + p1 * t;
            VecPoint P12 = p1 * (1 - t) + p2 * t;
            VecPoint P23 = p2 * (1 - t) + p3 * t;
            VecPoint P012 = P01 * (1 - t) + P12 * t;
            VecPoint P123 = P12 * (1 - t) + P23 * t;
            VecPoint P0123 = P012 * (1 - t) + P123 * t;

            CubicBezier newCurve = CubicBezier(p0, P01, P012, P0123);
            this->extremaCurves.push_back(newCurve);
            controlPoints[0] = P0123;
            controlPoints[1] = P123;
            controlPoints[2] = P23;
            controlPoints[3] = p3;
        }
        this->extremaCurves.push_back(CubicBezier(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3]));


    }


    //find the smallest cross track error given search space, using newton raphson method, and the starting search t value (signed)
    //initial guess is most likely going to be the previous guess
    float smallestDistance(VecPoint Q, float initialGuess, float maxIterations = 5, float tolerance = 1e-6) {
        float t = initialGuess;
        for (int i = 0; i < maxIterations; i++) {
            float f = this->distanceDerivative(t, Q);
            float f_prime = this->distanceSecondDerivative(t, Q);
            if (fabs(f_prime) < 1e-6) {
                break;
            }
            float t_new = t - f / f_prime;
            t_new = std::fmax(0.0, std::fmin(t_new, 1.0));
            if (fabs(t - t_new) < tolerance) {
                return t_new;
            }

            t = t_new;
        }
        return t;
    }

public:
    VecPoint P0;
    VecPoint P1;
    VecPoint P2;
    VecPoint P3;
    float numIterations;
    float totalCurveArcLength;
    std::vector<float> distanceTable; // To store the precomputed distances
    std::vector<CubicBezier> extremaCurves; // To store the t values of extrema
};