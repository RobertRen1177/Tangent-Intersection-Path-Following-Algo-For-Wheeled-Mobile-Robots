#include "Math/MathFunctions.hpp"
namespace CMath {
    double degToRad(double n) {
        return n * CMath::PI / 180.0;
    }

    double radToDeg(double n) {
        return n * 180.0 / CMath::PI;
    }

    double rescale180(double n, bool inRad) {
        n = inRad ? radToDeg(n) : n;
        n = n - 360.0 * std::floor((n + 180.0) * (1.0 / 360.0));
        if (inRad) { n = degToRad(n); }
        return n;
    }

    double rescale90(double n, bool inRad) {
        n = inRad ? radToDeg(n) : n;
        if (fabs(n) > 90) {
            n += 180;
            n = rescale180(n, false);
        }
        if (inRad) { n = degToRad(n); }
        return n;
    }

    double angleFlip(double n, bool inRad) {
        n = inRad ? radToDeg(n) : n;
        n = rescale180(n + 180, false);
        if (inRad) { n = degToRad(n); }
        return n;
    }

    double rescale360(double n, bool inRad) {
        n = inRad ? radToDeg(n) : n;
        double ans;
        if (n >= 0) {
            ans = n - 360 * (int(n / 360));
        }
        else {
            ans = 360 + (n - 360 * (int(n / 360)));
        }
        if (inRad) { return degToRad(ans); }
        return ans;
    }

    double sgn(double n) {
        return (n > 0) - (n < 0);
    }

    bool inRange(double num1, double num2, double threshold) {
        return fabs(num1 - num2) < threshold;
    }

    double ceiling(double val, double ceiling) {
        if (val > ceiling) {
            return ceiling;
        }
        return val;
    }

    double floor(double val, double floor) {
        if (val < floor) {
            return floor;
        }
        return val;
    }

    double angleError(float angle1, float angle2, bool radians) {
        float max = radians ? 2 * CMath::PI : 360;
        float half = radians ? CMath::PI : 180;
        angle1 = fmod(angle1, max);
        angle2 = fmod(angle2, max);
        float error = angle1 - angle2;
        if (error > half) error -= max;
        else if (error < -half) error += max;
        return error;
    }

    double angleErrorDirection(float angle1, float angle2, bool counterClockWise, bool radians) {
        float holder = radians ? 2 * CMath::PI : 360;
        angle1 = fmod(angle1, holder);
        angle2 = fmod(angle2, holder);
        float delta = angle1 - angle2;
        float returnVal;
        if (delta < 0) {
            returnVal = (counterClockWise) ? delta : (holder + delta); //adding since delta is negative
        }
        else {
            returnVal = (counterClockWise) ? (delta - holder) : delta;
        }
        return returnVal;
    }

    float avg(std::vector<float> values) {
        float sum = 0;
        for (float v : values) {
            sum += v;
        }
        return sum / values.size();
    }

    float slew(float target, float current, float maxChange) {
        float change = target - current;
        if (maxChange == 0) return target;
        if (change > maxChange) change = maxChange;
        else if (change < -maxChange) change = -maxChange;
        return current + change;
    }

    std::vector<float> solveQuadratic(float a, float b, float c) {
        std::vector<float> solutions;
        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return solutions;
        }
        else if (discriminant == 0) {
            solutions.push_back(-1.0 * b / 2.0 * a);
            return solutions;
        }

        if (a == 0.0) {
            solutions.push_back(-1.0 * c / b);
            return solutions;
        }
        else {
            float root1 = (-1 * b + sqrt(discriminant)) / (2 * a);
            float root2 = (-1 * b - sqrt(discriminant)) / (2 * a);
            solutions.push_back(root1);
            solutions.push_back(root2);
            return solutions;
        }
    }

    float regularCos(float a, float x){
        if(x >= PI / (2 * a)){
            return 0;
        }
        return cos(a * x);
    }
    float strongerMinimizingCos(float a, float x){
        if(x >= PI / (2 * a)){
            return 0;
        }
        return cos(a * x + PI/2) + 1;
    }

    float mediumMinimzingCos(float a, float x){
        if(x >= PI / (2 * a)){
            return 0;
        }
        return cos(a * x) * cos(a * x);
    }
}