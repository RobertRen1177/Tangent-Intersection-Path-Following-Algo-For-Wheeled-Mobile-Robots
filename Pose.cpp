#include "Math/Pose.hpp"
namespace CMath{
    Pose::Pose(float x, float y, float theta){
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    float Pose::angle(Pose p){
        float deltaX = p.x - this->x;
        float deltaY = p.y - this->y;
        return atan2(deltaX, deltaY);
    }

    float Pose::distance(Pose p){
        float deltaX = this->x - p.x;
        float deltaY = this->y - p.y;
        return sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    float Pose::curvature(Pose p){
        float x1 = this->x;
        float y1 = this->y;
        float m = 1.0 / tan(this->theta * 3.1415926535 / 180.0);
        float x2 = p.x;
        float y2 = p.y;
        if(fabs((y2 - y1) / (x2 - x1) - m) < 0.001){
            return -1; 
        }
        float part1 = pow((-(x1*x1) + 2*x1*x2 - x2*x2 - y1*y1 + 2*y1*y2 - y2*y2), 2);
        float part2 = pow((m*x1*x1 - 2*m*x1*x2 + m*x2*x2 + m*y1*y1 - 2*m*y1*y2 + m*y2*y2), 2);
        float part3 = 4 *pow(((m*x1 - m*x2 - y1 + y2)), 2);
        float r_squared = (part1 + part2) / part3; 
        return 1.0 / sqrt(r_squared);
    }

    Pose Pose::shiftViaAngle(float distance, float angle){
        float xMove = distance * sin(angle);
        float yMove = distance * cos(angle);
        Pose newPose = Pose(this->x + xMove, this->y + yMove, this->theta);
        return newPose; 
    }

    Pose Pose::shiftViaComponents(float x, float y, float theta){
        this->x += x;
        this->y += y;
        this->theta += theta; 
        return Pose(this->x + x, this->y + y, this->theta + theta);
    }

    bool Pose::thetaActive(){
        return fabs(this->theta) <= 360;
    }

    Pose Pose::operator-(Pose p) {
        return Pose(this->x - p.x, this->y - p.y, 361);
    }

    Pose Pose:: operator*(float scalar){
        return Pose(this->x * scalar, this->y * scalar, 361);
    }

    float Pose::vectorLength(){
        return std::sqrt(this->x * this->x + this->y * this->y);
    }

    Pose Pose::normalize(){
        float len = this->vectorLength();
        return Pose(this->x / len, this->y / len, 361);
    }

    Pose Pose::shiftTowardsRobot(float distance, Pose robotPose, Pose targetPoint){
        
        Pose direction = targetPoint - robotPose;
        Pose normalizedDirection = direction.normalize();
        Pose scaledDirection = normalizedDirection * distance;
        return targetPoint - scaledDirection;
    }
}