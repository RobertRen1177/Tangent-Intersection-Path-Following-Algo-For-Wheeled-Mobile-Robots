#pragma once
#include <iostream>
#include <cmath>

namespace CMath{
    class Pose{
        public:
            float x;
            float y;
            float theta;
            
            Pose(float x, float y, float theta);

            float angle(Pose p);
            float distance(Pose p);
            float curvature(Pose p);
            bool thetaActive();
            Pose shiftViaAngle(float distance, float angle);
            Pose shiftViaComponents(float x, float y, float theta);
            Pose operator-(Pose p);
            Pose operator*(float scalar);
            float vectorLength();
            Pose normalize();
            Pose shiftTowardsRobot(float distance, Pose robotPose, Pose targetPoint);
    };
}
