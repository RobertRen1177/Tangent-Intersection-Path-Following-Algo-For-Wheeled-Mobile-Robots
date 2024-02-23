#include "Math/CubicBezier.hpp"
#include "Math/Pose.hpp"

float angularVoltageConstant = 2;
float linearVoltageConstant = 8; 
void follow(CubicBezier curve, bool forwards, float timeout, bool splitCurve, CMath::Pose currentPosition){
   if(splitCurve){
        curve.findExtrema();
    }
    float closestTVal = 0.0;
    float prevClosestTVal = 0.0;
    float currentPathIndex = 0;
    float maxPathIndexes = curve.extremaCurves.size(); //length of the curve path thingy
    CubicBezier currentPath = maxPathIndexes == 0 ? curve : curve.extremaCurves[currentPathIndex];
    bool exitPath = false; 

    CMath::Pose endingPoint = CMath::Pose(curve.getPoint(1.0).x, curve.getPoint(1.0).y, 361);
    int timer = 0;
    while(timer < timeout){
        //First phase of the algorithm: find the cross track error and closet t value
        CMath::Pose currentPose = currentPosition; 
        if(!forwards) {
            currentPose.theta += CMath::PI; 
        }
        currentPose.theta = CMath::rescale180(currentPose.theta, true);
        VecPoint currentPoint = VecPoint(currentPose.x, currentPose.y);

        //previous closest t value is the initial guess
        while(true){
            closestTVal = currentPath.smallestDistance(currentPoint, prevClosestTVal);
            //if the closestTVal is equal to 1.0 (or greater) then we need to move on the next path
            if(closestTVal >= 0.985){
                //we finished entirity of the pat
                if(currentPathIndex == maxPathIndexes){
                    exitPath = true; 
                    break;
                }
                //we only finished a portion of the path, and we need to move on to the next portoin
                currentPathIndex++; 
                currentPath = curve.extremaCurves[currentPathIndex];
                prevClosestTVal = 0.0; 
            }
            else{ //TVal is between 0 and 1, so we take our closestTVal and dip
                break;
            }
        }
        prevClosestTVal = closestTVal; //used as the next estimation next round
        
        if(exitPath){
            break;
        }

        //signed cross track error, sign of distanceVector x tangentVector = sign of error
        VecPoint tangentVector = currentPath.getDerivative(closestTVal);
        VecPoint distanceVector = currentPoint - currentPath.getPoint(closestTVal);
        float signedCrossTrackError = currentPoint.distance(currentPath.getPoint(closestTVal)); //not signed yet
        signedCrossTrackError *= CMath::sgn(tangentVector.cross(distanceVector));

        //phase 2, finding the carrot point
        Line endPointTangentLine = currentPath.getTangentLine(1.0); //line through the end point
        Line closestPointLine = currentPath.getTangentLine(closestTVal);

        //the carrot point that we want to follow
        VecPoint carrotPoint = endPointTangentLine.intersection(closestPointLine);
        CMath::Pose carrotPointPose = CMath::Pose(carrotPoint.x, carrotPoint.y, 361);

        //write code to move towards carrotPoint (or turn towards carrot point)
        float angularError = CMath::angleError(currentPose.angle(carrotPointPose), currentPose.theta, true);
        float distanceError = currentPose.distance(endingPoint);

        float angularPower = angularVoltageConstant * (CMath::radToDeg(angularError));
        float lateralPower = linearVoltageConstant * (distanceError) * (forwards ? 1 : -1);
        lateralPower *= fabs(cos(angularError));

        float leftPower = lateralPower + angularPower;
        float rightPower = lateralPower - angularPower;

        //MOVE ROBOT WITH THE GIVEN ANGULAR POWER AND LATERAL POWER
        //DID NOT IMPLEMENT BECAUSE IMPLEMENTATION TO MOVE MOTORS BY VOLTAGE IS DIFFERENT FOR EVERY SYSTEM
        
        timer += 10;

    }

}