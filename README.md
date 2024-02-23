# Tangent-Intersection-Method-For-Path-Following

In the Repo above I’ve created a new algorithm of following continuous and structured paths for wheeled mobile robots.
The benefits of this algorithm compared to others are listed later in the document. 

Now here’s the actual algorithm explained:

Note: in my explanation I use a Cubic Bezier curve as the path to follow. 

1. Given the curve B(X), we find the line tangent to the very last point on this curve. This line will be called line_endingPointTangent. 
![First Step Of Algorithm](Images/Step1.png)
2.	Given our robots current position (x_start, y_start), we find the point that we are closest to on the curve B(X). This point is called (x_closest, y_closest).
![Second Step Of Algorithm](Images/Step2.png)
3.	We then find the line tangent to (x_closest, y_closest), called line_closestPointTangent
![Third Step Of Algorithm](Images/Step3.png)
4.	We then find the signed cross track error of our robot to the curve. We then rotate the line_closetPointTangent by an angle proportional to the signed cross track error to minimize cross track error. (This step is not illustrated above as it is explained in greater later in the document)
5.	The intersection point between the two lines is the carrot point that our robot attempts to follow. 
![Fifth Step Of Algorithm](Images/Step5.png)
6.	We calculate our angular and linear power needed to move to the carrot point and we then input them to the control system. 
7.	This is then repeated in a while loop until we reach the end of the curve. (So essentially, we are following a dynamically changing carrot point to follow our path). 


The whole Algorithm is modeled in the pictures (without step 4 for now for clarity) shown below: 
![Whole Algorithm](Images/WholeAlgorithm.png)
In the given picture, the blue curve is B(X)
The red line is line_endingPointTangent
The purple point labeled robot is our starting point.
And the green line is the tangent line closest to our robots’ starting position. 
And the carrot point is at the intersection of the green and red lines. 

The following images show more carrot points as the robot moves along the curve.
![Extra Steps Modeled](Images/MoreCarrotPoints1.png)
![Extra Steps Modeled](Images/MoreCarrotPoints2.png)
This is the Desmos to model this situation: https://www.desmos.com/calculator/axcqmxr9wz (scroll down to the t_c variable for more control)
As you can see, by following the carrot point created by the intersection of these two lines, the robot naturally follows the curve. It is constantly moving along the derivatives of the curve which means it will follow out the path created by the curve. 

Furthermore, it is extremely hard for the robot to stray away from the curve, as the robot is mathematically bounded by the ending point tangent line. In other words, due to the nature of carrot point generations, the robot is always behind the carrot point. And since the carrot points are all generated on the ending point tangent line, the robot will always be below the ending point tangent line (which means  it is bounded below the tangent line). This means that the robot will always be in the region below the ending point tangent line, which allows for more consistency, since you are guaranteed to be in a specific region while and after traversing the path. 
This also means that path following will be much more accurate even in the event of catastrophic failure, as the robot wont stray away extremely far from the path since the robot will be bounded by the tangent line (if the robot gets knocked above the tangent line, carrot point generation forces the robot to seek back underneath the tangent line). 
Another way to look at this is if you play the Desmos simulation, you can see that the robot is always below and behind the carrot point. And since the carrot point is always going to be on the ending tangent line, the robot physically won’t be able to go pass the ending point tangent line, meaning it is bounded by this line and thus has the benefits mentioned above. 
![Bounded by tangent](Images/Bounded.png)

The even better part of this is, there are SOME instances where by following the carrot point, the robot implicitly moves closer to the curve, limiting cross track error. If the blue point was the robot, it is implicitly moving closer to the curve. However, this is only true some of the time. For a more general way of moving closer to the curve, we have a cross track minimization algorithm which is explained later in the document. 
![Implicit XTE limiting](Images/ImplicitXTECorrection.png)

This algorithm is extremely well suited for differential drive robots. Since we are simply just moving to a dynamically changing point, there are no steering limitations or need for doing so. This means that there are no constraints on the angular velocity, allowing differential drives to move to their fullest potential. 
Furthermore, the algorithm is very simple and easy to implement, as it requires no feedforward or velocity control (by this, I mean you don’t need to know the current velocity of your robot and need to know how to convert voltage into the velocity of your robot). Controlling the robot simply through a plain PID voltage control, without the added complexity of converting it to the robot’s actual velocity, is more than enough to accurately follow the path. Additionally, the general idea of the algorithm is easy to implement because the algorithm is just effectively moving to a dynamically changing carrot point, meaning a large portion of the control section of code is just a simple move to point algorithm, which almost anyone can do. 
The algorithm is also extremely memory and time efficient if you are using a closed form curve as your path, like a Bezier curve, as you are not required to pre generate the points on the path before starting the motion algorithm, like you do with some algorithms like motion profiling. The only thing the algorithm needs is the derivative of the Bezier curve at a constant point (the end), and the closest point to the robot on the Bezier curve, and its derivative. This information can all be retrieved in O(1) time, as it goes along the curve using the closed form solution, meaning we can save tons of time and memory. 


