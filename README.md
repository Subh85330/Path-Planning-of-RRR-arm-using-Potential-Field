# Path-Planning-of-RRR-arm-using-Potential-Field
This repository contains the matlab programes to perform the path planning of the robot from initial to final configurations such that it does not hit the obstacles using  potential field (parabolic well potential).

Methodology of the project is given below:
  1. Develop kinematic model of the robot.
  2. First, determine initial and final positions of link origins (o1,  o2 , o3, and oE).
  3. Second, given the final position in the workspace.
    a. Use this to create an attractive potential field. Use parabolic well potential .
  4. Locate obstacle (here three points) in the workspace. 
    a. Create a repulsive potential field. 
  5. Make sure that self-collision of the links are also avoided 
  6. Find forces in the task space and subsequent joint toques. Sum the joint torques in the configuration space. 
  7. Use gradient descent to evolve the present state, and reach your target configuration 8. Animate the configuration for the given pat
  
 # Results
 
  <img src="images/result1_speed_change.gif" width="700" height="500">
