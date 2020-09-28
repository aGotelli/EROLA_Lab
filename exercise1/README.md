# Monday 28/09 Exercise

The exercise will be about a control of a holonomic robot in a 2d space. We want the robot to reach the target, and then ask for a new, random, target.
We will test the system with a simple 2d simulator, Stage. 

The node controlling the robot will subscribe to the topic **/odom** and it will publish the velocity on the topic **/cmd_vel**.

Please follow these steps:

1. Create a package, called **exercise1** with dependencies: *geometry_msgs*, *nav_msgs*, *roscpp*.
2. Put the source code **exercise.cpp** in the src directory of the new package.
3. Put the files **exercise.world** and **uoa_robotics_lab.png** in the world folder of the new package (you should create the folder).
4. Create another package, with dependencies: *roscpp*, *std_msgs*, *message_generation*.
5. Put the source code **PositionServer.cpp** in the src directory of the new package.
6. Now, create a service message (using this second package) with an empty request, and two floats (x and y) as response.
7. Modify **PositionServer.cpp** so as to implement a Ros service which use the service message that you have defined to return two random floats, x and y, respectively between *x_min* and *x_max* and *y_min* and *y_max* (the function **RandomFloat** is already defined). Some code should be added when you see comments in the form:
```
/* Bla bla bla

*/
```
8. Now modify **exercise1.cpp** so as to reach the target with the robot. When the target is reached, the node should require the service to send a new target position.
9. If needed, modify the **CMakeLists.txt** and **package.xml** of the two packages.

You can test the sistem with the simulator.
```
rosrun stage_ros stageros $(rospack find exercise1)/world/exercise.world
```
When launching the node controlling the robot, also set the parameters */exercise1/xt* and */exercise1/yt*. You can do that by running
```
rosrun exercise1 <name of the node> _xt:=2.0 _yt:=3.0
```
Have fun!
