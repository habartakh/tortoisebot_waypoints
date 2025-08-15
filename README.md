# Python Unit Tests in ROS 

The goal of this package is to write test cases for an action server in ROS.

The /tortoisebot_as action server provides a custom message in order to move the robot to a certain position using the coordinates [X,Y,Z].

The `test` directory contains the necessary files to check if:

- The end position [X, Y] of the robot is the expected one based on the goal sent.
- The end rotation [Yaw] of the robot is the expected one based on the goal sent.

The `waypoints_test.test` file does the following:
- Reset the robot position in the gazebo world.
- Launch an action client that sends a specific goal to the action server.
- Run the tests to check if the final position corresponds to the deried one.   


## Test different cases:



In order to test different cases, it is necessary to modify the desired goal position during tests. 

Here is how to do so:   

Open `waypoints_test.test` file, then change the arguments of the test node to the desired goal position [X, Y] and yaw.   

Then, build and source your workspace.

First, run the action server: 
```
rosrun tortoisebot_waypoints tortoisebot_action_server.py
```

Then run the tests using the following command:
```
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
```

### Test successful conditions 
Here are some of the goal values that the robot successfully reaches:
- X = 0.0 & Y = 0.5 & Yaw = 1.57  

- X = 0.4 & Y = 0.4 & Yaw = 0.78  

- X = -0.4 & Y = -0.4 & Yaw = -2.35  


### Test failing conditions

#### Case 1: X = -0.4 & Y = 0.0  & Yaw = 3.14

The robot is stuck trying to correct its orientation. Due to the absence of angle normalization and the strict correction angle threshold (2 degrees),
The robot angle fluctualtes between pi and -pi values and cannot stabilize. 
The test node then times out and the tests end up failing.

#### Case 2: X = none / Y = none / Yaw = none 

If any of the goal variables does not correspond to float values (e.g: string), the tests end up failing since they expect numerical values.

#### Case 3: X = 100 / Y = 100 

If the goal values are too big or out of the scope of the environment, the robot crashes into the obtacles or the wall while trying to go towards the goal.


