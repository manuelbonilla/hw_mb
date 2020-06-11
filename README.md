# hw_mb


## Overview

This repo is temporal. It just perform some test for the UR robot kinematic test. The node includes Part 1-3 from the homework


## Usage

After compillation, it is enough to load 
```roslaunch hw_mb show_robot.launch```


---------------------------------------------


Part 1 can be tested publishing the join references 
`rostopic pub  /moveToJointRef hw_mb/JointPoseRef "{q1: 0.0, q2: 0.0, q3: 0.0, q4: 0.0, q5: 0.0, q6: 0.0, time_exec: 1.0}`
To see streamed cartesian positions just
`rostopic echo /robot_cart_pose `
While joint positions are streamedn in 
`rostopic echo /joint_states`
The status of the controller can be read in
`rostopic echo /movement_completed`

---------------------------------------------

Part 2 is always active and the cartesian distance from the ball to the Robot TCP is streamed in 
`rostopic echo /sphere_dist `
To change the motion of the sphere use
`rosparam set /ball_motion_amplitude/x 1.0`
`rosparam set /ball_motion_amplitude/y 1.0`
`rosparam set /ball_motion_amplitude/z 1.0`


---------------------------------------------


Part 3 can be tested publishing the cartesian reference 
`rostopic pub /moveToCartRef geometry_msgs/Point "x: 0.5 y: 0.5 z: .5" `
This parameter speed up the resolved motion generation
`rosparam set /alpha_cart_to_joint_projection`


---------------------------------------------

This code was tested on Ubuntu 18.04 and ros melodic

