# simluation learning process 
1. taking inspiration from [autocarROS2](https://github.com/winstxnhdw/AutoCarROS2/tree/master/ngeeann_av_gazebo)
2. creating a xacro for the wheels (and making sure it is everything in the right place and turns on the right axis). the file is ackermann_wheels.xacro
3. to test each wheel seperately create a test robot which has only a body and a wheel or two (to see which way it falls and how does it react to forces and torques)
4. after getting the axes in the right direction, a major difficulty is getting all the ode coeffiects make sense (so the robot will move in a physical)
5. testing it come back from a weird rotation helped a lot 
6. try all the coeffiencts, one by one and try to get a sense of their effect in the simulation. 
7. the final robot with all the wheels is my_ackermann. 
8. now it's time to fine tune more the physical coefficients, and calibrate a little bit the ackermann loop. 
9. connecting the ackermann_wheels.xacro to the relevant robot model with it's sensors. 
10. looking at it's tf tree - TF_OLD_DATA ERROR
11. fixing it - making sure every relevant node uses simulated time - use_sim_time: 'True' 
12. more tf errors - the root cause is changes gazebo makes when it generates sdf from urdf. for more [information](https://nu-msr.github.io/me495_site/lecture10_sdf_gazebo.html). my fix is to that every fixed joint in the urdf wheel have a `<preserveFixedJoint>true</preserveFixedJoint>` gazebo tag. in this way the sdf parser won't remove this joints. this implicates to add the must have tags (inertia, visual and collision) for each link in these joints.
