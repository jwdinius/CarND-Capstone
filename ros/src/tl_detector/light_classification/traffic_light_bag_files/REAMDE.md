To replay the ROS bag, use the following steps:


1. Open a terminal and start `roscore`.
2. Open another terminal, run `rosbag play -l <path_to_your.bag>`
3. Open one more terminal and run `rviz`. You can change the RViz configuration to the Udacity download by navigating to your config file from File > Open Config in RViz. Alternatively, if you'd like to make the Udacity config file your default, you can replace the rviz config file found in ~/.rviz/default.rviz.
4. Start car `cd CarND-Capstone/ros` and `roslaunch launch/site.launch`


