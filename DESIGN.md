ROS Topics
==========

- /
	- autonomous (set whether mode is autonomous or not)
	- mobile\_base (turtlebot namespace)
		- data (data namespace)
			- pose [i2r_common/Pose] [published] (pose data topic)
			- twist [geometry_msgs/Twist] [published] (twist data topic)
		- commands (command namespace)
			- velocity [geometry_msgs/Twist] [subscribed] (velocity command topic)
	- obstacles (obstacle namespace)
		- positions [turtle_rv/ObstaclePositions] [published] (obstacle position list topic)


ROS Services
============

- /
	- gridpath (call NuSMV gridpath and get output path)
