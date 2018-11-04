#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <i2r_common/Pose.h>
#include <turtle_rv/ObstaclePoints.h>
#include <turtle_rv/ObstaclePositions.h>

#include <i2r_common/function.h>

#include "obstacle.h"

//frequency of obstacle calculation
double frequency = 4; //Hz

//near distance
double near = 1.5; //m

//safe distance
double safe = 0.75; //m

//ROS handles
ros::Subscriber obstacle_sub;
ros::Subscriber pose_sub;

ros::Publisher positions;
ros::Publisher sensor;

ros::Timer update_timer;

//stores pose of robot
turtle_rv::ObstaclePositions obstacles;
i2r_common::Pose pose;

//stores positions of obstacles
turtle_rv::ObstaclePositions obstacle_positions;

//check if there is an obstacle ahead using simulation data and robot pose
turtle_rv::ObstaclePositions check_obstacles(const turtle_rv::ObstaclePositions & obstacles, const i2r_common::Pose & pose) {
	//obstacles the robot is aware of (distinct from simulation obstacles in 'obstacles')
	turtle_rv::ObstaclePositions near_obstacles;

	//iterate over each simulation obstacle and record nearby obstacles
	for(std::vector<turtle_rv::ObstaclePosition>::const_iterator obstacle = obstacles.positions.begin(); obstacle != obstacles.positions.end(); obstacle++) {
		//if the obstacle is close enough
		if(i2r::planar(obstacle->position, pose.position) <= near) {
			//record it as a nearby obstacle
			near_obstacles.positions.push_back(*obstacle);
		}
	}

	return near_obstacles;
}

//store obstacles from simulation
void get_obstacles(const turtle_rv::ObstaclePositions & _obstacles) {
	obstacles = _obstacles;
}

//store pose from robot
void get_pose(const i2r_common::Pose & _pose) {
	pose = _pose;
}

//check, update, and publish obstacles
void update_obstacles(const ros::TimerEvent &) {
	turtle_rv::ObstaclePositions near_obstacles = check_obstacles(obstacles, pose);

	//get position in front of the robot
	turtle_rv::ObstaclePosition front_position;

	front_position.position.x = i2r::round(pose.position.x + std::cos(pose.orientation.yaw));
	front_position.position.y = i2r::round(pose.position.y + std::sin(pose.orientation.yaw));

	//publish whether there is an obstacle in front of the robot
	std_msgs::Bool sensor_val;
	std::vector<turtle_rv::ObstaclePosition>::iterator front_obstacle = i2r::find(near_obstacles.positions, front_position);
	sensor_val.data = front_obstacle != near_obstacles.positions.end() && i2r::planar(front_obstacle->position, pose.position) < safe;
	sensor.publish(sensor_val);

	//add every near obstacle to the list
	for(std::vector<turtle_rv::ObstaclePosition>::iterator obstacle = near_obstacles.positions.begin(); obstacle != near_obstacles.positions.end(); obstacle++) {
		//if it is not already contained in the list
		if(!i2r::contains(obstacle_positions.positions, *obstacle)) {
			//store id numbers in list elements
			obstacle->id = obstacle_positions.positions.size();

			//add data to lists
			obstacle_positions.positions.push_back(*obstacle);

			//publish new data
			positions.publish(obstacle_positions);
		}
	}
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "turtle_rv_obstacles");

	//initialize ros topics and timers
	ros::NodeHandle node;

	obstacle_sub = node.subscribe("simulation/obstacles", 1, get_obstacles);
	pose_sub = node.subscribe("mobile_base/data/pose", 1, get_pose);

	positions = node.advertise<turtle_rv::ObstaclePositions>("obstacles/positions", 10, true);
	sensor = node.advertise<std_msgs::Bool>("mobile_base/data/sensor", 10, true);

	update_timer = node.createTimer(ros::Duration(1.0/frequency), update_obstacles);

	//run ros
	ros::spin();

	//deinitialize ros timers
	update_timer.stop();

	return 0;
}
