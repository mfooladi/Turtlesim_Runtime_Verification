#include <cmath>
#include <limits>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <i2r_common/Pose.h>
#include <turtle_rv/ObstaclePositions.h>
#include <turtle_rv/ObstaclePosition.h>
#include <turtle_rv/SetMode.h>

#include <i2r_common/function.h>
#include <i2r_common/point.h>
#include <nusmv/gridpath.h>

int frequency = 20; //frequency of velocity calculations

//grid parameters
int grid_dimensions_default[] = {10, 10};
int grid_start_default[] = {0, 2};
int grid_obstacles_default[] = {};
int grid_goals_default[] = {9, 6};

//parameters for manual robot motion
double translation_manual_max = 0.5;
double rotation_manual_max = 0.3;

//parameters for robot motion (deciding when it needs to turn or move forward)
double distance_max = 0.08; //maximum distance before the robot is considered on top of a point
double rotation_max = 0.03; //maximum rotation before the robot is considered facing a point
double rotation_min = 0.13; //minimum rotation before the robot turns toward the point instead of moving forward

//proportional gain values for robot motion
double translation_kp = 0.8;
double rotation_kp = 0.5;

ros::Subscriber joy_sub;
ros::Subscriber goal_sub;
ros::Subscriber pose_sub;
ros::Subscriber obstacle_sub;
ros::Subscriber autonomous_sub;

ros::Publisher goal_pub;
ros::Publisher mode;
ros::Publisher cmd_vel;

ros::Timer update_timer;

sensor_msgs::Joy joy, last_joy; //joystick information
geometry_msgs::Point current_goal; //chosen goal
i2r_common::Pose pose; //pose information

turtle_rv::SetMode mode_manual, mode_autonomous;

bool autonomous = false; //whether to use NuSMV or joystick control
bool force = false; //whether to force the autonomous mode

bool target_lock = false; //whether autonomous control is locked to next point

//a bit of magic to convert above default values into vectors that can be updated with ROS parameters
std::vector<int> grid_dimensions(grid_dimensions_default, grid_dimensions_default + (sizeof(grid_dimensions_default)/sizeof(grid_dimensions_default[0])));
std::vector<int> grid_start(grid_start_default, grid_start_default + (sizeof(grid_start_default)/sizeof(grid_start_default[0])));
std::vector<int> grid_obstacles(grid_obstacles_default, grid_obstacles_default + (sizeof(grid_obstacles_default)/sizeof(grid_obstacles_default[0])));
std::vector<int> grid_goals(grid_goals_default, grid_goals_default + (sizeof(grid_goals_default)/sizeof(grid_goals_default[0])));

geometry_msgs::Point next_goal() {
	//find next goal in path
	for(std::vector<geometry_msgs::Point>::iterator goal_point = point; goal_point != path.end(); goal_point++) {
		if(i2r::contains(data.request.goals, *goal_point))
			return *goal_point;
	}

	geometry_msgs::Point empty;

	return empty;
}

geometry_msgs::Point closest_goal() {
	geometry_msgs::Point min_goal;

	//find closest goal
	double min_dist = std::numeric_limits<double>::max();
	for(std::vector<geometry_msgs::Point>::iterator goal = data.request.goals.begin(); goal != data.request.goals.end(); goal++) {
		double dist = i2r::distance(pose.position, *goal);
		if(dist < min_dist) {
			min_dist = dist;
			min_goal = *goal;
		}
	}

	return min_goal;
}

void recalculate() {
	//rerun NuSMV from current point
	start(i2r::round(pose.position.x), i2r::round(pose.position.y), false);
	run();
}

void get_joy(const sensor_msgs::Joy & _joy) {
	last_joy = joy;
	joy = _joy;

	//trigger toggles autonomous
	if(joy.buttons[0] && !last_joy.buttons[0] && !force) {
		//set appropriate mode
		if(autonomous)
			mode.publish(mode_manual);
		else
			mode.publish(mode_autonomous);
	}
}

void get_goal(const geometry_msgs::Point & _current_goal) {
	current_goal = _current_goal;
	goal_pub.publish(current_goal);
}

void get_pose(const i2r_common::Pose & _pose) {
	pose = _pose;
}

void update_obstacles(const turtle_rv::ObstaclePositions & obstacles) {
	bool added = false; //whether obstacle was added or not

	//add all obstacles from list
	for(std::vector<turtle_rv::ObstaclePosition>::const_iterator near_obstacle = obstacles.positions.begin(); near_obstacle != obstacles.positions.end(); near_obstacle++) {
		if(obstacle(near_obstacle->position.x, near_obstacle->position.y))
			added = true;
	}

	//if obstacle added and we are in autonomous mode, calculate path with new start position
	if(added && autonomous) {
		recalculate();

		//publish next_goal
		goal_pub.publish(next_goal());
	}
}

void set_autonomous(const turtle_rv::SetMode & mode) {
	autonomous = mode.autonomous;
	force = mode.force;

	if(autonomous) {
		recalculate();

		//publish first goal
		goal_pub.publish(next_goal());
	}
	else {
		//publish closest goal
		goal_pub.publish(closest_goal());
	}
}

void update_command(const ros::TimerEvent &) {
	//velocity message to send to turtle
	geometry_msgs::Twist twist;

	//autonomous control
	if(autonomous) {
		//if no path
		if(path.size() == 0) {
			//stop the turtle
			cmd_vel.publish(twist);
			return;
		}

		//get relevant motion data
		double dx = point->x - pose.position.x;
		double dy = point->y - pose.position.y;
		double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
		double rotation = i2r::rotation(std::atan2(dy, dx) - pose.orientation.yaw);

		//if we are targetting the next point or we are on top of our current point, target next point
		if(target_lock || distance < distance_max) {
			//if we are at the last point
			if(point + 1 == path.end()) {
				//reset goals and path, stop motion, and await further instruction
				target_lock = false;
				reset_goals();
				reset_path();
			}
			else {
				//look at next point
				double next_dx = (point + 1)->x - pose.position.x;
				double next_dy = (point + 1)->y - pose.position.y;
				double next_rotation = i2r::rotation(std::atan2(next_dy, next_dx) - pose.orientation.yaw);

				//if we need to rotate towards the next point
				if(std::abs(next_rotation) > rotation_max) {
					//lock targeting for next point
					target_lock = true;

					//rotate
					twist.angular.z = rotation_kp*next_rotation;
				}
				else {
					//unlock targeting from next point
					target_lock = false;

					//achieve possible goal we are on top of
					if(achieve(i2r::round(pose.position.x), i2r::round(pose.position.y)))
						//if goal achieved, publish next_goal
						goal_pub.publish(next_goal());

					//increment target point
					point++;
				}
			}
		}
		else if(std::abs(rotation) > rotation_min) {
			//rotate
			twist.angular.z = rotation_kp*rotation;
		}
		else {
			//go forward and rotate
			twist.linear.x = translation_kp*distance*std::cos(rotation);
			twist.angular.z = rotation_kp*rotation;
		}
	}
	else {
		//forward-backward on second axis
		twist.linear.x = joy.axes[1]*translation_manual_max;

		//rotation on first axis
		twist.angular.z = joy.axes[0]*rotation_manual_max;

		//achieve possible goal we are possibly on top of
		if(achieve(i2r::round(pose.position.x), i2r::round(pose.position.y)))
			//if goal achieved, publish next_goal
			goal_pub.publish(closest_goal());
	}

	//send calculated velocity command
	cmd_vel.publish(twist);
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "turtle_rv_control");

	init(); //init gridpath stuff

	//prevent problems with uninitialized joystick
	joy.axes.push_back(0);
	joy.axes.push_back(0);
	joy.buttons.push_back(0);

	//initialize mode set messages
	mode_manual.autonomous = false;
	mode_autonomous.autonomous = true;

	ros::NodeHandle home("~");

	//get parameters
	home.getParam("grid/dimensions", grid_dimensions);
	home.getParam("grid/start", grid_start);
	home.getParam("grid/obstacles", grid_obstacles);
	home.getParam("grid/goals", grid_goals);

	//add gridpath stuff from the parameters
	dimensions(grid_dimensions[0], grid_dimensions[1]);

	start(grid_start[0], grid_start[1]);

	for(std::vector<int>::size_type i = 0; i < grid_obstacles.size(); i += 2)
		obstacle(grid_obstacles[i], grid_obstacles[i + 1]);

	for(std::vector<int>::size_type i = 0; i < grid_goals.size(); i += 2)
		goal(grid_goals[i], grid_goals[i + 1]);

	ros::NodeHandle node;

	//connect to joystick, robot pose, and obstacle positions
	joy_sub = node.subscribe("joy", 10, get_joy);
	goal_sub = node.subscribe("move_base_simple/goal", 10, get_goal);
	pose_sub = node.subscribe("mobile_base/data/pose", 10, get_pose);
	obstacle_sub = node.subscribe("obstacles/positions", 10, update_obstacles);

	//subscribe to topic to change whether controller is autonomous or not
	autonomous_sub = node.subscribe("autonomous", 10, set_autonomous);

	//advertise currently targeted goal
	goal_pub = node.advertise<geometry_msgs::Point>("mobile_base/data/goal", 10, true);

	//advertise mode setting
	mode = node.advertise<turtle_rv::SetMode>("mode", 10);

	//advertise to the robot and wait until it is ready for commands
	cmd_vel = node.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 10);
	while(cmd_vel.getNumSubscribers() == 0)
		ros::Duration(0.1).sleep();

	//create control loop
	update_timer = node.createTimer(ros::Duration(1.0/frequency), update_command);

	//main ROS loop
	ros::spin();

	//avoid exceptions on shutdown
	update_timer.stop();

	return 0;
}
