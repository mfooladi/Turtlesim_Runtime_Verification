#include <cmath>
#include <limits>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <i2r_common/Pose.h>
#include <turtle_rv/ObstaclePositions.h>
#include <turtle_rv/ObstaclePosition.h>
#include <turtle_rv/SetMode.h>

#include <i2r_common/function.h>
#include <i2r_common/point.h>

double camera_half_fov = M_PI/6;

double safety_factor = 1;
double threshold_distance = 1;
double sensitivity = 100;

double A = 1;
double B1 = 1;
double B2 = 1;
double C1 = 1;
double C2 = 1;

int cMode = 0;

ros::Subscriber obstacles_sub;
ros::Subscriber goal_sub;
ros::Subscriber pose_sub;
ros::Subscriber mode_sub;

ros::Publisher trust_pub;
ros::Publisher utilization_pub;
ros::Publisher ph_pub;
ros::Publisher pr_pub;

ros::ServiceServer reload_srv;

turtle_rv::ObstaclePositions obstacles;
geometry_msgs::Point goal;

double distance_prev = 0.0;

double robot_performance_prev = 0.0;
double utilization_ratio_prev = 0.0;
double human_performance_prev = 0.0;

double trust_prev = 0.5;

void get_obstacles(const turtle_rv::ObstaclePositions & _obstacles) {
	obstacles = _obstacles;
}

void get_goal(const geometry_msgs::Point & _goal) {
	goal = _goal;
}

void get_mode(const turtle_rv::SetMode & _mode){
	if (_mode.autonomous)
		cMode = 1;
	else 
		cMode = 0;
}

void update_trust(const i2r_common::Pose & pose) {
	int obstacle_count = 0;

	geometry_msgs::Point closest_obstacle;
	double closest_distance = std::numeric_limits<double>::max();

	//count obstacles inside of camera and find closest obstacle
	for(std::vector<turtle_rv::ObstaclePosition>::iterator obstacle = obstacles.positions.begin(); obstacle != obstacles.positions.end(); obstacle++) {
		if(i2r::yaw(pose.position, obstacle->position) < camera_half_fov)
			obstacle_count++;

		double dist = i2r::distance(pose.position, obstacle->position);
		if(dist < closest_distance) {
			closest_distance = dist;
			closest_obstacle = obstacle->position;
		}
	}

	//distance to current goal
	double distance = i2r::distance(pose.position, goal);

	//calculate parameters
	double robot_performance = std::pow(std::abs(std::atan(safety_factor*(closest_distance - threshold_distance) + 0.5)/M_PI), distance/distance_prev);
	double utilization_ratio = (cMode + (sensitivity - 1)*utilization_ratio_prev)/sensitivity;
	double human_performance;
	if (cMode == 1)	
		human_performance = std::pow(utilization_ratio, obstacle_count + 1);
	else if (cMode == 0)
		human_performance = utilization_ratio;

	//calculate trust
	double trust = A*trust_prev + B1*robot_performance - B2*robot_performance_prev + C1*human_performance - C2*human_performance_prev;

	//prevent NaN from infecting trust
	if(trust != trust)
		trust = 0.0;

	//update previous values
	distance_prev = distance;

	robot_performance_prev = robot_performance;
	utilization_ratio_prev = utilization_ratio;
	human_performance_prev = human_performance;

	trust_prev = trust;

	//publish trust
	std_msgs::Float64 trust_msg;
	std_msgs::Float64 utilization_msg;
	std_msgs::Float64 ph_msg;
	std_msgs::Float64 pr_msg;
	trust_msg.data = trust;
	utilization_msg.data = utilization_ratio;
	ph_msg.data = human_performance;
	pr_msg.data = robot_performance;
	trust_pub.publish(trust_msg);
	utilization_pub.publish(utilization_msg);
	ph_pub.publish(ph_msg);
	pr_pub.publish(pr_msg);

}

void reload() {
	ros::NodeHandle home("~");

	home.getParam("camera_half_fov", camera_half_fov);

	home.getParam("safety_factor", safety_factor);
	home.getParam("threshold_distance", threshold_distance);
	home.getParam("sensitivity", sensitivity);

	home.getParam("A", A);
	home.getParam("B1", B1);
	home.getParam("B2", B2);
	home.getParam("C1", C1);
	home.getParam("C2", C2);
}

bool do_reload(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
	reload();

	return true;
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "turtle_rv_trust");

	ros::NodeHandle home("~");

	reload_srv = home.advertiseService("reload", do_reload);

	ros::NodeHandle node;

	obstacles_sub = node.subscribe("simulation/obstacles", 10, get_obstacles);
	goal_sub = node.subscribe("mobile_base/data/goal", 10, get_goal);
	pose_sub = node.subscribe("mobile_base/data/pose", 10, update_trust);
	mode_sub = node.subscribe("mode",10, get_mode);

	trust_pub = node.advertise<std_msgs::Float64>("trust", 10);
	utilization_pub = node.advertise<std_msgs::Float64>("mUtilization",10);
	ph_pub = node.advertise<std_msgs::Float64>("mPh",10);
	pr_pub = node.advertise<std_msgs::Float64>("mPr",10);

	reload();

	ros::spin();

	return 0;
}
