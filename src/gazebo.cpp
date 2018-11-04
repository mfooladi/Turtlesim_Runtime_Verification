#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <i2r_common/Pose.h>
#include <i2r_common/RPY.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <turtle_rv/ObstaclePosition.h>
#include <turtle_rv/ObstaclePositions.h>

#include <i2r_common/function.h>
#include <i2r_common/point.h>
#include <i2r_common/rpy.h>

//update time of gazebo models (blocks)
double update_time = 4; //s

std::string turtlebot_model = "turtlebot";
std::string frame_id = "world";

ros::Subscriber odom_sub;
ros::Subscriber state_sub;

ros::Publisher pose_pub;
ros::Publisher twist_pub;

ros::Publisher vis_simulation;

ros::Publisher simulation_obstacles;

ros::Timer update_timer;

gazebo_msgs::ModelStates models;

nav_msgs::Odometry odom;
i2r_common::Pose pose;
geometry_msgs::Twist twist;

void get_odom(const nav_msgs::Odometry & _odom) {
	static tf::TransformBroadcaster tf_broadcast;

	//store odom
	odom = _odom;

	//find odom pose
	i2r_common::Pose odom_pose;
	odom_pose.position = odom.pose.pose.position;
	odom_pose.orientation = i2r::quat_to_rpy(odom.pose.pose.orientation);

	//make transform
	geometry_msgs::Point origin = pose.position - odom_pose.position;
	i2r_common::RPY rotation = pose.orientation - odom_pose.orientation;

	//publish transform
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(origin.x, origin.y, origin.z));
	transform.setRotation(i2r::rpy_to_tf_quat(rotation));
	tf_broadcast.sendTransform(tf::StampedTransform(transform, odom.header.stamp, frame_id, odom.header.frame_id));
}

void update_pose(const gazebo_msgs::ModelStates & _models) {
	models = _models;

	//find turtlebot
	std::vector<std::string>::size_type index;
	for(index = 0; index < models.name.size() && models.name[index] != turtlebot_model; index++);

	//publish pose
	pose.position = models.pose[index].position;
	pose.orientation = i2r::quat_to_rpy(models.pose[index].orientation);
	pose_pub.publish(pose);

	//publish twist
	twist = models.twist[index];
	twist_pub.publish(twist);
}

void update_simulation(const ros::TimerEvent &) {
	//prepare marker
	ros::NodeHandle node;
	visualization_msgs::Marker marker;
	turtle_rv::ObstaclePositions obstacles;

	//for every model, add a point to the points vector in the marker
	std::vector<std::string>::size_type index;
	for(index = 0; index < models.name.size(); index++) {
		//if we find a model starting with "block_"
		if(models.name[index].compare(0, 6, "block_") == 0) {
			geometry_msgs::Point point;

			//use gazebo x and y, but put at 0.25 z
			point.x = models.pose[index].position.x;
			point.y = models.pose[index].position.y;
			point.z = 0.25;

			marker.points.push_back(point);

			turtle_rv::ObstaclePosition obstacle;

			obstacle.position.x = i2r::round(models.pose[index].position.x);
			obstacle.position.y = i2r::round(models.pose[index].position.y);

			obstacles.positions.push_back(obstacle);
		}
	}

	//set basic details of marker
	marker.header.frame_id = node.getNamespace().substr(1) + "world";

	//modify the 0 cube list each time
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.action = visualization_msgs::Marker::MODIFY;

	marker.pose.orientation.w = 1.0;

	//scale down a little bit
	marker.scale.x = 0.4;
	marker.scale.y = 0.4;
	marker.scale.z = 0.4;

	//set it to a transparent red
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 0.6;

	vis_simulation.publish(marker);

	simulation_obstacles.publish(obstacles);
}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "turtle_rv_gazebo");

	ros::NodeHandle home("~");

	home.getParam("turtlebot_model", turtlebot_model);
	home.getParam("frame_id", frame_id);

	ros::NodeHandle node;

	odom_sub = node.subscribe("odom", 10, get_odom);
	state_sub = node.subscribe("/gazebo/model_states", 10, update_pose);

	pose_pub = node.advertise<i2r_common::Pose>("mobile_base/data/pose", 10);
	twist_pub = node.advertise<geometry_msgs::Twist>("mobile_base/data/twist", 10);

	vis_simulation = node.advertise<visualization_msgs::Marker>("vis/simulation", 1000, true);

	simulation_obstacles = node.advertise<turtle_rv::ObstaclePositions>("simulation/obstacles", 10);

	update_timer = node.createTimer(ros::Duration(update_time), update_simulation);

	ros::spin();

	update_timer.stop();

	return 0;
}
