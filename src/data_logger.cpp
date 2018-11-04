#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <turtle_rv/SetMode.h>



ros::Subscriber trust_sub;
ros::Subscriber p_h_sub;
ros::Subscriber p_r_sub;
ros::Subscriber mode_sub;



double t0;
std::ofstream results_file;


double p_h, p_r, trust;

int cMode;

void get_mode(const turtle_rv::SetMode & _mode){
	if (_mode.autonomous)
		cMode = 1;
	else 
		cMode = 0;
}

void get_p_h(const std_msgs::Float64 & _data) {
	p_h=_data.data;	
}

void get_p_r(const std_msgs::Float64 & _data) {
	p_r=_data.data;		
}

void get_trust(const std_msgs::Float64 & _data) {
	trust=_data.data;	
}





int main(int argc, char * argv[]) {
	ros::init(argc, argv, "data_logger");
	ros::NodeHandle node;
	int data_frequency = 100; //Hz
	ros::Rate loop_rate(data_frequency);

	results_file.open((ros::package::getPath("turtle_rv") + "/results/"+"results.csv").c_str());

	
	results_file << "t,trust, ph, pr, mode" << std::endl;

	p_h_sub = node.subscribe("/mPh", 10, get_p_h);
	p_r_sub = node.subscribe("/mPr", 10, get_p_r);
	trust_sub = node.subscribe("/trust", 10, get_trust);
	mode_sub = node.subscribe("/mode",10, get_mode);

	double t;
	t0 = ros::Time::now().toSec();
	while(ros::ok()){
			t = ros::Time::now().toSec() - t0;
			results_file << t << "," << trust << "," << p_h << "," << p_r << "," << cMode<< std::endl;
			
			ros::spinOnce();
			loop_rate.sleep();
			
	}


	results_file.close();

}
