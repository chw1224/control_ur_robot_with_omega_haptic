#include "ros/ros.h"

#include "omega_haptic/omega_haptic_control.h"

int main (int argc, char** argv) {
	ros::init(argc,argv,"haptic_ros_driver");
	ros::NodeHandle nh;
	
	OmegaHaptic omega_haptic(nh, 1000, true);
	omega_haptic.Start();
	
	return 0;
}
