#include "omega_haptic/omega_haptic_control.h"

#include <std_msgs/Int8MultiArray.h>
#include <cmath>

OmegaHaptic::OmegaHaptic(ros::NodeHandle& node, float loop_rate, bool set_force): loop_rate_(loop_rate)
{
	nh_ = node;
	
	dev_id_ = -2;
	device_enabled_ = -1;
	
	set_force_ = set_force;
	
	for (int i = 0; i < 3; i++) {
		position_[i] = 0.0;
	}
	
	button0_state_ = false;
	keep_alive_ = false;
	force_released_ = true;
	
	force_.resize(3);
	force_[0] = 0.0;
	force_[1] = 0.0;
	force_[2] = 0.0;
	
	velocity_[0] = 0.0;
	velocity_[1] = 0.0;
	velocity_[2] = 0.0;
	
	SetForceLimit(10.0, 10.0, 10.0);
	
	device_count_ = dhdGetDeviceCount();
	
	if (device_count_ >= 1) {
		dev_id_ = dhdOpenID(0);
		if (dev_id_ < 0) {
			ROS_INFO("Error: Cannot Open Device!!! %s\n", dhdErrorGetLastStr());
			device_enabled_ = false;
			return;
		}
	} else {
		ROS_INFO("Error: No Device Found!!!\n");
		device_enabled_ = false;
		return;
	}
	
	device_enabled_ = true;
}


OmegaHaptic::~OmegaHaptic()
{
	dev_id_ = -1;
	device_count_ = 0;
	keep_alive_ = false;
	if (dev_op_thread_) {
		dev_op_thread_->join();
	}
}


void OmegaHaptic::PublishHapticData() {
	
	geometry_msgs::Vector3 delta_pos;
	delta_pos.x = delta_position_[0];
	delta_pos.y = delta_position_[1];
	delta_pos.z = delta_position_[2];
	
	delta_position_pub_.publish(delta_pos);
	
	geometry_msgs::Vector3 velocity;
	velocity.x = velocity_[0];
	velocity.y = velocity_[1];
	velocity.z = velocity_[2];
	
	velocity_pub_.publish(velocity);
}


void OmegaHaptic::RegisterCallback() {

	delta_position_topic_ = "/haptic/delta_position";
	velocity_topic_ = "/haptic/velocity";
	delta_position_pub_ = nh_.advertise<geometry_msgs::Vector3>(delta_position_topic_.c_str(), 1);
	velocity_pub_ = nh_.advertise<geometry_msgs::Vector3>(velocity_topic_.c_str(), 1);
	
	//position_topic_ = "/haptic/position";
	//buttons_topic_ = "/haptic/button_state";
	force_topic_ = "/haptic/force";
	
	//position_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(position_topic_.c_str(), 1);
	//button_state_pub_ = nh_.advertise<std_msgs::Int8MultiArray>(buttons_topic_.c_str(), 1);
	
	force_sub_ = nh_.subscribe<geometry_msgs::Vector3>(force_topic_.c_str(), 1, &OmegaHaptic::ForceCallback, this);
	
}


void OmegaHaptic::ForceCallback (const geometry_msgs::Vector3::ConstPtr &data) {
	SetForce(data->x, data->y, data->z);
}


void OmegaHaptic::GetHapticDataRun() {
	double feed_force[3] = {0.0, 0.0, 0.0};
	double current_position[3] = {0.0, 0.0, 0.0};
	double current_velocity[3] = {0.0, 0.0, 0.0};
	
	while (ros::ok() && (keep_alive_ == true)) {
		if (device_count_ >= 1 && dev_id_ >= 0) {
			dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
			current_time = clock();
			position_[0] = current_position[0];
			position_[1] = current_position[1];
			position_[2] = current_position[2];
			// ROS_INFO("position is : %f, %f, %f", position_[0], position_[1], position_[2]);
			if (!get_start_position) {
				start_position_[0] = position_[0];
				start_position_[1] = position_[1];
				start_position_[2] = position_[2];
				get_start_position = true;
			}
			delta_position_[0] = round((position_[0] - start_position_[0]) * 100) / 100;
			delta_position_[1] = round((position_[1] - start_position_[1]) * 100) / 100;
			delta_position_[2] = round((position_[2] - start_position_[2]) * 100) / 100;
			button0_state_ = dhdGetButton(0, dev_id_);
			
			if ((current_position[0] != prev_position[0]) || (current_position[1] != prev_position[1]) || (current_position[2] != prev_position[2]) ) {
				double delayed_time = (double)(current_time-prev_time)/CLOCKS_PER_SEC;
				current_velocity[0] = (current_position[0]-prev_position[0])/delayed_time;
				current_velocity[1] = (current_position[1]-prev_position[1])/delayed_time;
				current_velocity[2] = (current_position[2]-prev_position[2])/delayed_time;
				
				if (abs(current_velocity[0]) >= 1) {
					velocity_[0] = round(current_velocity[0]*100)/100;
				} else {
					velocity_[0] = 0.0;
				}
					
				if (abs(current_velocity[1]) >= 1) {
					velocity_[1] = round(current_velocity[1]*100)/100;
				} else {
					velocity_[1] = 0.0;
				}
					
				if (abs(current_velocity[2]) >= 1) {
					velocity_[2] = round(current_velocity[2]*100)/100;
				} else {
					velocity_[2] = 0.0;
				}
			} else {
				velocity_[0] = 0.0;
				velocity_[1] = 0.0;
				velocity_[2] = 0.0;
			}
			
			prev_time = current_time;
			prev_position[0] = current_position[0];
			prev_position[1] = current_position[1];
			prev_position[2] = current_position[2];
			
		}
		
		
		PublishHapticData();
		
		if (set_force_) {
			val_lock_.lock();
			feed_force[0] = force_[0];
			feed_force[1] = force_[1];
			feed_force[2] = force_[2];
			dhdSetForce(feed_force[0], feed_force[1], feed_force[2]);
			val_lock_.unlock();
		}
		
		loop_rate_.sleep();
	}
}


void OmegaHaptic::SetForce (double x, double y, double z) {
	double input_force[3] = {0.0, 0.0, 0.0};
	
	if (set_force_) {
		val_lock_.lock();
		input_force[0] = x;
		input_force[1] = y;
		input_force[2] = z;
		VerifyForceLimit(input_force, force_);
		force_released_ = false;
		val_lock_.unlock();
	}
}


void OmegaHaptic::SetForceLimit (double x, double y, double z) {
	force_x_limit_ = x;
	force_y_limit_ = y;
	force_z_limit_ = z;
}


void OmegaHaptic::VerifyForceLimit (double input_force[], std::vector<double> &output) {
	if (output.size() != 3) {
		output.resize(3);
	}
	if (input_force[0] < -force_x_limit_) output[0] = -force_x_limit_;
	if (input_force[1] < -force_y_limit_) output[1] = -force_y_limit_;
	if (input_force[2] < -force_z_limit_) output[2] = -force_z_limit_;
	if (input_force[0] > force_x_limit_) output[0] = force_x_limit_;
	if (input_force[1] > force_y_limit_) output[1] = force_y_limit_;
	if (input_force[2] > force_z_limit_) output[2] = force_z_limit_;
}


// void OmegaHaptic::


void OmegaHaptic::Start() {
	if (!device_enabled_) {
		return;
	}
	
	RegisterCallback();
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	dev_op_thread_ = std::make_shared<boost::thread>(boost::bind(&OmegaHaptic::GetHapticDataRun, this));
	keep_alive_ = true;
	
	while (ros::ok() && (keep_alive_ == true)) {
		ros::Duration(0.001).sleep();
	}
	
	keep_alive_ = false;
	spinner.stop();
}
