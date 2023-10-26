#ifndef _OMEGA_HAPTIC_CONTROL_H
#define _OMEGA_HAPTIC_CONTROL_H

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <mutex>
#include <vector>
#include <string>
#include <ctime>
#include "dhdc.h"

class OmegaHaptic
{
    public:
        OmegaHaptic(ros::NodeHandle& node, float loop_rate, bool set_force);
        virtual ~OmegaHaptic();
        
        void PublishHapticData();
        void RegisterCallback();
        void GetHapticDataRun();
        
        void SetForce(double x, double y, double z);
        void SetForceLimit(double x, double y, double z);
        void VerifyForceLimit(double input_force[], std::vector<double>& output);
        void ForceCallback(const geometry_msgs::Vector3::ConstPtr &data);
        
        void Start();
        
    protected:
    	std::shared_ptr<boost::thread> dev_op_thread_;
    	
    private:
        ros::NodeHandle nh_;
        ros::Rate loop_rate_;
        
        ros::Publisher delta_position_pub_;
        ros::Publisher velocity_pub_;
        //ros::Publisher button_state_pub_;
        ros::Subscriber force_sub_;
        
        int device_count_;
        int dev_id_;
        bool set_force_;
        bool keep_alive_ = false;
        bool device_enabled_ = false;
        std::mutex val_lock_;
        bool force_released_;
        bool button0_state_ = false;
        bool button1_state_ = false;
        
        double position_[3];
        double prev_position[3];
        double velocity_[3];
        double start_position_[3];
        double delta_position_[3];
        
        double force_x_limit_;
        double force_y_limit_;
        double force_z_limit_;
        std::vector<double> force_;
        
        bool get_start_position = false;
        
        clock_t current_time, prev_time;
        
        std::string delta_position_topic_;
        std::string velocity_topic_;
        //std::string buttons_topic_;
        std::string force_topic_;
        
        
};

#endif


