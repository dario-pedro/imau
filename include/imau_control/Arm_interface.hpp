#ifndef _ARM_ARM_INTERFACE_HPP__
#define _ARM_ARM_INTERFACE_HPP__


#define TOLERANCE 2

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
//#include <hardware_interface/joint_mode_interface.h>

#include <stdio.h>
#include <iostream>
#include <cmath>

// Boost
#include <boost/shared_ptr.hpp>
// ros_control
#include <controller_manager/controller_manager.h>


namespace ARM
{
    class Arm_interface : public hardware_interface::RobotHW
    {
        public:

            Arm_interface (std::string controller_name,boost::shared_ptr<controller_manager::ControllerManager> _controller_manager);
            Arm_interface (std::string controller_name);
            virtual ~Arm_interface (){};
            boost::shared_ptr<controller_manager::ControllerManager> share_controller_manager();
            void standard_arm(std::string controller_name);
            void create_controller_manager();
            void run();

        private:
            ros::NodeHandle                               n;
            ros::NodeHandle                               nh;//for absolute names
            std::vector<int>                              motorIDs;

            boost::shared_ptr<controller_manager::ControllerManager> controller_manager;

            sensor_msgs::JointState                       jointsState;
            sensor_msgs::JointState                       jointsStateT;
            std::vector<double>                           joint_position_command_;

            // Interfaces
            hardware_interface::JointStateInterface    js_interface_;
            //hardware_interface::JointModeInterface     jm_interface_;
           // hardware_interface::EffortJointInterface   ej_interface_;
            //hardware_interface::VelocityJointInterface vj_interface_;
            hardware_interface::PositionJointInterface pj_interface_;

            ros::Time                                       last_time;
            ros::Timer non_realtime_loop_;

            bool                                          waiting_motorMessage;
            bool                                          handleMotorRequestFLAG;
            bool                                          handleMotorPermition;
            //std::vector<int>                            motorIDs;   
            //int                                         motorIDs[20]; 
            int                                           current_motor;  
            int                                           motorCounter;   

            ros::Time                                     mTimer;  
            ros::Time                                     timer_for_ellapse;      
            ros::Publisher                                joints_pub;	
	        
            ros::Subscriber                               arm_interface_sub;

            //FUNCTIONS
            void print_motors_info();
            void write();
            void read();
            void update(const ros::TimerEvent& e);
            //control functions
            bool init(hardware_interface::JointStateInterface&    js_interface,
                      hardware_interface::PositionJointInterface& pj_interface);





    	    //CALLBACKS
            void handleAMotorResponse(const sensor_msgs::JointState::ConstPtr& arm_request);
    };
}

#endif
