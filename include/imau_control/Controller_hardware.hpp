#ifndef _ARM_CONTROLLER_HARDWARE_HPP__
#define _ARM_CONTROLLER_HARDWARE_HPP__


#define TOLERANCE 2

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>



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


using namespace transmission_interface;

namespace ARM
{
    class Controller_hardware 
    {
        public:
            Controller_hardware (std::string controller_name);
            virtual ~Controller_hardware (){};

        std::string controller_name_;

        void write();
        void read(sensor_msgs::JointState &jointState_complete);

        int                                           motorCounter;

        //control functions
        bool init(hardware_interface::JointStateInterface&    js_interface,
                  hardware_interface::PositionJointInterface& pj_interface,
                  sensor_msgs::JointState &jointState_complete);

        private:

            ros::NodeHandle                               n;
            ros::NodeHandle                               nh;//for absolute names
            std::vector<int>                              motorIDs;

            boost::shared_ptr<controller_manager::ControllerManager> controller_manager;

            sensor_msgs::JointState                       jointsState;
            sensor_msgs::JointState                       jointsState_output;
            sensor_msgs::JointState                       jointsStateActuator;
            std::vector<double>                           joint_position_command_;

            ros::Time                                       last_time;
            ros::Timer non_realtime_loop_;


            // Convert a joint states message to our ids
            std::vector<std::size_t> joint_id_to_joint_states_id_;



            ros::Time                                     mTimer;

            ros::Publisher                                joints_pub;
            ros::Subscriber                               arm_interface_sub;

            void standard_controller(std::string controller_name);

            void print_motors_info();

    	    //CALLBACKS
            void handleAMotorResponse(const sensor_msgs::JointState::ConstPtr& arm_request);
    };
}

#endif
