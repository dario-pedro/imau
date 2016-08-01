#ifndef _ARM_ARM_INTERFACE_HPP__
#define _ARM_ARM_INTERFACE_HPP__


#define TOLERANCE 2

#include "imau_control/Controller_hardware.hpp"

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
    class Arm_interface : public hardware_interface::RobotHW
    {
        public:
            Arm_interface (std::string controller_name,boost::shared_ptr<controller_manager::ControllerManager> _controller_manager);
            Arm_interface (std::string controller_name);
            Arm_interface ();
            virtual ~Arm_interface (){};
            boost::shared_ptr<controller_manager::ControllerManager> share_controller_manager();
            //void standard_controller(std::string controller_name);
            void create_controller_manager();
            void run();

        private:

            // TRANSMISSIONS
            RobotTransmissions robot_transmissions_;
           boost::scoped_ptr<TransmissionInterfaceLoader> transmission_loader_;

            // Transmission interfaces
            ActuatorToJointStateInterface    act_to_jnt_state; // For propagating current actuator state to joint space
            JointToActuatorPositionInterface jnt_to_act_pos;   // For propagating joint position commands to actuator space

            // Transmissions
            //SimpleTransmission       sim_trans;
            //DifferentialTransmission dif_trans;

            std::vector<Controller_hardware> controllers_vector;

            // Actuator and joint space variables: wrappers around raw data
            std::vector<ActuatorData> a_state_data; // Size 2: One per transmission
            std::vector<ActuatorData> a_cmd_data;

            std::vector<JointData> j_state_data;
            std::vector<JointData> j_cmd_data;


            ros::NodeHandle                               n;
            ros::NodeHandle                               nh;//for absolute names
            std::vector<int>                              motorIDs;

            boost::shared_ptr<controller_manager::ControllerManager> controller_manager;

            //sensor_msgs::JointState                       jointsState;
            sensor_msgs::JointState                       jointStates_complete;


            // Interfaces
            hardware_interface::JointStateInterface    js_interface_;
            hardware_interface::PositionJointInterface pj_interface_;

            ros::Time                                       last_time;


            int                                           total_motorCounter;


            ros::Publisher                                joints_pub;	
	        
            ros::Subscriber                               arm_interface_sub;

            //FUNCTIONS
            void constructor_global_function();
            void print_motors_info();
            void write();
            void read();

            //Callbacks
            void handleAMotorResponse(const sensor_msgs::JointState::ConstPtr& motor_response);

    };
}

#endif
