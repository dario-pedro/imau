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
//            void run();

        std::string controller_name_;

        void write();
        void read();

        //control functions
        bool init(hardware_interface::JointStateInterface&    js_interface,
                  hardware_interface::PositionJointInterface& pj_interface);

        private:

            // TRANSMISSIONS
           // RobotTransmissions robot_transmissions_;
            //boost::scoped_ptr<TransmissionInterfaceLoader> transmission_loader_;

            // Transmission interfaces
            ActuatorToJointStateInterface    act_to_jnt_state; // For propagating current actuator state to joint space
            JointToActuatorPositionInterface jnt_to_act_pos;   // For propagating joint position commands to actuator space

            // Transmissions
            //SimpleTransmission       sim_trans;
            //DifferentialTransmission dif_trans;

            // Actuator and joint space variables: wrappers around raw data
            std::vector<ActuatorData> a_state_data; // Size 2: One per transmission
            std::vector<ActuatorData> a_cmd_data;

            std::vector<JointData> j_state_data;
            std::vector<JointData> j_cmd_data;

            // Actuator and joint space variables - raw data:
            // The first actuator/joint are coupled through a reducer.
            // The last two actuators/joints are coupled through a differential.
            double a_curr_pos[3]; // Size 3: One per actuator
            double a_curr_vel[3];
            double a_curr_eff[3];
            std::vector<double> a_cmd_pos;

            double j_curr_pos[3]; // Size 3: One per joint
            double j_curr_vel[3];
            double j_curr_eff[3];
            double j_cmd_pos[3];





            ros::NodeHandle                               n;
            ros::NodeHandle                               nh;//for absolute names
            std::vector<int>                              motorIDs;

            boost::shared_ptr<controller_manager::ControllerManager> controller_manager;

            sensor_msgs::JointState                       jointsState;
            sensor_msgs::JointState                       jointsStateT;
            sensor_msgs::JointState                       jointsStateActuator;
            std::vector<double>                           joint_position_command_;

            // Interfaces
         //   hardware_interface::JointStateInterface    js_interface_;
            //hardware_interface::JointModeInterface     jm_interface_;
           // hardware_interface::EffortJointInterface   ej_interface_;
            //hardware_interface::VelocityJointInterface vj_interface_;
           // hardware_interface::PositionJointInterface pj_interface_;

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

            void standard_controller(std::string controller_name);

            void print_motors_info();







    	    //CALLBACKS
            void handleAMotorResponse(const sensor_msgs::JointState::ConstPtr& arm_request);
    };
}

#endif
