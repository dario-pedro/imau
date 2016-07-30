#include "imau_control/Controller_hardware.hpp"

using namespace ARM;



Controller_hardware::Controller_hardware(std::string controller_name):n("~"){
    //SUBSCRIBERS
    arm_interface_sub = n.subscribe("/actuator/motor_nanotech/joint_states", 1000,&Controller_hardware::handleAMotorResponse, this);

    //PUBLISHERS
  joints_pub = nh.advertise<sensor_msgs::JointState>(/*/imau*/"/actuator/motor_nanotec"+controller_name+"/joint_command/", 1000);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    last_time = ros::Time::now(); // marks the start
    standard_controller(controller_name);

}


/*! \brief Loads all controllers
 *         Loads and resizes all controllers found
 *
 *  @param [in] the controller name
 */
void Controller_hardware::standard_controller(std::string controller_name){

    controller_name_ = controller_name;
  //DINAMIC PARAMETERS LOADING
  if (!nh.hasParam(controller_name+"/joints")/* || !nh.hasParam(controller_name+"/motorIDs")*/){
     ROS_ERROR(" %s joint(s) do not exist in the parameter server!", controller_name.c_str());
     return;
  }
    std::cout << controller_name+"/joints"<< std::endl;
  if (nh.getParam(controller_name+"/joints",jointsState.name) /*&& nh.getParam(controller_name+"/motorIDs",motorIDs)*/){
    ROS_INFO("Read all joints names and IDs with success!");

    motorCounter = (int) jointsState.name.size();
    //motors controllers
    jointsState.position.resize(motorCounter);
    jointsState.velocity.resize(motorCounter);
    jointsState.effort.resize(motorCounter);
    joint_position_command_.resize(motorCounter);

    jointsStateT.name=jointsState.name;
    jointsStateT.position.resize(motorCounter);
    jointsStateT.velocity.resize(motorCounter);
    jointsStateT.effort.resize(motorCounter);


  for (std::size_t i = 0; i < motorCounter ; ++i){
      ROS_INFO("%s ", jointsState.name[i].c_str());

      jointsState.position[i] = 0.0;
      jointsState.velocity[i] = 0.0;
      jointsState.effort[i] = 0.0;
      joint_position_command_[i] = 0.0;

  }

  }
  else{
    ROS_ERROR("Failed to get param joints");
  }

  //CHECK IF ALL INFORMATION IS CORRECT
  print_motors_info();

}


/**
 * Prints the motor info for all motors found
 */

void Controller_hardware::print_motors_info(){
  for (std::vector<std::string>::iterator it = jointsState.name.begin(); it != jointsState.name.end(); ++it/*,++it2*/){
    std::cout << ' ' << *it ;
  	std::cout << '\n'; }
    std::cout << " Found " << motorCounter << " motors" << std::endl;
}

/**
 * Publishes commands from the controller to a topic to the motors.
 */
void Controller_hardware::write(){

    std::cout << "Name :" << controller_name_ << std::endl;
    std::cout << "joint states size: " << jointsState.name.size() << std::endl;
    std::cout << "State :" << jointsState.position[0] << std::endl;

    for (int i = 0; i< joint_position_command_.size(); i++) {
        std::cout << "Command :" << joint_position_command_[i] << std::endl;
    }


/*    std::vector<double>::iterator it2 = joint_position_command_.begin();// cant declare 2 different variables types inside the for loop
//          std::vector<double>::iterator it3 = jointsState.position.begin();// cant declare 2 different variables types inside the for loop
//          for (std::vector<std::string>::iterator it = jointsState.name.begin();
//               it != jointsState.name.end(); ++it, ++it2, ++it3) {
//              std::cout << ' ' << *it;
//              std::cout << " got command " << *it2;
//              std::cout << " and is at pos " << *it3;
//              std::cout << '\n';
//          }
*/
}

void Controller_hardware::read(){
	ROS_INFO("TODO");
}




bool Controller_hardware::init(
        hardware_interface::JointStateInterface&    js_interface,
        hardware_interface::PositionJointInterface& pj_interface)
{
    for (std::size_t i = 0; i < motorCounter; i++)
    {
        ROS_INFO("Init %s  at %.2f with speed %.2f doing effort %.2f ",jointsState.name[i].c_str(),jointsState.position[i],jointsState.velocity[i],jointsState.effort[i]);

        // Create joint state interface for all joints
        js_interface.registerHandle(hardware_interface::JointStateHandle(
                jointsState.name[i], &jointsState.position[i], &jointsState.velocity[i], &jointsState.effort[i]));
        ROS_INFO("Joint state interface registered ");


        // Create position joint interface
        pj_interface.registerHandle(hardware_interface::JointHandle(
                js_interface.getHandle(jointsState.name[i]),&joint_position_command_[i]));
        ROS_INFO("Joint position interface registered ");


    }
    return true;
}



void Controller_hardware::handleAMotorResponse(const sensor_msgs::JointState::ConstPtr& motor_response){
    ROS_INFO("Rcv a motor Response");
  // READ
    //    std::cout << "joint states size: " << jointsState.name.size() << " motor response size: " << motor_response->name.size() << std::endl;

//    for (int j = 0; j < jointsState.name.size(); j++){
//            for (int i = 0; i < motor_response->name.size(); i++){
//           ROS_INFO("A %s B %s", motor_response->name[i].c_str(), jointsState.name[j].c_str());
//                std::cout << "j: " << j << " i: " << i << std::endl;
//            if (motor_response->name[i].compare(jointsState.name[j]) == 0){
//                ROS_INFO("Found %s", motor_response->name[i].c_str());
//              jointsState.name[j] = motor_response->name[i] ;
//              jointsState.position[j] = motor_response->position[i];
//              jointsState.velocity[j] = motor_response->velocity[i];
//              jointsState.effort[j] = motor_response->effort[i];
//            }
//       }
//    }
  //jointsState = &request;
}



