#include "imau_control/Controller_hardware.hpp"

using namespace ARM;


/**
 *
 * @param controller_name
 * @return
 */
Controller_hardware::Controller_hardware(std::string controller_name):n("~"){
    //SUBSCRIBERS
    //arm_interface_sub = n.subscribe("/actuator/motor_nanotech/"+controller_name+"/joint_states", 1000,&Controller_hardware::handleAMotorResponse, this);

    //PUBLISHERS
  joints_pub = nh.advertise<sensor_msgs::JointState>(/*/imau*/"/actuator/nanotec/manipulation/"+controller_name+"/request", 1000);

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
    motorCounter = 0;
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

      jointsState_output.name = jointsState.name;
      jointsState_output.position.resize(motorCounter);
      jointsState_output.velocity.resize(motorCounter);
      jointsState_output.effort.resize(motorCounter);

      joint_id_to_joint_states_id_.resize(motorCounter);


      for (std::size_t i = 0; i < motorCounter; ++i) {
          ROS_INFO("%s ", jointsState.name[i].c_str());

          jointsState.position[i] = 0.0;
          jointsState.velocity[i] = 0.0;
          jointsState.effort[i] = 0.0;
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
 * @brief Publishes commands from the controller to a topic for the motors.
 */
void Controller_hardware::write(){
    std::cout << "WRITE pos[0]: " << controller_name_ << "go to" << joint_position_command_[0]<<std::endl;
    jointsState_output.position = joint_position_command_;
    jointsState_output.header.stamp = ros::Time::now();
    joints_pub.publish(jointsState_output);
}

/**
 * @brief - read to the controller its own joint states by parsing the complete joint states of the robot
 * @param jointState_complete - all the joints states
 */
void Controller_hardware::read(sensor_msgs::JointState &jointState_complete){
//    std::vector<std::string>::iterator it;
//    for(std::size_t i=0,j=0;i < jointState_complete.name.size() && j < jointsState.position.size() ;i++){
//        //std::cout << controller_name_ << " \n loop i" << i  << " loop j"<< j << std::endl;
//
//        it = std::find(jointState_complete.name.begin(), jointState_complete.name.end(), jointsState.name[i]);
//        // enter if condition if exists
//        if(it !=  jointState_complete.name.end()){
//            jointsState.position[j] = jointState_complete.position[i];
//            std::cout << "Setting " << jointsState.name[i]  << " at pos "<< i << " from complete to pos "
//                      << j << " of controller joints: max joint:  " << jointsState.position.size() << std::endl;
//            j++;
//        }
//        else
//            std::cout << " Not member of joints" << std::endl;

    // Copy state message to our datastructures
    std::cout << "READ ! pos[0]: " << controller_name_ << "go to" << jointState_complete.position[joint_id_to_joint_states_id_[0]] << std::endl;

    for (std::size_t i = 0; i < motorCounter; ++i)
    {
        //ROS_INFO_STREAM_NAMED("arm_hardware_interface","Joint " << i << "("<< joint_names_[i] << ") -> " << joint_id_to_joint_states_id_[i] << " position= " << state_msg->position[joint_id_to_joint_states_id_[i]]);
        jointsState.position[i] =joint_position_command_[i]; /* jointState_complete.position[joint_id_to_joint_states_id_[i]];*/
        //jointsState.velocity[i] = jointState_complete.velocity[joint_id_to_joint_states_id_[i]];
        /* Nanotech motor sends the max speed of the motor only, for that reason
         * the value must always be 0 to not interfere with the controller */
        //jointsState.velocity[i] = jointState_complete.velocity[joint_id_to_joint_states_id_[i]];
        //jointsState.effort[i] = jointState_complete.effort[joint_id_to_joint_states_id_[i]];
                    std::cout << "Speed " << jointsState.name[i]  << " at pos "<< i << " with value  " << jointsState.velocity[i] << std::endl;
    }
    //}
}




bool Controller_hardware::init(
        hardware_interface::JointStateInterface&    js_interface,
        hardware_interface::PositionJointInterface& pj_interface,
        sensor_msgs::JointState &jointState_complete)
{
    for (std::size_t i = 0; i < motorCounter; i++) {
        ROS_INFO("Init %s  at %.2f with speed %.2f doing effort %.2f ", jointsState.name[i].c_str(),
                 jointsState.position[i], jointsState.velocity[i], jointsState.effort[i]);

        // Create joint state interface for all joints
        js_interface.registerHandle(hardware_interface::JointStateHandle(
                jointsState.name[i], &jointsState.position[i], &jointsState.velocity[i], &jointsState.effort[i]));
        ROS_INFO("Joint state interface registered ");


        // Create position joint interface
        pj_interface.registerHandle(hardware_interface::JointHandle(
                js_interface.getHandle(jointsState.name[i]), &joint_position_command_[i]));
        ROS_INFO("Joint position interface registered ");

        // Make a mapping of joint names to indexes in the joint_states message
        std::vector<std::string>::const_iterator iter = std::find(jointState_complete.name.begin(),
                                                                  jointState_complete.name.end(), jointsState.name[i]);
        size_t joint_states_id = std::distance<std::vector<std::string>::const_iterator>(jointState_complete.name.begin(), iter);
        if (joint_states_id == jointState_complete.name.size()) {
            ROS_ERROR_STREAM("Unable to find joint " << i << " named " << jointState_complete.name[i]
                                                     << " in joint state message");
        }
        else {
            joint_id_to_joint_states_id_[i] = joint_states_id;

            ROS_DEBUG_STREAM(
                    "Found joint " << i << " at " << joint_states_id << " named " << jointState_complete.name[i]);

            //Populate commands
            joint_position_command_[i] = jointState_complete.position[joint_id_to_joint_states_id_[i]];
        }

        ROS_INFO("Controller %s has been regist with success!",controller_name_.c_str());

    }

    return true;
}






