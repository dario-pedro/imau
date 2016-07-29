#include "imau_control/Controller_hardware.hpp"

using namespace ARM;



Controller_hardware::Controller_hardware(std::string controller_name):n("~"){
    //SUBSCRIBERS
    arm_interface_sub = n.subscribe("/actuator/motor_nanotec/joint_states", 1000,&Controller_hardware::handleAMotorResponse, this);

    //PUBLISHERS
  joints_pub = nh.advertise<sensor_msgs::JointState>(/*/imau*/"/actuator/motor_nanotec"+controller_name+"/joint_command/", 1000);


    ros::AsyncSpinner spinner(1);
    spinner.start();



        //CONTROL VARIABLES
    waiting_motorMessage = false;
    handleMotorRequestFLAG = false;
    handleMotorPermition = false;


    last_time = ros::Time::now(); // marks the start
    standard_controller(controller_name);

}





void Controller_hardware::standard_controller(std::string controller_name){

    controller_name_ = controller_name;


  //DINAMIC PARAMETERS LOADING
  if (!nh.hasParam(controller_name+"/joints")/* || !nh.hasParam(controller_name+"/motorIDs")*/){
     ROS_ERROR("Joints DOES NOT EXIST!");
     return;
  }
  if (nh.getParam(controller_name+"/joints",jointsState.name) /*&& nh.getParam(controller_name+"/motorIDs",motorIDs)*/){
    ROS_INFO("Read all joints names and IDs with success!");

    motorCounter = (int) jointsState.name.size();
    //motors controllers
    jointsState.position.resize(motorCounter);
    jointsState.velocity.resize(motorCounter);
    jointsState.effort.resize(motorCounter);
    jointsStateT.name=jointsState.name;
    jointsStateT.position.resize(motorCounter);
    jointsStateT.velocity.resize(motorCounter);
    jointsStateT.effort.resize(motorCounter);
    joint_position_command_.resize(motorCounter);

    //transmissions
     a_state_data.resize(motorCounter);
     j_state_data.resize(motorCounter);
     a_cmd_data.resize(motorCounter);
     j_cmd_data.resize(motorCounter);
     jointsStateActuator.position.resize(motorCounter);
     jointsStateActuator.velocity.resize(motorCounter);
     jointsStateActuator.effort.resize(motorCounter);
     a_cmd_pos.resize(motorCounter);


  for (std::size_t i = 0; i < motorCounter ; ++i){
      ROS_INFO("%s ", jointsState.name[i].c_str());


      jointsState.position[i] = 0.0;
      jointsState.velocity[i] = 0.0;
      jointsState.effort[i] = 0.0;
      joint_position_command_[i] = 0.0;

      //TRANSMISSIONS
      jointsStateActuator.name.push_back(jointsState.name[i]+"_actuator");

      a_state_data[i].position.push_back(&jointsStateActuator.position[0]);
      // Wrap simple transmission raw data - current state
      a_state_data[i].position.push_back(&jointsStateActuator.position[0]);
      a_state_data[i].velocity.push_back(&jointsStateActuator.velocity[0]);
      a_state_data[i].effort.push_back(&jointsStateActuator.effort[0]);

      j_state_data[i].position.push_back(&jointsState.position[i]);
      j_state_data[i].velocity.push_back(&jointsState.velocity[i]);
      j_state_data[i].effort.push_back(&joint_position_command_[i]);

      // Wrap simple transmission raw data - position command
      a_cmd_data[i].position.push_back(&a_cmd_pos[i]); // Velocity and effort vectors are unused

      j_cmd_data[i].position.push_back(&joint_position_command_[i]); // Velocity and effort vectors are unused

  }
    current_motor = 0;
  }
  else{
    ROS_ERROR("Failed to get param joints");
  }

  //CHECK IF ALL INFORMATION IS CORRECT
  print_motors_info();

}




void Controller_hardware::print_motors_info(){
  //std::vector<int>::iterator it2 = motorIDs.begin();// cant declare 2 different variables types inside the for loop
  for (std::vector<std::string>::iterator it = jointsState.name.begin(); it != jointsState.name.end(); ++it/*,++it2*/){
    std::cout << ' ' << *it ;
   // std::cout << " with ID = " << *it2;
  	std::cout << '\n'; }
  std::cout << " Found " << motorCounter << " motors\n";
}

void Controller_hardware::write(){

    ROS_INFO("Joint %s got command %f",jointsState.name[0].c_str(),joint_position_command_[0]);
//
//     ROS_INFO("---START PUBLISH ALL---");
//
//	//jointsStateT.position.clear();
//	//jointsStateT.velocity.clear();
//	for(int i = 0; i<motorCounter;i++){
//       // if(std::abs(joint_position_command_[i]-jointsStateT.position[i])*20 > TOLERANCE) {
//            jointsStateT.position[i] = /*joint_position_command_[i] * */joint_position_command_[i];
//            jointsStateT.velocity[i] = (i * 1000);
//            jointsStateT.effort[i] = (i * 1005);
//            joints_pub.publish(jointsStateT);
//      //  }
//	}
   // ROS_INFO("---END PUBLISH ALL---");


}

void Controller_hardware::read(){
	ROS_INFO("TODO");
}




bool Controller_hardware::init(
  hardware_interface::JointStateInterface&    js_interface,
  hardware_interface::PositionJointInterface& pj_interface)
{
//  std::vector<SimpleTransmission> sim_array;
  //SimpleTransmission *sim_trans = new SimpleTransmission(-10, 1);
  for (std::size_t i = 0; i < motorCounter; i++)
  {
    ROS_INFO("Init %s  at %.2f with speed %.2f doing effort %.2f ",jointsState.name[i].c_str(),jointsState.position[i],jointsState.velocity[i],jointsState.effort[i]);


//          sim_array.push_back(*new SimpleTransmission(-10, 1));
//
//          // Create joint transmissions interfaces
//          act_to_jnt_state.registerHandle(ActuatorToJointStateHandle("sim_trans",
//                                                                     sim_array[i],
//                                                                     a_state_data[i],
//                                                                     j_state_data[i]));
//
//          jnt_to_act_pos.registerHandle(JointToActuatorPositionHandle("sim_trans",
//                                                                      sim_array[i],
//                                                                      a_cmd_data[i],
//                                                                      j_cmd_data[i]));

    // Create joint state interface for all joints
    js_interface.registerHandle(hardware_interface::JointStateHandle(
            jointsState.name[i], &jointsState.position[i], &jointsState.velocity[i], &jointsState.effort[i]));

    // Create position joint interface
    pj_interface.registerHandle(hardware_interface::JointHandle(
            js_interface.getHandle(jointsState.name[i]),&joint_position_command_[i]));

  }
  return true;
}


//void Controller_hardware::run(){
//  ros::Rate go(10);
//
//  //double previous_for_debug=joint_position_command_[0];
//  while (ros::ok()){
//
//      ROS_INFO("Joint %s got command %f",jointsState.name[0].c_str(),joint_position_command_[0]);
//
// // ROS_INFO("Update on run!");
////     if(previous_for_debug!=joint_position_command_[0]) {
///*          ROS_INFO("Checking joint_commands");
//          std::vector<double>::iterator it2 = joint_position_command_.begin();// cant declare 2 different variables types inside the for loop
//          std::vector<double>::iterator it3 = jointsState.position.begin();// cant declare 2 different variables types inside the for loop
//          for (std::vector<std::string>::iterator it = jointsState.name.begin();
//               it != jointsState.name.end(); ++it, ++it2, ++it3) {
//              std::cout << ' ' << *it;
//              std::cout << " got command " << *it2;
//              std::cout << " and is at pos " << *it3;
//              std::cout << '\n';
//          }
//          previous_for_debug = joint_position_command_[0];*/
////      }
//      // Control
//
//  	  go.sleep();
//  }
//}


void Controller_hardware::handleAMotorResponse(const sensor_msgs::JointState::ConstPtr& motor_response){
	//ROS_INFO("Rcv a motor Response");
  // READ 
  jointsState.name = motor_response->name ;
  jointsState.position = motor_response->position;
  jointsState.velocity = motor_response->velocity;
  jointsState.effort = motor_response->effort;
  act_to_jnt_state.propagate();
  //jointsState = &request;
}



