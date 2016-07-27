#include "imau_control/Arm_interface.hpp"

using namespace ARM;
int TO_REMOVEE = 0;


Arm_interface::Arm_interface(std::string controller_name,boost::shared_ptr<controller_manager::ControllerManager> _controller_manager):n("~"){

  standard_arm(controller_name);
  controller_manager = _controller_manager;
}

Arm_interface::Arm_interface(std::string controller_name):n("~"){
  standard_arm(controller_name);
  create_controller_manager();
}


void Arm_interface::create_controller_manager(){
  // Create the controller manager
  ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");
  controller_manager.reset(new controller_manager::ControllerManager(this, nh));
}



void Arm_interface::standard_arm(std::string controller_name){
    //PUBLISHERS
  joints_pub = nh.advertise<sensor_msgs::JointState>(/*/imau*/"/actuator/motor_nanotec/joint_command", 1000);

  //SUBSCRIBERS
  arm_interface_sub = n.subscribe("/actuator/motor_nanotec/joint_states", 1000,&Arm_interface::handleAMotorResponse, this);


  //CONTROL VARIABLES
  waiting_motorMessage = false;
  handleMotorRequestFLAG = false;
  handleMotorPermition = false;

  //create timer
  ros::Duration update_freq = ros::Duration(1.0/10);
  non_realtime_loop_ = n.createTimer(update_freq, &Arm_interface::update, this);

  last_time = ros::Time::now(); // marks the start



  ROS_INFO("Name %s",controller_name.c_str());
  //DINAMIC PARAMETERS LOADING
  std::string teste = controller_name+"/joints";
  if (!nh.hasParam(controller_name+"/joints")/* || !nh.hasParam(controller_name+"/motorIDs")*/){
     ROS_ERROR("Joints DOES NOT EXIST!");
     return;
  }

  if (nh.getParam(controller_name+"/joints",jointsState.name) /*&& nh.getParam(controller_name+"/motorIDs",motorIDs)*/){
    ROS_INFO("Read all joints names and IDs with sucess!");

    motorCounter = (int) jointsState.name.size();
    jointsState.position.resize(motorCounter);
    jointsState.velocity.resize(motorCounter);
    jointsState.effort.resize(motorCounter);
    jointsStateT.name=jointsState.name;
    jointsStateT.position.resize(motorCounter);
    jointsStateT.velocity.resize(motorCounter);
    jointsStateT.effort.resize(motorCounter);
    joint_position_command_.resize(motorCounter);

  for (std::size_t i = 0; i < motorCounter ; ++i){
      ROS_INFO("%s ", jointsState.name[i].c_str());
      jointsState.position[i] = 0.0;
      jointsState.velocity[i] = 0.0;
      jointsState.effort[i] = 0.0;
      joint_position_command_[i] = 0.0;
    }
    current_motor = 0;
  }
  else{
    ROS_ERROR("Failed to get param joints");
  }

  //CHECK IF ALL INFORMATION IS CORRECT
  print_motors_info();

  // Initialize

  init(js_interface_, pj_interface_);

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
}

boost::shared_ptr<controller_manager::ControllerManager> Arm_interface::share_controller(){
  return controller_manager;
}



void Arm_interface::print_motors_info(){
  //std::vector<int>::iterator it2 = motorIDs.begin();// cant declare 2 different variables types inside the for loop
  for (std::vector<std::string>::iterator it = jointsState.name.begin(); it != jointsState.name.end(); ++it/*,++it2*/){
    std::cout << ' ' << *it ;
   // std::cout << " with ID = " << *it2;
  	std::cout << '\n'; }
  std::cout << " Found " << motorCounter << " motors\n";
}

void Arm_interface::write(){

	//jointsStateT.position.clear();
	//jointsStateT.velocity.clear();
	for(int i =1; i<=motorCounter;i++){
        if(std::abs(joint_position_command_[0]-jointsStateT.position[0])*20 > TOLERANCE) {
            jointsStateT.position[0] = joint_position_command_[0] * 10;
            jointsStateT.velocity.push_back(i * 1000);
            jointsStateT.position.push_back(i * 1000 + TO_REMOVEE * 20);
            joints_pub.publish(jointsStateT);
        }
	}

}

void Arm_interface::read(){
	ROS_INFO("TODO");
}








void Arm_interface::update(const ros::TimerEvent& e)
{

  ros::Duration elapsed_time_ = ros::Duration(e.current_real - e.last_real);

  // Input
  //right_arm_hw_->read(state_msg_);
 // left_arm_hw_->read(state_msg_);




  // Control
 controller_manager->update(ros::Time::now(), elapsed_time_);

 //WRITE
 write();



  // Output
  //right_arm_hw_->write(elapsed_time_);
  //left_arm_hw_->write(elapsed_time_);
}


bool Arm_interface::init(
  hardware_interface::JointStateInterface&    js_interface,
  hardware_interface::PositionJointInterface& pj_interface)
{

  for (std::size_t i = 0; i < motorCounter; i++)
  {
    ROS_INFO("Motor %s is at %f with speed %f doing effort %f ",jointsState.name[i].c_str(),jointsState.position[i],jointsState.velocity[i],jointsState.effort[i]);

    // Create joint state interface for all joints
    js_interface.registerHandle(hardware_interface::JointStateHandle(
            jointsState.name[i], &jointsState.position[i], &jointsState.velocity[i], &jointsState.effort[i]));

    // Create position joint interface
    pj_interface.registerHandle(hardware_interface::JointHandle(
            js_interface.getHandle(jointsState.name[i]),&joint_position_command_[i]));

  }

 
  ROS_INFO("Loaded imau_hardware_interface.");
  return true;
}


void Arm_interface::run(){
 // ros::Rate go(100);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  double previous_for_debug=joint_position_command_[0];
  int i = 0;
  while (ros::ok()){
    
  //ROS_INFO("Update!");
      if(previous_for_debug!=joint_position_command_[0]) {
          ROS_INFO("Checking joint_commands");
          std::vector<double>::iterator it2 = joint_position_command_.begin();// cant declare 2 different variables types inside the for loop
          std::vector<double>::iterator it3 = jointsState.position.begin();// cant declare 2 different variables types inside the for loop
          for (std::vector<std::string>::iterator it = jointsState.name.begin();
               it != jointsState.name.end() && i < motorCounter; i++, ++it, ++it2, ++it3) {
              std::cout << ' ' << *it;
              std::cout << " got command " << *it2;
              std::cout << " and is at pos " << *it3;
              std::cout << '\n';
          }
          previous_for_debug = joint_position_command_[0];
      }
      // Control
      ros::spinOnce();
      i=0;
  	//  go.sleep();
  }
}


void Arm_interface::handleAMotorResponse(const sensor_msgs::JointState::ConstPtr& motor_response){
	//ROS_INFO("Rcv a motor Response");
  // READ 
  jointsState.name = motor_response->name ;
  jointsState.position = motor_response->position;
  jointsState.velocity = motor_response->velocity;
  jointsState.effort = motor_response->effort;
  //jointsState = &request;
}



