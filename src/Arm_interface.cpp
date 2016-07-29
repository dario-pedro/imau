#include "imau_control/Arm_interface.hpp"

using namespace ARM;


Arm_interface::Arm_interface(std::string controller_name,boost::shared_ptr<controller_manager::ControllerManager> _controller_manager):n("~"){
    constructor_global_fuction();

//  standard_controller(controller_name);
  controller_manager = _controller_manager;
}

Arm_interface::Arm_interface(std::string controller_name):n("~"){
    constructor_global_fuction();
  //standard_controller(controller_name);
    // Initialize transmission loader
 /*   try
    {
        transmission_loader_.reset(new TransmissionInterfaceLoader(this, &robot_transmissions_));
    }
    catch(const std::invalid_argument& ex)
    {
        ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    }
    catch(const pluginlib::LibraryLoadException& ex)
    {
        ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    }

    std::string robot_description;
    // ...load URDF from parameter server or file

    // Perform actual transmission loading
    if (!transmission_loader_->load(robot_description)) {
        ROS_ERROR("Could not load robot_descripions to transmissions_loader");
    }*/
  create_controller_manager();
}

Arm_interface::Arm_interface():n("~"){
    constructor_global_fuction();
    std::vector<std::string> controller_name_array;
    if(nh.getParam("controllers",controller_name_array)) {
      ROS_INFO("Load all controllers correctly into array");
    }
    else{
      ROS_FATAL("Couldnt Load controller correctly");
    }

    for(int i=0;i<controller_name_array.size();i++){
       // ROS_INFO("TEST DINAYMIC VECTOR LOADING %s",controller_name_array[i].c_str());
        //{
        //Controller_hardware mch(controller_name_array[i]);
        controllers_vector.push_back(Controller_hardware(controller_name_array[i]));
        controllers_vector[i].init(js_interface_, pj_interface_);
        //}
        //standard_controller(controller_name_array[i]);
    }

    create_controller_manager();
}

void Arm_interface::create_controller_manager(){
  // Create the controller manager
  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");
  controller_manager.reset(new controller_manager::ControllerManager(this, nh));
}


void Arm_interface::constructor_global_fuction(){

    //SUBSCRIBERS
//    arm_interface_sub = n.subscribe("/actuator/motor_nanotec/joint_states", 1000,&Arm_interface::handleAMotorResponse, this);


    //CONTROL VARIABLES
    waiting_motorMessage = false;
    handleMotorRequestFLAG = false;
    handleMotorPermition = false;

    //create timer
    // ros::Duration update_freq = ros::Duration(1.0/10);
    //non_realtime_loop_ = n.createTimer(update_freq, &Arm_interface::update, this);

    last_time = ros::Time::now(); // marks the start
}

boost::shared_ptr<controller_manager::ControllerManager> Arm_interface::share_controller_manager(){
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

//    controllers_vector[0].write();
    for(int i=0;i<controllers_vector.size();i++) {
       // ROS_INFO("Writting %s",controllers_vector[i].controller_name_.c_str());
        controllers_vector[i].write();
    }

}

void Arm_interface::read(){
	ROS_INFO("TODO");
}



bool Arm_interface::init(
  hardware_interface::JointStateInterface&    js_interface,
  hardware_interface::PositionJointInterface& pj_interface)
{
  std::vector<SimpleTransmission> sim_array;
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


void Arm_interface::run(){

  ros::Rate go(10);
  ros::AsyncSpinner spinner(1);
  spinner.start();
   // controllers_vector[0].run();
  //  for(int i=0;i<controllers_vector.size();i++) {
  //      controllers_vector[i].run();
 //   }
    std::cout << "duration " << std::endl;

  while (ros::ok()){

     // Controller manager loop
      // read is on callback
      std::cout << "duration " << ros::Duration( ros::Time::now()- last_time) << std::endl;
     controller_manager->update(ros::Time::now(), ros::Duration( ros::Time::now()- last_time));
     last_time = ros::Time::now();
     //WRITE
     write();
    
  // ROS_INFO("Update on run!");
  /*    if(previous_for_debug!=joint_position_command_[0]) {
          ROS_INFO("Checking joint_commands");
          std::vector<dwriteouble>::iterator it2 = joint_position_command_.begin();// cant declare 2 different variables types inside the for loop
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
      //ros::spinOnce();
      i=0;*/
  	  go.sleep();
  }
}


//void Arm_interface::handleAMotorResponse(const sensor_msgs::JointState::ConstPtr& motor_response){
//	//ROS_INFO("Rcv a motor Response");
//  // READ
//  jointsState.name = motor_response->name ;
//  jointsState.position = motor_response->position;
//  jointsState.velocity = motor_response->velocity;
//  jointsState.effort = motor_response->effort;
//  act_to_jnt_state.propagate();
//  //jointsState = &request;
//}



