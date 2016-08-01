#include "imau_control/Arm_interface.hpp"

using namespace ARM;


Arm_interface::Arm_interface(std::string controller_name,boost::shared_ptr<controller_manager::ControllerManager> _controller_manager):n("~"){
    constructor_global_function();
    controller_manager = _controller_manager;
}

Arm_interface::Arm_interface(std::string controller_name):n("~"){
    constructor_global_function();
    create_controller_manager();
}

Arm_interface::Arm_interface():n("~"){
    constructor_global_function();
    jointStates_complete.name.resize(total_motorCounter);
    jointStates_complete.position.resize(total_motorCounter);
    jointStates_complete.velocity.resize(total_motorCounter);
    jointStates_complete.effort.resize(total_motorCounter);
    jointStates_complete.header.stamp.fromSec(0);

    // Loop until we get our first joint_state message to avoid starting with no joint messages
    while(ros::ok() && jointStates_complete.header.stamp.toSec() == 0)
    {
        ROS_WARN("Waiting for joint state message");
        ros::spinOnce();
        ros::Duration(0.25).sleep();
    }


    std::vector<std::string> controller_name_array;
    if(nh.getParam("controllers",controller_name_array)) {
        ROS_INFO(" Name(s) of  all controller(s) loaded correctly");
    }
    else{
        ROS_FATAL("Couldn't load controller(s)");
    }

    for(int i=0;i<controller_name_array.size();i++){
        ROS_INFO("Controller: %s",controller_name_array[i].c_str());
        controllers_vector.push_back(Controller_hardware(controller_name_array[i]));
        total_motorCounter+=controllers_vector[i].motorCounter;
        controllers_vector[i].init(js_interface_, pj_interface_,jointStates_complete);
    }

    ROS_INFO("All controllers loaded and initiated");

    create_controller_manager();
}


void Arm_interface::constructor_global_function(){

    //set motorCounter to 0
    total_motorCounter= 0;

    //SUBSCRIBERS
    arm_interface_sub = n.subscribe("/actuator/motor_nanotech/joint_states", 1000,&Arm_interface::handleAMotorResponse, this);


    //CONTROL VARIABLES
    last_time = ros::Time::now(); // marks the start

}

void Arm_interface::create_controller_manager(){
    // Create the controller manager
    // Register interfaces
    registerInterface(&js_interface_);
    registerInterface(&pj_interface_);

//
//    for (int i = 0; i < js_interface_.getNames().size(); i++) {
//        std::cout << "joint State interface :" <<js_interface_.getNames()[i] << std::endl;
//    }
//    for (int i = 0; i < pj_interface_.getNames().size(); i++) {
//        std::cout << "position interface :" <<pj_interface_.getNames()[i] << std::endl;
//    }
    ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");
    controller_manager.reset(new controller_manager::ControllerManager(this, nh));
    ROS_INFO("Controller manager reset");


}

boost::shared_ptr<controller_manager::ControllerManager> Arm_interface::share_controller_manager(){
  return controller_manager;
}


void Arm_interface::write(){
    for(int i=0;i<controllers_vector.size();i++) {
        controllers_vector[i].write();
    }
}

void Arm_interface::read(){
    if(jointStates_complete.name[0].empty()){
        return;
    }
    for(int i=0;i<controllers_vector.size();i++) {
        controllers_vector[i].read(jointStates_complete);
    }
}



void Arm_interface::run(){
//
  ros::Rate go(10);

  ros::AsyncSpinner spinner(5);
  spinner.start();

    // Controller manager loop
    // read is on callback
  while (ros::ok()){

      std::cout << "duration " << ros::Duration( ros::Time::now()- last_time) << std::endl;

      /*Controller cycle - read, update, write */
      read();
      controller_manager->update(ros::Time::now(), ros::Duration( ros::Time::now()- last_time));
      write();

      last_time = ros::Time::now();
      go.sleep();
  }
}
/**
 * @brief - receives the robot's joint states to update the controllers
 * @param motor_response
 */
void Arm_interface::handleAMotorResponse(const sensor_msgs::JointStateConstPtr& motor_response){

    jointStates_complete = *motor_response;

}




