
#include "imau_control/Arm_interface.hpp"
using namespace ARM;

int main(int argc, char **argv){
	ros::init(argc, argv, "arm_interface_node");

	ARM::Arm_interface ai_elev("imau_elevator_controller");
	ARM::Arm_interface ai_r_gripper("imau_right_gripper_controller",ai_elev.share_controller());
	ARM::Arm_interface ai_l_gripper("imau_left_gripper_controller",ai_elev.share_controller());

	//ai_elev.create_controller_manager();

	ai_elev.run();
	ai_r_gripper.run();
	ai_l_gripper.run();
 	return 0;
} 	
