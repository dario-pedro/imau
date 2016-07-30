
#include "imau_control/Arm_interface.hpp"
using namespace ARM;

int main(int argc, char **argv){
	ros::init(argc, argv, "arm_interface_node");

	ARM::Arm_interface ai_elev;
	ai_elev.run();

 	return 0;
} 	
