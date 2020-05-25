/*
 * main.cpp
 *
 *  Created on: Apr 13, 2017
 *      Author: rene
 *  A simple program that publishes messages to Jointstate to visualize robot movement.
 *  Only the forearm moves between 0 - 90 degrees. The degrees are converted to radian before send.
 *  Todo: receive messages from motion and send those to the Jointstate topic.
 *  Todo: Split into different header files.
 */

#include "JointStatePublisher.h"



int main(int argc, char** argv) {
ros::init(argc, argv, "MoveJoint");
JointStatePublisher x;
ros::Rate loop_rate(1);

while (ros::ok()) {
    //update joint_state
	// update here with received radian
    x.publishJointStates();
    loop_rate.sleep();
}
return 0;
}


