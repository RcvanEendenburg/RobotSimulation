/*
 * JointStatePublisher.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: rene
 */

#include "JointStatePublisher.h"

JointStatePublisher::JointStatePublisher() {

	joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	joint_state.name.resize(6);
	joint_state.position.resize(6);
	joint_state.velocity.resize(6);
	joint_state.name[0] = "base_link2turret";
	joint_state.name[1] = "turret2upperarm";
	joint_state.name[2] = "upperarm2forearm";
	joint_state.name[3] = "forearm2wrist";
	joint_state.name[4] = "wrist2hand";
	joint_state.name[5] = "gripper_left2hand";
	defaultSpeed();
	rot4=0;

}

JointStatePublisher::~JointStatePublisher() {
	// TODO Auto-generated destructor stub
}

void JointStatePublisher::publishJointStates() {
	joint_state.header.stamp = ros::Time::now();
	joint_state.position[0] = 0;
	joint_state.position[1] = 0;
	joint_state.position[2] = 0;
	joint_state.position[3] = rot4 * degree;
	joint_state.position[4] = 0;
	joint_state.position[5] = 0;


	// update transform
	// (moving in a circle with radius=2)
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.transform.translation.x = 0;
	odom_trans.transform.translation.y = 0;
	odom_trans.transform.translation.z = 0;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

	//send the joint state and transform
	joint_pub.publish(joint_state);
	broadcaster.sendTransform(odom_trans);

	if (rot4 == 0) rot4 = 90;



}






void JointStatePublisher::speed(std::vector<double> degrees, double time) {
	//calculate difference with last set.
//	std::vector<double> difference;
//	if(degrees.size() == joint_state.)
//	for(short i = 0; i <degrees.size(); ++i){
//		difference.push_back(joint_state.position[i] - degrees[i]);
//	}
//
//
//	if (time == 0){
//
//	}

}

void JointStatePublisher::defaultSpeed() {
	joint_state.velocity[0] = baseSpeed;
	joint_state.velocity[1] = shoulderSpeed;
	joint_state.velocity[2] = elbowSpeed;
	joint_state.velocity[3] = wristSpeed;
	joint_state.velocity[4] = wristRotateSpeed;
	joint_state.velocity[5] = gripperSpeed;
}
