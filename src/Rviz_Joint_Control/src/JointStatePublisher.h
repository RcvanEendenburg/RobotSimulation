/*
 * JointStatePublisher.h
 *
 *  Created on: Apr 24, 2017
 *      Author: rene
 */

#ifndef SRC_RVIZ_JOINT_CONTROL_SRC_JOINTSTATEPUBLISHER_H_
#define SRC_RVIZ_JOINT_CONTROL_SRC_JOINTSTATEPUBLISHER_H_

#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

class JointStatePublisher {
public:
	JointStatePublisher();
	virtual ~JointStatePublisher();
	void publishJointStates();
	void speed(std::vector<double> degrees, double time);
	void defaultSpeed();

private:
	ros::NodeHandle n;
	ros::Publisher joint_pub;
	tf::TransformBroadcaster broadcaster;
	const double degree = M_PI/180;
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	//debug value
	double rot4;

	/*
	 * Servo max velocity declared in rad/s
	 */
	float baseSpeed = 0.18;
	float shoulderSpeed = 0.14;
	float elbowSpeed = 0.23;
	float wristSpeed = 1;
	float wristRotateSpeed = 0.18;
	float gripperSpeed = 0.16;



};

#endif /* SRC_RVIZ_JOINT_CONTROL_SRC_JOINTSTATEPUBLISHER_H_ */
