//
// Created by derk on 4-6-20.
//

#include "cup/robot.h"

Robot::Robot(std::string gripper_left, std::string gripper_right) : n_(),
gripper_left_(std::move(gripper_left)), gripper_right_(std::move(gripper_right)),
left_sensor_("left_sensor", visualization_msgs::Marker::SPHERE),
right_sensor_("right_sensor", visualization_msgs::Marker::SPHERE)
{
    left_sensor_.setScale(0.01, 0.01, 0.01);
    left_sensor_.setColor(1, 0, 0);
    right_sensor_.setScale(0.01, 0.01, 0.01);
    right_sensor_.setColor(1, 0, 0);
}

bool Robot::grabbedMarker(const Marker::Transformable& marker) const
{
    return marker.collide(left_sensor_) && marker.collide(right_sensor_);
}

void Robot::followGripper(Marker::Transformable& marker) const
{
    marker.attach(gripper_right_);
}

void Robot::updateSensors()
{
    left_sensor_.attach(gripper_left_, 0, -0.01, 0.015);
    right_sensor_.attach(gripper_right_, 0, 0.01, 0.015);
    left_sensor_.sendUpdate();
    right_sensor_.sendUpdate();
}