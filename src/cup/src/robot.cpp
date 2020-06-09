//
// Created by derk on 4-6-20.
//

#include "cup/robot.h"

Robot::Robot(std::string gripper_left, std::string gripper_right) : n_(),
gripper_left_(std::move(gripper_left)), gripper_right_(std::move(gripper_right)),
left_sensor_("left_sensor"),
right_sensor_("right_sensor"),
left_outer_sensor_("left_outer_sensor"),
right_outer_sensor_("right_outer_sensor")

{
}

bool Robot::grabbedMarker(const Marker::Transformable& marker) const
{
    return left_sensor_.detectedMarker(marker) && right_sensor_.detectedMarker(marker);
}

bool Robot::pushedMarkerLeft(const Marker::Transformable& marker) const
{
    if(left_outer_sensor_.available())
    {
        static double previous_left_outer_sensor_y = left_outer_sensor_.getWorldPosition().y();
        double new_y = left_outer_sensor_.getWorldPosition().y();
        if (new_y > previous_left_outer_sensor_y)
        {
            previous_left_outer_sensor_y = new_y;
            return left_outer_sensor_.detectedMarker(marker);
        }
    }
    return false;
}

bool Robot::pushedMarkerRight(const Marker::Transformable& marker) const
{
    if(right_outer_sensor_.available())
    {
        static double previous_right_outer_sensor_y = right_outer_sensor_.getWorldPosition().y();
        double new_y = right_outer_sensor_.getWorldPosition().y();
        if (new_y < previous_right_outer_sensor_y)
        {
            previous_right_outer_sensor_y = new_y;
            return right_outer_sensor_.detectedMarker(marker);
        }
    }
    return false;
}

void Robot::followGripper(Marker::Transformable& marker) const
{
    marker.attach(gripper_right_);
}

void Robot::pushLeft(Marker::Transformable& marker)
{
    marker.attach(gripper_left_);
}

void Robot::pushRight(Marker::Transformable& marker)
{
    marker.attach(gripper_right_);
}

void Robot::updateSensors()
{
    left_sensor_.attach(gripper_left_, 0, -0.01, 0.015);
    right_sensor_.attach(gripper_right_, 0, 0.01, 0.015);
    left_outer_sensor_.attach(gripper_left_, 0, 0.01, 0.015);
    right_outer_sensor_.attach(gripper_right_, 0, -0.01, 0.015);
    left_sensor_.sendUpdate();
    right_sensor_.sendUpdate();
    left_outer_sensor_.sendUpdate();
    right_outer_sensor_.sendUpdate();
}