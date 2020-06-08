//
// Created by derk on 4-6-20.
//

#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <vector>

#include <cup/transformable.h>

class Robot
{
public:
    /**
     * Create the robot.
     * @param gripper_left
     * @param gripper_right
     */
    Robot(std::string gripper_left, std::string gripper_right);
    ~Robot() = default;

    /**
     * Checks if the gripper grabbed the marker.
     * @param marker The marker.
     * @return True if grabbed.
     */
    bool grabbedMarker(const Marker::Transformable& marker) const;

    /**
     * Let the marker follow the gripper by attaching it to the gripper.
     * @param marker The marker.
     */
    void followGripper(Marker::Transformable& marker) const;

    /**
     * Update the sensors.
     */
    void updateSensors();
private:
    ros::NodeHandle n_;
    std::string gripper_left_;
    std::string gripper_right_;
    Marker::Transformable left_sensor_;
    Marker::Transformable right_sensor_;
};


#endif //ROBOT_H
