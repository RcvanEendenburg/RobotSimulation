//
// Created by derk on 4-6-20.
//

#ifndef ROBOT_H
#define ROBOT_H

#include <string>
#include <vector>

#include <cup/touch_sensor.h>
#include <cup/distance_sensor.h>

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

    bool pushedMarkerLeft(const Marker::Transformable& marker) const;
    bool pushedMarkerRight(const Marker::Transformable& marker) const;

    void pushLeft(Marker::Transformable& marker);
    void pushRight(Marker::Transformable& marker);

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
    Sensor::DistanceSensor left_sensor_;
    Sensor::DistanceSensor right_sensor_;
    Sensor::TouchSensor left_outer_sensor_;
    Sensor::TouchSensor right_outer_sensor_;
};


#endif //ROBOT_H
