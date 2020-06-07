//
// Created by derk on 4-6-20.
//

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <vector>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>


class Robot
{
public:
    Robot(std::string gripper_left, std::string gripper_right);
    ~Robot() = default;

    /**
     * Checks if the gripper grabbed the marker.
     * @param listener The transform listener.
     * @param marker The marker.
     * @return True if grabbed.
     */
    bool grabbedMarker(const tf::TransformListener& listener, const visualization_msgs::Marker& marker) const;

    /**
     * Let the marker follow the gripper by attaching it to the gripper.
     * @param listener The transform listener.
     * @param transform The transform of the marker.
     * @param marker The marker.
     */
    void followGripper(const tf::TransformListener& listener, tf::StampedTransform& transform,
                              visualization_msgs::Marker& marker) const;

    /**
     * Create the sensors.
     */
    void createSensors();

    /**
     * Update the sensors.
     * @param listener The transform listener.
     * @param broadcaster The transform broadcaster.
     */
    void updateSensors(const tf::TransformListener &listener, tf::TransformBroadcaster &broadcaster);
private:
    ros::NodeHandle n_;
    std::string gripper_left_;
    std::string gripper_right_;
    visualization_msgs::Marker left_sensor;
    visualization_msgs::Marker right_sensor;
    ros::Publisher sensor_publisher_;
};


#endif //ROBOT_HPP
