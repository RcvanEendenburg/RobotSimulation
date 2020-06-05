//
// Created by derk on 4-6-20.
//

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <string>
#include <vector>

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>


class Robot
{
public:
    Robot(std::string gripper_left, std::string gripper_right);
    ~Robot() = default;

    bool grabbedMarker(tf::TransformListener& listener, const visualization_msgs::Marker& marker) const;

    void followGripper(tf::TransformListener& listener, tf::StampedTransform& transform,
                              visualization_msgs::Marker& marker) const;
    bool setTransforms(tf::TransformListener& listener, visualization_msgs::Marker& marker);
private:
    std::string gripper_left_;
    std::string gripper_right_;
    tf::StampedTransform left_transform_;
    tf::StampedTransform right_transform_;
};


#endif //ROBOT_HPP
