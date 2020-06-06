//
// Created by derk on 4-6-20.
//

#include "cup/robot.hpp"

Robot::Robot(std::string gripper_left, std::string gripper_right) :
gripper_left_(std::move(gripper_left)), gripper_right_(std::move(gripper_right))
{

}

bool Robot::grabbedMarker(tf::TransformListener& listener, const visualization_msgs::Marker& marker) const
{
    double left_x = left_transform_.getOrigin().x();
    double left_y = left_transform_.getOrigin().y();
    double left_z = left_transform_.getOrigin().z();

    double right_x = right_transform_.getOrigin().x();
    double right_y = right_transform_.getOrigin().y();
    double right_z = right_transform_.getOrigin().z();

    double marker_x = marker.scale.x / 2;
    double marker_y = marker.scale.y / 2;
    double marker_z = marker.scale.z;

    double delta_y_left = std::abs(left_y - marker_y);
    double delta_y_right = std::abs(right_y - marker_y);

    return left_x <= marker_x && marker_x <= right_x &&
        delta_y_right <= 0.04 && delta_y_left <= 0.04 &&
        left_z <= marker_z && right_z <= marker_z;
}

void Robot::followGripper(tf::TransformListener& listener, tf::StampedTransform& transform,
    visualization_msgs::Marker& marker) const
{
    transform.frame_id_ = gripper_right_;
    transform.setRotation(right_transform_.getRotation());
    transform.setOrigin(right_transform_.getOrigin());

    marker.header.frame_id = gripper_right_;
    marker.pose.orientation.x = right_transform_.getRotation().x();
    marker.pose.orientation.y = right_transform_.getRotation().y();
    marker.pose.orientation.z = right_transform_.getRotation().z();
    marker.pose.orientation.w = right_transform_.getRotation().w();
    marker.pose.position.x = right_transform_.getOrigin().x();
    marker.pose.position.y = right_transform_.getOrigin().y();
    marker.pose.position.z = right_transform_.getOrigin().z();
}

bool Robot::setTransforms(tf::TransformListener& listener, visualization_msgs::Marker& marker)
{
    if(listener.canTransform(gripper_left_, marker.ns, ros::Time(0)) &&
        listener.canTransform(gripper_right_, marker.ns, ros::Time(0)))
    {
        listener.lookupTransform(marker.ns, gripper_left_, ros::Time(0), left_transform_);
        listener.lookupTransform(gripper_right_, marker.ns, ros::Time(0), right_transform_);
        return true;
    }
    return false;
}