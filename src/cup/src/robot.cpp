//
// Created by derk on 4-6-20.
//

#include "cup/robot.hpp"

Robot::Robot(std::string gripper_left, std::string gripper_right) : n_(),
gripper_left_(std::move(gripper_left)), gripper_right_(std::move(gripper_right)),
sensor_publisher_(n_.advertise<visualization_msgs::Marker>("gripper_sensors", 1))
{

}

bool Robot::grabbedMarker(const tf::TransformListener& listener, const visualization_msgs::Marker& marker) const
{
    tf::StampedTransform left_marker_tf, right_marker_tf;
    if(listener.canTransform(marker.ns, right_sensor.ns, ros::Time(0)) &&
        listener.canTransform(marker.ns, left_sensor.ns, ros::Time(0)))
    {
        listener.lookupTransform(marker.ns, right_sensor.ns, ros::Time(0), left_marker_tf);
        listener.lookupTransform(marker.ns, left_sensor.ns, ros::Time(0), right_marker_tf);

        double left_x = left_marker_tf.getOrigin().x();
        double left_y = left_marker_tf.getOrigin().y();
        double left_z = left_marker_tf.getOrigin().z();

        double right_x = right_marker_tf.getOrigin().x();
        double right_y = right_marker_tf.getOrigin().y();
        double right_z = right_marker_tf.getOrigin().z();

        double radius = marker.scale.x;
        double height = marker.scale.z;

        const double error_margin = 0.001;

        return(std::abs(left_y) <= (radius/2) + error_margin && std::abs(right_y) <= (radius/2) + error_margin &&
           left_z > 0 && left_z <= radius + error_margin &&
            right_z > 0 && right_z <= radius + error_margin &&
            left_x > 0 && left_x <= (height/2) + error_margin &&
            right_x > 0 && right_x <= (height/2) + error_margin);
    }
    return false;
}

void Robot::followGripper(const tf::TransformListener& listener, tf::StampedTransform& transform,
    visualization_msgs::Marker& marker) const
{
    tf::StampedTransform gripper_right_to_cup_tf;
    if(listener.canTransform(gripper_right_, marker.ns, ros::Time(0)))
    {
        listener.lookupTransform(gripper_right_, marker.ns, ros::Time(0), gripper_right_to_cup_tf);
        transform.frame_id_ = gripper_right_;
        transform.setRotation(gripper_right_to_cup_tf.getRotation());
        transform.setOrigin(gripper_right_to_cup_tf.getOrigin());

        marker.header.frame_id = gripper_right_;
        marker.pose.orientation.x = gripper_right_to_cup_tf.getRotation().x();
        marker.pose.orientation.y = gripper_right_to_cup_tf.getRotation().y();
        marker.pose.orientation.z = gripper_right_to_cup_tf.getRotation().z();
        marker.pose.orientation.w = gripper_right_to_cup_tf.getRotation().w();
        marker.pose.position.x = gripper_right_to_cup_tf.getOrigin().x();
        marker.pose.position.y = gripper_right_to_cup_tf.getOrigin().y();
        marker.pose.position.z = gripper_right_to_cup_tf.getOrigin().z();
    }
}

void Robot::createSensors()
{
    left_sensor.ns = "left_sensor";
    left_sensor.type = visualization_msgs::Marker::SPHERE;

    left_sensor.scale.x = 0.01;
    left_sensor.scale.y = 0.01;
    left_sensor.scale.z = 0.01;

    left_sensor.action = visualization_msgs::Marker::ADD;
    left_sensor.lifetime = ros::Duration();

    left_sensor.color.r = 1.0f;
    left_sensor.color.g = 0.0f;
    left_sensor.color.b = 0.0f;
    left_sensor.color.a = 1.0;

    right_sensor.ns = "right_sensor";
    right_sensor.type = visualization_msgs::Marker::SPHERE;

    right_sensor.scale.x = 0.01;
    right_sensor.scale.y = 0.01;
    right_sensor.scale.z = 0.01;

    right_sensor.action = visualization_msgs::Marker::ADD;
    right_sensor.lifetime = ros::Duration();

    right_sensor.color.r = 1.0f;
    right_sensor.color.g = 0.0f;
    right_sensor.color.b = 0.0f;
    right_sensor.color.a = 1.0;
}

void Robot::updateSensors(const tf::TransformListener &listener, tf::TransformBroadcaster &broadcaster)
{
    tf::StampedTransform gripper_left_tf, gripper_right_tf;

    if(listener.canTransform(gripper_left_, gripper_left_, ros::Time(0)) &&
        listener.canTransform(gripper_right_, gripper_right_, ros::Time(0)))
    {
        listener.lookupTransform(gripper_left_, gripper_left_, ros::Time(0), gripper_left_tf);
        listener.lookupTransform(gripper_right_, gripper_right_, ros::Time(0), gripper_right_tf);

        left_sensor.header.frame_id = gripper_left_;
        left_sensor.header.stamp = ros::Time::now();
        left_sensor.pose.orientation.x = gripper_left_tf.getRotation().x();
        left_sensor.pose.orientation.y = gripper_left_tf.getRotation().y();
        left_sensor.pose.orientation.z = gripper_left_tf.getRotation().z();
        left_sensor.pose.orientation.w = gripper_left_tf.getRotation().w();
        left_sensor.pose.position.x = gripper_left_tf.getOrigin().x();
        left_sensor.pose.position.y = gripper_left_tf.getOrigin().y() - 0.01;
        left_sensor.pose.position.z = gripper_left_tf.getOrigin().z() + 0.015;

        right_sensor.header.frame_id = gripper_right_;
        right_sensor.header.stamp = ros::Time::now();
        right_sensor.pose.orientation.x = gripper_right_tf.getRotation().x();
        right_sensor.pose.orientation.y = gripper_right_tf.getRotation().y();
        right_sensor.pose.orientation.z = gripper_right_tf.getRotation().z();
        right_sensor.pose.orientation.w = gripper_right_tf.getRotation().w();
        right_sensor.pose.position.x = gripper_right_tf.getOrigin().x();
        right_sensor.pose.position.y = gripper_right_tf.getOrigin().y() + 0.01;
        right_sensor.pose.position.z = gripper_right_tf.getOrigin().z() + 0.015;

        tf::StampedTransform left_marker_tf;
        left_marker_tf.frame_id_ = gripper_left_;
        left_marker_tf.child_frame_id_ = left_sensor.ns;
        left_marker_tf.stamp_ = ros::Time::now();
        left_marker_tf.setOrigin(tf::Vector3(left_sensor.pose.position.x,
                                             left_sensor.pose.position.y,
                                             left_sensor.pose.position.z));
        left_marker_tf.setRotation(tf::Quaternion(left_sensor.pose.orientation.x, left_sensor.pose.orientation.y,
                                                  left_sensor.pose.orientation.z, left_sensor.pose.orientation.w));

        tf::StampedTransform right_marker_tf;
        right_marker_tf.frame_id_ = gripper_right_;
        right_marker_tf.child_frame_id_ = right_sensor.ns;
        right_marker_tf.stamp_ = ros::Time::now();
        right_marker_tf.setOrigin(tf::Vector3(right_sensor.pose.position.x,
                                              right_sensor.pose.position.y,
                                              right_sensor.pose.position.z));
        right_marker_tf.setRotation(tf::Quaternion(right_sensor.pose.orientation.x, right_sensor.pose.orientation.y,
                                                   right_sensor.pose.orientation.z, right_sensor.pose.orientation.w));

        broadcaster.sendTransform(right_marker_tf);
        broadcaster.sendTransform(left_marker_tf);
        sensor_publisher_.publish(left_sensor);
        sensor_publisher_.publish(right_sensor);
    }
}