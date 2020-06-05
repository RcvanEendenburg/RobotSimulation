#include <cup/cup.hpp>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <thread>
#include <chrono>

Cup::Cup(const std::string& name, unsigned short id, std::tuple<double,double,double> position,
    World& world) :
listener_(n_), broadcaster_(), name_(name + std::to_string(id)),
publisher_(n_.advertise<visualization_msgs::Marker>(name_, 1)),
speed_publisher_(n_.advertise<std_msgs::Float64>(name_ + "_speed", 1)),
velocity_publisher_(n_.advertise<geometry_msgs::Vector3>(name_ + "_velocity", 1)),
orientation_publisher_(n_.advertise<geometry_msgs::Quaternion>(name_ + "_orientation", 1)),
state_(State::Ungrabbed), world_(world)
{
    create(std::move(position), id);
}

void Cup::create(std::tuple<double,double,double> position, unsigned short id)
{
    cup_.ns = name_;
    cup_.id = id;
    cup_.type = visualization_msgs::Marker::CYLINDER;

    cup_.pose.orientation.x = 0.0;
    cup_.pose.orientation.y = 0.0;
    cup_.pose.orientation.z = 0.0;
    cup_.pose.orientation.w = 1.0;

    cup_.scale.x = 0.019;
    cup_.scale.y = 0.019;
    cup_.scale.z = 0.05;

    world_.createMarker(cup_, transform_, std::move(position));
    publisher_.publish(cup_);
    broadcaster_.sendTransform(transform_);
}

void Cup::display()
{
    cup_.header.stamp = ros::Time::now();
    transform_.stamp_ = ros::Time::now();

    if(world_.setTransforms(listener_, cup_)) {
        double speed = calculateSpeed();
        publishSpeed(speed);
        publishVelocity(speed);
        publishOrientation();

        if (world_.robotGrabbedMarker(listener_, cup_))
            state_ = State::Grabbed;
        else
            state_ = State::Ungrabbed;
        update();
        setBeginSpeed();
        setBeginVelocity();
    }
    publisher_.publish(cup_);
    broadcaster_.sendTransform(transform_);
}

void Cup::setGrabbedColor()
{
    cup_.color.r = 1.0f;
    cup_.color.g = 0.0f;
    cup_.color.b = 0.0f;
    cup_.color.a = 1.0;
}

void Cup::setUngrabbedColor()
{
    cup_.color.r = 0.0f;
    cup_.color.g = 1.0f;
    cup_.color.b = 0.0f;
    cup_.color.a = 1.0;
}

void Cup::update()
{
    switch(state_)
    {
    case State::Grabbed:
        setGrabbedColor();
        world_.followRobot(listener_, transform_, cup_);
        break;
    case State::Ungrabbed:
        setUngrabbedColor();
        world_.applyGravity(listener_, transform_, cup_);
        break;
    }
}

void Cup::setBeginSpeed()
{
    auto cup_position = world_.getMarkerPosition(cup_);
    time_difference_.first = ros::Time::now();
    magnitude_difference_.first = std::sqrt(std::pow(cup_position.x(), 2) + std::pow(cup_position.y(), 2) +
        std::pow(cup_position.z(),2));
}

void Cup::setBeginVelocity()
{
    position_difference_.first = world_.getMarkerPosition(cup_);
}

void Cup::publishVelocity(double speed)
{
    position_difference_.second = world_.getMarkerPosition(cup_);
    double d_x = position_difference_.second.x() - position_difference_.first.x();
    double d_y = position_difference_.second.y() - position_difference_.first.y();
    double d_z = position_difference_.second.z() - position_difference_.first.z();
    double d = std::sqrt(std::pow(d_x, 2) + std::pow(d_y, 2) + std::pow(d_z,2));

    if(std::isnan(d_x/d))
        d = std::numeric_limits<double>::min();
    double v_x = d_x/d * speed;
    double v_y = d_y/d * speed;
    double v_z = d_z/d * speed;

    geometry_msgs::Vector3 velocity_msg;
    velocity_msg.x = v_x;
    velocity_msg.y = v_y;
    velocity_msg.z = v_z;
    velocity_publisher_.publish(velocity_msg);
}

void Cup::publishOrientation()
{
    auto orientation = world_.getMarkerOrientation(cup_);
    geometry_msgs::Quaternion orientation_msg;
    orientation_msg.x = orientation.x();
    orientation_msg.y = orientation.y();
    orientation_msg.z = orientation.z();
    orientation_msg.w = orientation.w();
    orientation_publisher_.publish(orientation_msg);
}

double Cup::calculateSpeed()
{
    auto cup_position = world_.getMarkerPosition(cup_);
    time_difference_.second = ros::Time::now();
    magnitude_difference_.second = std::sqrt(std::pow(cup_position.x(), 2) + std::pow(cup_position.y(), 2) +
        std::pow(cup_position.z(),2));

    ros::Duration time_difference = time_difference_.second - time_difference_.first;
    auto secs = time_difference.toSec();
    double magnitude_difference = std::abs(magnitude_difference_.first - magnitude_difference_.second);
    return magnitude_difference / secs;
}

void Cup::publishSpeed(double speed)
{
    std_msgs::Float64 speed_msg;
    speed_msg.data = speed;
    speed_publisher_.publish(speed_msg);
}