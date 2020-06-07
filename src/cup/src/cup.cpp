#include <cup/cup.hpp>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <thread>
#include <chrono>

Cup::Cup(std::string name, unsigned short id, std::tuple<double,double,double> position,
    World& world) :
name_(std::move(name)),
publisher_(n_.advertise<visualization_msgs::Marker>(name_, 1)),
speed_publisher_(n_.advertise<std_msgs::Float64>(name_ + std::to_string(id) + "_speed", 1)),
velocity_publisher_(n_.advertise<geometry_msgs::Vector3>(name_ + std::to_string(id) + "_velocity", 1)),
orientation_publisher_(n_.advertise<geometry_msgs::Quaternion>(name_ + std::to_string(id) + "_orientation", 1)),
state_(State::Ungrabbed), world_(world)
{
    create(std::move(position), id);
}

void Cup::create(std::tuple<double,double,double> position, unsigned short id)
{
    marker_.ns = name_ + std::to_string(id);
    marker_.type = visualization_msgs::Marker::CYLINDER;

    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    marker_.scale.x = 0.022;
    marker_.scale.y = 0.022;
    marker_.scale.z = 0.05;

    world_.createMarker(marker_, transform_, std::move(position));
    world_.createRobotSensors();
}

void Cup::display()
{
    double speed = calculateSpeed();
    publishSpeed(speed);
    publishVelocity(speed);
    publishOrientation();

    if (world_.robotGrabbedMarker(marker_))
        state_ = State::Grabbed;
    else
        state_ = State::Ungrabbed;
    update();
    setBeginSpeed();
    setBeginVelocity();
    world_.updateRobotSensors();
    publisher_.publish(marker_);
}

void Cup::setGrabbedColor()
{
    marker_.color.r = 1.0f;
    marker_.color.g = 0.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;
}

void Cup::setUngrabbedColor()
{
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;
}

void Cup::update()
{
    switch(state_)
    {
    case State::Grabbed:
        setGrabbedColor();
        world_.followRobot(transform_, marker_);
        break;
    case State::Ungrabbed:
        setUngrabbedColor();
        world_.applyGravity(transform_, marker_);
        break;
    }
}

void Cup::setBeginSpeed()
{
    auto cup_position = world_.getMarkerPosition(marker_);
    time_difference_.first = ros::Time::now();
    magnitude_difference_.first = std::sqrt(std::pow(cup_position.x(), 2) + std::pow(cup_position.y(), 2) +
        std::pow(cup_position.z(),2));
}

void Cup::setBeginVelocity()
{
    position_difference_.first = world_.getMarkerPosition(marker_);
}

void Cup::publishVelocity(double speed)
{
    position_difference_.second = world_.getMarkerPosition(marker_);
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
    auto orientation = world_.getMarkerOrientation(marker_);
    geometry_msgs::Quaternion orientation_msg;
    orientation_msg.x = orientation.x();
    orientation_msg.y = orientation.y();
    orientation_msg.z = orientation.z();
    orientation_msg.w = orientation.w();
    orientation_publisher_.publish(orientation_msg);
}

double Cup::calculateSpeed()
{
    auto cup_position = world_.getMarkerPosition(marker_);
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