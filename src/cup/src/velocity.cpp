//
// Created by derk on 8-6-20.
//

#include <cup/velocity.h>

namespace Marker
{
Velocity::Velocity(const std::string &name, const Marker::Transformable &transformable) :
MeasureAttribute(transformable), speed_(0)
{
    publisher_ = node_.advertise<geometry_msgs::Vector3>(name + "_velocity", 1);
}

void Velocity::beginMeasurement()
{
    if(transformable_.available())
        position_difference_.first = transformable_.getWorldPosition();
}

void Velocity::endMeasurement()
{
    if(transformable_.available())
    {
        position_difference_.second = transformable_.getWorldPosition();
    }
}

void Velocity::setSpeed(double speed)
{
    speed_ = speed;
}

void Velocity::publish()
{
    double d_x = position_difference_.second.x() - position_difference_.first.x();
    double d_y = position_difference_.second.y() - position_difference_.first.y();
    double d_z = position_difference_.second.z() - position_difference_.first.z();
    double d = std::sqrt(std::pow(d_x, 2) + std::pow(d_y, 2) + std::pow(d_z,2));

    if(std::isnan(d_x/d))
        d = std::numeric_limits<double>::min();
    double v_x = d_x/d * speed_;
    double v_y = d_y/d * speed_;
    double v_z = d_z/d * speed_;

    geometry_msgs::Vector3 velocity_msg;
    velocity_msg.x = v_x;
    velocity_msg.y = v_y;
    velocity_msg.z = v_z;
    publisher_.publish(velocity_msg);
}
}
