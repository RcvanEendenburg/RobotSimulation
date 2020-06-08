//
// Created by derk on 7-6-20.
//

#include <cup/speed.h>
#include <std_msgs/Float64.h>

namespace Marker
{
Speed::Speed(const std::string &name, const Transformable& transformable) : MeasureAttribute(transformable), speed_(0)
{
    publisher_ = node_.advertise<std_msgs::Float64>(name + "_speed", 1);
}

void Speed::beginMeasurement()
{
    if(transformable_.available())
    {
        auto position = transformable_.getWorldPosition();
        time_difference_.first = ros::Time::now();
        magnitude_difference_.first = std::sqrt(std::pow(position.x(), 2) + std::pow(position.y(), 2) +
            std::pow(position.z(), 2));
    }
}

void Speed::endMeasurement()
{
    if(transformable_.available())
    {
        auto position = transformable_.getWorldPosition();
        time_difference_.second = ros::Time::now();
        magnitude_difference_.second = std::sqrt(std::pow(position.x(), 2) + std::pow(position.y(), 2) +
            std::pow(position.z(), 2));

        ros::Duration time_difference = time_difference_.second - time_difference_.first;
        auto secs = time_difference.toSec();
        double magnitude_difference = std::abs(magnitude_difference_.first - magnitude_difference_.second);
        speed_ = magnitude_difference / secs;
    }
}

void Speed::publish()
{
    std_msgs::Float64 message;
    message.data = speed_;
    publisher_.publish(message);
}

double Speed::getSpeed() const
{
    return speed_;
}

}
