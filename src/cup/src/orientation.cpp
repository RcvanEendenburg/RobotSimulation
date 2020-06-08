//
// Created by derk on 8-6-20.
//

#include <cup/orientation.h>

namespace Marker
{
Orientation::Orientation(const std::string &name, const Marker::Transformable &transformable) :
    Attribute(transformable)
{
    publisher_ = node_.advertise<geometry_msgs::Quaternion>(name + "_orientation", 1);
}

void Orientation::publish()
{
    if(transformable_.available())
    {
        auto orientation = transformable_.getWorldOrientation();
        geometry_msgs::Quaternion orientation_msg;
        orientation_msg.x = orientation.x();
        orientation_msg.y = orientation.y();
        orientation_msg.z = orientation.z();
        orientation_msg.w = orientation.w();
        publisher_.publish(orientation_msg);
    }
}

}