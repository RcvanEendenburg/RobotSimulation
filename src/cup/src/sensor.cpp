//
// Created by derk on 9-6-20.
//

#include <cup/sensor.h>

namespace Sensor
{
Sensor::Sensor(const std::string &name) : Marker::Transformable(name, visualization_msgs::Marker::SPHERE)
{
    setScale(0.01, 0.01, 0.01);
    setColor(1, 0, 0);
}
}