//
// Created by derk on 9-6-20.
//

#include <cup/touch_sensor.h>
#include <cup/transform_manager.h>

namespace Sensor
{
TouchSensor::TouchSensor(const std::string &name) : Sensor(name)
{
}

bool TouchSensor::detectedMarker(const Marker::Transformable &marker) const
{
    const auto &listener = TransformManager::get().getListener();
    tf::StampedTransform tf;
    if(listener.canTransform(marker.getMarker().ns, marker_.ns, ros::Time(0)))
    {
        listener.lookupTransform(marker.getMarker().ns, marker_.ns, ros::Time(0), tf);

        double x = tf.getOrigin().x();
        double y = tf.getOrigin().y();
        double z = tf.getOrigin().z();

        double sensor_radius = marker_.scale.x / 2;
        double radius = marker.getMarker().scale.x / 2;
        double half_height = marker.getMarker().scale.z / 2;

        return(std::abs(y) <= radius + sensor_radius &&
            std::abs(z) <= radius + sensor_radius &&
            std::abs(x) <= half_height + sensor_radius);
    }
    return false;
}
}
