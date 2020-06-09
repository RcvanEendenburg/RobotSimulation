//
// Created by derk on 9-6-20.
//

#include <cup/distance_sensor.h>
#include <cup/transform_manager.h>

namespace Sensor
{
DistanceSensor::DistanceSensor(const std::string &name) : Sensor(name)
{
}

bool DistanceSensor::detectedMarker(const Marker::Transformable &marker) const
{
    const auto &listener = TransformManager::get().getListener();
    tf::StampedTransform tf;
    if(listener.canTransform(marker.getMarker().ns, marker_.ns, ros::Time(0)))
    {
        listener.lookupTransform(marker.getMarker().ns, marker_.ns, ros::Time(0), tf);

        double x = tf.getOrigin().x();
        double y = tf.getOrigin().y();
        double z = tf.getOrigin().z();

        double radius = marker.getMarker().scale.x / 2;
        double half_height = marker.getMarker().scale.z / 2;

        return(std::abs(y) <= radius &&
            std::abs(z) <= radius + 0.005 &&
            std::abs(x) <= half_height);
    }
    return false;
}
}
