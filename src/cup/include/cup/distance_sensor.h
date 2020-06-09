//
// Created by derk on 9-6-20.
//

#ifndef CUP_INCLUDE_CUP_DISTANCE_SENSOR_HPP
#define CUP_INCLUDE_CUP_DISTANCE_SENSOR_HPP

#include <cup/sensor.h>

namespace Sensor
{
class DistanceSensor : public Sensor
{
public:
    explicit DistanceSensor(const std::string& name);
    ~DistanceSensor() override = default;

    /**
     * Checks if marker collides with the center of the sensor.
     * @param marker
     * @return
     */
    bool detectedMarker(const Transformable& marker) const override;
};
}
#endif //CUP_INCLUDE_CUP_DISTANCE_SENSOR_HPP
