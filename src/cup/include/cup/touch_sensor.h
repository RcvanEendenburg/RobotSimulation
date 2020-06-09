//
// Created by derk on 9-6-20.
//

#ifndef CUP_INCLUDE_CUP_TOUCH_SENSOR_HPP
#define CUP_INCLUDE_CUP_TOUCH_SENSOR_HPP

#include <cup/sensor.h>

namespace Sensor
{
class TouchSensor : public Sensor
{
public:
    explicit TouchSensor(const std::string& name);
    ~TouchSensor() override = default;

    /**
     * Checks if marker collides with the sensor.
     * @param marker
     * @return
     */
    bool detectedMarker(const Transformable& marker) const override;
};
}
#endif //CUP_INCLUDE_CUP_TOUCH_SENSOR_HPP
