//
// Created by derk on 9-6-20.
//

#ifndef CUP_INCLUDE_CUP_SENSOR_HPP
#define CUP_INCLUDE_CUP_SENSOR_HPP

#include <cup/transformable.h>

namespace Sensor
{
class Sensor : public Marker::Transformable
{
public:
    explicit Sensor(const std::string& name);

    ~Sensor() override = default;

    virtual bool detectedMarker(const Transformable& marker) const = 0;
};
}
#endif //CUP_INCLUDE_CUP_SENSOR_HPP
