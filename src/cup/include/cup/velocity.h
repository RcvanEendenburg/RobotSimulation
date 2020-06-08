//
// Created by derk on 8-6-20.
//

#ifndef VELOCITY_H
#define VELOCITY_H

#include <cup/measure_attribute.h>

namespace Marker
{
class Velocity : public MeasureAttribute
{
public:
    /**
     * Creates the velocity attribute.
     * @param name The name of the marker.
     * @param transformable A reference to the marker.
     */
    Velocity(const std::string &name, const Transformable& transformable);
    ~Velocity() override = default;

    /**
     * @see MeasureAttribute::beginMeasurement
     */
    void beginMeasurement() override;

    /**
     * @see MeasureAttribute::endMeasurement
     */
    void endMeasurement() override;

    /**
     * Sets the speed for calculating velocity.
     * @param speed
     */
    void setSpeed(double speed);

    /**
     * @see MeasureAttribute::publish
     */
    void publish() override;
private:
    std::pair<tf::Vector3, tf::Vector3> position_difference_;
    double speed_;
};
}

#endif //VELOCITY_H
