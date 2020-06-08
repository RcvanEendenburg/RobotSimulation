//
// Created by derk on 7-6-20.
//

#ifndef SPEED_H
#define SPEED_H

#include <cup/measure_attribute.h>

namespace Marker
{
class Speed : public MeasureAttribute
{
public:
    /**
     * Creates the speed attribute.
     * @param name Name of the marker.
     * @param transformable Marker
     */
    Speed(const std::string &name, const Transformable& transformable);
    ~Speed() override = default;

    /**
     * @see MeasureAttribute::beginMeasurement
     */
    void beginMeasurement() override;

    /**
     * @see MeasureAttribute::endMeasurement
     */
    void endMeasurement() override;

    /**
     * @see MeasureAttribute::publish
     */
    void publish() override;

    /**
     * Gets the calculated speed (after begin and end measurement).
     * @return
     */
    double getSpeed() const;
private:
    std::pair<ros::Time, ros::Time> time_difference_;
    std::pair<double, double> magnitude_difference_;
    double speed_;
};
}

#endif //SPEED_H
