//
// Created by derk on 8-6-20.
//

#ifndef MEASURE_ATTRIBUTE_H
#define MEASURE_ATTRIBUTE_H

#include <cup/attribute.h>

namespace Marker
{
class MeasureAttribute : public Attribute
{
public:
    /**
     * Creates an attribute that uses time to measure.
     * @param transformable
     */
    explicit MeasureAttribute(const Transformable& transformable) : Attribute(transformable) {}
    ~MeasureAttribute() override = default;

    /**
     * Starts the measurement.
     */
    virtual void beginMeasurement() = 0;

    /**
     * Ends the measurement.
     */
    virtual void endMeasurement() = 0;
};

}


#endif //MEASURE_ATTRIBUTE_H
