//
// Created by derk on 8-6-20.
//

#ifndef ATTRIBUTE_H
#define ATTRIBUTE_H

#include <ros/ros.h>
#include <cup/transformable.h>

namespace Marker
{
class Attribute
{
public:
    /**
     * Create an attribute of an marker.
     * @param transformable A reference to the marker.
     */
    explicit Attribute(const Transformable& transformable)
        : node_(), publisher_(), transformable_(transformable)
    {}
    virtual ~Attribute() = default;

    /**
     * Publishes a message.
     */
    virtual void publish() = 0;
protected:
    ros::NodeHandle node_;
    ros::Publisher publisher_;
    const Transformable& transformable_;
};

}

#endif //ATTRIBUTE_H
