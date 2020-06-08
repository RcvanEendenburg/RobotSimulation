//
// Created by derk on 8-6-20.
//

#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <cup/attribute.h>

namespace Marker
{
class Orientation : public Attribute
{
public:
    /**
     * Creates orientation attribute.
     * @param name
     * @param transformable
     */
    Orientation(const std::string &name, const Transformable& transformable);
    ~Orientation() override = default;

    /**
     * @see Attribute::publish
     */
    void publish() override;
};
}

#endif //ORIENTATION_H
