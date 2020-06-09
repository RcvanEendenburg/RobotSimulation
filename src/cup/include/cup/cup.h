#ifndef CUP_H
#define CUP_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <cup/world.h>

#include <cup/transformable.h>
#include <cup/speed.h>
#include <cup/orientation.h>
#include <cup/velocity.h>

class Cup
{
    enum class State
    {
        Grabbed,
        Ungrabbed,
        PushedLeft,
        PushedRight,
    };

public:
    /**
     * Create the cup
     * @param id The id of the cup.
     * @param position The position of the cup.
     * @param world Reference to the world.
     */
    Cup(unsigned short id, std::tuple<double,double,double> position, World& world);
    ~Cup() = default;

    /**
     * Displays the cup.
     */
    void display();
private:
    /**
     * Updates the cup based on its state.
     */
    void update();

    Marker::Transformable marker_;
    Marker::Speed speed_;
    Marker::Velocity velocity_;
    Marker::Orientation orientation_;
    State state_;
    World& world_;
};

#endif //CUP_H