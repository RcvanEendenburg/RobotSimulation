#include <cup/cup.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <thread>
#include <chrono>

Cup::Cup(unsigned short id, std::tuple<double,double,double> position,
    World& world) : marker_("cup" + std::to_string(id), visualization_msgs::Marker::CYLINDER),
    speed_("cup" + std::to_string(id), marker_),
    velocity_("cup" + std::to_string(id), marker_),
    orientation_("cup" + std::to_string(id), marker_),
    state_(State::Ungrabbed),
    world_(world)
{
    marker_.setPosition(std::get<0>(position), std::get<1>(position), std::get<2>(position) + world.getGroundLevel());
    marker_.setOrientation(0, 0, 0, 1);
    marker_.setScale(0.022, 0.022, 0.05);
}

void Cup::display()
{
    speed_.endMeasurement();
    velocity_.endMeasurement();
    velocity_.setSpeed(speed_.getSpeed());

    speed_.publish();
    velocity_.publish();
    orientation_.publish();

    if (world_.robotGrabbedMarker(marker_))
        state_ = State::Grabbed;
    else if(world_.robotPushedMarkerLeft(marker_))
        state_ = State::PushedLeft;
    else if(world_.robotPushedMarkerRight(marker_))
        state_ = State::PushedRight;
    else
        state_ = State::Ungrabbed;
    update();
    marker_.sendUpdate();

    speed_.beginMeasurement();
    velocity_.beginMeasurement();
}

void Cup::update()
{
    switch(state_)
    {
    case State::Grabbed:
        marker_.setColor(1, 0, 0);
        world_.followRobot(marker_);
        break;
    case State::Ungrabbed:
        marker_.setColor(0, 1, 0);
        world_.applyGravity(marker_);
        break;
    case State::PushedLeft:
        marker_.setColor(0,0,1);
        world_.robotPushLeft(marker_);
        break;
    case State::PushedRight:
        marker_.setColor(0,0,1);
        world_.robotPushRight(marker_);
        break;
    }
}
