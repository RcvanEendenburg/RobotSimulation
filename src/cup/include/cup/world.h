#ifndef WORLD_H
#define WORLD_H

#include <cup/robot.h>
#include <string>

#include <cup/transformable.h>

class World
{
public:
    /**
     * Create the world.
     * @param fixed_frame Fixed frame of the simulation.
     * @param ground_level Ground level of the cup.
     * @param robot Reference to the robot.
     */
    World(std::string fixed_frame, double ground_level, Robot& robot);
    ~World() = default;

    /**
     * Apply gravity to the marker.
     * @param marker
     */
    void applyGravity(Marker::Transformable& marker);

    /**
     * Checks if robot grabbed the marker.
     * @param marker
     * @return True if robot grabbed the marker.
     */
    bool robotGrabbedMarker(const Marker::Transformable& marker) const;

    bool robotPushedMarkerLeft(const Marker::Transformable& marker) const;
    bool robotPushedMarkerRight(const Marker::Transformable& marker) const;

    void robotPushLeft(Marker::Transformable& marker);
    void robotPushRight(Marker::Transformable& marker);

    /**
     * Let the marker follow the robot.
     * @param marker
     */
    void followRobot(Marker::Transformable& marker);

    /**
     * Get ground level of the world
     */
    double getGroundLevel() const;
private:
    std::string fixed_frame_;
    double ground_level_;
    Robot& robot_;
};


#endif //WORLD_H
