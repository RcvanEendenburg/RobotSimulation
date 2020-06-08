#ifndef WORLD_H
#define WORLD_H

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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
    ros::NodeHandle n_;
    std::string fixed_frame_;
    double ground_level_;
    Robot& robot_;
    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;
};


#endif //WORLD_H
