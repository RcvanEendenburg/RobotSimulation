#ifndef WORLD_HPP
#define WORLD_HPP

#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cup/robot.hpp>
#include <string>

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
     * Marker will be created. Transform of the marker will also be updated, but not sent.
     * @param marker
     * @param marker_transform Transform of the marker.
     * @param position
     */
    void createMarker(visualization_msgs::Marker& marker, tf::StampedTransform& marker_transform,
        std::tuple<double,double,double> position) const;

    /**
     * Apply gravity. Transform of the marker will be updated and send and marker will be updated.
     * @param marker_transform Transform of the marker.
     * @param marker
     */
    void applyGravity(tf::StampedTransform& marker_transform, visualization_msgs::Marker& marker);

    /**
     * Checks if robot grabbed the marker.
     * @param marker
     * @return True if robot grabbed the marker.
     */
    bool robotGrabbedMarker(const visualization_msgs::Marker& marker) const;

    /**
     * Follow the robot. Transform of the marker will be updated and send.
     * @param marker_transform Transform of the marker.
     * @param marker
     */
    void followRobot(tf::StampedTransform& marker_transform, visualization_msgs::Marker& marker);

    /**
     * Get the marker position based on the transform between world and the marker.
     * @param marker
     * @return The marker position.
     */
    tf::Vector3 getMarkerPosition(const visualization_msgs::Marker& marker) const;

    /**
     * Get the marker orientation based on the transform between world and the marker.
     * @param marker
     * @return The marker orientation.
     */
    tf::Quaternion getMarkerOrientation(const visualization_msgs::Marker& marker) const;

    /**
     * Create the sensors of the robot gripper.
     */
    void createRobotSensors();

    /**
     * Update the sensors of the robot gripper.
     */
    void updateRobotSensors();
private:
    ros::NodeHandle n_;
    std::string fixed_frame_;
    double ground_level_;
    Robot& robot_;
    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;
};


#endif //WORLD_HPP
