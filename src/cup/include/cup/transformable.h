//
// Created by derk on 8-6-20.
//

#ifndef TRANSFORMABLE_H
#define TRANSFORMABLE_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

namespace Marker
{
class Transformable
{
public:
    /**
     * Creates the transformable marker.
     * @param name The name of the marker.
     * @param type The shape of the marker.
     */
    Transformable(const std::string& name, int type);

    virtual ~Transformable() = default;

    /**
     * Gets the position relative to the world frame.
     * @return The world position.
     */
    const tf::Vector3& getWorldPosition() const;

    /**
     * Gets the position of the current transform.
     * @return The position.
     */
    const tf::Vector3& getPosition() const;

    /**
     * Gets the orientation relative to the world frame.
     * @return The world orientation.
     */
    tf::Quaternion getWorldOrientation() const;

    /**
     * Sets the color of the marker. Alpha wil be 1.
     * @param r Red
     * @param g Green
     * @param b Blue
     */
    void setColor(double r, double g, double b);

    /**
     * Sets the position.
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     */
    void setPosition(double x, double y, double z);

    /**
     * Sets the position.
     * @param position The position vector.
     */
    void setPosition(const tf::Vector3& position);

    /**
     * Sets the orientation.
     * @param x
     * @param y
     * @param z
     * @param w
     */
    void setOrientation(double x, double y, double z, double w);

    /**
     * Sets the orientation.
     * @param quaternion
     */
    void setOrientation(const tf::Quaternion& quaternion);

    /**
     * Sets the scale of the marker.
     * @param x
     * @param y
     * @param z
     */
    void setScale(double x, double y, double z);

    /**
     * Checks if the marker can be transformed.
     * @return True if marker can be transformed.
     */
    bool available() const;

    /**
     * Updates the visual marker and updates the transform.
     */
    void sendUpdate();

    /**
     * Attaches the marker to a frame.
     * @param frame
     */
    void attach(const std::string& frame);

    /**
     * Attaches the marker to a frame with an offset.
     * @param frame
     * @param x_offset
     * @param y_offset
     * @param z_offset
     */
    void attach(const std::string& frame, double x_offset, double y_offset, double z_offset);

    /**
     * Detaches the marker from the last set frame.
     */
    void detach();

    /**
     * Gets the z
     * @return
     */
    double getZ() const;

    /**
     * Sets the z.
     * @param z
     */
    void setZ(double z);

    /**
     * Checks if marker is attached to a frame.
     * @return
     */
    bool isAttached() const;

    const visualization_msgs::Marker& getMarker() const;

private:
    ros::NodeHandle node_;
protected:
    visualization_msgs::Marker marker_;
private:
    tf::StampedTransform transform_;
    ros::Publisher publisher_;
    bool attached_;
};
}

#endif //TRANSFORMABLE_H
