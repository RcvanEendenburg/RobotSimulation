#ifndef WORLD_HPP
#define WORLD_HPP

#include <visualization_msgs/Marker.h>
#include <cup/robot.hpp>
#include <string>

class World
{
public:
    World(std::string fixed_frame, double ground_level, Robot& robot);
    ~World() = default;

    void createMarker(visualization_msgs::Marker& marker, tf::StampedTransform& transform,
        std::tuple<double,double,double> position) const;
    void applyGravity(tf::TransformListener& listener, tf::StampedTransform& transform,
                             visualization_msgs::Marker& marker);
    const std::string& getFixedFrame() const;

    bool robotGrabbedMarker(tf::TransformListener& listener, const visualization_msgs::Marker& marker) const;

    void followRobot(tf::TransformListener& listener, tf::StampedTransform& transform,
                            visualization_msgs::Marker& marker) const;

    bool setTransforms(tf::TransformListener& listener, visualization_msgs::Marker& marker);

    const tf::Vector3& getMarkerPosition(const visualization_msgs::Marker& marker) const;
    tf::Quaternion getMarkerOrientation(const visualization_msgs::Marker& marker) const;
private:
    std::string fixed_frame_;
    double ground_level_;
    Robot& robot_;
    tf::StampedTransform world_transform_;
};


#endif //WORLD_HPP
