#ifndef CUP_H
#define CUP_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#include <cup/world.hpp>

class Cup
{
    enum class State
    {
        Grabbed,
        Ungrabbed
    };

public:
    Cup(const std::string& name, unsigned short id, std::tuple<double,double,double> position, World& world);
    ~Cup() = default;
    void display();
private:
    void update();
    void setGrabbedColor();
    void setUngrabbedColor();
    void create(std::tuple<double,double,double> position, unsigned short id);
    void publishSpeed(double speed);
    void setBeginSpeed();
    double calculateSpeed();
    void setBeginVelocity();
    void publishVelocity(double speed);
    void publishOrientation();

    ros::NodeHandle n_;
        tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;
    std::string name_;
    ros::Publisher publisher_;
    ros::Publisher speed_publisher_;
    ros::Publisher velocity_publisher_;
    ros::Publisher orientation_publisher_;
    visualization_msgs::Marker cup_;
    State state_;
    World& world_;
    tf::StampedTransform transform_;
    std::pair<ros::Time, ros::Time> time_difference_;
    std::pair<double, double> magnitude_difference_;
    std::pair<tf::Vector3, tf::Vector3> position_difference_;
};

#endif //CUP_H