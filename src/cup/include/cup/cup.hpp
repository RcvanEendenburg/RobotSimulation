#ifndef CUP_H
#define CUP_H

#include <ros/ros.h>
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
    /**
     * Create the cup
     * @param name The name of the cup.
     * @param id The id of the cup.
     * @param position The position of the cup.
     * @param world Reference to the world.
     */
    Cup(std::string name, unsigned short id, std::tuple<double,double,double> position, World& world);
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

    /**
     * Set the color when the cup is grabbed.
     */
    void setGrabbedColor();

    /**
     * Set the color when the cup is not grabbed.
     */
    void setUngrabbedColor();

    /**
     * Creates the cup.
     * @param position Position of the cup.
     * @param id Id of the cup.
     */
    void create(std::tuple<double,double,double> position, unsigned short id);

    /**
     * Publish the speed of the cup.
     * @param speed Speed of the cup.
     */
    void publishSpeed(double speed);

    /**
     * Sets the begin time and begin magnitude for calculating speed.
     */
    void setBeginSpeed();

    /**
     * Sets end time and end magnitude and calculates speed based on this.
     * @return The speed.
     */
    double calculateSpeed();

    /**
     * Sets the begin time and begin position for calculating velocity.
     */
    void setBeginVelocity();

    /**
     * Publishes and calculates velocity.
     * @param speed
     */
    void publishVelocity(double speed);

    /**
     * Publishes the orientation.
     */
    void publishOrientation();

    ros::NodeHandle n_;
    std::string name_;
    ros::Publisher publisher_;
    ros::Publisher speed_publisher_;
    ros::Publisher velocity_publisher_;
    ros::Publisher orientation_publisher_;
    visualization_msgs::Marker marker_;
    tf::StampedTransform transform_;
    State state_;
    World& world_;
    std::pair<ros::Time, ros::Time> time_difference_;
    std::pair<double, double> magnitude_difference_;
    std::pair<tf::Vector3, tf::Vector3> position_difference_;
};

#endif //CUP_H