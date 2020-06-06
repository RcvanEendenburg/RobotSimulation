#include <RobotSimulation.hpp>
#include <memory>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "al5d_simulation");

    const std::string defaultRequestTopic = "SSC32U_request_topic";
    const std::string defaultResponseTopic = "SSC32U_response_topic"; // Wont respond for now

    RobotSimulation::Al5dSimulation al5dSim("al5d/robot_description");
    al5dSim.startListening(defaultRequestTopic);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return EXIT_SUCCESS;

}