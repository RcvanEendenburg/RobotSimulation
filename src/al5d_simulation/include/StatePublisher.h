//
// Created by rene on 01-06-20.
//

#ifndef AL5D_SIMULATION_STATEPUBLISHER_H
#define AL5D_SIMULATION_STATEPUBLISHER_H

#include <vector>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "DegreeOfFreedom.hpp"


namespace RobotSimulation {

    class StatePublisher {
    public:
        explicit StatePublisher(std::vector<DegreeOfFreedom>& servos);
        void Initialize(std::string rosNamespace);
        void StartPublishing();
        void StopPublishing();
    private:
        void Publish();
        bool publishing = false;
        std::vector<DegreeOfFreedom> &servos;
        geometry_msgs::TransformStamped odom_trans;
        sensor_msgs::JointState joint_state;
        tf::TransformBroadcaster broadcaster;
        ros::Publisher jointPub;
        ros::NodeHandle n;
    };

    }

#endif //AL5D_SIMULATION_STATEPUBLISHER_H
