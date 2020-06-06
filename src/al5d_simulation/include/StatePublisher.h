
#ifndef AL5D_SIMULATION_STATEPUBLISHER_H
#define AL5D_SIMULATION_STATEPUBLISHER_H

#include <vector>
#include <atomic>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "DegreeOfFreedom.hpp"

namespace RobotSimulation {

    class StatePublisher {
    public:
        explicit StatePublisher(std::vector<std::shared_ptr<DegreeOfFreedom>>& servos);
        void Initialize(std::string rosNamespace);
        void StartPublishing();
        void StopPublishing();
    private:
        void Publish();
        std::atomic<bool> publishing;
        std::vector<std::shared_ptr<DegreeOfFreedom>> &servos;
        geometry_msgs::TransformStamped odom_trans;
        sensor_msgs::JointState joint_state;
        tf::TransformBroadcaster broadcaster;
        ros::Publisher jointPub;
        ros::NodeHandle n;
    };

    }

#endif //AL5D_SIMULATION_STATEPUBLISHER_H
