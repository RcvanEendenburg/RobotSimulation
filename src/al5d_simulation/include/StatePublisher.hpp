
#ifndef AL5D_SIMULATION_STATEPUBLISHER_H
#define AL5D_SIMULATION_STATEPUBLISHER_H

#include <vector>
#include <atomic>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "DegreeOfFreedom.hpp"

namespace robot_simulation {

    class StatePublisher {
    public:
        /**
         * Create a StatePublisher that sends out joint_states
         * @param servos Reference to the vector which contains the servo's
         */
        explicit StatePublisher(std::vector<std::shared_ptr<DegreeOfFreedom>>& servos);
        /**
         * @param ros_namespace The namespace the joint_states will be published too
         */
        void Initialize(std::string ros_namespace);
        /**
         * Calls the function Publish in a new thread
         */
        void StartPublishing();
        /**
         * Stop publishing the servo states
         */
        void StopPublishing();
    private:
        /**
         * Looping function that continuously publishes the state of the servos
         */
        void Publish();
        std::atomic<bool> publishing_;
        std::vector<std::shared_ptr<DegreeOfFreedom>> &servos_;
        geometry_msgs::TransformStamped odom_trans_;
        sensor_msgs::JointState joint_state_;
        tf::TransformBroadcaster broadcaster_;
        ros::Publisher joint_pub_;
        ros::NodeHandle n_;
        ros::Rate rate_;
    };

    }

#endif //AL5D_SIMULATION_STATEPUBLISHER_H
