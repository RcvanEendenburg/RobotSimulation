#include <StatePublisher.hpp>
#include <thread>

namespace robot_simulation {

    StatePublisher::StatePublisher(std::vector<std::shared_ptr<DegreeOfFreedom>>& servos) : servos_(servos), rate_(10)
    {

    }

    void StatePublisher::initialize(std::string ros_namespace)
    {
        joint_pub_ = n_.advertise<sensor_msgs::JointState>(ros_namespace, 100);

        joint_state_.name.resize(servos_.size());
        joint_state_.position.resize(servos_.size());
        joint_state_.velocity.resize(servos_.size());

        for (uint (i) = 0; (i) < servos_.size(); ++(i)) {
            joint_state_.name[i] = servos_.at(i)->getServoName();
        }
        odom_trans_.header.frame_id = "odom";
        odom_trans_.child_frame_id = "base_link";
    }

    void StatePublisher::StatePublisher::startPublishing()
    {
        publishing_ = true;
        std::thread publishThread(&StatePublisher::publish, this);// create a new thread to update the servo
        publishThread.detach();
    }

    void StatePublisher::stopPublishing()
    {
        publishing_ = false;
    }

    void StatePublisher::publish()
    {
        while (ros::ok() && publishing_)
        {
            joint_state_.header.stamp = ros::Time::now();

            odom_trans_.header.stamp = ros::Time::now();
            odom_trans_.transform.translation.x = 0;
            odom_trans_.transform.translation.y = 0;
            odom_trans_.transform.translation.z = 0;
            odom_trans_.transform.rotation = tf::createQuaternionMsgFromYaw(0);
            for (uint (i) = 0; (i) < servos_.size(); ++(i)) {
                joint_state_.position[i] = servos_.at(i)->getCurrentPos();
            }

            joint_pub_.publish(joint_state_);
            broadcaster_.sendTransform(odom_trans_);
            rate_.sleep();
        }
    }

};

