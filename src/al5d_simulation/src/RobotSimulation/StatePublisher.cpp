//
// Created by rene on 01-06-20.
//

#include <StatePublisher.h>
#include <thread>

namespace RobotSimulation {

    StatePublisher::StatePublisher(std::vector<std::shared_ptr<DegreeOfFreedom>>& servos) : servos(servos)
    {

    }

    void StatePublisher::Initialize(std::string rosNamespace)
    {
        jointPub = n.advertise<sensor_msgs::JointState>(rosNamespace, 100);

        joint_state.name.resize(servos.size());
        joint_state.position.resize(servos.size());
        joint_state.velocity.resize(servos.size());

        for (uint (i) = 0; (i) < servos.size(); ++(i)) {
            joint_state.name[i] = servos.at(i)->getServoName();
        }
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = joint_state.name[0];
    }

    void StatePublisher::StatePublisher::StartPublishing()
    {
        publishing = true;
        std::thread publishThread(&StatePublisher::Publish, this);// create a new thread to update the servo
        publishThread.detach();
    }

    void StatePublisher::StopPublishing()
    {
        publishing = false;
    }

    void StatePublisher::Publish()
    {
        ros::Rate r(10);
        while (ros::ok() && publishing)
        {
            joint_state.header.stamp = ros::Time::now();

            odom_trans.header.stamp = ros::Time::now();
            odom_trans.transform.translation.x = 0;
            odom_trans.transform.translation.y = 0;
            odom_trans.transform.translation.z = 0;
            odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
            for (uint (i) = 0; (i) < servos.size(); ++(i)) {
                joint_state.position[i] = servos.at(i)->getCurrentPos();
            }

            jointPub.publish(joint_state);
            broadcaster.sendTransform(odom_trans);
            r.sleep();
        }
    }

};

