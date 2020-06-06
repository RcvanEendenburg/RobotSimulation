#ifndef VIRTUAL_AL5D_ROBOT_HPP_
#define VIRTUAL_AL5D_ROBOT_HPP_

#include "std_msgs/String.h"
#include "DegreeOfFreedom.hpp"
#include "StatePublisher.h"
#include <SSC32UCommand.hpp>
#include <ros/ros.h>
#include <memory>
#include <thread>

namespace RobotSimulation {

    class Al5dSimulation {
    public:
        explicit Al5dSimulation(std::string modelDataNamespace);

        virtual ~Al5dSimulation();

        void startListening(std::string topicName)
        {
            sub = n.subscribe(topicName, 1000, &Al5dSimulation::handleRequest, this);
        }

        /**
         * @brief Handles incoming SSC32U requests
         *
         * @param SSC32U request
         */
        void handleRequest(const std_msgs::String::ConstPtr& msg);

        /**
         * @brief This function maps input_val values to radial based on min and max angle
         *
         * @param inputVal The value to set to radians
         * @param maxAngle Max angle in PWM
         * @param minAngle Min angle in PWM
         * @param maxRad Max angle in radian
         * @param minRad Min angle in radian
         * @return radians between min & max radian
         */
        double
        toRadian(short servoNr, double inputVal, double maxAngle, double minAngle, double maxRad, double minRad) const;

        /**
         * @brief function that converts a speed in pwm to a speed in rad/s  *
         * @param aSpeed the speed in pwm
         * @param servoNr the servo id
         * @return double the speed in rad/s
         */
        double speedToRad(double aSpeed, short servoNr);

    private:
        /**
         * @brief Get the Servo PWM object
         *
         * @param servoID the id of the servo
         * @return the current pwm
         */
        double getServoPWM(short servoID);

        /**
         * @brief function thats moves a servo
         *
         * @param aServoNr the id of the servo to be moved
         * @param aPwm the pwm the servo should move to
         * @param aSpeed the speed that the servo should be moved with
         */
        void moveServo(short aServoNr, double aPwm, double aSpeed);
        /**
         * @brief checks if the arm moves
         * @return true if moving
         */
        bool armIsMoving();
        /**
         * @brief stops the arm from moving
         */
        void stop();

        ros::NodeHandle n;
        ros::Subscriber sub;
        std::unique_ptr<StatePublisher> statePublisher = std::make_unique<StatePublisher>(servos);
        const uint16_t servoMin[7] = {2500, 500, 1950, 500, 2650, 2500, 500};
        const uint16_t servoMax[7] = {500, 1950, 500, 2500, 500, 500, 2500};
        const short gripperANr = 5;
        const short gripperBNr = 6;
        urdf::Model model;
        std::vector<std::shared_ptr<DegreeOfFreedom>> servos;
    };
}
#endif// ROBOT_SIMULATION_HPP_