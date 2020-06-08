#ifndef VIRTUAL_AL5D_ROBOT_HPP_
#define VIRTUAL_AL5D_ROBOT_HPP_

#include "std_msgs/String.h"
#include "DegreeOfFreedom.hpp"
#include "StatePublisher.hpp"
#include <SSC32UCommand.hpp>
#include <ros/ros.h>
#include <memory>
#include <thread>

namespace robot_simulation {

    class Al5dSimulation {
    public:
        explicit Al5dSimulation(std::string model_data_namespace);

        virtual ~Al5dSimulation();

        void startListening(std::string topic_name)
        {
                sub_ = n_.subscribe(topic_name, 1000, &Al5dSimulation::handleRequest, this);
        }

        /**
         * @brief Handles incoming SSC32U requests
         *
         * @param SSC32U request
         */
        void handleRequest(const std_msgs::String::ConstPtr& msg);

        /**
         * @brief This function maps input_val values to radial based on min and max angle
         * @param servo_id the servo id
         * @param input_val The value to set to radians
         * @param max_angle Max angle in PWM
         * @param min_angle Min angle in PWM
         * @param max_rad Max angle in radian
         * @param min_rad Min angle in radian
         * @return radians between min & max radian
         */
        double
        toRadian(short servo_id, double input_val, double max_angle, double min_angle, double max_rad, double min_rad) const;

        /**
         * @brief function that converts a speed in pwm to a speed in rad/s  *
         * @param speed the speed in pwm
         * @param servo_id the servo id
         * @return double the speed in rad/s
         */
        double speedToRad(double speed, short servo_id);

    private:
        /**
         * @brief Get the Servo PWM object
         *
         * @param servo_id the id of the servo
         * @return the current pwm
         */
        double getServoPWM(short servo_id);

        /**
         * @brief function thats moves a servo
         *
         * @param servo_id the id of the servo to be moved
         * @param pwm the pwm the servo should move to
         * @param speed the speed that the servo should be moved with
         */
        void moveServo(short servo_id, double pwm, double speed);
        /**
         * @brief checks if the arm moves
         * @return true if moving_
         */
        bool armIsMoving();
        /**
         * @brief stops the arm from moving_
         */
        void stop();

        ros::NodeHandle n_;
        ros::Subscriber sub_;
        std::unique_ptr<StatePublisher> state_publisher_ = std::make_unique<StatePublisher>(servos_);
        const uint16_t servo_min_[7] = {2500, 500, 1950, 500, 2650, 2500, 500};
        const uint16_t servo_max_[7] = {500, 1950, 500, 2500, 500, 500, 2500};
        const short gripper_a_channel_ = 5;
        const short gripper_b_channel_ = 6;
        urdf::Model model_;
        std::vector<std::shared_ptr<DegreeOfFreedom>> servos_;
    };
}
#endif// ROBOT_SIMULATION_HPP_