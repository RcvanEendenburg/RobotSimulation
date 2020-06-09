#ifndef DEGREE_OF_FREEDOM_HPP_
#define DEGREE_OF_FREEDOM_HPP_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "urdf/model.h"
#include <string>
#include <mutex>

#define NOTSET -1
#define UPDATE_RATE 50// Publish rate of updates in Hz

namespace robot_simulation {
/**
 * @brief Each degree of freedom has its own characteristics (such as speed, current position)
 */
    class DegreeOfFreedom{
    public:
        /**
         * @brief Construct a new Degree Of Freedom object
         * @author Rene van Eendenburg
         *
         * @param joint_shared_ptr pointer to the joint in the urdf model
         * @param aNamespace the robotname namespace
         */
        DegreeOfFreedom(urdf::JointConstSharedPtr joint_shared_ptr);

        virtual ~DegreeOfFreedom() = default;

        /**
         * @brief function that starts the position updating thread
         * @author Rene van Eendenburg
         */
        void startUpdateThread();

        /**
         * @brief function that notifies a servo its position is about to change
         * @author Rene van Eendenburg
         */
        void notifyChange();

        /**
         * @author Rene van Eendenburg
         * Function to stop the servo from moving_.
         */
        void stopMovement();

        /** Simple getters and setters */
        bool isUpdateReceived() const;

        static void setTime(long atime);

        static double getTime();

        void setSpeed(double speed);

        void setTargetPos(double target_pos);

        double getCurrentPos() const;

        double getMaxSpeed() const;

        double getMaxRad() const;

        double getMinRad() const;

        bool isMoving() const;

        std::string getServoName() const;

    private:
        /**
         * @author Rene van Eendenburg
         * Determines rather to use Time_ or Speed_ for the servo in question.
         * If the destination is reached within the time with set speed, Time will be used.
         * If the destination isn't reached within the time with set speed, Speed will be used.
         * Only used by AL5D
         * @return true if speed is the variable used for next calculations
         */
        bool useSpeed();

        /**
         * @author Rene van Eendenburg
         * Uses object variables to determine the PWM/Degree steps to take per tick to reach destination in given time.
         * Only used by AL5D
         */
        double stepsToTargetTime() const;

        /**
         * @author Rene van Eendenburg
         * Uses object variables to determine the PWM/Degree steps to take per tick to reach destination with given speed.
         */
        double stepsToTargetSpeed() const;

        /**
         * @author Rene van Eendenburg
         * checks set variables and determines which to use. Then updates the servo for every update tick
         */
        void updateServo();

        std::string servo_name_;
        double current_pos_;
        double target_pos_;
        double speed_;
        const double max_speed_;
        const double min_rad_;
        const double max_rad_;
        bool update_received_;
        bool moving_;
        ros::Rate rate_;
        static long cmd_time_;
        mutable std::mutex mutex_pos_;
    };
}


#endif// DEGREE_OF_FREEDOM_HPP_