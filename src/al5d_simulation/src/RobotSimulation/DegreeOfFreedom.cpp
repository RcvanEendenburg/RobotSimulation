#include <DegreeOfFreedom.hpp>
#include <thread>

namespace robot_simulation {
    long DegreeOfFreedom::cmd_time_ = NOTSET;

    DegreeOfFreedom::DegreeOfFreedom(urdf::JointConstSharedPtr joint_shared_ptr)
            : servo_name_(joint_shared_ptr->name),
              current_pos_(0),
              target_pos_(0),
              speed_(0),
              max_speed_(joint_shared_ptr->limits->velocity),
              min_rad_(joint_shared_ptr->limits->lower),
              max_rad_(joint_shared_ptr->limits->upper),
              update_received_(false),
              moving_(false),
              rate_(UPDATE_RATE) {
        speed_ = max_speed_;
    }

    double DegreeOfFreedom::stepsToTargetTime() const {
        return ((target_pos_ - current_pos_) / cmd_time_) * UPDATE_RATE;
    }

    double DegreeOfFreedom::stepsToTargetSpeed() const {
        if (target_pos_ - current_pos_ > 0) {
            return speed_ / UPDATE_RATE;
        }
        return -(speed_ / UPDATE_RATE);
    }

    bool DegreeOfFreedom::useSpeed() {
        return (std::abs(target_pos_ - current_pos_)) / speed_ > cmd_time_ / 1000;
    }

    void DegreeOfFreedom::stopMovement() {
        target_pos_ = current_pos_;
        update_received_ = true;
    }

    void DegreeOfFreedom::updateServo() {
        update_received_ = false;// update is handled in this function so we can set update_received_ to false
        moving_ = true;

        ROS_DEBUG_STREAM("Servoinfo: \n currentpos: " + std::to_string(current_pos_) + "\n targetpos: " +
                         std::to_string(target_pos_) + "\n time: " + std::to_string(cmd_time_) + "\n speed: " +
                         std::to_string(speed_));

        double step = 0;
        if (speed_ == NOTSET && cmd_time_ == NOTSET) {
            step = target_pos_ - current_pos_;
        } else if ((int) speed_ == NOTSET && cmd_time_ >= 1) {
            step = stepsToTargetTime();
        } else if (speed_ > 0 && cmd_time_ == NOTSET) {
            step = stepsToTargetSpeed();
        } else// this step is reached if both Time and Speed are set.
        {
            if (useSpeed()) {
                step = stepsToTargetSpeed();
            } else {
                step = stepsToTargetTime();
            }
        }
        ROS_DEBUG_STREAM("step = " + std::to_string(step));

        // Add epsilon to make sure diff is bigger then the smallest double steps and we can compare it.
        double diff = std::fabs(step) + std::numeric_limits<double>::epsilon();

        while (!update_received_)// as long we don't receive a new update. stay in this loop.
        {
            std::unique_lock<std::mutex> lock(mutex_pos_);
            if (std::fabs(target_pos_ - current_pos_) <= diff) {
                current_pos_ = target_pos_;
                break;
            }
            current_pos_ += step;
            lock.unlock();
            rate_.sleep();
        }
        moving_ = false;
    }

    void DegreeOfFreedom::setSpeed(double speed) {
        speed_ = speed;
    }

    void DegreeOfFreedom::setTargetPos(double target_pos) {
        if (target_pos > max_rad_) {
            target_pos_ = max_rad_;
        } else if (target_pos < min_rad_) {
            target_pos_ = min_rad_;
        } else {
            target_pos_ = target_pos;
        }
    }

    void DegreeOfFreedom::notifyChange() {
        update_received_ = true;
    }

    bool DegreeOfFreedom::isUpdateReceived() const {
        return update_received_;
    }

    void DegreeOfFreedom::startUpdateThread() {
        std::thread servoThread(&DegreeOfFreedom::updateServo, this);// create a new thread to update the servo
        servoThread.detach();                                        // detach and let the thread handle itself
    }

    double DegreeOfFreedom::getCurrentPos() const{
        std::unique_lock<std::mutex> lock(mutex_pos_);
        return current_pos_;
    }

    double DegreeOfFreedom::getMaxSpeed() const {
        return max_speed_;
    }

    double DegreeOfFreedom::getMaxRad() const {
        return max_rad_;
    }

    double DegreeOfFreedom::getMinRad() const {
        return min_rad_;
    }

    bool DegreeOfFreedom::isMoving() const {
        return moving_;
    }

    void DegreeOfFreedom::setTime(long a_time) {
        cmd_time_ = a_time;
    }

    double DegreeOfFreedom::getTime() {
        return cmd_time_;
    }

    std::string DegreeOfFreedom::getServoName() const
    {
        return servo_name_;
    }
}