#include <DegreeOfFreedom.hpp>
#include <thread>

namespace RobotSimulation {
    long DegreeOfFreedom::cmdTime = NOTSET;

    DegreeOfFreedom::DegreeOfFreedom(urdf::JointConstSharedPtr jointSharedPtr)
            : servoName(jointSharedPtr->name),
              currentPos(0),
              targetPos(0),
              speed(0),
              maxSpeed(jointSharedPtr->limits->velocity),
              minRad(jointSharedPtr->limits->lower),
              maxRad(jointSharedPtr->limits->upper),
              updateReceived(false),
              moving(false),
              rate(UPDATE_RATE) {
        speed = maxSpeed;
    }

    double DegreeOfFreedom::stepsToTargetTime() const {
        return ((targetPos - currentPos) / cmdTime) * UPDATE_RATE;
    }

    double DegreeOfFreedom::stepsToTargetSpeed() const {
        if (targetPos - currentPos > 0) {
            return speed / UPDATE_RATE;
        }
        return -(speed / UPDATE_RATE);
    }

    bool DegreeOfFreedom::useSpeed() {
        return (std::abs(targetPos - currentPos)) / speed > cmdTime / 1000;
    }

    void DegreeOfFreedom::stopMovement() {
        targetPos = currentPos;
        updateReceived = true;
    }

    void DegreeOfFreedom::updateServo() {
        updateReceived = false;// update is handled in this function so we can set updateReceived to false
        moving = true;

        ROS_DEBUG_STREAM("Servoinfo: \n currentpos: " + std::to_string(currentPos) + "\n targetpos: " +
                         std::to_string(targetPos) + "\n time: " + std::to_string(cmdTime) + "\n speed: " +
                         std::to_string(speed));

        double step = 0;
        if (speed == NOTSET && cmdTime == NOTSET) {
            step = targetPos - currentPos;
        } else if ((int) speed == NOTSET && cmdTime >= 1) {
            step = stepsToTargetTime();
        } else if (speed > 0 && cmdTime == NOTSET) {
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

        while (!updateReceived)// as long we don't receive a new update. stay in this loop.
        {
            std::unique_lock<std::mutex> lock(mutexPos);
            if (std::fabs(targetPos - currentPos) <= diff) {
                currentPos = targetPos;
                break;
            }
            currentPos += step;
            lock.unlock();
            rate.sleep();
        }
        moving = false;
    }

    void DegreeOfFreedom::setSpeed(double aspeed) {
        speed = aspeed;
    }

    void DegreeOfFreedom::setTargetPos(double atargetPos) {
        if (atargetPos > maxRad) {
            targetPos = maxRad;
        } else if (atargetPos < minRad) {
            targetPos = minRad;
        } else {
            targetPos = atargetPos;
        }
    }

    void DegreeOfFreedom::notifyChange() {
        updateReceived = true;
    }

    bool DegreeOfFreedom::isUpdateReceived() const {
        return updateReceived;
    }

    void DegreeOfFreedom::startUpdateThread() {
        std::thread servoThread(&DegreeOfFreedom::updateServo, this);// create a new thread to update the servo
        servoThread.detach();                                        // detach and let the thread handle itself
    }

    double DegreeOfFreedom::getCurrentPos() const{
        std::unique_lock<std::mutex> lock(mutexPos);
        return currentPos;
    }

    double DegreeOfFreedom::getMaxSpeed() const {
        return maxSpeed;
    }

    double DegreeOfFreedom::getMaxRad() const {
        return maxRad;
    }

    double DegreeOfFreedom::getMinRad() const {
        return minRad;
    }

    bool DegreeOfFreedom::isMoving() const {
        return moving;
    }

    void DegreeOfFreedom::setTime(long atime) {
        cmdTime = atime;
    }

    double DegreeOfFreedom::getTime() {
        return cmdTime;
    }

    std::string DegreeOfFreedom::getServoName() const
    {
        return servoName;
    }
}