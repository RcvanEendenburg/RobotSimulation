#include <RobotSimulation.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace RobotSimulation {

    Al5dSimulation::Al5dSimulation(std::string urdfFile) {
        model.initFile(urdfFile);
        std::string ns = "/joint_states";
      servos.push_back(DegreeOfFreedom(model.getJoint("base_link2turret")));
      servos.push_back(DegreeOfFreedom(model.getJoint("turret2upperarm")));
      servos.push_back(DegreeOfFreedom(model.getJoint("upperarm2forearm")));
      servos.push_back(DegreeOfFreedom(model.getJoint("forearm2wrist")));
      servos.push_back(DegreeOfFreedom(model.getJoint("wrist2hand")));
      servos.push_back(DegreeOfFreedom(model.getJoint("gripper_left2hand")));
      servos.push_back(DegreeOfFreedom(model.getJoint("gripper_right2hand")));
      statePublisher.Initialize( ns);
      statePublisher.StartPublishing();
    }

    Al5dSimulation::~Al5dSimulation() {
        statePublisher.StopPublishing();
    }

    void Al5dSimulation::handleRequest( std_msgs::String::ConstPtr& msg) {
        std::shared_ptr<SSC32UCommand> parsedCommand = SSC32UCommand::fromString(msg->data.c_str());

        switch (parsedCommand->commandType) {
            case SSC32UCommandType::STOP: {
                stop();
                break;
            }
            case SSC32UCommandType::QUERY_MOVEMENT_STATUS: {
                // TODO: Are we actually replying or is that not necessary?


                /*
                if (armIsMoving()) {
                    response = "+";
                } else {
                    response = ".";
                }
                responseAvailable = true;
                 */
                break;
            }
            case SSC32UCommandType::QUERY_PULSE_WIDTH: {
                // TODO: Are we actually replying or is that not necessary?


                /*
                auto queryPWCommand = std::static_pointer_cast<PulseWidthQuery>(parsedCommand);

                response = std::to_string((int) getServoPWM(queryPWCommand->channel) / 10);
                responseAvailable = true;
                 */
                break;
            }
            case SSC32UCommandType::SERVO_COMMAND: {
                auto servoCommand = std::static_pointer_cast<ServoCommand>(parsedCommand);

                if (servoCommand->timeSet)// Calculate the speed for each servo if not set
                {
                    DegreeOfFreedom::setTime(-1);
                    for (auto &singleCommand : servoCommand->commands) {
                        if (singleCommand.servoMovementSpeed == (int) NOTSET) {
                            double currentPosPwm = getServoPWM(singleCommand.channel);
                            double change = std::fabs(singleCommand.pulseWidth - currentPosPwm);
                            singleCommand.servoMovementSpeed = std::fabs(
                                    (change * 1000.0) / (double) servoCommand->time);
                        }
                    }
                }
                for (auto const &singleCommand : servoCommand->commands) {
                    moveServo(singleCommand.channel, singleCommand.pulseWidth, singleCommand.servoMovementSpeed);
                    if (singleCommand.channel == gripperANr) {
                        moveServo(gripperBNr, singleCommand.pulseWidth, singleCommand.servoMovementSpeed);
                    }
                }
                break;
            }
        }
    }

    double Al5dSimulation::getServoPWM(short servoNr) {
        return toRadian(servoNr,
                        servos.at(servoNr).getCurrentPos(),
                        servos.at(servoNr).getMaxRad(),
                        servos.at(servoNr).getMinRad(),
                        servoMax[servoNr],
                        servoMin[servoNr]);
    }

    void Al5dSimulation::moveServo(short aServoNr, double aPwm, double aSpeed) {
        servos.at(aServoNr).notifyChange();
        servos.at(aServoNr).setTargetPos(toRadian(aServoNr,
                                                  aPwm,
                                                  servoMax[aServoNr],
                                                  servoMin[aServoNr],
                                                  servos.at(aServoNr).getMaxRad(),
                                                  servos.at(aServoNr).getMinRad()));
        servos.at(aServoNr).setSpeed(speedToRad(aSpeed, aServoNr));
    }

    double Al5dSimulation::toRadian(short servoNr,
                                    double inputVal,
                                    double maxAngle,
                                    double minAngle,
                                    double maxRad,
                                    double minRad) const {
        if (servoNr == gripperANr || servoNr == gripperBNr) {
            if (servoNr == gripperANr) {
                if (std::fabs(inputVal - servoMin[gripperANr]) <= std::fabs(inputVal - servoMax[gripperANr])) {
                    inputVal = servoMax[gripperANr] + std::fabs(inputVal - servoMin[gripperANr]);
                } else {
                    inputVal = servoMin[gripperANr] - std::fabs(inputVal - servoMax[gripperANr]);
                }
            }
            double r = std::fabs((inputVal - servoMin[servoNr]) * ((0.04 / (servoMax[servoNr] - servoMin[servoNr])))) -
                       0.02;
            return r;
        }
        return (inputVal - minAngle) * (maxRad - minRad) / (maxAngle - minAngle) + minRad;
    }

    double Al5dSimulation::speedToRad(double aSpeed, short servoNr) {
        if ((int) aSpeed == (int) -1) {
            return -1;
        }
        double servoRangeRad = std::fabs(servos.at(servoNr).getMinRad() - servos.at(servoNr).getMaxRad());
        double servoRangePWM = std::fabs(servoMin[servoNr] - servoMax[servoNr]);
        return (double) std::fabs(aSpeed * std::fabs(servoRangeRad / (servoRangePWM)));
    }

    bool Al5dSimulation::armIsMoving() {
        for(auto& servo : servos)
        {
            if(servo.isMoving())
                return true;
        }
        return false;
    }

    void Al5dSimulation::stop() {
        for(auto& servo : servos)
        {
            servo.stopMovement();
        }
    }
}