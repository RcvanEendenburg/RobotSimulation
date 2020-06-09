#include <RobotSimulation.hpp>
#include <cmath>
#include <sstream>

namespace robot_simulation {

    Al5dSimulation::Al5dSimulation(std::string model_data_namespace) {
        model_.initParam(model_data_namespace);
        std::string ns = "/joint_states";
        servos_.push_back(std::make_shared<DegreeOfFreedom>(model_.getJoint("base_link2turret")));
        servos_.push_back(std::make_shared<DegreeOfFreedom>(model_.getJoint("turret2upperarm")));
        servos_.push_back(std::make_shared<DegreeOfFreedom>(model_.getJoint("upperarm2forearm")));
        servos_.push_back(std::make_shared<DegreeOfFreedom>(model_.getJoint("forearm2wrist")));
        servos_.push_back(std::make_shared<DegreeOfFreedom>(model_.getJoint("wrist2hand")));
        servos_.push_back(std::make_shared<DegreeOfFreedom>(model_.getJoint("gripper_left2hand")));
        servos_.push_back(std::make_shared<DegreeOfFreedom>(model_.getJoint("gripper_right2hand")));
        state_publisher_->initialize(ns);
        state_publisher_->startPublishing();
    }

    Al5dSimulation::~Al5dSimulation() {
        state_publisher_->stopPublishing();
    }

    void Al5dSimulation::handleRequest(const std_msgs::String::ConstPtr& msg) {
        std::shared_ptr<SSC32UCommand> parsed_command = SSC32UCommand::fromString(msg->data.c_str());

        switch (parsed_command->command_type_) {
            case SSC32UCommandType::STOP: {
                stop();
                break;
            }
            case SSC32UCommandType::QUERY_MOVEMENT_STATUS: {
                // TODO: Are we actually replying or is that not necessary?
                break;
            }
            case SSC32UCommandType::QUERY_PULSE_WIDTH: {
                // TODO: Are we actually replying or is that not necessary?
                break;
            }
            case SSC32UCommandType::SERVO_COMMAND: {
                auto servo_command = std::static_pointer_cast<ServoCommand>(parsed_command);

                if (servo_command->time_set_)// Calculate the speed for each servo if not set
                {
                    DegreeOfFreedom::setTime(-1);
                    for (auto &singleCommand : servo_command->commands_) {
                        if(singleCommand.channel_ == (short)servos_.size())
                            break;
                        if (singleCommand.servo_movement_speed_ == (int) NOTSET) {
                            double currentPosPwm = getServoPWM(singleCommand.channel_);
                            double change = std::fabs(singleCommand.pulse_width_ - currentPosPwm);
                            singleCommand.servo_movement_speed_ = std::fabs(
                                    (change * 1000.0) / (double) servo_command->time_);
                        }
                    }
                }
                for (auto const &singleCommand : servo_command->commands_) {
                    if(singleCommand.channel_ == (short) servos_.size())
                        break;
                    moveServo(singleCommand.channel_, singleCommand.pulse_width_, singleCommand.servo_movement_speed_);
                    if (singleCommand.channel_ == gripper_a_channel_) {
                        moveServo(gripper_b_channel_, singleCommand.pulse_width_, singleCommand.servo_movement_speed_);
                    }
                }
                break;
            }
        }
    }

    double Al5dSimulation::getServoPWM(short servo_id) {
        return toRadian(servo_id,
                        servos_.at(servo_id)->getCurrentPos(),
                        servos_.at(servo_id)->getMaxRad(),
                        servos_.at(servo_id)->getMinRad(),
                        servo_max_[servo_id],
                        servo_min_[servo_id]);
    }

    void Al5dSimulation::moveServo(short servo_id, double pwm, double speed) {
        servos_.at(servo_id)->notifyChange();
        servos_.at(servo_id)->setTargetPos(toRadian(servo_id,
                                                    pwm,
                                                    servo_max_[servo_id],
                                                    servo_min_[servo_id],
                                                    servos_.at(servo_id)->getMaxRad(),
                                                    servos_.at(servo_id)->getMinRad()));
        servos_.at(servo_id)->setSpeed(speedToRad(speed, servo_id));
        servos_.at(servo_id)->startUpdateThread();
    }

    double Al5dSimulation::toRadian(short servo_id,
                                    double input_val,
                                    double max_angle,
                                    double min_angle,
                                    double max_rad,
                                    double min_rad) const {
        if (servo_id == gripper_a_channel_ || servo_id == gripper_b_channel_) {
            if (servo_id == gripper_a_channel_) {
                if (std::fabs(input_val - servo_min_[gripper_a_channel_]) <= std::fabs(input_val - servo_max_[gripper_a_channel_])) {
                    input_val = servo_max_[gripper_a_channel_] + std::fabs(input_val - servo_min_[gripper_a_channel_]);
                } else {
                    input_val = servo_min_[gripper_a_channel_] - std::fabs(input_val - servo_max_[gripper_a_channel_]);
                }
            }
            double r = std::fabs((input_val - servo_min_[servo_id]) * ((0.04 / (servo_max_[servo_id] - servo_min_[servo_id])))) -
                       0.02;
            return r;
        }
        return (input_val - min_angle) * (max_rad - min_rad) / (max_angle - min_angle) + min_rad;
    }

    double Al5dSimulation::speedToRad(double speed, short servo_id) {
        if ((int) speed == (int) -1) {
            return -1;
        }
        double servoRangeRad = std::fabs(servos_.at(servo_id)->getMinRad() - servos_.at(servo_id)->getMaxRad());
        double servoRangePWM = std::fabs(servo_min_[servo_id] - servo_max_[servo_id]);
        return (double) std::fabs(speed * std::fabs(servoRangeRad / (servoRangePWM)));
    }

    bool Al5dSimulation::armIsMoving() {
        for(auto& servo : servos_)
        {
            if(servo->isMoving())
                return true;
        }
        return false;
    }

    void Al5dSimulation::stop() {
        for(auto& servo : servos_)
        {
            servo->stopMovement();
        }
    }
}