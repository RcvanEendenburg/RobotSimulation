#include <SSC32UCommand.hpp>
#include <sstream>

namespace robot_simulation {

    std::string getValue(std::string::iterator &it) {
        std::string value;
        ++it;

        // A character is part of the value if it is numeric
        for (char c = *it; (c >= '0') && (c <= '9'); c = *(++it)) {
            value += c;
        }
        return value;
    }

    std::shared_ptr<SSC32UCommand> SSC32UCommand::fromString(const std::string &command) {
        std::stringstream ss;
        ss << command;

        std::string verified_command;
        std::getline(ss, verified_command, '\r');
        verified_command += '\r';

        auto it = verified_command.begin();

        if (command == "STOP\r") {
            SSC32UCommand cmd;
            cmd.command_type_ = SSC32UCommandType::STOP;
            return std::make_shared<SSC32UCommand>(cmd);
        } else if (*it == '#') {
            ServoCommand servo_command;
            servo_command.command_type_ = SSC32UCommandType::SERVO_COMMAND;
            servo_command.time_set_ = false;

            while (true) {
                switch (*it) {
                    case '#': {
                        SingleServoCommand single_servo_command;
                        single_servo_command.servo_movement_speed_ = -1;
                        single_servo_command.channel_ = static_cast<short>((*(++it))++ - '0');
                        ++it;

                        single_servo_command.pulse_width_ = (unsigned long) (std::atoi(getValue(it).c_str()));
                        servo_command.commands_.push_back(single_servo_command);
                        break;
                    }
                    case 'T': {
                        servo_command.time_ = (unsigned long long) std::atoi(getValue(it).c_str());
                        servo_command.time_set_ = true;
                        break;
                    }
                    case 'S': {
                        servo_command.commands_.back().servo_movement_speed_ = (unsigned long) std::atoi(
                                getValue(it).c_str());
                        break;
                    }
                    case '\r': {
                        return std::make_shared<ServoCommand>(servo_command);
                    }
                }
            }
        } else {
            if (*it == 'Q' && *(++it) != 'P') {
                SSC32UCommand cmd;
                cmd.command_type_ = SSC32UCommandType::QUERY_MOVEMENT_STATUS;
                return std::make_shared<SSC32UCommand>(cmd);
            } else if (it != verified_command.end() - 1) {
                PulseWidthQuery cmd;
                cmd.command_type_ = SSC32UCommandType::QUERY_PULSE_WIDTH;
                cmd.channel = (uint8_t) (std::atoi(getValue(it).c_str()));
                return std::make_shared<PulseWidthQuery>(cmd);
            }
        }
        return std::make_shared<SSC32UCommand>();
    }
}
