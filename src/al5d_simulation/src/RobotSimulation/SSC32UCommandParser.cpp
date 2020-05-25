#include <SSC32UCommand.hpp>
#include <sstream>

namespace RobotSimulation {

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

        std::string verifiedCommand;
        std::getline(ss, verifiedCommand, '\r');
        verifiedCommand += '\r';

        auto it = verifiedCommand.begin();

        if (command == "STOP\r") {
            SSC32UCommand cmd;
            cmd.commandType = SSC32UCommandType::STOP;
            return std::make_shared<SSC32UCommand>(cmd);
        } else if (*it == '#') {
            ServoCommand servoCommand;
            servoCommand.commandType = SSC32UCommandType::SERVO_COMMAND;
            servoCommand.timeSet = false;

            while (true) {
                switch (*it) {
                    case '#': {
                        SingleServoCommand single_servo_command;
                        single_servo_command.servoMovementSpeed = -1;
                        single_servo_command.channel = static_cast<short>((*(++it))++ - '0');
                        ++it;

                        single_servo_command.pulseWidth = (unsigned long) (std::atoi(getValue(it).c_str()));
                        servoCommand.commands.push_back(single_servo_command);
                        break;
                    }
                    case 'T': {
                        servoCommand.time = (unsigned long long) std::atoi(getValue(it).c_str());
                        servoCommand.timeSet = true;
                        break;
                    }
                    case 'S': {
                        servoCommand.commands.back().servoMovementSpeed = (unsigned long) std::atoi(
                                getValue(it).c_str());
                        break;
                    }
                    case '\r': {
                        return std::make_shared<ServoCommand>(servoCommand);
                    }
                }
            }
        } else {
            if (*it == 'Q' && *(++it) != 'P') {
                SSC32UCommand cmd;
                cmd.commandType = SSC32UCommandType::QUERY_MOVEMENT_STATUS;
                return std::make_shared<SSC32UCommand>(cmd);
            } else if (it != verifiedCommand.end() - 1) {
                PulseWidthQuery cmd;
                cmd.commandType = SSC32UCommandType::QUERY_PULSE_WIDTH;
                cmd.channel = (uint8_t) (std::atoi(getValue(it).c_str()));
                return std::make_shared<PulseWidthQuery>(cmd);
            }
        }
        return std::make_shared<SSC32UCommand>();
    }
}
