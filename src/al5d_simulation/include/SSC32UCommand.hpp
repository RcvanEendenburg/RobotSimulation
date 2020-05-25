#ifndef SSC32U_COMMAND_HPP
#define SSC32U_COMMAND_HPP

#include <memory>
#include <string>
#include <vector>

namespace RobotSimulation {
/**
 * @brief this enum class contains all the SS32U Command types that are supported (and can be parsed)
 */
    enum class SSC32UCommandType {
        SERVO_COMMAND,
        STOP,
        QUERY_MOVEMENT_STATUS,
        QUERY_PULSE_WIDTH
    };

/**
 * @brief The SSC32UCommand super class: each SSC32U command must derive from this class
 */
    struct SSC32UCommand {
        static std::shared_ptr<SSC32UCommand> fromString(const std::string &command);

        SSC32UCommandType commandType;
    };

/**
 * @brief a command for a single servo command SSC32U
 */
    struct SingleServoCommand : public SSC32UCommand {
        short channel;
        double pulseWidth;
        double servoMovementSpeed;
    };

/**
 * @brief a command for a SSC32U servocommand (multiple single servo commands)
 */
    struct ServoCommand : public SSC32UCommand {
        std::vector<SingleServoCommand> commands;
        unsigned long long time;

        bool timeSet;
    };

/**
 * @brief a command for a SSC32U pulse width query command
 */
    struct PulseWidthQuery : public SSC32UCommand {
        uint8_t channel;
    };
}


#endif// SSC32U_COMMAND_HPP