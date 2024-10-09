#ifndef LX16A_CONSTS_HPP
#define LX16A_CONSTS_HPP

#include <cstdint>

namespace lx16a {

constexpr uint8_t SERVO_ID_ALL = 0xfe;
constexpr uint8_t SERVO_FRAME_HEADER = 0x55;

// CMDS
constexpr uint8_t SERVO_MOVE_TIME_WRITE = 1;
constexpr uint8_t SERVO_MOVE_TIME_READ = 2;
constexpr uint8_t SERVO_MOVE_TIME_WAIT_WRITE = 7;
constexpr uint8_t SERVO_MOVE_TIME_WAIT_READ = 8;
constexpr uint8_t SERVO_MOVE_START = 11;
constexpr uint8_t SERVO_MOVE_STOP = 12;
constexpr uint8_t SERVO_ID_WRITE = 13;
constexpr uint8_t SERVO_ID_READ = 14;
constexpr uint8_t SERVO_ANGLE_OFFSET_ADJUST = 17;
constexpr uint8_t SERVO_ANGLE_OFFSET_WRITE = 18;
constexpr uint8_t SERVO_ANGLE_OFFSET_READ = 19;
constexpr uint8_t SERVO_ANGLE_LIMIT_WRITE = 20;
constexpr uint8_t SERVO_ANGLE_LIMIT_READ = 21;
constexpr uint8_t SERVO_VIN_LIMIT_WRITE = 22;
constexpr uint8_t SERVO_VIN_LIMIT_READ = 23;
constexpr uint8_t SERVO_TEMP_MAX_LIMIT_WRITE = 24;
constexpr uint8_t SERVO_TEMP_MAX_LIMIT_READ = 25;
constexpr uint8_t SERVO_TEMP_READ = 26;
constexpr uint8_t SERVO_VIN_READ = 27;
constexpr uint8_t SERVO_POS_READ = 28;
constexpr uint8_t SERVO_OR_MOTOR_MODE_WRITE = 29;
constexpr uint8_t SERVO_OR_MOTOR_MODE_READ = 30;
constexpr uint8_t SERVO_LOAD_OR_UNLOAD_WRITE = 31;
constexpr uint8_t SERVO_LOAD_OR_UNLOAD_READ = 32;
constexpr uint8_t SERVO_LED_CTRL_WRITE = 33;
constexpr uint8_t SERVO_LED_CTRL_READ = 34;
constexpr uint8_t SERVO_LED_ERROR_WRITE = 35;
constexpr uint8_t SERVO_LED_ERROR_READ = 36;

// ERRORS
constexpr uint8_t SERVO_ERROR_OVER_TEMPERATURE = 1;
constexpr uint8_t SERVO_ERROR_OVER_VOLTAGE = 2;
constexpr uint8_t SERVO_ERROR_LOCKED_ROTOR = 4;

// SERVOS
constexpr uint8_t MOTOR_LEFT_FRONT = 1;
constexpr uint8_t MOTOR_LEFT_MIDDLE = 2;
constexpr uint8_t MOTOR_LEFT_BACK = 3;
constexpr uint8_t MOTOR_RIGHT_FRONT = 4;
constexpr uint8_t MOTOR_RIGHT_MIDDLE = 5;
constexpr uint8_t MOTOR_RIGHT_BACK = 6;

constexpr uint8_t SERVO_LEFT_FRONT = 7;
constexpr uint8_t SERVO_RIGHT_FRONT = 8;
constexpr uint8_t SERVO_LEFT_BACK = 9;
constexpr uint8_t SERVO_RIGHT_BACK = 10;

} // namespace lx16a
#endif
