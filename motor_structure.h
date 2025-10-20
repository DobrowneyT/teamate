/**
 * @file motor_structure.h
 * @brief Motor control structures for the TeaMate robot
 * @date 2025-03-24
 */

#ifndef MOTOR_STRUCTURE_H
#define MOTOR_STRUCTURE_H

#include "MeMegaPiWrapper.h" // Use wrapper instead of direct include

// =============================================
// Hardware Configuration Structures
// =============================================

/**
 * @struct motor_config
 * @brief Permanent configuration parameters for a motor
 */
struct motor_config {
  uint8_t slot;          ///< MegaPi expansion slot (SLOT1-SLOT4)
  float gear_ratio;      ///< Total gear reduction (motor_shaft:output_shaft)
  int encoder_ppr;       ///< Encoder pulses per revolution (PPR)
  float max_rpm;         ///< Maximum safe RPM at output shaft
  float position_pid_p;  ///< Proportional gain for position PID controller
  float position_pid_i;  ///< Integral gain for position PID controller
  float position_pid_d;  ///< Derivative gain for position PID controller
  float speed_pid_p;     ///< Proportional gain for speed PID controller
  float speed_pid_i;     ///< Integral gain for speed PID controller
  float speed_pid_d;     ///< Derivative gain for speed PID controller
  String name;           ///< Descriptive name for debugging
};

/**
 * @struct StallDetectionState
 * @brief Tracks state for stall detection during calibration
 */
struct stall_detection_state {
  float prev_pos;            ///< Previous position reading
  unsigned long last_check;  ///< Last check timestamp
  unsigned long start_time;  ///< Phase start timestamp
  bool was_stalled;          ///< Stalled state flag
};

/**
 * @struct calibration_params
 * @brief Parameters for automatic motor calibration
 */
struct calibration_params {
  int travel_limit;              ///< Maximum encoder counts for calibration moves
  float calibration_speed;       ///< RPM for calibration movements
  int stall_threshold;           ///< Position delta threshold for stall detection
  unsigned long timeout;         ///< Maximum calibration time (ms)
  unsigned long startup_delay;   ///< Initial movement delay before stall checks
  unsigned long check_interval;  ///< Time between stall checks
  float safe_position;           ///< Position to move dependent joints to before calibration
};

/**
 * @struct encoder_motor
 * @brief Complete state container for a motor controller
 */
struct encoder_motor {
  MeEncoderOnBoard* encoder;      ///< Pointer to motor controller instance
  motor_config config;            ///< Static configuration parameters
  calibration_params cal;         ///< Calibration settings
  stall_detection_state cal_state;  ///< Current calibration state
  float joint_gear_ratio;         ///< Output/Input of gear ratio attached to motor shaft
  float current_pos;              ///< Current position in degrees
  float neutral_pos;              ///< Home position
  float min_angle;                ///< Minimum safe angle
  float max_angle;                ///< Maximum safe angle
  bool is_calibrated;             ///< Calibration status
  bool arrived;                   ///< Movement status
};

// // Declare external global variables
// extern encoder_motor base_motor;
// extern encoder_motor joint1_motor;
// extern encoder_motor joint2_motor;

#endif /* MOTOR_STRUCTURE_H */