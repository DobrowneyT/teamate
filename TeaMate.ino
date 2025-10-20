/**
 * @file TeaMate.ino
 * @brief Main control system for 3DOF robotic arm with claw mechanism
 * @author Tristan Dobrowney & Evan Evaniuk
 * @date 2025-03-24
 * @version 1.0
 * 
 * TeaMate is an automated tea steeping robot that uses a 3DOF robotic arm
 * to grab a tea bag, steep it in a cup for a user-specified duration,
 * and then dispose of it in a trash bin.
 */

// =============================================
// Debug Configuration
// =============================================
#define DEBUG 1  // Set to 1 to enable debug prints, 0 to disable

// =============================================
// Library Includes
// =============================================
#define MEMEGAPI_IMPLEMENTATION
#include <MeMegaPi.h>
#include <U8g2lib.h>

#include "motor_structure.h"
#include "TeaMate_Tones.h"

// =============================================
// Hardware Pin Definitions
// =============================================
const byte ROTARY_CLK = 29;  // Rotary encoder clock pin
const byte ROTARY_DT = 26;   // Rotary encoder data pin
const byte ROTARY_SW = 27;   // Rotary encoder switch pin
const byte BUZZER_PIN = 24;  // Buzzer pin
const byte RED_LED = 30;     // Red LED pin
const byte YELLOW_LED = 39;  // Yellow LED pin
const byte GREEN_LED = 28;   // Green LED pin

// =============================================
// Timer Configuration
// =============================================
const byte DEFAULT_TIME = 10;              // Default timer value in seconds
const int MAX_TIMER_VALUE = 99 * 60 + 59;  // Maximum timer value (99:59)

// Timing Constants
const byte DEBOUNCE_DELAY = 50;             // Debounce delay for button press in ms
const int COUNTDOWN_INTERVAL = 1000;        // Interval for countdown updates in ms
const int DISPLAY_BLINK_INTERVAL = 500;     // Interval for display blinking in ms
const int BUZZER_DURATION = 1000;           // Duration the buzzer plays in ms
const int BUZZER_REMINDER_INTERVAL = 3000;  // Interval between buzzer reminders in ms
const int BUTTON_COOLDOWN = 750;            // Cooldown after reset to prevent immediate start

// Sound Constants
const int NOTE_A5 = 880;               // A5 frequency in Hz
const int NOTE_C6 = 1047;              // C6 frequency in Hz
const int NOTE_E6 = 1319;              // E6 frequency in Hz
const int NOTE_G6 = 1568;              // G6 frequency in Hz
const int NOTE_DURATION_SHORT = 200;   // Duration of short notes in ms
const int NOTE_DURATION_MEDIUM = 350;  // Duration of medium notes in ms
const int NOTE_DURATION_LONG = 400;    // Duration of long notes in ms
const int NOTE_PAUSE = 50;             // Pause between notes in ms

// =============================================
// Robot Configuration
// =============================================
const int JOINT_LIMIT_THRESHOLD = 50;
const float BASE_GEAR_RATIO = 9.0f;  // 72/8 teeth ratio
const float ARM_GEAR_RATIO = 7.0f;   // 56/8 teeth ratio
const float TEABAG_MOVE_SPEED = 35;

const uint8_t ULTRASONIC_SENSOR_PORT = PORT_8;

// =============================================
// State Machine Definition
// =============================================
enum State : byte {
  SETUP,          // Initial setup state for adjusting time
  PLACE_CUP,      // Waiting for user to place cup
  GET_TEABAG,     // Robot grabbing teabag
  TEA_IN_CUP,     // Initial placement of teabag in cup
  COUNTING_DOWN,  // Timer actively counting down while steeping
  TEA_IN_GARBAGE, // Moving teabag to garbage
  FINISHED        // Timer completed, alerting user
};

struct state_vars {
  bool done;
  bool steeped;
  bool in_cup;
};

// =============================================
// Global Variables
// =============================================
byte currentSeconds = DEFAULT_TIME;
byte currentStateCLK;
byte lastStateCLK;
unsigned long lastButtonPress = 0;
unsigned long lastCountdownUpdate = 0;
State currentState = SETUP;
bool buttonCooldown = false;
unsigned long cooldownStartTime = 0;

// Display object
U8G2_SH1106_128X64_NONAME_F_HW_I2C oled_display(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// =============================================
// Claw Control Structure
// =============================================
enum claw_state {
  CLAW_OPEN,
  CLAW_CLOSED
};

struct claw {
  MeMegaPiDCMotor motor;
  claw_state state;
  uint8_t open_speed;
  uint8_t close_speed;
  unsigned long move_time;
};

// =============================================
// Hardware Objects
// =============================================
MeLineFollower lineFinder(PORT_5);
MeUltrasonicSensor ultra_sensor(ULTRASONIC_SENSOR_PORT);

// Motor configurations
const motor_config BASE_CONFIG = {
  SLOT1, 75.0f, 8, 86.0f,
  7.5f, 2.5f, 1.0f, 0.5f, 0.0f, 0.2f,
  "Base"
};

const motor_config JOINT_CONFIG_1 = {
  SLOT2, 46.0f, 8, 185.0f,
  1.8f, 0.0f, 1.2f, 0.18f, 0.0f, 0.0f,
  "Joint 1"
};

const motor_config JOINT_CONFIG_2 = {
  SLOT3, 46.0f, 8, 185.0f,
  7.5f, 2.5f, 1.0f, 0.5f, 0.0f, 0.2f,
  "Joint 2"
};

const calibration_params BASE_CALIBRATION = {
  3240, 20.0f, 5, 10000, 1000, 200
};

const calibration_params JOINT_CALIBRATION = {
  2520, 70.0f, 3, 10000, 1500, 150, 90.0f
};

// Global motor objects
encoder_motor base_motor;
encoder_motor joint1_motor;
encoder_motor joint2_motor;

claw claw = {
  MeMegaPiDCMotor(PORT4A),
  CLAW_CLOSED,
  200,
  200,
  2000
};

// Movement command structure
struct MovementCommand {
  float base;
  float shoulder;
  float elbow;
  float speed;
};

// Pre-defined movement sequences
MovementCommand NeutralToTeagBagCommand[] = {
  { 86, 109, 253, TEABAG_MOVE_SPEED },
  { 86, 144, 263, TEABAG_MOVE_SPEED },
  { 86, 159, 263, TEABAG_MOVE_SPEED },
  { 76, 159, 263, TEABAG_MOVE_SPEED },
  { 180, 174, 230, TEABAG_MOVE_SPEED },
};

MovementCommand TeaBagToCupCommand[] = {
  { 180, 179, 260, 25 },
  { 180, 156, 260, 25 }
};

MovementCommand CupToTrashCommand[] = {
  { 180, 174, 230, TEABAG_MOVE_SPEED },
  { 270, 174, 260, TEABAG_MOVE_SPEED },
  { 270, 141, 260, TEABAG_MOVE_SPEED }
};

encoder_motor* all_motors[] = { &base_motor, &joint1_motor, &joint2_motor };
const uint8_t MOTOR_COUNT = sizeof(all_motors) / sizeof(all_motors[0]);

state_vars state_vars = { false, false, false };

// =============================================
// Function Prototypes
// =============================================
void initialize_motor(encoder_motor* motor, const motor_config& config, const calibration_params& cal, float joint_ratio);
bool check_stall(encoder_motor* motor);
void safe_delay(unsigned long delay_ms);
void calibrate_motor(encoder_motor* motor);
void system_calibration();
void move_motor(encoder_motor* motor, int target_deg, float speed_rpm);
void update_all_motors();
bool all_motors_reached();
void open_claw();
void close_claw();
void home_all_motors();
void emergency_stop();
float joint_to_motor_angle(float joint_angle_deg, float gear_ratio);
float motor_to_joint_angle(float motor_angle_deg, float gear_ratio);
void move_motor_joint_angle(encoder_motor* motor, float joint_angle_deg, float speed_rpm);
void NeutralToTeaBag();
void TeaBagInCup();
void TeaBagOutCup();
void cupToTrash();
void TrashToNeutral();
void init_GPIO();
void handleSetupState();
void handleCountdownState();
void handleFinishedState();
void resetToDefault();
void updateDisplays();
void updateOLEDNormal();
void updateOLEDFinished();
void updateOLEDPlaceCup();
void set_gpio_to_cal();
void reset_stall_state(encoder_motor* motor);
void safe_relative_move(encoder_motor* motor, int dir, float speed, unsigned long duration_ms);
void calibrate_base_motor_with_line_follower(encoder_motor* motor);

// =============================================
// Interrupt Service Routines
// =============================================
void isr_process_encoder1() {
  if (digitalRead(base_motor.encoder->getPortB())) {
    base_motor.encoder->pulsePosPlus();
  } else {
    base_motor.encoder->pulsePosMinus();
  }
}

void isr_process_encoder2() {
  if (digitalRead(joint1_motor.encoder->getPortB())) {
    joint1_motor.encoder->pulsePosPlus();
  } else {
    joint1_motor.encoder->pulsePosMinus();
  }
}

void isr_process_encoder3() {
  if (digitalRead(joint2_motor.encoder->getPortB())) {
    joint2_motor.encoder->pulsePosPlus();
  } else {
    joint2_motor.encoder->pulsePosMinus();
  }
}

// =============================================
// Setup and Main Loop
// =============================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  if (DEBUG) Serial.println("Initializing motors...");
  initialize_motor(&base_motor, BASE_CONFIG, BASE_CALIBRATION, BASE_GEAR_RATIO);
  initialize_motor(&joint1_motor, JOINT_CONFIG_1, JOINT_CALIBRATION, ARM_GEAR_RATIO);
  initialize_motor(&joint2_motor, JOINT_CONFIG_2, JOINT_CALIBRATION, ARM_GEAR_RATIO);

  attachInterrupt(base_motor.encoder->getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(joint1_motor.encoder->getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(joint2_motor.encoder->getIntNum(), isr_process_encoder3, RISING);

  init_GPIO();
  system_calibration();
  resetToDefault();
}

void loop() {
  if (buttonCooldown && (millis() - cooldownStartTime >= BUTTON_COOLDOWN)) {
    buttonCooldown = false;
  }
  
  switch (currentState) {
    case SETUP:
      handleSetupState();
      if (state_vars.done) {
        currentState = PLACE_CUP;
      }
      break;

    case PLACE_CUP:
      if (ultra_sensor.distanceCm() < 5.00) {
        currentState = GET_TEABAG;
      } else {
        updateOLEDPlaceCup();
      }
      break;

    case GET_TEABAG:
      updateDisplays();
      NeutralToTeaBag();
      safe_delay(2000);
      TeaBagInCup();
      state_vars.in_cup = true;
      state_vars.done = false;
      while (!all_motors_reached()) {
        update_all_motors();
      }
      currentState = COUNTING_DOWN;
      break;

    case COUNTING_DOWN:
      while (!state_vars.done && !state_vars.steeped) {
        handleCountdownState();
        update_all_motors();
        if (all_motors_reached()) {
          if (state_vars.in_cup) {
            state_vars.in_cup = false;
            TeaBagOutCup();
          } else {
            TeaBagInCup();
            state_vars.in_cup = true;
          }
        }
        if (state_vars.steeped)
          currentState = TEA_IN_GARBAGE;
      }
      break;

    case TEA_IN_GARBAGE:
      cupToTrash();
      state_vars.done = false;
      currentState = FINISHED;
      break;

    case FINISHED:
      TrashToNeutral();
      if (!state_vars.done) {
        handleFinishedState();
      } 
      if (!all_motors_reached()) {
        update_all_motors();
      } 
      if (state_vars.done && all_motors_reached()) {
        resetToDefault();
        state_vars.done = false;
        currentState = SETUP;
      }
      break;
  }
}

// =============================================
// Initialization Functions
// =============================================

/**
 * @brief Initialize a motor controller
 * @param motor Pointer to motor control structure
 * @param config Hardware configuration parameters
 * @param cal Calibration parameters
 * @details Creates encoder instance with proper slot and configures PID
 */
void initialize_motor(encoder_motor* motor, const motor_config& config, const calibration_params& cal, float joint_ratio) {
  // Create encoder instance with correct slot
  motor->encoder = new MeEncoderOnBoard(config.slot);

  // Store configuration - copy the string properly
  motor->config = config;
  motor->config.name = String(config.name);
  motor->cal = cal;

  // Configure encoder parameters
  motor->encoder->setPulse(config.encoder_ppr);
  motor->encoder->setRatio(config.gear_ratio);
  motor->joint_gear_ratio = joint_ratio;

  // Set PID parameters
  motor->encoder->setPosPid(config.position_pid_p,
                            config.position_pid_i,
                            config.position_pid_d);
  motor->encoder->setSpeedPid(config.speed_pid_p,
                              config.speed_pid_i,
                              config.speed_pid_d);

  // Initialize position tracking
  motor->current_pos = 0.0;
  motor->is_calibrated = false;
  motor->arrived = false;
}

// =============================================
// Motion Control with Callbacks
// =============================================

/**
 * @brief Callback handler for completed movements
 * @param motor_id ID of the motor that completed its move
 * @details Called automatically when a motor reaches its target position
 *          1. Stops the motor completely
 *          2. Prints status update to serial
 */
void on_move_complete(int16_t motor_id) {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    if (all_motors[i]->config.slot == motor_id) {
      all_motors[i]->encoder->runSpeed(0);  // Stop actively holding position
      if (DEBUG) {
        Serial.print(all_motors[i]->config.name);
        Serial.println(" reached target position");
      }
      all_motors[i]->arrived = true;
      return;
    }
  }
  if (DEBUG) Serial.println("Unknown motor completed movement - THIS SHOULD NEVER HAPPEN!!! PANIK11");
}

/**
 * @brief Move motor to specified position with callback
 * @param motor Motor to move
 * @param target_deg Target position in degrees
 * @param speed_rpm Movement speed in RPM
 */
void move_motor(encoder_motor* motor, int target_deg, float speed_rpm) {
  // if (!motor->is_calibrated) {
  //   Serial.println("Motor not calibrated!");
  //   return;
  // }

  motor->arrived = false;

  // Constrain target to safe limits
  target_deg = constrain(target_deg, motor->min_angle, motor->max_angle);

  // Use slot number as unique ID for callback
  motor->encoder->moveTo(
    target_deg,
    speed_rpm,
    motor->config.slot,
    [](int16_t id) {
      on_move_complete(id);
    });
}

/**
 * @brief Update all motor controllers
 * @details Must be called frequently to maintain smooth motion
 */
void update_all_motors() {
  // Update all motors and current positions
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    all_motors[i]->encoder->loop();
    all_motors[i]->current_pos = all_motors[i]->encoder->getCurPos();
  }
}

/**
 * @brief Return all motors to their neutral positions
 * @details Uses callback system to automatically stop motors at destination
 */
void home_all_motors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    move_motor(all_motors[i], all_motors[i]->neutral_pos, 50);
  }
}

/**
 * @brief Immediate stop of all motion
 * @details Bypasses normal stopping routine for emergency situations
 */
void emergency_stop() {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    all_motors[i]->encoder->runSpeed(0);
  }
  claw.motor.stop();
  Serial.println("EMERGENCY STOP ACTIVATED");
}

// =============================================
// Utility Functions
// =============================================

/**
 * @brief Non-blocking delay with motor updates
 * @param delay_ms Delay duration in milliseconds
 */
void safe_delay(unsigned long delay_time_ms) {
  long curr_time = millis();
  long prev_time = curr_time;

  while (curr_time - prev_time < delay_time_ms) {
    update_all_motors();
    curr_time = millis();
  }
}


/**
 * @brief Check if all motors have reached targets
 * @return True if all motors are at target positions
 */
bool all_motors_reached() {
  return (base_motor.arrived == true) && (joint1_motor.arrived == true) && (joint2_motor.arrived == true);
}

// =============================================
// Claw Control Functions
// =============================================

/**
 * @brief Open the claw mechanism
 * @details Runs motor in reverse for predetermined time
 */
void open_claw() {
  if (claw.state != CLAW_OPEN) {
    claw.motor.run(claw.open_speed);
    delay(claw.move_time);
    claw.motor.stop();
    claw.state = CLAW_OPEN;
  }
}

/**
 * @brief Close the claw mechanism
 * @details Runs motor forward for predetermined time
 */
void close_claw() {
  if (claw.state != CLAW_CLOSED) {
    claw.motor.run(-claw.close_speed);
    delay(claw.move_time);
    claw.motor.stop();
    claw.state = CLAW_CLOSED;
  }
}

// =============================================
// Calibration Implementation
// =============================================

/**
 * @brief Reset stall detection state for a new calibration phase
 * @param motor Pointer to motor control structure
 */
void reset_stall_state(encoder_motor* motor) {
  motor->cal_state.prev_pos = motor->encoder->getCurPos();
  motor->cal_state.last_check = millis();
  motor->cal_state.start_time = millis();
  motor->cal_state.was_stalled = false;
}

/**
 * @brief Enhanced stall detection logic
 * @param motor Pointer to motor control structure
 * @return True if stall detected, false otherwise
 * @details Detects both active stalling (motor trying to move) and hard stops
 */
bool check_stall(encoder_motor* motor) {
  // Only check at configured intervals
  if (millis() - motor->cal_state.last_check < motor->cal.check_interval) {
    return false;
  }

  float current_pos = motor->encoder->getCurPos();
  float current_speed = motor->encoder->getCurrentSpeed();

  // Calculate actual movement
  float position_delta = abs(current_pos - motor->cal_state.prev_pos);

  // Detect two types of stalls:
  // 1. "Active stall" - motor is trying to move (speed > 0) but not moving
  // 2. "Hard stop" - motor has completely stopped (speed == 0)
  bool is_stalled = (position_delta < motor->cal.stall_threshold) && ((abs(current_speed) > 1.0) ||  // Active stall
                                                                      (abs(current_speed) == 0.0));  // Hard stop

  // Update state tracking
  motor->cal_state.prev_pos = current_pos;
  motor->cal_state.last_check = millis();

  if (is_stalled) {
    if (!motor->cal_state.was_stalled) {
      Serial.print(motor->config.name);
      Serial.println(" - Initial stall detected!");
      motor->cal_state.was_stalled = true;
      return true;
    }
    // Require consecutive stall detections
    Serial.print(motor->config.name);
    Serial.println(" - Confirmed stall!");
    return true;
  }

  // Reset stall state if movement detected
  motor->cal_state.was_stalled = false;
  return false;
}

// =============================================
// Coordinated Calibration System
// =============================================

/**
 * @brief Move joint to safe position using relative positioning
 * @param motor Pointer to motor control structure
 * @param dir Direction multiplier (1 or -1)
 * @param speed RPM for movement
 * @param duration_ms Movement duration
 */
void safe_relative_move(encoder_motor* motor, int dir, float speed, unsigned long duration_ms) {
  Serial.print("Moving ");
  Serial.print(motor->config.name);
  Serial.println(" to safe position");

  motor->encoder->runSpeed(dir * speed);

  unsigned long start = millis();
  while (millis() - start < duration_ms) {
    motor->encoder->loop();
  }
  motor->encoder->runSpeed(0);
}

/**
 * @brief Perform full calibration routine for a motor
 * @param motor Pointer to motor control structure
 */
void calibrate_motor(encoder_motor* motor) {
  if (motor == nullptr) return;

  Serial.print("Starting calibration for ");
  Serial.println(motor->config.name);

  // Phase 1: Find lower limit
  Serial.println("Finding lower limit...");
  motor->encoder->moveTo(-motor->cal.travel_limit, motor->cal.calibration_speed);
  reset_stall_state(motor);
  //   Serial.print("Free memory: ");
  // Serial.println(freeMemory());

  unsigned long phase_start = millis();
  while (millis() - phase_start < motor->cal.timeout) {
    motor->encoder->loop();

    // Skip stall checks during startup delay
    if (millis() - motor->cal_state.start_time > motor->cal.startup_delay) {
      if (check_stall(motor)) break;
    }
  }

  // Finalize lower limit
  motor->encoder->runSpeed(0);
  safe_delay(200);
  motor->encoder->setPulsePos(-JOINT_LIMIT_THRESHOLD);
  motor->min_angle = 0;
  Serial.print("Lower limit set: ");
  Serial.println(motor->min_angle);

  // Phase 2: Find upper limit
  if (motor->config.slot == SLOT2) {
    safe_relative_move(&joint2_motor, 1, 60.0f, 2000.0);  // Move elbow up
  }

  Serial.println("Finding upper limit...");
  motor->encoder->moveTo(motor->cal.travel_limit, motor->cal.calibration_speed);
  reset_stall_state(motor);

  phase_start = millis();
  while (millis() - phase_start < motor->cal.timeout) {
    motor->encoder->loop();

    if (millis() - motor->cal_state.start_time > motor->cal.startup_delay) {
      if (check_stall(motor)) break;
    }
  }

  // Finalize upper limit
  motor->encoder->runSpeed(0);
  safe_delay(200);
  motor->max_angle = motor->encoder->getCurPos() - JOINT_LIMIT_THRESHOLD;
  Serial.print("Upper limit set: ");
  Serial.println(motor->max_angle);

  motor->is_calibrated = true;
  Serial.print(motor->config.name);
  Serial.print(" calibration complete with range of [");
  Serial.print(motor->min_angle);
  Serial.print(", ");
  Serial.print(motor->max_angle);
  Serial.println("]");
  safe_delay(200);

  // Calculate and move to neutral position
  motor->neutral_pos = (motor->min_angle + motor->max_angle) / 2.0f;
  Serial.print("Moving to neutral: ");
  Serial.println(motor->neutral_pos);

  move_motor(motor, motor->neutral_pos, motor->cal.calibration_speed);
  while (!motor->arrived) {
    motor->encoder->loop();
  }
}


/**
 * @brief Calibrate the base motor using the line follower sensor
 * @param motor Pointer to motor control structure
 */
void calibrate_base_motor_with_line_follower(encoder_motor* motor) {
  if (motor == nullptr) return;

  Serial.println("Starting base motor calibration using line follower");

  // Move base motor to find the first black line (lower limit)
  Serial.println("Finding first black line...");
  motor->encoder->runSpeed(-motor->cal.calibration_speed);
  safe_delay(1000);
  while (true) {
    motor->encoder->loop();
    int sensorState = lineFinder.readSensors();
    if (sensorState == S1_OUT_S2_OUT) {
      motor->encoder->runSpeed(0);
      safe_delay(200);
      motor->encoder->setPulsePos(0);
      motor->min_angle = 0;
      Serial.println("First black line detected");
      break;
    }
  }

  // Add a delay to ensure the motor moves past the first black line
  Serial.println("Waiting before detecting second black line...");
  motor->encoder->runSpeed(motor->cal.calibration_speed);
  safe_delay(1000);  // Wait for 2 seconds


  // Move base motor to find the second black line (upper limit)
  Serial.println("Finding second black line...");

  while (true) {
    motor->encoder->loop();
    int sensorState = lineFinder.readSensors();
    if (sensorState == S1_OUT_S2_OUT) {
      motor->encoder->runSpeed(0);
      safe_delay(200);
      motor->max_angle = motor->encoder->getCurPos();
      Serial.println("Second black line detected");
      break;
    }
  }

  // Set the neutral position to be half of the calibrated distance
  motor->max_angle = motor->max_angle + (90 * BASE_GEAR_RATIO);  // add 90 joint degrees
  motor->neutral_pos = motor->max_angle / 2.0;
  Serial.print("Neutral position set to: ");
  Serial.println(motor->neutral_pos);

  motor->is_calibrated = true;
  Serial.println("Base motor calibration complete");
}


/**
 * @brief Full system calibration sequence
 * @details Coordinates joint movements to avoid physical interference
 */
void system_calibration() {
  set_gpio_to_cal();
  // 1. Calibrate base motor using line follower
  calibrate_base_motor_with_line_follower(&base_motor);

  // 2. Prepare elbow for shoulder calibration
  Serial.println("Preparing elbow for shoulder calibration");
  safe_relative_move(&joint2_motor, -1, 60.0f, 2000.0);  // Move elbow up

  // 3. Calibrate shoulder
  calibrate_motor(&joint1_motor);

  // 4. Calibrate elbow
  calibrate_motor(&joint2_motor);

  // 5. Final verification
  Serial.println("Moving system to neutral positions...");
  move_motor(&joint1_motor, joint1_motor.neutral_pos, joint1_motor.cal.calibration_speed * 2);
  move_motor(&joint2_motor, joint2_motor.neutral_pos, joint2_motor.cal.calibration_speed * 2);
  move_motor(&base_motor, base_motor.neutral_pos, base_motor.cal.calibration_speed * 2);
  while (!all_motors_reached()) {  //TODO: Change this back once cal routine is verified.
                                   // while (!joint1_motor.arrived && joint2_motor.arrived) {
    update_all_motors();
  }

  // Report calibrated workspace
  Serial.println("\n----- Workspace Analysis -----");

  // Convert to joint angles for reporting
  float base_min = motor_to_joint_angle(base_motor.min_angle, BASE_GEAR_RATIO);
  float base_max = motor_to_joint_angle(base_motor.max_angle, BASE_GEAR_RATIO);
  float shoulder_min = motor_to_joint_angle(joint1_motor.min_angle, ARM_GEAR_RATIO);
  float shoulder_max = motor_to_joint_angle(joint1_motor.max_angle, ARM_GEAR_RATIO);
  float elbow_min = motor_to_joint_angle(joint2_motor.min_angle, ARM_GEAR_RATIO);
  float elbow_max = motor_to_joint_angle(joint2_motor.max_angle, ARM_GEAR_RATIO);

  Serial.println("Joint Ranges (degrees):");
  Serial.print("  Base: ");
  Serial.print(base_min);
  Serial.print(" to ");
  Serial.println(base_max);

  Serial.print("  Shoulder: ");
  Serial.print(shoulder_min);
  Serial.print(" to ");
  Serial.println(shoulder_max);

  Serial.print("  Elbow: ");
  Serial.print(elbow_min);
  Serial.print(" to ");
  Serial.println(elbow_max);

  // Report neutral position
  // float x, y, z;
  float base_neutral = motor_to_joint_angle(base_motor.neutral_pos, BASE_GEAR_RATIO);
  float shoulder_neutral = motor_to_joint_angle(joint1_motor.neutral_pos, ARM_GEAR_RATIO);
  float elbow_neutral = motor_to_joint_angle(joint2_motor.neutral_pos, ARM_GEAR_RATIO);

  // Initialize claw
  open_claw();

  Serial.println("System calibration complete!");
  // Serial.println("TeaMate robot ready - Commands:");
  // Serial.println("p [1-3|claw] [angle|action] [speed] - Joint position control");
  // Serial.println("c [x] [y] [z] [speed] - Cartesian position control (mm)");
  // Serial.println("t [x] [y] [z] [speed] - Path planning with obstacle avoidance");
  // Serial.println("o [type] [x] [y] [z] [dim1] [dim2] [dim3] - Define obstacle");
  // Serial.println("cl - Clear all obstacles");
  // Serial.println("s [1-3|claw|xyz] - State query");
  // Serial.println("h - Home all motors");
  // Serial.println("e - Emergency stop");
}

// =============================================
// Forward and Inverse Kinematics Functions
// =============================================

/**
 * @brief Convert joint angle (after gear reduction) to motor angle
 * @param joint_angle_deg Angle in joint degrees
 * @param gear_ratio The gear ratio to apply
 * @return Angle in motor degrees
 */
float joint_to_motor_angle(float joint_angle_deg, float gear_ratio) {
  return joint_angle_deg * gear_ratio;
}

/**
 * @brief Convert motor angle to joint angle (after gear reduction)
 * @param motor_angle_deg Angle in motor degrees
 * @param gear_ratio The gear ratio to apply
 * @return Angle in joint degrees
 */
float motor_to_joint_angle(float motor_angle_deg, float gear_ratio) {
  return motor_angle_deg / gear_ratio;
}

/**
 * @brief Modified move_motor that takes joint angles instead of motor angles
 * @param motor Motor to move
 * @param joint_angle_deg Target joint angle in degrees
 * @param speed_rpm Movement speed in RPM
 */
void move_motor_joint_angle(encoder_motor* motor, float joint_angle_deg, float speed_rpm) {
  // if (!motor->is_calibrated) {
  //   Serial.println("Motor not calibrated!");
  //   return;
  // }

  motor->arrived = false;

  // Determine the gear ratio to use based on which motor we're controlling
  float gear_ratio = (motor == &base_motor) ? BASE_GEAR_RATIO : ARM_GEAR_RATIO;

  // Convert joint angle to motor angle
  float motor_angle_deg = joint_to_motor_angle(joint_angle_deg, gear_ratio);

  // Constrain motor angle to safe limits
  motor_angle_deg = constrain(motor_angle_deg, motor->min_angle, motor->max_angle);

  // Use slot number as unique ID for callback
  motor->encoder->moveTo(
    motor_angle_deg,
    speed_rpm,
    motor->config.slot,
    [](int16_t id) {
      on_move_complete(id);
    });
}


// =============================================
// Serial Command Handling
// =============================================

/**
 * @brief Process incoming serial commands
 * @param input Raw command string from serial port
 * @details Parses command string and dispatches to appropriate handlers.
 *          Supported command formats:
 *          - Position: "p [1-3|claw] [angle|action] [speed]"
 *          - Cartesian: "c [x] [y] [z] [speed]"
 *          - Path: "t [x] [y] [z] [speed]"
 *          - Obstacle: "o [type] [x] [y] [z] [dim1] [dim2] [dim3]"
 *          - Clear obstacles: "cl"
 *          - State: "s [1-3|claw|xyz]"
 *          - Home: "h"
 *          - Emergency: "e"
 */
void process_command(String input) {
  input.trim();
  input.toLowerCase();

  // Find command component boundaries
  int space1 = input.indexOf(' ');
  if (space1 == -1) {
    // Single-word commands
    if (input == CMD_HOME) {
      home_all_motors();
      return;
    } else if (input == CMD_EMERGENCY) {
      emergency_stop();
      return;
    }
    else {
      Serial.println("Invalid command. Type 'help' for available commands.");
      return;
    }
  }

  // Extract command and parameters
  String command = input.substring(0, space1);
  String params = input.substring(space1 + 1);

  // Find parameter boundaries
  int space2 = params.indexOf(' ');
  int space3 = space2 != -1 ? params.indexOf(' ', space2 + 1) : -1;
  int space4 = space3 != -1 ? params.indexOf(' ', space3 + 1) : -1;
  int space5 = space4 != -1 ? params.indexOf(' ', space4 + 1) : -1;
  int space6 = space5 != -1 ? params.indexOf(' ', space5 + 1) : -1;
  int space7 = space6 != -1 ? params.indexOf(' ', space6 + 1) : -1;

  // Extract parameters
  String param1 = space2 != -1 ? params.substring(0, space2) : params;
  String param2 = space2 != -1 ? (space3 != -1 ? params.substring(space2 + 1, space3) : params.substring(space2 + 1)) : "";
  String param3 = space3 != -1 ? (space4 != -1 ? params.substring(space3 + 1, space4) : params.substring(space3 + 1)) : "";
  String param4 = space4 != -1 ? (space5 != -1 ? params.substring(space4 + 1, space5) : params.substring(space4 + 1)) : "";
  String param5 = space5 != -1 ? (space6 != -1 ? params.substring(space5 + 1, space6) : params.substring(space5 + 1)) : "";
  String param6 = space6 != -1 ? (space7 != -1 ? params.substring(space6 + 1, space7) : params.substring(space6 + 1)) : "";
  String param7 = space7 != -1 ? params.substring(space7 + 1) : "";

  // Command routing
  if (command == CMD_POSITION) {
    handle_position_command(param1, param2, param3);
  }
  // else if (command == CMD_CARTESIAN) {
  //   handle_cartesian_command(param1, param2, param3, param4);
  // } else if (command == CMD_PATH) {
  //   handle_path_command(param1, param2, param3, param4);
  // } else if (command == CMD_OBSTACLE) {
  //   handle_obstacle_command(param1, param2, param3, param4, param5, param6, param7);
  // }
  else if (command == CMD_STATE) {
    handle_state_command(param1);
  } else if (command == "help") {
    // Print help information
    Serial.println("TeaMate robot commands:");
    Serial.println("p [1-3|claw] [angle|action] [speed] - Joint position control");
    Serial.println("c [x] [y] [z] [speed] - Cartesian position control (mm)");
    Serial.println("t [x] [y] [z] [speed] - Path planning with obstacle avoidance");
    Serial.println("o [type(1-3)] [x] [y] [z] [dim1] [dim2] [dim3] - Define obstacle");
    Serial.println("cl - Clear all obstacles");
    Serial.println("s [1-3|claw|xyz] - State query");
    Serial.println("h - Home all motors");
    Serial.println("e - Emergency stop");
  } else {
    Serial.println("Invalid command. Type 'help' for available commands.");
  }
}

/**
 * @brief Handle position control commands
 * @param motor_spec Motor identifier ("1"-"3" or "claw")
 * @param pos_spec Target position/action ("open"/"close" for claw)
 * @param speed_spec Movement speed (RPM for joints, ignored for claw)
 */
void handle_position_command(String motor_spec, String pos_spec, String speed_spec) {
  if (motor_spec == "claw") {
    handle_claw_command(pos_spec);
    return;
  }

  int motor_num = motor_spec.toInt();
  if (motor_num < 1 || motor_num > 3) {
    Serial.println("Invalid motor (1-3)");
    return;
  }

  float target = pos_spec.toFloat();
  float speed = constrain(speed_spec.toFloat(), 0, 100);

  // Stop trajectory if active
  // if (trajectory_active) {
  //   trajectory_active = false;
  //   Serial.println("Stopping active trajectory for direct motor control");
  // }

  switch (motor_num) {
    case 1:
      move_motor_joint_angle(&base_motor, target, speed);
      Serial.print("Moving base motor to ");
      Serial.print(target);
      Serial.print(" degrees at ");
      Serial.print(speed);
      Serial.println(" RPM");
      break;
    case 2:
      move_motor_joint_angle(&joint1_motor, target, speed);
      Serial.print("Moving shoulder joint to ");
      Serial.print(target);
      Serial.print(" degrees at ");
      Serial.print(speed);
      Serial.println(" RPM");
      break;
    case 3:
      move_motor_joint_angle(&joint2_motor, target, speed);
      Serial.print("Moving elbow joint to ");
      Serial.print(target);
      Serial.print(" degrees at ");
      Serial.print(speed);
      Serial.println(" RPM");
      break;
  }
}

/**
 * @brief Handle claw open/close commands
 * @param action_spec Requested action ("open" or "close")
 */
void handle_claw_command(String action_spec) {
  if (action_spec == "open") {
    Serial.println("Opening claw");
    open_claw();
  } else if (action_spec == "close") {
    Serial.println("Closing claw");
    close_claw();
  } else {
    Serial.println("Invalid claw action. Use 'open' or 'close'");
  }
}

/**
 * @brief Handle system state queries
 * @param motor_spec Motor identifier ("1"-"3" or "claw" or "xyz")
 */
void handle_state_command(String motor_spec) {
  if (motor_spec == "claw") {
    Serial.print("Claw: ");
    Serial.println(claw.state == CLAW_OPEN ? "Open" : "Closed");
    return;
  }

  // When reporting individual motor state:
  int motor_num = motor_spec.toInt();
  if (motor_num < 1 || motor_num > 3) {
    Serial.println("Invalid motor (1-3). Use '1' for base, '2' for shoulder, '3' for elbow, 'claw', 'xyz', or 'all'");
    return;
  }

  encoder_motor* m = nullptr;
  switch (motor_num) {
    case 1: m = &base_motor; break;
    case 2: m = &joint1_motor; break;
    case 3: m = &joint2_motor; break;
  }

  // Get the appropriate gear ratio
  float gear_ratio = (m == &base_motor) ? BASE_GEAR_RATIO : ARM_GEAR_RATIO;

  // Calculate joint angle ranges
  float joint_min_angle = motor_to_joint_angle(m->min_angle, gear_ratio);
  float joint_max_angle = motor_to_joint_angle(m->max_angle, gear_ratio);
  float joint_current_pos = motor_to_joint_angle(m->current_pos, gear_ratio);

  Serial.print(m->config.name);
  Serial.print(" Position: ");
  Serial.print(joint_current_pos);
  Serial.print("° (range: ");
  Serial.print(joint_min_angle);
  Serial.print("° to ");
  Serial.print(joint_max_angle);
  Serial.print("°), Moving: ");
  Serial.println(m->arrived == false ? "Yes" : "No");
}

// =============================================
// Movement and Control Sequences
// =============================================

/**
 * @brief 
 */
void NeutralToTeaBag() {
  // Start by opening the claw
  open_claw();
  // Make sure the arm is in neutral
  move_motor(&joint1_motor, joint1_motor.neutral_pos, joint1_motor.cal.calibration_speed * 2);
  move_motor(&joint2_motor, joint2_motor.neutral_pos, joint2_motor.cal.calibration_speed * 2);
  move_motor(&base_motor, base_motor.neutral_pos, base_motor.cal.calibration_speed * 2);
  while (!all_motors_reached()) {
    update_all_motors();
  }


  // Using the MovementCommand Neutral to teabag
  move_motor_joint_angle(&joint1_motor, NeutralToTeagBagCommand[0].shoulder, NeutralToTeagBagCommand[0].speed);
  move_motor_joint_angle(&joint2_motor, NeutralToTeagBagCommand[0].elbow, NeutralToTeagBagCommand[0].speed);
  move_motor_joint_angle(&base_motor, NeutralToTeagBagCommand[0].base, NeutralToTeagBagCommand[0].speed);
  while (!all_motors_reached()) {
    update_all_motors();
  }
  // small delay
  safe_delay(200);

  // close the claw
  close_claw();

   // Command 2
   move_motor_joint_angle(&joint1_motor, NeutralToTeagBagCommand[1].shoulder, NeutralToTeagBagCommand[1].speed);
   move_motor_joint_angle(&joint2_motor, NeutralToTeagBagCommand[1].elbow, NeutralToTeagBagCommand[1].speed);
   move_motor_joint_angle(&base_motor, NeutralToTeagBagCommand[1].base, NeutralToTeagBagCommand[1].speed);
   while (!all_motors_reached()) {
     update_all_motors();
   }
   safe_delay(200);

  // Command 3
  move_motor_joint_angle(&joint1_motor, NeutralToTeagBagCommand[2].shoulder, NeutralToTeagBagCommand[2].speed);
  move_motor_joint_angle(&joint2_motor, NeutralToTeagBagCommand[2].elbow, NeutralToTeagBagCommand[2].speed);
  move_motor_joint_angle(&base_motor, NeutralToTeagBagCommand[2].base, NeutralToTeagBagCommand[2].speed);
  while (!all_motors_reached()) {
    update_all_motors();
  }
  safe_delay(200);

  // Command 4
  move_motor_joint_angle(&joint1_motor, NeutralToTeagBagCommand[3].shoulder, NeutralToTeagBagCommand[3].speed);
  move_motor_joint_angle(&joint2_motor, NeutralToTeagBagCommand[3].elbow, NeutralToTeagBagCommand[3].speed);
  move_motor_joint_angle(&base_motor, NeutralToTeagBagCommand[3].base, NeutralToTeagBagCommand[3].speed);
  while (!all_motors_reached()) {
    update_all_motors();
  }
  safe_delay(200);

  // Final Command
  move_motor_joint_angle(&joint1_motor, NeutralToTeagBagCommand[4].shoulder, NeutralToTeagBagCommand[4].speed);
  move_motor_joint_angle(&joint2_motor, NeutralToTeagBagCommand[4].elbow, NeutralToTeagBagCommand[4].speed);
  move_motor_joint_angle(&base_motor, NeutralToTeagBagCommand[4].base, NeutralToTeagBagCommand[4].speed);
  while (!all_motors_reached()) {
    update_all_motors();
  }
  safe_delay(200);
}

void TeaBagInCup() {
  // We assume we are where we are after the Neutral to Cup fucntion.
  // I think we will do a steeping action here? for now we can just do an in and an out, and repeat this as many times as we need.

  move_motor_joint_angle(&joint1_motor, TeaBagToCupCommand[0].shoulder, TeaBagToCupCommand[0].speed);
  move_motor_joint_angle(&joint2_motor, TeaBagToCupCommand[0].elbow, TeaBagToCupCommand[0].speed);
  move_motor_joint_angle(&base_motor, TeaBagToCupCommand[0].base, TeaBagToCupCommand[0].speed);
  safe_delay(200);
}

void TeaBagOutCup() {
  move_motor_joint_angle(&joint1_motor, TeaBagToCupCommand[1].shoulder, TeaBagToCupCommand[1].speed);
  move_motor_joint_angle(&joint2_motor, TeaBagToCupCommand[1].elbow, TeaBagToCupCommand[1].speed);
  move_motor_joint_angle(&base_motor, TeaBagToCupCommand[1].base, TeaBagToCupCommand[1].speed);
  // while (!all_motors_reached()) {
  //   update_all_motors();
  // }

  // small delay
  safe_delay(200);
}

void cupToTrash() {
  // command 1, lift out of cup
  move_motor_joint_angle(&joint1_motor, CupToTrashCommand[0].shoulder, CupToTrashCommand[0].speed);
  move_motor_joint_angle(&joint2_motor, CupToTrashCommand[0].elbow, CupToTrashCommand[0].speed);
  while (!all_motors_reached()) {
    update_all_motors();
  }
  safe_delay(2000); // let it drip

  // command 2, rotate to garbage and deposit teabag
  move_motor_joint_angle(&base_motor, CupToTrashCommand[1].base, CupToTrashCommand[1].speed);
  move_motor_joint_angle(&joint1_motor, CupToTrashCommand[1].shoulder, CupToTrashCommand[1].speed);
  move_motor_joint_angle(&joint2_motor, CupToTrashCommand[1].elbow, CupToTrashCommand[1].speed);
  while (!all_motors_reached()) {
    update_all_motors();
  }

  move_motor_joint_angle(&base_motor, CupToTrashCommand[2].base, CupToTrashCommand[1].speed);
  move_motor_joint_angle(&joint1_motor, CupToTrashCommand[2].shoulder, CupToTrashCommand[1].speed);
  move_motor_joint_angle(&joint2_motor, CupToTrashCommand[2].elbow, CupToTrashCommand[1].speed);
  while (!all_motors_reached()) {
    update_all_motors();
  }

  // now open the claw
  open_claw();
  safe_delay(200);
}

void TrashToNeutral() {
  move_motor(&joint1_motor, joint1_motor.neutral_pos, joint1_motor.cal.calibration_speed * 2);
  move_motor(&joint2_motor, joint2_motor.neutral_pos, joint2_motor.cal.calibration_speed * 2);
  move_motor(&base_motor, base_motor.neutral_pos, base_motor.cal.calibration_speed * 2);
}

/**
 * @brief Setup function to initialize hardware and state
 * @details Configures displays, sets pin modes, and initializes the system
 */
void init_GPIO() {
  // Configure displays
  // timer_display.setBrightness(6);  // Set moderate brightness (0-7)
  oled_display.begin();
  oled_display.setFont(u8g2_font_6x10_tr);  // Use smaller font to save memory

  // Set up pins
  pinMode(ROTARY_CLK, INPUT);
  pinMode(ROTARY_DT, INPUT);
  pinMode(ROTARY_SW, INPUT_PULLUP);  // Use internal pull-up resistor for switch
  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize state
  // resetToDefault();
  lastStateCLK = digitalRead(ROTARY_CLK);  // Initialize last state for edge detection
}

/**
 * @brief Handles the SETUP state
 * @details Manages rotary encoder input for time adjustment and button press to start countdown
 */
void handleSetupState() {

  // Read rotary encoder for time adjustment
  currentStateCLK = digitalRead(ROTARY_CLK);
  // bool finished = false;

  // Detect rotation using edge detection
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
    // CLK pin transitioned from LOW to HIGH (rising edge)
    if (digitalRead(ROTARY_DT) != currentStateCLK) {
      // Counter-clockwise rotation (DT and CLK out of phase)
      if (currentSeconds > 0) currentSeconds--;
    } else {
      // Clockwise rotation (DT and CLK in phase)
      if (currentSeconds < MAX_TIMER_VALUE) currentSeconds++;
    }
    // Update displays with new time value
    updateDisplays();
  }
  lastStateCLK = currentStateCLK;  // Save CLK state for next comparison

  // Check button press to start countdown
  // Only respond if:
  // 1. Not in cooldown period
  // 2. Button is pressed (LOW)
  // 3. Enough time has passed since last press (debounce)
  if (!buttonCooldown && digitalRead(ROTARY_SW) == LOW && millis() - lastButtonPress > DEBOUNCE_DELAY) {
    lastButtonPress = millis();
    state_vars.done = true;

    // Update LEDs to show counting state (yellow)
    digitalWrite(RED_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);

    // Initialize countdown timer
    lastCountdownUpdate = millis();
    updateDisplays();
    return;
  }

  delay(1);  // Small delay for stability
}

/**
 * @brief Handles the COUNTING_DOWN state
 * @details Decrements timer and checks for completion
 */
void handleCountdownState() {
  // Check if a second has passed since last update
  // bool done = false;
  if (millis() - lastCountdownUpdate >= COUNTDOWN_INTERVAL) {
    lastCountdownUpdate = millis();

    // Decrement time if not already at zero
    if (currentSeconds > 0) {
      currentSeconds--;
      updateDisplays();
    }

    // Check if countdown is complete
    if (currentSeconds <= 0) {
      // Transition to FINISHED state
      // currentState = FINISHED;
      state_vars.steeped = true;

      // Update LEDs to show finished state (green)
      digitalWrite(RED_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);

      updateDisplays();
      return;
    }
  }

  // Read encoder input but don't respond to it during countdown
  // This prevents false triggers when returning to SETUP state
  lastStateCLK = digitalRead(ROTARY_CLK);
}

/**
 * @brief Handles the FINISHED state
 * @details Manages buzzer alerts, display blinking, and reset functionality
 */
void handleFinishedState() {
  // Static variables preserve state between function calls
  static unsigned long lastBuzzerTime = 0;       // Time of last buzzer activation
  static unsigned long lastAnimationTime = 0;    // Time of last animation update
  static bool initialBuzzerDone = false;         // Flag for initial alert sound
  static bool buzzerActive = false;              // Flag indicating buzzer is currently on
  const unsigned long ANIMATION_INTERVAL = 300;  // Milliseconds between steam animation frames

  // Check for reset button press
  if (ultra_sensor.distanceCm() < 5.00 && millis() - lastButtonPress > DEBOUNCE_DELAY) {
    lastButtonPress = millis();
    hardwareNoTone(BUZZER_PIN);  // Make sure buzzer is off when resetting
    state_vars.done = true;
    // resetToDefault();

    // Set cooldown flag and start time to prevent immediate start
    buttonCooldown = true;
    cooldownStartTime = millis();

    // Reset the static variables for buzzer to ensure clean state
    lastBuzzerTime = 0;
    lastAnimationTime = 0;
    initialBuzzerDone = false;
    buzzerActive = false;

    return;
  }

  // Handle initial alert sound when first entering finished state
  if (!initialBuzzerDone) {
    playTeaDoneMelody();  // Use the new tea done melody
    lastBuzzerTime = millis();
    initialBuzzerDone = true;
    buzzerActive = true;
  }

  // Handle periodic buzzer alerts
  if (buzzerActive && millis() - lastBuzzerTime >= BUZZER_DURATION) {
    // Turn off buzzer after specified duration
    buzzerActive = false;
    lastBuzzerTime = millis();
  } else if (!buzzerActive && millis() - lastBuzzerTime >= BUZZER_REMINDER_INTERVAL) {
    // Turn on buzzer for reminder after interval has passed
    playTeaDoneMelody();  // Use the new tea done melody
    buzzerActive = true;
    lastBuzzerTime = millis();
  }

  // Update the teacup animation at regular intervals
  if (millis() - lastAnimationTime >= ANIMATION_INTERVAL) {
    lastAnimationTime = millis();
    updateOLEDFinished();  // No parameter needed since we're not using the inversion anymore
  }

  // Small delay for stability
  delay(1);
}

/**
 * @brief Resets the system to default state
 * @details Resets timer, LEDs, buzzer, and updates displays
 */
void resetToDefault() {
  // Reset timer value
  currentSeconds = DEFAULT_TIME;

  // Reset state
  currentState = SETUP;

  // Set LEDs to show setup state (red)
  digitalWrite(RED_LED, HIGH);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(GREEN_LED, LOW);

  // Turn off buzzer
  hardwareNoTone(BUZZER_PIN);  // Use the new function

  // Update displays with new state
  updateDisplays();
  state_vars.done = false;
  state_vars.steeped = false;
}

/**
 * @brief Updates both displays based on current state
 * @details Updates TM1637 with time and OLED with appropriate display
 */
void updateDisplays() {
  // Calculate minutes and seconds for display
  byte minutes = currentSeconds / 60;
  byte secs = currentSeconds % 60;

  // Update 4-digit display with MM:SS format
  // 0x40 turns on the colon between digits
  // timer_display.showNumberDecEx(minutes * 100 + secs, 0x40, true);

  // Update OLED display based on current state
  if (currentState == FINISHED) {
    updateOLEDFinished();  // Normal finished display
  } else {
    updateOLEDNormal();  // Standard time display
  }
}

/**
 * @brief Updates OLED with normal time display
 * @details Shows time and current state information
 */
void updateOLEDNormal() {
  oled_display.clearBuffer();  // Clear display buffer

  // Calculate time values
  byte minutes = currentSeconds / 60;
  byte secs = currentSeconds % 60;

  // Draw timer in large font for visibility
  oled_display.setFont(u8g2_font_logisoso16_tr);  // Use larger font for time
  char timeStr[6];
  sprintf(timeStr, "%02d:%02d", minutes, secs);  // Format as MM:SS with leading zeros

  // Center the time string on display
  byte strWidth = oled_display.getStrWidth(timeStr);
  oled_display.drawStr((128 - strWidth) / 2, 30, timeStr);

  // Draw different state text based on current state
  oled_display.setFont(u8g2_font_6x10_tr);
  const char* stateText;

  if (currentState == SETUP) {
    stateText = "Enter Steep Duration";
  } else if (currentState == COUNTING_DOWN) {
    stateText = "Steeping the Tea...";
  } else {
    stateText = "TeaMate";  // Default text
  }

  strWidth = oled_display.getStrWidth(stateText);
  oled_display.drawStr((128 - strWidth) / 2, 55, stateText);

  // Add cooldown indicator if in cooldown period
  if (buttonCooldown && currentState == SETUP) {
    const char* cooldownText = "WAIT...";
    strWidth = oled_display.getStrWidth(cooldownText);
    oled_display.drawStr((128 - strWidth) / 2, 12, cooldownText);
  }

  // Send the buffer to the display
  oled_display.sendBuffer();
}

/**
 * @brief Updates OLED with finished state display
 * @param inverted Whether to show inverted display for blinking effect
 * @details Shows either a Pepe frog face or inverted "TIMER DONE" message
 */
void updateOLEDFinished() {
  static byte steamPhase = 0;  // Use this to animate the steam

  oled_display.clearBuffer();  // Clear display buffer

  // Draw teacup body
  // Cup body
  oled_display.drawEllipse(64, 45, 20, 8);  // Bottom ellipse
  oled_display.drawLine(44, 45, 44, 30);    // Left side
  oled_display.drawLine(84, 45, 84, 30);    // Right side
  oled_display.drawEllipse(64, 30, 20, 5);  // Top ellipse

  // Cup handle
  oled_display.drawEllipse(88, 38, 6, 10);
  oled_display.drawLine(84, 33, 88, 28);  // Top handle connection
  oled_display.drawLine(84, 43, 88, 48);  // Bottom handle connection

  // Saucer
  oled_display.drawEllipse(64, 52, 30, 6);

  // Draw steam lines (animated)
  if (steamPhase == 0) {
    // Steam phase 1
    oled_display.drawLine(55, 25, 50, 15);
    oled_display.drawLine(64, 25, 64, 12);
    oled_display.drawLine(73, 25, 78, 15);
  } else if (steamPhase == 1) {
    // Steam phase 2
    oled_display.drawLine(55, 25, 53, 13);
    oled_display.drawLine(64, 25, 67, 10);
    oled_display.drawLine(73, 25, 75, 13);
  } else {
    // Steam phase 3
    oled_display.drawLine(55, 25, 57, 13);
    oled_display.drawLine(64, 25, 61, 10);
    oled_display.drawLine(73, 25, 71, 13);
  }

  // Increment steam phase for animation
  steamPhase = (steamPhase + 1) % 3;

  // Draw "Tea is served" text
  oled_display.setFont(u8g2_font_8x13B_tr);
  const char* text = "Tea is served";
  byte width = oled_display.getStrWidth(text);
  oled_display.drawStr((128 - width) / 2, 62, text);

  // Send the buffer to the display
  oled_display.sendBuffer();
}

void updateOLEDPlaceCup() {
  // Display a message prompting user to place cup
  oled_display.clearBuffer();

  // Draw a title
  oled_display.setFont(u8g2_font_8x13B_tr);
  const char* titleText = "TeaMate";
  byte titleWidth = oled_display.getStrWidth(titleText);
  oled_display.drawStr((128 - titleWidth) / 2, 15, titleText);

  // Draw the instruction
  oled_display.setFont(u8g2_font_6x10_tr);
  const char* instructText = "Please place your";
  byte instrWidth = oled_display.getStrWidth(instructText);
  oled_display.drawStr((128 - instrWidth) / 2, 35, instructText);

  const char* instrText2 = "cup on the tray";
  byte instr2Width = oled_display.getStrWidth(instrText2);
  oled_display.drawStr((128 - instr2Width) / 2, 48, instrText2);

  // Send the buffer to the display
  oled_display.sendBuffer();
}

void set_gpio_to_cal() {
  // Set LEDs to show cal state (yellow)
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);

  // Clear buffer first to remove any previous content
  oled_display.clearBuffer();

  // Draw calibration title
  oled_display.setFont(u8g2_font_8x13B_tr);
  const char* titleText = "TeaMate Setup";
  byte titleWidth = oled_display.getStrWidth(titleText);
  oled_display.drawStr((128 - titleWidth) / 2, 20, titleText);

  // Draw calibration message
  oled_display.setFont(u8g2_font_6x10_tr);
  const char* text = "CALIBRATING...";
  byte width = oled_display.getStrWidth(text);
  oled_display.drawStr((128 - width) / 2, 35, text);

  // Draw a progress line
  oled_display.drawLine(44, 45, 84, 45);

  // Draw a small moving indicator
  int pos = (millis() / 200) % 40;
  oled_display.drawBox(44 + pos, 43, 4, 4);

  // Send the buffer to the display
  oled_display.sendBuffer();
}
