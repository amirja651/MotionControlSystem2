/**
 * High Precision Motion Control for ESP32 with Stepper Motor and Encoder Feedback
 * Components:
 * - Nema 11 Stepper Motor (1.8deg, 200 steps/rev)
 * - Magnetic Encoder (1000PPR/4000CPR)
 * - Stepper Driver with microstepping
 * - ESP32 Controller
 * - Linear Stage with Ultra-Fine-Thread Ball-Point Set Screw
 *
 * Target precision: < 1 micrometer
 */

//==============================================================================
// INCLUDE LIBRARIES
//==============================================================================
#include <Arduino.h>
#include <driver/pcnt.h>     // ESP32 Pulse Counter
#include "esp_timer.h"       // For high-resolution timing
#include "soc/gpio_struct.h" // For GPIO register access
#include "driver/gpio.h"     // For GPIO functions

//==============================================================================
// PIN DEFINITIONS
//==============================================================================
// Motor and driver pins
#define STEP_PIN 32
#define DIR_PIN 33
#define ENABLE_PIN 14

// Encoder pins for position feedback
#define ENCODER_A_PIN 23
#define ENCODER_B_PIN 22
#define ENCODER_INDEX_PIN 21 // Optional index pin if available

// Limit switch pin (single switch at the end of travel)
#define LIMIT_SWITCH_PIN 13 // Using a lower priority pin

#define PIN_VOLTAGE A0
#define returnedVoltage 3.2
#define VOLTAGE_THRESHOLD 0.02
volatile float cutoffVoltage = 3.1 * 4095.0 / 3.3;
bool save_on_cutoffVoltage = false;

//==============================================================================
// SYSTEM PARAMETERS
//==============================================================================
#define MICROSTEPS 16                                         // Set according to driver SW1-SW4 settings
#define STEPS_PER_REV 200                                     // Motor steps per revolution
#define ENCODER_PPR 1000                                      // Pulses per revolution (4000 CPR with quadrature)
#define LEAD_SCREW_PITCH 0.5                                  // Lead screw pitch in mm (adjust based on actual screw)
#define PIXEL_SIZE 5.2                                        // Size of one pixel in micrometers
#define TOTAL_TRAVEL_MM 30                                    // Total travel distance in mm (from home to end)
#define TOTAL_TRAVEL_MICRONS (TOTAL_TRAVEL_MM * 1000)         // Total travel in microns
#define REL_TRAVEL_LIMIT_MM 3                                 // Relative travel limit in mm (±3mm from relative zero)
#define REL_TRAVEL_LIMIT_MICRONS (REL_TRAVEL_LIMIT_MM * 1000) // Relative limit in microns
#define STATUS_UPDATE_MS 400                                  // Status update interval in milliseconds

// Derived parameters
#define MOTOR_STEPS_PER_MICRON ((STEPS_PER_REV * MICROSTEPS) / (LEAD_SCREW_PITCH * 1000))
#define ENCODER_COUNTS_PER_MICRON ((ENCODER_PPR * 4) / (LEAD_SCREW_PITCH * 1000))

// Motion control parameters
#define MAX_SPEED 5000       // Maximum step frequency in Hz
#define ACCELERATION 10000   // Steps per second per second
#define PID_UPDATE_FREQ 1000 // PID update frequency in Hz

// PID Controller parameters (to be tuned)
#define KP 0.8            // Proportional gain
#define KI 0.1            // Integral gain
#define KD 0.05           // Derivative gain
#define MAX_INTEGRAL 1000 // Anti-windup limit

//==============================================================================
// GLOBAL VARIABLES
//==============================================================================
// Encoder quadrature pulse counter
pcnt_unit_t encoder_pcnt_unit = PCNT_UNIT_0;
volatile int32_t encoder_position = 0; // Current encoder position
int32_t target_position = 0;           // Target position
int32_t last_encoder_position = 0;     // Previous encoder position

// Position tracking variables
int32_t absolute_zero_position = 0; // Encoder position at absolute zero (HOME)
int32_t relative_zero_position = 0; // Encoder position at relative zero (RESET)

// PID variables
float integral = 0;
float last_error = 0;
int32_t output = 0;
uint64_t last_pid_time = 0;

// Task handles
TaskHandle_t pidTaskHandle = NULL;
TaskHandle_t motionTaskHandle = NULL;
TaskHandle_t statusTaskHandle = NULL; // Task for periodic status updates

// Motion profile variables
int32_t current_step_position = 0;
float current_speed = 0; // Speed in steps per second
uint64_t last_step_time = 0;

// System state flags
volatile bool limitSwitchTriggered = false;
volatile bool emergencyStop = false;

// Status update variables
uint32_t last_status_time = 0; // Last status update time

//==============================================================================
// FUNCTION PROTOTYPES
//==============================================================================
static void IRAM_ATTR encoderOverflowISR(void *arg);
static void IRAM_ATTR limitSwitchISR(void *arg);
void setupEncoder();
void setupLimitSwitch();
int32_t readEncoder();
int32_t micronsToSteps(float microns);
float pixelsToMicrons(float pixels);
int32_t pixelsToSteps(float pixels);
float encoderToMicrons(int32_t counts);
float encoderToPixels(int32_t counts);
void generateStep();
void setDirection(bool dir);
uint32_t calculateStepInterval(float speed);
void pidControlTask(void *parameter);
void motionControlTask(void *parameter);
void statusUpdateTask(void *parameter);
void moveToPosition(float position_microns, bool calibration);
void moveToPositionPixels(float position_pixels);
void moveRelative(float distance_microns);
void moveRelativePixels(float distance_pixels);
bool waitForMotionComplete(float tolerance_microns, uint32_t timeout_ms);
void calibrateSystem();
void resetRelativeZero();
float getAbsolutePosition();
float getRelativePosition();
void printStatusUpdate(bool showStatus = 0);

//==============================================================================
// INTERRUPT SERVICE ROUTINES (ISRs)
//==============================================================================
/**
 * ISR for pulse counter overflow/underflow
 * Extends the 16-bit counter to 32-bit
 */
static void IRAM_ATTR encoderOverflowISR(void *arg)
{
  uint32_t status = 0;
  pcnt_get_event_status(encoder_pcnt_unit, &status);

  if(status & PCNT_EVT_H_LIM)
    {
      encoder_position += 32767;
      pcnt_counter_clear(encoder_pcnt_unit);
    }
  else if(status & PCNT_EVT_L_LIM)
    {
      encoder_position -= 32768;
      pcnt_counter_clear(encoder_pcnt_unit);
    }
}

/**
 * ISR for limit switch interrupt
 */
static void IRAM_ATTR limitSwitchISR(void *arg)
{
  limitSwitchTriggered = true;
  emergencyStop = true;
}

//==============================================================================
// SETUP FUNCTIONS
//==============================================================================
/**
 * Setup encoder pulse counter for quadrature decoding
 * Using ESP32's hardware pulse counter for precise position tracking
 */
void setupEncoder()
{
  // Configure pulse counter unit
  pcnt_config_t pcnt_config = {
      .pulse_gpio_num = ENCODER_A_PIN, // Channel A
      .ctrl_gpio_num = ENCODER_B_PIN,  // Channel B
      .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting when B=1
      .hctrl_mode = PCNT_MODE_KEEP,    // Keep counting when B=0
      .pos_mode = PCNT_COUNT_INC,      // Count up on rising edge A
      .neg_mode = PCNT_COUNT_DEC,      // Count down on falling edge A
      .counter_h_lim = 32767,          // Max hardware limit
      .counter_l_lim = -32768,         // Min hardware limit
      .unit = encoder_pcnt_unit,
      .channel = PCNT_CHANNEL_0,
  };

  pcnt_unit_config(&pcnt_config);

  // Set up pulse counter events
  pcnt_event_enable(encoder_pcnt_unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(encoder_pcnt_unit, PCNT_EVT_L_LIM);

  // Filter out glitches (adjust these values based on encoder specs)
  pcnt_set_filter_value(encoder_pcnt_unit, 100);
  pcnt_filter_enable(encoder_pcnt_unit);

  // Start the counter
  pcnt_counter_pause(encoder_pcnt_unit);
  pcnt_counter_clear(encoder_pcnt_unit);
  pcnt_counter_resume(encoder_pcnt_unit);

  // Set up interrupt for overflow handling
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(encoder_pcnt_unit, encoderOverflowISR, NULL);
}

/**
 * Setup limit switch with interrupt
 */
void setupLimitSwitch()
{
  // Configure pin with internal pull-up resistor
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Attach interrupt for limit switch (trigger on falling edge - switch closed)
  attachInterrupt(
      digitalPinToInterrupt(LIMIT_SWITCH_PIN),
      []() {
        limitSwitchISR(NULL);
      },
      FALLING);

  Serial.println("Limit switch configured on pin " + String(LIMIT_SWITCH_PIN));
}

//==============================================================================
// UTILITY FUNCTIONS
//==============================================================================
/**
 * Read the current position from the encoder in quadrature counts
 */
int32_t readEncoder()
{
  int16_t count = 0;
  pcnt_get_counter_value(encoder_pcnt_unit, &count);
  return encoder_position + count;
}

/**
 * Convert microns to motor steps
 */
int32_t micronsToSteps(float microns) { return roundf(microns * MOTOR_STEPS_PER_MICRON); }

/**
 * Convert pixels to microns
 */
float pixelsToMicrons(float pixels) { return pixels * PIXEL_SIZE; }

/**
 * Convert pixels to motor steps
 */
int32_t pixelsToSteps(float pixels) { return micronsToSteps(pixelsToMicrons(pixels)); }

/**
 * Convert encoder counts to microns
 */
float encoderToMicrons(int32_t counts) { return (float)counts / ENCODER_COUNTS_PER_MICRON; }

/**
 * Convert encoder counts to pixels
 */
float encoderToPixels(int32_t counts) { return encoderToMicrons(counts) / PIXEL_SIZE; }

/**
 * Get absolute position in microns (relative to HOME/absolute zero)
 */
float getAbsolutePosition()
{
  int32_t current_position = readEncoder();
  return encoderToMicrons(current_position - absolute_zero_position);
}

/**
 * Get relative position in microns (relative to user-set zero)
 */
float getRelativePosition()
{
  int32_t current_position = readEncoder();
  return encoderToMicrons(current_position - relative_zero_position);
}

/**
 * Reset the relative zero position to current position
 */
void resetRelativeZero()
{
  relative_zero_position = readEncoder();
  Serial.println("Relative zero position reset at current position");
}

/**
 * Generate step pulse with precise timing
 * Uses digitalWrite for compatibility but can be optimized with direct register access
 */
void IRAM_ATTR generateStep()
{
  // Using standard digitalWrite for compatibility
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2); // Minimum pulse width (check driver specs)
  digitalWrite(STEP_PIN, LOW);
}

/**
 * Set motor direction (1 = forward, 0 = backward)
 */
void setDirection(bool dir)
{
  digitalWrite(DIR_PIN, dir);
  delayMicroseconds(5); // Direction setup time
}

/**
 * Calculate time for next step based on current speed
 */
uint32_t calculateStepInterval(float speed)
{
  if(abs(speed) < 1)
    return 0;                  // Prevent division by zero
  return 1000000 / abs(speed); // Convert Hz to microseconds
}

//==============================================================================
// Voltage Monitor Task
//==============================================================================
void voltageMonitorTask(void *parameter)
{
  for(;;)
    {
      int adcValue = analogRead(PIN_VOLTAGE);
      if(adcValue < cutoffVoltage)
        {
          // Serial.println("⚡ " + String((adcValue * 3.3 / 4095.0), 2) + "V, ADC: " + String(adcValue));
          if(!save_on_cutoffVoltage)
            {
              save_on_cutoffVoltage = true;
            }
        }
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

//==============================================================================
// MOTION CONTROL TASKS
//==============================================================================
/**
 * PID Task - Runs at high priority for closed-loop position control
 */
void pidControlTask(void *parameter)
{
  // Initialize timing
  last_pid_time = esp_timer_get_time();

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 1000 / PID_UPDATE_FREQ; // Convert to milliseconds

  while(true)
    {
      // Get current position
      int32_t current_position = readEncoder();

      // Calculate error (in encoder counts)
      float error = target_position - current_position;

      // Calculate time delta
      uint64_t now = esp_timer_get_time();
      float dt = (now - last_pid_time) / 1000000.0; // Convert to seconds
      last_pid_time = now;

      // Calculate PID terms
      float proportional = KP * error;
      integral += KI * error * dt;

      // Apply anti-windup
      if(integral > MAX_INTEGRAL)
        integral = MAX_INTEGRAL;
      if(integral < -MAX_INTEGRAL)
        integral = -MAX_INTEGRAL;

      float derivative = 0;
      if(dt > 0)
        {
          derivative = KD * (error - last_error) / dt;
        }

      // Calculate output
      output = proportional + integral + derivative;

      // Store values for next iteration
      last_error = error;
      last_encoder_position = current_position;

      // Use vTaskDelayUntil for precise timing
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * Motion control task - Handles step generation based on PID output
 */
void motionControlTask(void *parameter)
{
  // Initialize timings
  last_step_time = esp_timer_get_time();

  while(true)
    {
      // Check for emergency stop condition
      if(emergencyStop)
        {
          current_speed = 0;
          emergencyStop = false; // Reset flag
          Serial.println("EMERGENCY STOP: Limit switch triggered!");
          vTaskDelay(100); // Give time for other tasks
          continue;
        }

      // Calculate desired speed from PID output
      float desired_speed = constrain(output, -MAX_SPEED, MAX_SPEED);

      // Apply acceleration limits
      float max_speed_change = ACCELERATION / PID_UPDATE_FREQ;
      if(desired_speed > current_speed + max_speed_change)
        {
          current_speed += max_speed_change;
        }
      else if(desired_speed < current_speed - max_speed_change)
        {
          current_speed -= max_speed_change;
        }
      else
        {
          current_speed = desired_speed;
        }

      // Set direction based on speed sign
      bool direction = current_speed >= 0;

      // Safety check - don't move if limit switch is triggered
      if(limitSwitchTriggered)
        {
          // Only allow movement away from the limit switch
          // Since the switch is at the minimum position (opposite end),
          // only allow positive movement (away from the switch)
          if(!direction)
            {                    // Moving toward limit switch (negative direction)
              current_speed = 0; // Stop motion in this direction
              continue;
            }
        }

      setDirection(direction);

      // Calculate step interval
      uint32_t step_interval = calculateStepInterval(current_speed);

      // Generate step if needed and enough time has passed
      if(step_interval > 0)
        {
          uint64_t now = esp_timer_get_time();
          if(now - last_step_time >= step_interval)
            {
              generateStep();

              // Update position
              if(direction)
                {
                  current_step_position++;
                }
              else
                {
                  current_step_position--;
                }

              last_step_time = now;
            }
        }

      // Yield to other tasks
      vTaskDelay(1);
    }
}

/**
 * Status update task - Handles periodic status updates
 */
void statusUpdateTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = STATUS_UPDATE_MS; // Convert to milliseconds

  while(true)
    {
      float rel_position = getRelativePosition();
      float rel_target = encoderToMicrons(target_position - relative_zero_position);

      if(abs(rel_position - rel_target) > 0.2)
        printStatusUpdate();

      // Use vTaskDelayUntil for precise timing
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * Print a status update to the serial port
 */
void printStatusUpdate(bool showStatus)
{
  int32_t current_position = readEncoder();

  // Calculate positions
  float rel_position = getRelativePosition();
  float abs_position = getAbsolutePosition();
  float rel_target = encoderToMicrons(target_position - relative_zero_position);

  // Calculate error based on relative position
  float error = rel_target - rel_position;

  // Calculate motor frequency
  float motor_frequency = abs(current_speed); // Steps per second

  // Calculate relative position as percentage of relative travel limit
  float rel_travel_percent = (rel_position / REL_TRAVEL_LIMIT_MICRONS) * 100;

  if(motor_frequency > 10 || showStatus)
    {
      // Print status update
      Serial.printf("POS(rel): %.3f µm (%.1f%% of ±%d mm), TARGET: %.3f µm, ERROR: %.3f µm\n", rel_position, abs(rel_travel_percent),
                    REL_TRAVEL_LIMIT_MM, rel_target, error);

      Serial.printf("POS(abs): %.3f µm, Travel: %.1f%% of %.1f mm\n", abs_position, (abs_position / TOTAL_TRAVEL_MICRONS) * 100,
                    TOTAL_TRAVEL_MM);

      Serial.printf("Motor Freq: %.1f Hz, Speed: %.3f mm/s, Limit Switch: %s\n", motor_frequency,
                    (motor_frequency / (STEPS_PER_REV * MICROSTEPS)) * LEAD_SCREW_PITCH, limitSwitchTriggered ? "TRIGGERED" : "clear");

      Serial.println("-------------------------------");
      Serial.print("\n\n\n\n\n\n\n\n");
    }
}

//==============================================================================
// MOTION COMMANDS
//==============================================================================
/**
 * Move to absolute position in microns (relative to relative zero)
 */
void moveToPosition(float position_microns, bool calibration = 0)
{
  // Check if within relative travel limits
  if(!calibration && position_microns < -REL_TRAVEL_LIMIT_MICRONS || position_microns > REL_TRAVEL_LIMIT_MICRONS)
    {
      Serial.printf("ERROR: Position %.3f µm exceeds relative travel limits (±%d mm)\n", position_microns, REL_TRAVEL_LIMIT_MM);
      return;
    }

  // Convert microns to encoder counts and adjust for relative zero
  target_position = relative_zero_position + (position_microns * ENCODER_COUNTS_PER_MICRON);

  Serial.printf("Moving to: %.3f µm (relative) (Target encoder: %ld)\n", position_microns, target_position);
}

/**
 * Move to absolute position in pixels (relative to relative zero)
 */
void moveToPositionPixels(float position_pixels)
{
  float position_microns = pixelsToMicrons(position_pixels);

  // Check if within relative travel limits
  if(position_microns < -REL_TRAVEL_LIMIT_MICRONS || position_microns > REL_TRAVEL_LIMIT_MICRONS)
    {
      Serial.printf("ERROR: Position %.3f px (%.3f µm) exceeds relative travel limits (±%d mm)\n", position_pixels, position_microns,
                    REL_TRAVEL_LIMIT_MM);
      return;
    }

  target_position = relative_zero_position + (position_microns * ENCODER_COUNTS_PER_MICRON);

  Serial.printf("Moving to: %.3f px (%.3f µm relative) (Target encoder: %ld)\n", position_pixels, position_microns, target_position);
}

/**
 * Move relative to current position in microns
 */
void moveRelative(float distance_microns)
{
  // Check if move would exceed relative travel limits
  float current_rel_pos = getRelativePosition();
  float new_rel_pos = current_rel_pos + distance_microns;

  if(new_rel_pos < -REL_TRAVEL_LIMIT_MICRONS || new_rel_pos > REL_TRAVEL_LIMIT_MICRONS)
    {
      Serial.printf("ERROR: Relative move to %.3f µm exceeds relative travel limits (±%d mm)\n", new_rel_pos, REL_TRAVEL_LIMIT_MM);
      return;
    }

  int32_t current_position = readEncoder();
  target_position = current_position + (distance_microns * ENCODER_COUNTS_PER_MICRON);

  Serial.printf("Moving: %.3f µm (%.3f px) (Target encoder: %ld)\n", distance_microns, distance_microns / PIXEL_SIZE, target_position);
}

/**
 * Move relative to current position in pixels
 */
void moveRelativePixels(float distance_pixels)
{
  float distance_microns = pixelsToMicrons(distance_pixels);

  // Check if move would exceed relative travel limits
  float current_rel_pos = getRelativePosition();
  float new_rel_pos = current_rel_pos + distance_microns;

  if(new_rel_pos < -REL_TRAVEL_LIMIT_MICRONS || new_rel_pos > REL_TRAVEL_LIMIT_MICRONS)
    {
      Serial.printf("ERROR: Relative move to %.3f µm exceeds relative travel limits (±%d mm)\n", new_rel_pos, REL_TRAVEL_LIMIT_MM);
      return;
    }

  int32_t current_position = readEncoder();
  target_position = current_position + (distance_microns * ENCODER_COUNTS_PER_MICRON);

  Serial.printf("Moving: %.3f px (%.3f µm) (Target encoder: %ld)\n", distance_pixels, distance_microns, target_position);
}

/**
 * Wait for motion to complete within tolerance
 */
bool waitForMotionComplete(float tolerance_microns, uint32_t timeout_ms)
{
  int32_t tolerance_counts = tolerance_microns * ENCODER_COUNTS_PER_MICRON;
  uint32_t start_time = millis();

  while(millis() - start_time < timeout_ms)
    {
      int32_t current_position = readEncoder();
      int32_t position_error = abs(target_position - current_position);

      if(position_error <= tolerance_counts && abs(current_speed) < 10)
        {
          return true; // Motion complete
        }

      delay(10); // Small delay to prevent CPU hogging
    }

  return false; // Timeout occurred
}

//==============================================================================
// CALIBRATION & HOMING
//==============================================================================
/**
 * Calibrate the system by finding home position
 * For a single limit switch setup, this assumes the switch is at the minimum position
 */
void calibrateSystem()
{
  Serial.println("Starting system calibration...");

  // Reset limit switch flag
  limitSwitchTriggered = false;

  // First move away from the limit switch if already triggered
  if(limitSwitchTriggered)
    {
      Serial.println("Limit switch already triggered. Moving away...");
      // Move in the positive direction (away from limit switch)
      moveRelative(100); // Move 100 microns away
      waitForMotionComplete(0.1, 5000);

      // Check if we're clear of the limit switch
      if(limitSwitchTriggered)
        {
          Serial.println("Error: Could not move away from limit switch!");
          return;
        }
    }

  // Now find the minimum position by moving toward the limit switch
  Serial.println("Finding minimum position (home)...");
  moveToPosition(-99999, 1); // Move to a large negative value

  // Wait for limit switch to trigger or timeout
  unsigned long startTime = millis();
  while(!limitSwitchTriggered && (millis() - startTime < 30000))
    {
      delay(10);
    }

  if(!limitSwitchTriggered)
    {
      Serial.println("Calibration failed: Limit switch not triggered!");
      return;
    }

  // Stop motion
  current_speed = 0;
  delay(500); // Allow system to settle

  // Move away from limit switch
  Serial.println("Moving away from limit switch...");
  moveRelative(50); // Move 50 microns away
  waitForMotionComplete(0.1, 5000);

  // Define this position as our absolute zero
  absolute_zero_position = readEncoder();

  // Also set as relative zero by default
  relative_zero_position = absolute_zero_position;

  // Reset target to current position
  target_position = readEncoder();

  // Reset limit flag
  limitSwitchTriggered = false;

  Serial.println("System calibrated. Absolute zero set at current position.");
}

//==============================================================================
// ARDUINO SETUP & LOOP
//==============================================================================
/**
 * Setup hardware and initialize the system
 */
void setup()
{
  // Initialize serial for debugging
  Serial.begin(250000);
  Serial.println("High Precision Motion Control System");

  // Configure motor control pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Active LOW enable
  digitalWrite(ENABLE_PIN, LOW); // Enable the motor driver

  // Setup encoder hardware
  setupEncoder();

  // Setup limit switch
  setupLimitSwitch();

  // Create tasks with appropriate priorities
  xTaskCreatePinnedToCore(pidControlTask, "PID Control", 4096, NULL,
                          3, // Higher priority
                          &pidTaskHandle,
                          1 // Core 1
  );

  xTaskCreatePinnedToCore(motionControlTask, "Motion Control", 4096, NULL,
                          2, // Lower than PID but still high
                          &motionTaskHandle,
                          1 // Core 1
  );

  xTaskCreatePinnedToCore(statusUpdateTask, "Status Updates", 4096, NULL,
                          1, // Lower priority
                          &statusTaskHandle,
                          0 // Core 0
  );

  xTaskCreatePinnedToCore(voltageMonitorTask, "Voltage Monitor", 4096, NULL, 2, NULL, 0);

  // Allow time for tasks to start
  delay(100);

  // Initialize relative and absolute zeros
  absolute_zero_position = readEncoder();
  relative_zero_position = absolute_zero_position;

  // Optional: Calibrate system on startup
  // calibrateSystem();  // Uncomment if you want automatic calibration at startup

  Serial.println("System initialized and ready!");
  Serial.println("Available commands:");
  Serial.println("  HOME - Calibrate and set home position");
  Serial.println("  MOVE x - Move to x microns from relative zero");
  Serial.println("  MOVEPX x - Move to x pixels from relative zero");
  Serial.println("  REL x - Move x microns relative to current position");
  Serial.println("  RELPX x - Move x pixels relative to current position");
  Serial.println("  RESET_POS - Reset relative zero to current position");
  Serial.println("  RESET_LIMIT - Reset limit switch flag");
  Serial.println("  STATUS - Print current status");
}

/**
 * Main loop for command processing
 */
void loop()
{
  // Check for commands from serial port
  if(Serial.available() > 0)
    {
      String command = Serial.readStringUntil('\n');
      command.trim();
      command.toUpperCase();

      // Parse command
      if(command.startsWith("MOVE "))
        {
          float position = command.substring(5).toFloat();
          moveToPosition(position);
        }
      else if(command.startsWith("MOVEPX "))
        {
          float position_pixels = command.substring(7).toFloat();
          moveToPositionPixels(position_pixels);
        }
      else if(command.startsWith("REL "))
        {
          float distance = command.substring(4).toFloat();
          moveRelative(distance);
        }
      else if(command.startsWith("RELPX "))
        {
          float distance_pixels = command.substring(6).toFloat();
          moveRelativePixels(distance_pixels);
        }
      else if(command == "HOME")
        {
          calibrateSystem();
        }
      else if(command == "STATUS")
        {
          printStatusUpdate(true);
        }
      else if(command == "RESET_LIMIT")
        {
          limitSwitchTriggered = false;
          Serial.println("Limit switch flag reset");
        }
      else if(command == "RESET_POS")
        {
          resetRelativeZero();
        }
      else
        {
          Serial.println("Unknown command");
        }
    }

  // Main loop runs at lower priority
  delay(10);
}