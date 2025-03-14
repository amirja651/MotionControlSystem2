/**
 * ESP32 High-Precision Motion Control System
 * Stepper Driver Implementation
 */

#include "StepperDriver.h"

namespace MotionSystem {

    StepperDriver::StepperDriver(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin,
                                 uint16_t microsteps, uint16_t stepsPerRev)
        : m_stepPin(stepPin),
          m_dirPin(dirPin),
          m_enablePin(enablePin),
          m_microsteps(microsteps),
          m_stepsPerRev(stepsPerRev),
          m_currentSpeed(0),
          m_currentStepPosition(0),
          m_lastStepTime(0),
          m_direction(true),
          m_enabled(false) {}

    void StepperDriver::begin() {
        // Configure motor control pins
        pinMode(m_stepPin, OUTPUT);
        pinMode(m_dirPin, OUTPUT);
        pinMode(m_enablePin, OUTPUT);

        // Initialize pins to safe state
        digitalWrite(m_stepPin, LOW);
        digitalWrite(m_dirPin, LOW);
        digitalWrite(m_enablePin, HIGH);  // Most drivers use active LOW enable

        m_enabled = false;
    }

    void StepperDriver::enable(bool enable) {
        if (enable) {
            digitalWrite(m_enablePin, LOW);  // Active LOW enable
            m_enabled = true;
        } else {
            digitalWrite(m_enablePin, HIGH);
            m_enabled = false;
        }
    }

    /**
     * Set motor direction (1 = forward, 0 = backward)
     */
    void StepperDriver::setDirection(bool forward) {
        m_direction = forward;
        digitalWrite(m_dirPin, forward ? HIGH : LOW);

        // Direction setup time
        delayMicroseconds(5);
    }

    /**
     * Generate step pulse with precise timing
     * Uses digitalWrite for compatibility but can be optimized with direct register access
     */
    void IRAM_ATTR StepperDriver::step() {
        // Generate a single step pulse
        digitalWrite(m_stepPin, HIGH);
        delayMicroseconds(2);  // Minimum pulse width
        digitalWrite(m_stepPin, LOW);

        // Update position counter
        if (m_direction) {
            m_currentStepPosition++;
        } else {
            m_currentStepPosition--;
        }
    }

    void StepperDriver::setSpeed(float speed) {
        // Set direction based on speed sign
        bool direction = speed >= 0;

        // Only change direction if it's different
        if (direction != m_direction) {
            setDirection(direction);
        }

        // Update current speed
        m_currentSpeed = speed;
    }

    bool IRAM_ATTR StepperDriver::generateStep() {
        // If speed is zero or driver is disabled, no step needed
        if (m_currentSpeed == 0 || !m_enabled) {
            return false;
        }

        // Calculate time for current step interval
        uint32_t step_interval = calculateStepInterval();

        // Get current time
        uint64_t now = esp_timer_get_time();

        // Generate step if enough time has passed
        if (step_interval > 0 && now - m_lastStepTime >= step_interval) {
            step();
            m_lastStepTime = now;
            return true;
        }

        return false;
    }

    /**
     * Calculate time for next step based on current speed
     */
    uint32_t StepperDriver::calculateStepInterval() const {
        float speed = abs(m_currentSpeed);

        // Prevent division by zero
        if (speed < 1) {
            return 0;
        }

        // Convert Hz to microseconds
        return 1000000 / speed;
    }

    /**
     * Convert microns to motor steps
     */
    int32_t StepperDriver::micronsToSteps(float microns) const {
        return roundf(microns * Config::MotionParams::MOTOR_STEPS_PER_MICRON);
    }

    /**
     * Convert pixels to motor steps
     */
    int32_t StepperDriver::pixelsToSteps(float pixels) const {
        return micronsToSteps(pixels * Config::MotionParams::PIXEL_SIZE);
    }

}  // namespace MotionSystem