/**
 * ESP32 High-Precision Motion Control System
 * Stepper Motor Driver Module
 */

#ifndef STEPPERDRIVER_H
#define STEPPERDRIVER_H

#include <Arduino.h>
#include "Config.h"

namespace MotionSystem {

    /**
     * Manages stepper motor control including step generation and direction
     */
    class StepperDriver {
    public:
        /**
         * Constructor
         * @param stepPin Step pulse pin
         * @param dirPin Direction pin
         * @param enablePin Driver enable pin
         * @param microsteps Microstepping setting
         * @param stepsPerRev Full steps per revolution
         */
        StepperDriver(uint8_t  stepPin     = Config::Pins::STEP_PIN,
                      uint8_t  dirPin      = Config::Pins::DIR_PIN,
                      uint8_t  enablePin   = Config::Pins::ENABLE_PIN,
                      uint16_t microsteps  = Config::MotionParams::MICROSTEPS,
                      uint16_t stepsPerRev = Config::MotionParams::STEPS_PER_REV);

        /**
         * Initialize pins and driver
         */
        void begin();

        /**
         * Enable the motor driver
         * @param enable true=enable, false=disable
         */
        void enable(bool enable = true);

        /**
         * Set motor direction
         * @param forward true=forward, false=backward
         */
        void setDirection(bool forward);

        /**
         * Generate a single step pulse
         * Uses fast direct port manipulation for timing-critical operations
         */
        void IRAM_ATTR step();

        /**
         * Set the current speed in steps per second
         * @param speed Desired speed in steps/second
         */
        void setSpeed(float speed);

        /**
         * Get current speed
         * @return Current speed in steps/second
         */
        float getSpeed() const {
            return m_currentSpeed;
        }

        /**
         * Update step position based on elapsed time
         * @param now Current time in microseconds
         * @return true if step was generated, false otherwise
         */
        bool IRAM_ATTR update(uint64_t now);

        /**
         * Get current step position
         * @return Current step position
         */
        int32_t getCurrentPosition() const {
            return m_currentStepPosition;
        }

        /**
         * Calculate time for next step based on current speed
         * @return Interval in microseconds
         */
        uint32_t calculateStepInterval() const;

        /**
         * Convert microns to motor steps
         * @param microns Position in microns
         * @return Equivalent motor steps
         */
        int32_t micronsToSteps(float microns) const;

        /**
         * Convert pixels to motor steps
         * @param pixels Position in pixels
         * @return Equivalent motor steps
         */
        int32_t pixelsToSteps(float pixels) const;

    private:
        uint8_t m_stepPin;    // Step pulse pin
        uint8_t m_dirPin;     // Direction pin
        uint8_t m_enablePin;  // Driver enable pin

        uint16_t m_microsteps;   // Microstepping setting
        uint16_t m_stepsPerRev;  // Full steps per revolution

        float    m_currentSpeed;         // Current speed in steps/second
        int32_t  m_currentStepPosition;  // Current position in steps
        uint64_t m_lastStepTime;         // Time of last step pulse (microseconds)

        bool m_direction;  // Current direction (true=forward, false=backward)
        bool m_enabled;    // Whether driver is enabled
    };

}  // namespace MotionSystem

#endif  // STEPPERDRIVER_H