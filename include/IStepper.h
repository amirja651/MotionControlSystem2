/**
 * ESP32 High-Precision Motion Control System
 * Stepper Driver Interface
 */

#ifndef ISTEPPER_H
#define ISTEPPER_H

#include <Arduino.h>

namespace MotionSystem {

    /**
     * Interface for stepper drivers (real or simulated)
     */
    class IStepper {
    public:
        /**
         * Initialize the stepper driver
         */
        virtual void begin() = 0;

        /**
         * Enable the motor driver
         * @param enable true=enable, false=disable
         */
        virtual void enable(bool enable = true) = 0;

        /**
         * Set motor direction
         * @param forward true=forward, false=backward
         */
        virtual void setDirection(bool forward) = 0;

        /**
         * Generate a single step pulse
         */
        virtual void step() = 0;

        /**
         * Set the current speed in steps per second
         * @param speed Desired speed in steps/second
         */
        virtual void setSpeed(float speed) = 0;

        /**
         * Get current speed
         * @return Current speed in steps/second
         */
        virtual float getSpeed() const = 0;

        /**
         * Update step position based on elapsed time
         * @param now Current time in microseconds
         * @return true if step was generated, false otherwise
         */
        virtual bool update(uint64_t now) = 0;

        /**
         * Get current step position
         * @return Current step position
         */
        virtual int32_t getCurrentPosition() const = 0;

        /**
         * Virtual destructor
         */
        virtual ~IStepper() = default;
    };

}  // namespace MotionSystem

#endif  // ISTEPPER_H