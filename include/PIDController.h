/**
 * ESP32 High-Precision Motion Control System
 * PID Controller Module
 */

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>
#include "Config.h"

namespace MotionSystem {

    /**
     * PID Controller for closed-loop position control
     */
    class PIDController {
    public:
        /**
         * Constructor
         * @param kp Proportional gain
         * @param ki Integral gain
         * @param kd Derivative gain
         * @param maxIntegral Anti-windup limit
         */
        PIDController(float kp = Config::Control::KP, float ki = Config::Control::KI,
                      float   kd          = Config::Control::KD,
                      int16_t maxIntegral = Config::Control::MAX_INTEGRAL);

        /**
         * Configure PID parameters
         * @param kp Proportional gain
         * @param ki Integral gain
         * @param kd Derivative gain
         */
        void setParameters(float kp, float ki, float kd);

        /**
         * Set the maximum integral value for anti-windup
         * @param maxIntegral Maximum integral value
         */
        void setMaxIntegral(int16_t maxIntegral);

        /**
         * Set the target position
         * @param target Target position in encoder counts
         */
        void setTarget(int32_t target);

        /**
         * Get the current target position
         * @return Target position in encoder counts
         */
        int32_t getTarget() const {
            return m_target;
        }

        /**
         * Compute PID output based on current position
         * @param currentPosition Current position in encoder counts
         * @param dt Time delta in seconds since last update
         * @return Control output (speed in steps/sec)
         */
        float compute(int32_t currentPosition, float dt);

        /**
         * Reset internal state (integral, last error)
         */
        void reset();

        /**
         * Get last calculated error
         * @return Last calculated position error
         */
        float getLastError() const {
            return m_lastError;
        }

        /**
         * Get proportional term of last calculation
         * @return Proportional term
         */
        float getLastProportional() const {
            return m_lastProportional;
        }

        /**
         * Get integral term of last calculation
         * @return Integral term
         */
        float getLastIntegral() const {
            return m_integral;
        }

        /**
         * Get derivative term of last calculation
         * @return Derivative term
         */
        float getLastDerivative() const {
            return m_lastDerivative;
        }

    private:
        float   m_kp;           // Proportional gain
        float   m_ki;           // Integral gain
        float   m_kd;           // Derivative gain
        int16_t m_maxIntegral;  // Anti-windup limit

        int32_t m_target;        // Target position in encoder counts
        float   m_lastError;     // Previous position error
        float   m_integral;      // Accumulated integral
        int32_t m_lastPosition;  // Previous position

        // Diagnostic values
        float m_lastProportional;  // Last proportional term
        float m_lastDerivative;    // Last derivative term
    };

}  // namespace MotionSystem

#endif  // PIDCONTROLLER_H