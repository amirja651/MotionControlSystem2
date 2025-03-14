/**
 * ESP32 High-Precision Motion Control System
 * PID Controller Implementation
 */

#include "PIDController.h"

namespace MotionSystem {

    PIDController::PIDController(float kp, float ki, float kd, int16_t maxIntegral)
        : m_kp(kp),
          m_ki(ki),
          m_kd(kd),
          m_maxIntegral(maxIntegral),
          m_target(0),
          m_lastError(0),
          m_integral(0),
          m_lastPosition(0),
          m_lastProportional(0),
          m_lastDerivative(0) {}

    void PIDController::setParameters(float kp, float ki, float kd) {
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
    }

    void PIDController::setMaxIntegral(int16_t maxIntegral) {
        m_maxIntegral = maxIntegral;
    }

    void PIDController::setTarget(int32_t target) {
        m_target = target;
    }

    float PIDController::compute(int32_t currentPosition, float dt) {
        // Calculate error (in encoder counts)
        float error = m_target - currentPosition;

        // Calculate PID terms
        m_lastProportional = m_kp * error;

        // Integrate error
        m_integral += m_ki * error * dt;

        // Apply anti-windup (limit integral term)
        if (m_integral > m_maxIntegral) {
            m_integral = m_maxIntegral;
        } else if (m_integral < -m_maxIntegral) {
            m_integral = -m_maxIntegral;
        }

        // Calculate derivative term (only if dt is valid)
        m_lastDerivative = 0;
        if (dt > 0) {
            // Use position change instead of error change to avoid derivative kick
            int32_t positionChange = m_lastPosition - currentPosition;
            m_lastDerivative       = m_kd * positionChange / dt;
        }

        // Store values for next iteration
        m_lastError    = error;
        m_lastPosition = currentPosition;

        // Calculate overall output (speed in steps/sec)
        float output = m_lastProportional + m_integral + m_lastDerivative;

        return output;
    }

    void PIDController::reset() {
        m_integral         = 0;
        m_lastError        = 0;
        m_lastPosition     = 0;
        m_lastProportional = 0;
        m_lastDerivative   = 0;
    }

}  // namespace MotionSystem