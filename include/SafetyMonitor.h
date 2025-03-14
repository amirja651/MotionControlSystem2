/**
 * ESP32 High-Precision Motion Control System
 * Safety Monitor Module
 */

#ifndef SAFETYMONITOR_H
#define SAFETYMONITOR_H

#include <Arduino.h>
#include "Config.h"

namespace MotionSystem {

    // Safety event types
    enum class SafetyEvent {
        LIMIT_SWITCH_TRIGGERED,
        POWER_VOLTAGE_LOW,
        POWER_VOLTAGE_HIGH,
        MOTOR_STALL_DETECTED,
        OVERCURRENT_DETECTED,
        EMERGENCY_STOP_REQUESTED,
        TRAVEL_LIMIT_EXCEEDED,
        NORMAL_OPERATION
    };

    // Safety event callback function type
    using SafetyEventCallback = void (*)(SafetyEvent event);

    /**
     * Monitors system safety and triggers emergency stops
     */
    class SafetyMonitor {
    public:
        /**
         * Constructor
         * @param limitSwitchPin Limit switch input pin
         * @param voltageSensePin Analog pin for voltage monitoring
         */
        SafetyMonitor(uint8_t limitSwitchPin  = Config::Pins::LIMIT_SWITCH_PIN,
                      uint8_t voltageSensePin = Config::Pins::VOLTAGE_SENSE_PIN);

        /**
         * Initialize safety systems
         */
        void begin();

        /**
         * Check all safety parameters
         * @return Current safety status
         */
        SafetyEvent checkSafety();

        /**
         * Register a callback for safety events
         * @param callback Function to call when safety event occurs
         */
        void setEventCallback(SafetyEventCallback callback);

        /**
         * Check if limit switch is triggered
         * @return true if limit switch is triggered
         */
        bool isLimitSwitchTriggered() const {
            return m_limitSwitchTriggered;
        }

        /**
         * Reset limit switch flag
         */
        void resetLimitSwitch() {
            m_limitSwitchTriggered = false;
        }

        /**
         * Check if system is in emergency stop state
         * @return true if emergency stop is active
         */
        bool isEmergencyStop() const {
            return m_emergencyStop;
        }

        /**
         * Trigger emergency stop
         * @param reason Reason for emergency stop
         */
        void triggerEmergencyStop(SafetyEvent reason);

        /**
         * Clear emergency stop state
         */
        void clearEmergencyStop();

        /**
         * Check if position is within safe travel limits
         * @param relativePosition Current position in microns relative to zero
         * @return true if position is within safe limits
         */
        bool isPositionSafe(float relativePosition) const;

        /**
         * Read system voltage
         * @return Current voltage level
         */
        float readVoltage() const;

        /**
         * Static ISR for limit switch
         */
        static void IRAM_ATTR limitSwitchISR(void* arg);

    private:
        uint8_t m_limitSwitchPin;   // Limit switch input pin
        uint8_t m_voltageSensePin;  // Analog pin for voltage monitoring

        volatile bool m_limitSwitchTriggered;  // Whether limit switch is triggered
        volatile bool m_emergencyStop;         // System in emergency stop state

        SafetyEventCallback m_eventCallback;  // User callback for safety events
        SafetyEvent         m_lastEvent;      // Last safety event

        // Configure the limit switch pin and interrupt
        void setupLimitSwitch();
    };

}  // namespace MotionSystem

#endif  // SAFETYMONITOR_H