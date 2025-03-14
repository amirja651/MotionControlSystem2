/**
 * ESP32 High-Precision Motion Control System
 * Safety Monitor Implementation
 */

#include "SafetyMonitor.h"

namespace MotionSystem {

    // Static instance pointer for ISR callback
    static SafetyMonitor* s_instance = nullptr;

    SafetyMonitor::SafetyMonitor(uint8_t limitSwitchPin, uint8_t voltageSensePin)
        : m_limitSwitchPin(limitSwitchPin),
          m_voltageSensePin(voltageSensePin),
          m_limitSwitchTriggered(false),
          m_emergencyStop(false),
          m_eventCallback(nullptr),
          m_lastEvent(SafetyEvent::NORMAL_OPERATION) {
        // Store instance pointer for static callback
        s_instance = this;
    }

    void SafetyMonitor::begin() {
        // Configure limit switch pin with internal pull-up resistor
        pinMode(m_limitSwitchPin, INPUT_PULLUP);

        // Configure analog input for voltage sensing
        pinMode(m_voltageSensePin, INPUT);

        // Setup limit switch with interrupt
        setupLimitSwitch();
    }

    void SafetyMonitor::setupLimitSwitch() {
        // Attach interrupt for limit switch (trigger on falling edge - switch closed)
        attachInterrupt(
            digitalPinToInterrupt(m_limitSwitchPin),
            []() {
                if (s_instance) {
                    s_instance->limitSwitchISR(nullptr);
                }
            },
            FALLING);

        Serial.println("Limit switch configured on pin " + String(m_limitSwitchPin));
    }

    void SafetyMonitor::setEventCallback(SafetyEventCallback callback) {
        m_eventCallback = callback;
    }

    SafetyEvent SafetyMonitor::checkSafety() {
        SafetyEvent currentEvent = SafetyEvent::NORMAL_OPERATION;

        // Check limit switch status
        if (m_limitSwitchTriggered) {
            currentEvent = SafetyEvent::LIMIT_SWITCH_TRIGGERED;
        }

        // Check system voltage if enabled
        if (Config::Power::MONITORING_ENABLED) {
            float voltage = readVoltage();

            if (voltage < Config::Power::VOLTAGE_MIN) {
                currentEvent = SafetyEvent::POWER_VOLTAGE_LOW;
            } else if (voltage > Config::Power::VOLTAGE_MAX) {
                currentEvent = SafetyEvent::POWER_VOLTAGE_HIGH;
            }
        }

        // If event state changed and we have a callback, notify
        if (currentEvent != m_lastEvent && m_eventCallback != nullptr) {
            m_eventCallback(currentEvent);
        }

        // Update last event
        m_lastEvent = currentEvent;

        return currentEvent;
    }

    float SafetyMonitor::readVoltage() const {
        // Read analog value and convert to voltage
        int adcValue = analogRead(m_voltageSensePin);
        return (adcValue * 3.3f) / 4095.0f;  // For ESP32 (12-bit ADC)
    }

    bool SafetyMonitor::isPositionSafe(float relativePosition) const {
        // Check if position is within relative travel limits
        return (relativePosition >= -Config::MotionParams::REL_TRAVEL_LIMIT_MICRONS &&
                relativePosition <= Config::MotionParams::REL_TRAVEL_LIMIT_MICRONS);
    }

    void SafetyMonitor::triggerEmergencyStop(SafetyEvent reason) {
        m_emergencyStop = true;

        // Notify via callback if available
        if (m_eventCallback != nullptr) {
            m_eventCallback(reason);
        }
    }

    void SafetyMonitor::clearEmergencyStop() {
        m_emergencyStop = false;
    }

    void IRAM_ATTR SafetyMonitor::limitSwitchISR(void* arg) {
        // Static member functions can't access instance variables directly
        // We must use the static instance pointer
        if (s_instance) {
            s_instance->m_limitSwitchTriggered = true;
            s_instance->m_emergencyStop        = true;
        }
    }

}  // namespace MotionSystem