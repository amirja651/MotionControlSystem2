/**
 * ESP32 High-Precision Motion Control System
 * Global Configuration
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace MotionSystem {
    namespace Config {
        // System Identification
        constexpr char SYSTEM_VERSION[] = "1.0.0";
        constexpr char SYSTEM_NAME[]    = "ESP32 Motion Control System";

        // Debug and Logging
        constexpr bool    DEBUG_ENABLED = true;
        constexpr uint8_t LOG_LEVEL     = 5;  // 0=OFF, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG, 5=VERBOSE
        constexpr unsigned long SERIAL_BAUD_RATE     = 115200;
        constexpr bool          COLOR_OUTPUT_ENABLED = false;

        // Pin Definitions
        namespace Pins {
            // Motor and driver pins
            constexpr uint8_t STEP_PIN   = 32;
            constexpr uint8_t DIR_PIN    = 33;
            constexpr uint8_t ENABLE_PIN = 14;

            // Encoder pins
            constexpr uint8_t ENCODER_A_PIN     = 23;
            constexpr uint8_t ENCODER_B_PIN     = 22;
            constexpr uint8_t ENCODER_INDEX_PIN = 21;

            // Limit switch
            constexpr uint8_t LIMIT_SWITCH_PIN = 13;

            // ADC
            constexpr uint8_t VOLTAGE_SENSE_PIN = 36;  // ADC1_CH0
        }  // namespace Pins

        // System Parameters
        namespace MotionParams {
            constexpr uint16_t MICROSTEPS          = 16;
            constexpr uint16_t STEPS_PER_REV       = 200;
            constexpr uint16_t ENCODER_PPR         = 1000;
            constexpr float    LEAD_SCREW_PITCH    = 0.5f;  // mm
            constexpr float    PIXEL_SIZE          = 5.2f;  // um
            constexpr float    TOTAL_TRAVEL_MM     = 30.0f;
            constexpr float    REL_TRAVEL_LIMIT_MM = 3.0f;

            // Derived parameters (calculated at compile time)
            constexpr float TOTAL_TRAVEL_MICRONS     = TOTAL_TRAVEL_MM * 1000.0f;
            constexpr float REL_TRAVEL_LIMIT_MICRONS = REL_TRAVEL_LIMIT_MM * 1000.0f;
            constexpr float MOTOR_STEPS_PER_MICRON =
                (STEPS_PER_REV * MICROSTEPS) / (LEAD_SCREW_PITCH * 1000.0f);
            constexpr float ENCODER_COUNTS_PER_MICRON =
                (ENCODER_PPR * 4.0f) / (LEAD_SCREW_PITCH * 1000.0f);
        }  // namespace MotionParams

        // Motion Control
        namespace Control {
            constexpr uint16_t MAX_SPEED       = 5000;   // steps/sec
            constexpr uint16_t ACCELERATION    = 10000;  // steps/sec^2
            constexpr uint16_t PID_UPDATE_FREQ = 1000;   // Hz

            // PID Parameters
            constexpr float   KP           = 0.8f;
            constexpr float   KI           = 0.1f;
            constexpr float   KD           = 0.05f;
            constexpr int16_t MAX_INTEGRAL = 1000;

            // Status update interval
            constexpr uint16_t STATUS_UPDATE_MS = 400;
        }  // namespace Control

        // Power Monitoring
        namespace Power {
            constexpr bool  MONITORING_ENABLED        = true;
            constexpr float VOLTAGE_MIN               = 3.1f;
            constexpr float VOLTAGE_MAX               = 3.3f;
            constexpr float VOLTAGE_WARNING_THRESHOLD = 0.5f;
            constexpr float CUTOFF_VOLTAGE            = 3.1f * 4095.0f / 3.3f;
        }  // namespace Power

        // FreeRTOS Task Configuration
        namespace Tasks {
            constexpr uint16_t STACK_SIZE = 4096;

            constexpr uint8_t PID_PRIORITY    = 3;
            constexpr uint8_t MOTION_PRIORITY = 2;
            constexpr uint8_t STATUS_PRIORITY = 1;

            constexpr uint8_t PID_CORE    = 1;
            constexpr uint8_t MOTION_CORE = 1;
            constexpr uint8_t STATUS_CORE = 0;
        }  // namespace Tasks
    }  // namespace Config
}  // namespace MotionSystem

#endif  // CONFIG_H