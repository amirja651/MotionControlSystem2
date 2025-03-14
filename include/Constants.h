/**
 * ESP32 High-Precision Motion Control System
 * Constants and System Parameters
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

namespace MotionSystem {
    namespace Constants {
        // System constants
        namespace System {
            constexpr char     VERSION[]           = "1.0.0";
            constexpr char     NAME[]              = "ESP32 High-Precision Motion Control";
            constexpr uint32_t SERIAL_BAUD_RATE    = 250000;
            constexpr uint16_t COMMAND_BUFFER_SIZE = 64;
        }  // namespace System

        // Motor and drivetrain parameters
        namespace Motor {
            constexpr uint16_t STEPS_PER_REV    = 200;   // Full steps per revolution
            constexpr uint16_t MICROSTEPS       = 16;    // Microstepping setting
            constexpr float    LEAD_SCREW_PITCH = 0.5f;  // Lead screw pitch in mm
            constexpr float    STEPS_PER_MM     = (STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_PITCH;
            constexpr float    STEPS_PER_MICRON = STEPS_PER_MM / 1000.0f;

            // Motor control parameters
            constexpr uint16_t MAX_SPEED            = 5000;   // Maximum step frequency in Hz
            constexpr uint16_t ACCELERATION         = 10000;  // Steps per second per second
            constexpr uint16_t MIN_STEP_PULSE_WIDTH = 2;      // Minimum pulse width in microseconds
            constexpr uint16_t DIR_SETUP_TIME       = 5;  // Direction setup time in microseconds
        }  // namespace Motor

        // Encoder parameters
        namespace Encoder {
            constexpr uint16_t PPR               = 1000;     // Pulses per revolution
            constexpr uint16_t CPR               = PPR * 4;  // Counts per revolution (quadrature)
            constexpr float    COUNTS_PER_MM     = CPR / Motor::LEAD_SCREW_PITCH;
            constexpr float    COUNTS_PER_MICRON = COUNTS_PER_MM / 1000.0f;
            constexpr uint16_t FILTER_VALUE      = 100;  // Pulse counter filter value
        }  // namespace Encoder

        // Motion control parameters
        namespace Control {
            // PID controller parameters
            constexpr float   KP           = 0.8f;   // Proportional gain
            constexpr float   KI           = 0.1f;   // Integral gain
            constexpr float   KD           = 0.05f;  // Derivative gain
            constexpr int16_t MAX_INTEGRAL = 1000;   // Anti-windup limit

            constexpr uint16_t PID_UPDATE_FREQ  = 1000;  // PID update frequency in Hz
            constexpr uint16_t STATUS_UPDATE_MS = 400;   // Status update interval in milliseconds
        }  // namespace Control

        // Travel limits and safety parameters
        namespace Travel {
            constexpr float TOTAL_TRAVEL_MM          = 30.0f;  // Total travel distance in mm
            constexpr float REL_TRAVEL_LIMIT_MM      = 3.0f;   // Relative travel limit in mm
            constexpr float TOTAL_TRAVEL_MICRONS     = TOTAL_TRAVEL_MM * 1000.0f;
            constexpr float REL_TRAVEL_LIMIT_MICRONS = REL_TRAVEL_LIMIT_MM * 1000.0f;

            constexpr float POSITION_TOLERANCE_MICRONS = 0.1f;   // Position tolerance in microns
            constexpr float HOME_OFFSET_MICRONS        = 50.0f;  // Offset from limit switch

            constexpr float PIXEL_SIZE = 5.2f;  // Size of one pixel in micrometers
        }  // namespace Travel

        // Power monitoring parameters
        namespace Power {
            constexpr bool  MONITORING_ENABLED        = true;   // Enable power monitoring
            constexpr float VOLTAGE_MIN               = 3.1f;   // Minimum safe voltage
            constexpr float VOLTAGE_MAX               = 3.3f;   // Maximum safe voltage
            constexpr float VOLTAGE_WARNING_THRESHOLD = 0.02f;  // Voltage warning threshold
            constexpr float CUTOFF_VOLTAGE            = 3.1f * 4095.0f / 3.3f;  // ADC cutoff value
        }  // namespace Power

        // FreeRTOS task parameters
        namespace Tasks {
            constexpr uint16_t STACK_SIZE = 4096;  // Stack size for tasks

            constexpr uint8_t PID_PRIORITY    = 3;  // Higher priority
            constexpr uint8_t MOTION_PRIORITY = 2;  // Medium priority
            constexpr uint8_t STATUS_PRIORITY = 1;  // Lower priority

            constexpr uint8_t PID_CORE    = 1;  // Core for PID task
            constexpr uint8_t MOTION_CORE = 1;  // Core for motion task
            constexpr uint8_t STATUS_CORE = 0;  // Core for status task
        }  // namespace Tasks

        // Timing parameters
        namespace Timing {
            constexpr uint32_t HOMING_TIMEOUT_MS = 30000;  // Timeout for homing
            constexpr uint32_t MOTION_TIMEOUT_MS = 10000;  // Default timeout for motion
            constexpr uint32_t SETTLE_DELAY_MS   = 500;    // Delay for system to settle
        }  // namespace Timing
    }  // namespace Constants
}  // namespace MotionSystem

#endif  // CONSTANTS_H