/**
 * ESP32 High-Precision Motion Control System
 * Motion Controller Module
 */

#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "Encoder.h"
#include "PIDController.h"
#include "SafetyMonitor.h"
#include "StepperDriver.h"

namespace MotionSystem {

    // Motion status
    enum class MotionStatus { IDLE, MOVING, HOMING, CALIBRATING, ERROR };

    /**
     * High-level motion control integrating all subsystems
     */
    class MotionController {
    public:
        /**
         * Constructor
         * @param encoder Reference to encoder object
         * @param pid Reference to PID controller
         * @param driver Reference to stepper driver
         * @param safety Reference to safety monitor
         */
        MotionController(Encoder& encoder, PIDController& pid, StepperDriver& driver,
                         SafetyMonitor& safety);

        /**
         * Initialize the motion controller
         */
        void begin();

        /**
         * Start the motion control tasks
         */
        void startTasks();

        /**
         * PID control task - runs at fixed frequency
         * @param parameter Task parameter (pointer to MotionController)
         */
        static void pidControlTask(void* parameter);

        /**
         * Motion control task - handles step generation
         * @param parameter Task parameter (pointer to MotionController)
         */
        static void motionControlTask(void* parameter);

        /**
         * Status update task - periodic status reporting
         * @param parameter Task parameter (pointer to MotionController)
         */
        static void statusUpdateTask(void* parameter);

        /**
         * Move to absolute position in microns (from relative zero)
         * @param positionMicrons Target position in microns
         * @param calibration Whether this is a calibration move (ignores limits)
         * @return true if move command was accepted
         */
        bool moveToPosition(float positionMicrons, bool calibration = false);

        /**
         * Move to absolute position in pixels (from relative zero)
         * @param positionPixels Target position in pixels
         * @return true if move command was accepted
         */
        bool moveToPositionPixels(float positionPixels);

        /**
         * Move relative to current position in microns
         * @param distanceMicrons Distance to move in microns
         * @return true if move command was accepted
         */
        bool moveRelative(float distanceMicrons);

        /**
         * Move relative to current position in pixels
         * @param distancePixels Distance to move in pixels
         * @return true if move command was accepted
         */
        bool moveRelativePixels(float distancePixels);

        /**
         * Wait for motion to complete within tolerance
         * @param toleranceMicrons Position tolerance in microns
         * @param timeoutMs Maximum wait time in milliseconds
         * @return true if motion completed within tolerance, false if timeout
         */
        bool waitForMotionComplete(float toleranceMicrons, uint32_t timeoutMs);

        /**
         * Calibrate the system by finding home position
         * @return true if calibration was successful
         */
        bool calibrateSystem();

        /**
         * Reset the relative zero position to current position
         */
        void resetRelativeZero();

        /**
         * Get absolute position in microns (from absolute zero)
         * @return Current position in microns
         */
        float getAbsolutePosition() const;

        /**
         * Get relative position in microns (from relative zero)
         * @return Current position in microns
         */
        float getRelativePosition() const;

        /**
         * Print status update to serial port
         * @param forceDisplay Force display even if not moving
         */
        void printStatusUpdate(bool forceDisplay = false) const;

        /**
         * Get current motion status
         * @return Current motion status
         */
        MotionStatus getStatus() const {
            return m_status;
        }

        /**
         * Called when a safety event occurs
         * @param event Safety event that occurred
         */
        void handleSafetyEvent(SafetyEvent event);

        /**
         * Process a safety event from the safety monitor
         * @param event Event type
         */
        static void safetyEventCallback(SafetyEvent event);

    private:
        Encoder&       m_encoder;  // Reference to encoder object
        PIDController& m_pid;      // Reference to PID controller
        StepperDriver& m_driver;   // Reference to stepper driver
        SafetyMonitor& m_safety;   // Reference to safety monitor

        TaskHandle_t m_pidTaskHandle;     // Handle to PID task
        TaskHandle_t m_motionTaskHandle;  // Handle to motion task
        TaskHandle_t m_statusTaskHandle;  // Handle to status task

        MotionStatus m_status;  // Current motion status

        int32_t m_absoluteZeroPosition;  // Encoder position at absolute zero (HOME)
        int32_t m_relativeZeroPosition;  // Encoder position at relative zero (RESET)

        uint64_t m_lastPidTime;     // Last PID update time
        uint32_t m_lastStatusTime;  // Last status update time

        // Static pointer to singleton instance for use in static callbacks
        static MotionController* s_instance;
    };

}  // namespace MotionSystem

#endif  // MOTIONCONTROLLER_H