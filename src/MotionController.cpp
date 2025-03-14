/**
 * ESP32 High-Precision Motion Control System
 * Motion Controller Implementation
 */

#include "MotionController.h"
#include "esp_timer.h"

namespace MotionSystem {

    // Initialize static member
    MotionController* MotionController::s_instance = nullptr;

    MotionController::MotionController(Encoder& encoder, PIDController& pid, StepperDriver& driver,
                                       SafetyMonitor& safety)
        : m_encoder(encoder),
          m_pid(pid),
          m_driver(driver),
          m_safety(safety),
          m_pidTaskHandle(nullptr),
          m_motionTaskHandle(nullptr),
          m_statusTaskHandle(nullptr),
          m_status(MotionStatus::IDLE),
          m_absoluteZeroPosition(0),
          m_relativeZeroPosition(0),
          m_lastPidTime(0),
          m_lastStatusTime(0) {
        // Store instance pointer for callbacks
        s_instance = this;
    }

    void MotionController::begin() {
        // Enable stepper driver
        m_driver.enable(true);

        // Initialize timing
        m_lastPidTime    = esp_timer_get_time();
        m_lastStatusTime = millis();

        // Initialize relative and absolute zeros to current position
        int32_t currentPos     = m_encoder.readPosition();
        m_absoluteZeroPosition = currentPos;
        m_relativeZeroPosition = currentPos;

        // Initialize PID controller with current position as target
        m_pid.setTarget(currentPos);
    }

    void MotionController::startTasks() {
        // Create PID control task
        xTaskCreatePinnedToCore(pidControlTask, "PID Control", Config::Tasks::STACK_SIZE, this,
                                Config::Tasks::PID_PRIORITY, &m_pidTaskHandle,
                                Config::Tasks::PID_CORE);

        // Create motion control task
        xTaskCreatePinnedToCore(motionControlTask, "Motion Control", Config::Tasks::STACK_SIZE,
                                this, Config::Tasks::MOTION_PRIORITY, &m_motionTaskHandle,
                                Config::Tasks::MOTION_CORE);

        // Create status update task
        xTaskCreatePinnedToCore(statusUpdateTask, "Status Updates", Config::Tasks::STACK_SIZE, this,
                                Config::Tasks::STATUS_PRIORITY, &m_statusTaskHandle,
                                Config::Tasks::STATUS_CORE);
    }

    // Static callback for safety events
    void MotionController::safetyEventCallback(SafetyEvent event) {
        if (s_instance) {
            s_instance->handleSafetyEvent(event);
        }
    }

    void MotionController::handleSafetyEvent(SafetyEvent event) {
        switch (event) {
            case SafetyEvent::LIMIT_SWITCH_TRIGGERED:
                Serial.println("SAFETY: Limit switch triggered!");
                m_driver.setSpeed(0);
                break;

            case SafetyEvent::POWER_VOLTAGE_LOW:
                Serial.println("SAFETY: Low voltage detected!");
                m_driver.setSpeed(0);
                break;

            case SafetyEvent::POWER_VOLTAGE_HIGH:
                Serial.println("SAFETY: High voltage detected!");
                m_driver.setSpeed(0);
                break;

            case SafetyEvent::TRAVEL_LIMIT_EXCEEDED:
                Serial.println("SAFETY: Travel limit exceeded!");
                m_driver.setSpeed(0);
                break;

            case SafetyEvent::EMERGENCY_STOP_REQUESTED:
                Serial.println("SAFETY: Emergency stop requested!");
                m_driver.setSpeed(0);
                break;

            case SafetyEvent::NORMAL_OPERATION:
                // Normal operation, no action needed
                break;

            default:
                Serial.println("SAFETY: Unknown safety event!");
                break;
        }
    }

    /**
     * PID Task - Runs at high priority for closed-loop position control
     */
    void MotionController::pidControlTask(void* parameter) {
        MotionController* controller = static_cast<MotionController*>(parameter);

        // PID update timing
        TickType_t       xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency =
            1000 / Config::Control::PID_UPDATE_FREQ;  // Convert to milliseconds

        while (true) {
            // Get current time
            uint64_t now = esp_timer_get_time();

            // Calculate time delta in seconds
            float dt                  = (now - controller->m_lastPidTime) / 1000000.0f;
            controller->m_lastPidTime = now;

            // Get current position
            int32_t currentPosition = controller->m_encoder.readPosition();

            // Compute PID output
            float output = controller->m_pid.compute(currentPosition, dt);

            // Set motor speed based on PID output
            float speed =
                constrain(output, -Config::Control::MAX_SPEED, Config::Control::MAX_SPEED);
            controller->m_driver.setSpeed(speed);

            // Use vTaskDelayUntil for precise timing
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void MotionController::motionControlTask(void* parameter) {
        MotionController* controller = static_cast<MotionController*>(parameter);

        while (true) {
            // Check for emergency stop condition
            if (controller->m_safety.isEmergencyStop()) {
                controller->m_driver.setSpeed(0);
                controller->m_status = MotionStatus::ERROR;

                // Give time for other tasks
                vTaskDelay(100);
                continue;
            }

            // Check all safety parameters
            controller->m_safety.checkSafety();

            // Update stepper motor - generate steps if needed
            controller->m_driver.generateStep();

            // Determine motion status
            float speed = controller->m_driver.getSpeed();
            if (abs(speed) < 10) {
                controller->m_status = MotionStatus::IDLE;
            } else {
                controller->m_status = MotionStatus::MOVING;
            }

            // Yield to other tasks
            vTaskDelay(1);
        }
    }

    /**
     * Status update task - Handles periodic status updates
     */
    void MotionController::statusUpdateTask(void* parameter) {
        MotionController* controller = static_cast<MotionController*>(parameter);

        TickType_t       xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency    = Config::Control::STATUS_UPDATE_MS;

        while (true) {
            // Get current positions
            float relPosition = controller->getRelativePosition();
            float relTarget   = controller->m_encoder.countsToMicrons(
                controller->m_pid.getTarget() - controller->m_relativeZeroPosition);

            // Print status update if moving or significant error
            if (controller->m_status == MotionStatus::MOVING ||
                abs(relPosition - relTarget) > 0.2f) {
                controller->printStatusUpdate();
            }

            // Use vTaskDelayUntil for precise timing
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    /**
     * Move to absolute position in microns (relative to relative zero)
     */
    bool MotionController::moveToPosition(float positionMicrons, bool calibration) {
        // Check if within relative travel limits
        if (!calibration && !m_safety.isPositionSafe(positionMicrons)) {
            // Fix: Change %d to %.1f for the double value
            Serial.printf("ERROR: Position %.3f µm exceeds relative travel limits (±%.1f mm)\n",
                          positionMicrons, Config::MotionParams::REL_TRAVEL_LIMIT_MM);
            return false;
        }

        // Convert microns to encoder counts and adjust for relative zero
        int32_t targetPosition = m_relativeZeroPosition + m_encoder.micronsToCount(positionMicrons);

        // Set target position in PID controller
        m_pid.setTarget(targetPosition);

        // Fix: Change %ld to %d for int32_t on this platform
        Serial.printf("Moving to: %.3f µm (relative) (Target encoder: %d)\n", positionMicrons,
                      targetPosition);

        return true;
    }

    /**
     * Move to absolute position in pixels (relative to relative zero)
     */
    bool MotionController::moveToPositionPixels(float positionPixels) {
        // Convert pixels to microns
        float positionMicrons = positionPixels * Config::MotionParams::PIXEL_SIZE;

        // Use the moveToPosition method for consistency
        bool result = moveToPosition(positionMicrons);

        if (result) {
            Serial.printf("Moving to: %.3f px (%.3f µm relative)\n", positionPixels,
                          positionMicrons);
        }

        return result;
    }

    /**
     * Move relative to current position in microns
     */
    bool MotionController::moveRelative(float distanceMicrons) {
        // Get current relative position
        float currentRelPos = getRelativePosition();

        // Calculate new position
        float newRelPos = currentRelPos + distanceMicrons;

        // Move to the calculated position
        bool result = moveToPosition(newRelPos);

        if (result) {
            Serial.printf("Moving: %.3f µm relative (to %.3f µm)\n", distanceMicrons, newRelPos);
        }

        return result;
    }

    /**
     * Move relative to current position in pixels
     */
    bool MotionController::moveRelativePixels(float distancePixels) {
        // Convert pixels to microns
        float distanceMicrons = distancePixels * Config::MotionParams::PIXEL_SIZE;

        // Use the moveRelative method for consistency
        bool result = moveRelative(distanceMicrons);

        if (result) {
            Serial.printf("Moving: %.3f px (%.3f µm) relative\n", distancePixels, distanceMicrons);
        }

        return result;
    }

    /**
     * Wait for motion to complete within tolerance
     */
    bool MotionController::waitForMotionComplete(float toleranceMicrons, uint32_t timeoutMs) {
        int32_t  toleranceCounts = m_encoder.micronsToCount(toleranceMicrons);
        uint32_t startTime       = millis();

        while (millis() - startTime < timeoutMs) {
            // Get current position
            int32_t currentPosition = m_encoder.readPosition();

            // Calculate error
            int32_t positionError = abs(m_pid.getTarget() - currentPosition);

            // Check if within tolerance and not moving
            if (positionError <= toleranceCounts && abs(m_driver.getSpeed()) < 10) {
                return true;  // Motion complete
            }

            delay(10);  // Small delay to prevent CPU hogging
        }

        return false;  // Timeout occurred
    }

    /**
     * Calibrate the system by finding home position
     * For a single limit switch setup, this assumes the switch is at the minimum position
     */
    bool MotionController::calibrateSystem() {
        Serial.println("Starting system calibration...");

        // Reset limit switch flag
        m_safety.resetLimitSwitch();

        // First move away from the limit switch if already triggered
        if (m_safety.isLimitSwitchTriggered()) {
            Serial.println("Limit switch already triggered. Moving away...");

            // Move in the positive direction (away from limit switch)
            moveRelative(100);  // Move 100 microns away
            waitForMotionComplete(0.1, 5000);

            // Check if we're clear of the limit switch
            if (m_safety.isLimitSwitchTriggered()) {
                Serial.println("Error: Could not move away from limit switch!");
                return false;
            }
        }

        // Now find the minimum position by moving toward the limit switch
        Serial.println("Finding minimum position (home)...");
        moveToPosition(-99999, true);  // Move to a large negative value, ignoring limits

        // Wait for limit switch to trigger or timeout
        unsigned long startTime = millis();
        while (!m_safety.isLimitSwitchTriggered() && (millis() - startTime < 30000)) {
            delay(10);
        }

        if (!m_safety.isLimitSwitchTriggered()) {
            Serial.println("Calibration failed: Limit switch not triggered!");
            return false;
        }

        // Stop motion
        m_driver.setSpeed(0);
        delay(500);  // Allow system to settle

        // Move away from limit switch
        Serial.println("Moving away from limit switch...");
        m_safety.resetLimitSwitch();
        moveRelative(50);  // Move 50 microns away
        waitForMotionComplete(0.1, 5000);

        // Define this position as our absolute zero
        m_absoluteZeroPosition = m_encoder.readPosition();

        // Also set as relative zero by default
        m_relativeZeroPosition = m_absoluteZeroPosition;

        // Reset target to current position
        m_pid.setTarget(m_encoder.readPosition());

        Serial.println("System calibrated. Absolute zero set at current position.");
        return true;
    }

    /**
     * Reset the relative zero position to current position
     */
    void MotionController::resetRelativeZero() {
        m_relativeZeroPosition = m_encoder.readPosition();
        Serial.println("Relative zero position reset at current position");
    }

    /**
     * Get absolute position in microns (relative to HOME/absolute zero)
     */
    float MotionController::getAbsolutePosition() const {
        int32_t currentPosition = m_encoder.getLastPosition();
        return m_encoder.countsToMicrons(currentPosition - m_absoluteZeroPosition);
    }

    /**
     * Get relative position in microns (relative to user-set zero)
     */
    float MotionController::getRelativePosition() const {
        int32_t currentPosition = m_encoder.getLastPosition();
        return m_encoder.countsToMicrons(currentPosition - m_relativeZeroPosition);
    }

    /**
     * Print a status update to the serial port
     */
    void MotionController::printStatusUpdate(bool forceDisplay) const {
        // Get current positions
        int32_t currentPosition = m_encoder.getLastPosition();
        int32_t encoderCounter  = m_encoder.getEncoderCounter();
        float   relPosition = m_encoder.countsToMicrons(currentPosition - m_relativeZeroPosition);
        float   absPosition = m_encoder.countsToMicrons(currentPosition - m_absoluteZeroPosition);

        // Get target position
        float relTarget = m_encoder.countsToMicrons(m_pid.getTarget() - m_relativeZeroPosition);

        // Calculate error based on relative position
        float error = relTarget - relPosition;

        // Calculate motor frequency - Fix: use m_driver instead of m_stepper
        float motorFrequency = fabs(m_driver.getSpeed());  // Steps per second

        // Only print if moving or forced display
        if (motorFrequency > 10 || forceDisplay) {
            // Calculate relative position as percentage of relative travel limit
            float relTravelPercent =
                (relPosition / Config::MotionParams::REL_TRAVEL_LIMIT_MICRONS) * 100;

            // Fix: Changed %d to %.1f for the double value
            Serial.printf(
                "POS(rel): %.3f µm (%.1f%% of ±%.1f mm), TARGET: %.3f µm, ERROR: %.3f µm, Encoder "
                "Pos: %" PRId32 ", Encoder Cnt: %" PRId32 "\n",
                relPosition, fabs(relTravelPercent), Config::MotionParams::REL_TRAVEL_LIMIT_MM,
                relTarget, error, currentPosition, encoderCounter);

            /*Serial.printf("POS(abs): %.3f µm, Travel: %.1f%% of %.1f mm\n", absPosition,
                          (absPosition / Config::MotionParams::TOTAL_TRAVEL_MICRONS) * 100,
                          Config::MotionParams::TOTAL_TRAVEL_MM);

            Serial.printf(
                "Motor Freq: %.1f Hz, Speed: %.3f mm/s, Limit Switch: %s\n", motorFrequency,
                (motorFrequency /
                 (Config::MotionParams::STEPS_PER_REV * Config::MotionParams::MICROSTEPS)) *
                    Config::MotionParams::LEAD_SCREW_PITCH,
                m_safety.isLimitSwitchTriggered() ? "TRIGGERED" : "clear");*/

            Serial.println("-------------------------------");
        }
    }

}  // namespace MotionSystem