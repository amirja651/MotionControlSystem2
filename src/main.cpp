/**
 * High Precision Motion Control for ESP32 with Stepper Motor and Encoder Feedback
 *
 * Components:
 * - Nema 11 Stepper Motor (1.8deg, 200 steps/rev)
 * - Magnetic Encoder (1000PPR/4000CPR)
 * - Stepper Driver with microstepping
 * - ESP32 Controller
 * - Linear Stage with Ultra-Fine-Thread Ball-Point Set Screw
 *
 * Target precision: < 1 micrometer
 */

#include <Arduino.h>
#include "CommandHandler.h"
#include "Config.h"
#include "Encoder.h"
#include "MotionController.h"
#include "PIDController.h"
#include "SafetyMonitor.h"
#include "StepperDriver.h"

using namespace MotionSystem;

// System components
Encoder          encoder;
PIDController    pidController;
StepperDriver    stepperDriver;
SafetyMonitor    safetyMonitor;
MotionController motionController(encoder, pidController, stepperDriver, safetyMonitor);
CommandHandler   commandHandler(motionController);

void setup() {
    // Initialize serial port for debugging
    Serial.begin(Config::SERIAL_BAUD_RATE);
    Serial.println("High Precision Motion Control System");
    Serial.println("Version: " + String(Config::SYSTEM_VERSION));

    // Initialize all subsystems
    encoder.begin();
    stepperDriver.begin();
    safetyMonitor.begin();

    // Set safety event callback
    safetyMonitor.setEventCallback(MotionController::safetyEventCallback);

    // Initialize and start the motion controller and tasks
    motionController.begin();
    motionController.startTasks();

    // Initialize command handler
    commandHandler.begin();

    // Print help information
    commandHandler.printHelp();

    Serial.println("System initialized and ready!");
}

void loop() {
    // Process serial commands
    commandHandler.update();

    // Main loop runs at lower priority
    delay(10);
}