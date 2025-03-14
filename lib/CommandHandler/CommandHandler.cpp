/**
 * ESP32 High-Precision Motion Control System
 * Command Handler Implementation
 */

#include "CommandHandler.h"

namespace MotionSystem {

    CommandHandler::CommandHandler(MotionController& controller, Stream& serial)
        : m_controller(controller),
          m_serial(serial),
          m_commandBuffer(""),
          m_commandComplete(false) {}

    void CommandHandler::begin() {
        m_commandBuffer.reserve(64);  // Pre-allocate buffer space
        m_commandComplete = false;
    }

    void CommandHandler::update() {
        // Process any incoming serial data
        while (m_serial.available() > 0) {
            char inChar = (char)m_serial.read();

            // Process end of line
            if (inChar == '\n' || inChar == '\r') {
                if (m_commandBuffer.length() > 0) {
                    m_commandComplete = true;
                }
            } else {
                // Add character to buffer
                m_commandBuffer += inChar;
            }
        }

        // Process complete command
        if (m_commandComplete) {
            processCommand(m_commandBuffer);
            m_commandBuffer   = "";
            m_commandComplete = false;
        }
    }

    void CommandHandler::processCommand(const String& command) {
        // Convert to uppercase for case-insensitive comparison
        String cmd = command;
        cmd.trim();
        cmd.toUpperCase();

        // Parse and execute command
        if (cmd.startsWith("MOVE ")) {
            float position = cmd.substring(5).toFloat();
            bool  success  = m_controller.moveToPosition(position);

            if (success) {
                m_serial.printf("Moving to: %.3f µm\n", position);
            }
        } else if (cmd.startsWith("MOVEPX ")) {
            float positionPixels = cmd.substring(7).toFloat();
            bool  success        = m_controller.moveToPositionPixels(positionPixels);

            if (success) {
                m_serial.printf("Moving to: %.3f px\n", positionPixels);
            }
        } else if (cmd.startsWith("REL ")) {
            float distance = cmd.substring(4).toFloat();
            bool  success  = m_controller.moveRelative(distance);

            if (success) {
                m_serial.printf("Moving by: %.3f µm\n", distance);
            }
        } else if (cmd.startsWith("RELPX ")) {
            float distancePixels = cmd.substring(6).toFloat();
            bool  success        = m_controller.moveRelativePixels(distancePixels);

            if (success) {
                m_serial.printf("Moving by: %.3f px\n", distancePixels);
            }
        } else if (cmd == "HOME") {
            m_serial.println("Starting home calibration...");
            bool success = m_controller.calibrateSystem();

            if (success) {
                m_serial.println("Home calibration complete.");
            } else {
                m_serial.println("Home calibration failed!");
            }
        } else if (cmd == "STATUS") {
            m_controller.printStatusUpdate(true);
        } else if (cmd == "RESET_POS") {
            m_controller.resetRelativeZero();
            m_serial.println("Relative zero position reset at current position.");
        } else if (cmd == "HELP") {
            printHelp();
        } else {
            m_serial.println("Unknown command. Type HELP for available commands.");
        }
    }

    void CommandHandler::printHelp() const {
        m_serial.println("\nAvailable commands:");
        m_serial.println("  HOME           - Calibrate and set home position");
        m_serial.println("  MOVE x         - Move to x microns from relative zero");
        m_serial.println("  MOVEPX x       - Move to x pixels from relative zero");
        m_serial.println("  REL x          - Move x microns relative to current position");
        m_serial.println("  RELPX x        - Move x pixels relative to current position");
        m_serial.println("  RESET_POS      - Reset relative zero to current position");
        m_serial.println("  STATUS         - Print current status");
        m_serial.println("  HELP           - Display this help message");
        m_serial.println();
    }

}  // namespace MotionSystem