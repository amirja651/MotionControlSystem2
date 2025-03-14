/**
 * ESP32 High-Precision Motion Control System
 * Serial Command Handler Module
 */

#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include <Arduino.h>
#include "MotionController.h"

namespace MotionSystem {

    /**
     * Handles serial port commands
     */
    class CommandHandler {
    public:
        /**
         * Constructor
         * @param controller Reference to motion controller
         * @param serial Reference to Serial port (default Serial)
         */
        CommandHandler(MotionController& controller, Stream& serial = Serial);

        /**
         * Initialize the command handler
         */
        void begin();

        /**
         * Process incoming serial commands
         * Should be called in the main loop
         */
        void update();

        /**
         * Print help information
         */
        void printHelp() const;

    private:
        MotionController& m_controller;  // Reference to motion controller
        Stream&           m_serial;      // Reference to serial port

        String m_commandBuffer;    // Buffer for incoming command
        bool   m_commandComplete;  // Flag for complete command

        /**
         * Process a complete command
         * @param command Command string
         */
        void processCommand(const String& command);
    };

}  // namespace MotionSystem

#endif  // COMMANDHANDLER_H