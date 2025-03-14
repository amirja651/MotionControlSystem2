/**
 * ESP32 High-Precision Motion Control System
 * Quadrature Encoder Module
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <driver/pcnt.h>
#include "Config.h"
#include "IEncoder.h"

namespace MotionSystem {

    /**
     * Manages encoder hardware interface and position tracking
     * Uses ESP32's hardware pulse counter for precise position tracking
     */
    class Encoder : public IEncoder {
    public:
        /**
         * Constructor
         * @param aPin Channel A pin
         * @param bPin Channel B pin
         * @param indexPin Optional index pin (Z channel)
         * @param pcntUnit ESP32 pulse counter unit to use
         */
        Encoder(uint8_t     aPin     = Config::Pins::ENCODER_A_PIN,
                uint8_t     bPin     = Config::Pins::ENCODER_B_PIN,
                uint8_t     indexPin = Config::Pins::ENCODER_INDEX_PIN,
                pcnt_unit_t pcntUnit = PCNT_UNIT_0);

        /**
         * Initialize the encoder hardware
         */
        void begin() override;

        /**
         * Read current encoder position in counts
         * @return Current position as 32-bit integer
         */
        int32_t readPosition() override;

        /**
         * Get last read position without updating
         * @return Last read position
         */
        int32_t getLastPosition() const override {
            return m_position;
        }

        /**
         * Get last read 16-bit counter value
         * @return Read encoder counter
         */
        int32_t getEncoderCounter() const override {
            return m_lastCount;
        }

        /**
         * Reset encoder counter to zero
         */
        void resetCounter() override;

        /**
         * Convert encoder counts to microns
         * @param counts Encoder counts
         * @return Position in microns
         */
        float countsToMicrons(int32_t counts) override;

        /**
         * Convert microns to encoder counts
         * @param microns Position in microns
         * @return Equivalent encoder counts
         */
        int32_t micronsToCount(float microns) override;

        /**
         * Convert encoder counts to pixels
         * @param counts Encoder counts
         * @return Position in pixels
         */
        static float countsToPixels(int32_t counts);

        /**
         * Convert pixels to encoder counts
         * @param pixels Position in pixels
         * @return Equivalent encoder counts
         */
        static int32_t pixelsToCount(float pixels);

        /**
         * Interrupt handler for pulse counter overflow/underflow
         */
        static void IRAM_ATTR overflowHandler(void* arg);

    private:
        uint8_t     m_aPin;      // Encoder A channel pin
        uint8_t     m_bPin;      // Encoder B channel pin
        uint8_t     m_indexPin;  // Optional index pin
        pcnt_unit_t m_pcntUnit;  // ESP32 pulse counter unit

        volatile int32_t m_position;   // Current encoder position
        int16_t          m_lastCount;  // Last read 16-bit counter value

        bool m_initialized;  // Whether encoder has been initialized

        // Extended position tracking for 32-bit count from 16-bit hardware counter
        static volatile int32_t s_overflowCount;
    };

}  // namespace MotionSystem

#endif  // ENCODER_H