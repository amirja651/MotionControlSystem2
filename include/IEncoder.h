/**
 * ESP32 High-Precision Motion Control System
 * Encoder Interface
 */

#ifndef IENCODER_H
#define IENCODER_H

#include <Arduino.h>

namespace MotionSystem {

    /**
     * Interface for encoders (real or simulated)
     */
    class IEncoder {
    public:
        /**
         * Initialize the encoder
         */
        virtual void begin() = 0;

        /**
         * Read current encoder position in counts
         * @return Current position as 32-bit integer
         */
        virtual int32_t readPosition() = 0;

        /**
         * Get last read position without updating
         * @return Last read position
         */
        virtual int32_t getLastPosition() const = 0;

        /**
         * Reset encoder counter to zero
         */
        virtual void resetCounter() = 0;

        /**
         * Convert encoder counts to microns
         * @param counts Encoder counts
         * @return Position in microns
         */
        virtual float countsToMicrons(int32_t counts) = 0;

        /**
         * Convert microns to encoder counts
         * @param microns Position in microns
         * @return Equivalent encoder counts
         */
        virtual int32_t micronsToCount(float microns) = 0;

        /**
         * Virtual destructor
         */
        virtual ~IEncoder() = default;
    };

}  // namespace MotionSystem

#endif  // IENCODER_H