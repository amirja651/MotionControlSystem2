/**
 * ESP32 High-Precision Motion Control System
 * Encoder Implementation
 */

#include "Encoder.h"

namespace MotionSystem {

    // Initialize static variables
    volatile int32_t Encoder::s_overflowCount = 0;

    Encoder::Encoder(uint8_t aPin, uint8_t bPin, uint8_t indexPin, pcnt_unit_t pcntUnit)
        : m_aPin(aPin),
          m_bPin(bPin),
          m_indexPin(indexPin),
          m_pcntUnit(pcntUnit),
          m_position(0),
          m_lastCount(0),
          m_initialized(false) {}

    /**
     * Setup encoder pulse counter for quadrature decoding
     * Using ESP32's hardware pulse counter for precise position tracking
     */
    void Encoder::begin() {
        if (m_initialized)
            return;

        // Configure pulse counter unit for quadrature decoding
        pcnt_config_t pcnt_config = {
            .pulse_gpio_num = m_aPin,             // Channel A
            .ctrl_gpio_num  = m_bPin,             // Channel B
            .lctrl_mode     = PCNT_MODE_REVERSE,  // Reverse counting when B=1
            .hctrl_mode     = PCNT_MODE_KEEP,     // Keep counting when B=0
            .pos_mode       = PCNT_COUNT_INC,     // Count up on rising edge A
            .neg_mode       = PCNT_COUNT_DEC,     // Count down on falling edge A
            .counter_h_lim  = 32767,              // Max hardware limit
            .counter_l_lim  = -32768,             // Min hardware limit
            .unit           = m_pcntUnit,
            .channel        = PCNT_CHANNEL_0,
        };

        pcnt_unit_config(&pcnt_config);

        // Set up pulse counter events
        pcnt_event_enable(m_pcntUnit, PCNT_EVT_H_LIM);
        pcnt_event_enable(m_pcntUnit, PCNT_EVT_L_LIM);

        // Filter out glitches (adjust these values based on encoder specs)
        pcnt_set_filter_value(m_pcntUnit, 100);
        pcnt_filter_enable(m_pcntUnit);

        // Start the counter
        pcnt_counter_pause(m_pcntUnit);
        pcnt_counter_clear(m_pcntUnit);
        pcnt_counter_resume(m_pcntUnit);

        // Set up interrupt for overflow handling
        pcnt_isr_service_install(0);
        pcnt_isr_handler_add(m_pcntUnit, overflowHandler, this);

        m_initialized = true;
    }

    /**
     * Read the current position from the encoder in quadrature counts
     */
    int32_t Encoder::readPosition() {
        int16_t count = 0;
        pcnt_get_counter_value(m_pcntUnit, &count);

        // Calculate 32-bit position from hardware counter and overflow counter
        m_position  = s_overflowCount + count;
        m_lastCount = count;

        return m_position;
    }

    /**
     * Reset the current position
     */
    void Encoder::resetCounter() {
        pcnt_counter_pause(m_pcntUnit);
        pcnt_counter_clear(m_pcntUnit);
        pcnt_counter_resume(m_pcntUnit);

        m_position      = 0;
        s_overflowCount = 0;
    }

    /**
     * Convert encoder counts to microns
     */
    float Encoder::countsToMicrons(int32_t counts) {
        return static_cast<float>(counts) / Config::MotionParams::ENCODER_COUNTS_PER_MICRON;
    }

    /**
     * Convert microns to motor steps
     */
    int32_t Encoder::micronsToCount(float microns) {
        return static_cast<int32_t>(microns * Config::MotionParams::ENCODER_COUNTS_PER_MICRON);
    }

    /**
     * Convert encoder counts to pixels
     */
    float Encoder::countsToPixels(int32_t counts) {
        // Implement the conversion directly instead of calling countsToMicrons
        return (static_cast<float>(counts) / Config::MotionParams::ENCODER_COUNTS_PER_MICRON) /
               Config::MotionParams::PIXEL_SIZE;
    }

    /**
     * Convert pixels to motor steps
     */
    int32_t Encoder::pixelsToCount(float pixels) {
        // Implement the conversion directly instead of calling micronsToCount
        return static_cast<int32_t>((pixels * Config::MotionParams::PIXEL_SIZE) *
                                    Config::MotionParams::ENCODER_COUNTS_PER_MICRON);
    }

    /**
     * ISR for pulse counter overflow/underflow
     * Extends the 16-bit counter to 32-bit
     */
    void IRAM_ATTR Encoder::overflowHandler(void* arg) {
        uint32_t status  = 0;
        Encoder* encoder = static_cast<Encoder*>(arg);

        // Get the triggered events
        pcnt_get_event_status(encoder->m_pcntUnit, &status);

        // Handle counter overflow
        if (status & PCNT_EVT_H_LIM) {
            s_overflowCount += 32767;
            pcnt_counter_clear(encoder->m_pcntUnit);
        }
        // Handle counter underflow
        else if (status & PCNT_EVT_L_LIM) {
            s_overflowCount -= 32768;
            pcnt_counter_clear(encoder->m_pcntUnit);
        }
    }

}  // namespace MotionSystem