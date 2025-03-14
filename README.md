# ESP32 High-Precision Motion Control System Simulator

This project provides a virtual simulator for the ESP32 High-Precision Motion Control System. It allows testing and development of motion control software without requiring physical hardware connections.

## Features

- Simulates stepper motor, encoder, limit switch, and voltage sensor
- Adjustable simulation fidelity levels (basic, realistic, challenging)
- Compatible with the actual motion control system code
- Enables software development and testing without hardware
- Includes realistic motor physics simulation (inertia, non-linearity, missed steps)
- Simulates encoder noise and occasional reading errors

## How It Works

The simulator replaces hardware interface classes with software implementations that:

1. **Simulate Physical Behavior**: Mimic real-world physics including inertia, backlash, and imperfections
2. **Coordinate Component Interaction**: Connect virtual stepper movements to encoder feedback
3. **Emulate System Events**: Trigger limit switches and voltage changes
4. **Provide Control Interface**: Allow controlling simulation parameters via serial commands

## Simulation Levels

The simulator offers three levels of simulation fidelity:

- **Level 0 (Basic)**: Ideal components with perfect behavior
- **Level 1 (Realistic)**: Moderate inertia, slight non-linearity, minor encoder noise
- **Level 2 (Challenging)**: Significant inertia, non-linearity, occasional missed steps, encoder errors

## Using the Simulator

### Standard Commands

All standard motion control commands work with the simulator:

- `HOME` - Calibrate and set home position
- `MOVE x` - Move to x microns from relative zero
- `MOVEPX x` - Move to x pixels from relative zero
- `REL x` - Move x microns relative to current position
- `RELPX x` - Move x pixels relative to current position
- `RESET_POS` - Reset relative zero to current position
- `STATUS` - Print current status

### Simulator-Specific Commands

Additional commands to control the simulation:

- `SIM_LEVEL n` - Set simulation level (0=basic, 1=realistic, 2=challenging)
- `SIM_LIMIT_ON` - Trigger limit switch
- `SIM_LIMIT_OFF` - Clear limit switch
- `SIM_VOLTAGE v` - Set voltage level to v volts
- `SIM_STATUS` - Show simulator status

## Project Structure

- `EncoderSimulator` - Simulates quadrature encoder behavior
- `StepperSimulator` - Simulates stepper motor physics and behavior
- `HardwareSimulator` - Coordinates all hardware simulations
- Standard motion control components use these simulators instead of real hardware

## Setup Instructions

1. Use this code with PlatformIO and the ESP32 platform
2. Build and upload the code with the simulation flag:
   ```
   pio run -e simulator -t upload
   ```
3. Open serial monitor at 250000 baud
4. Start with `SIM_LEVEL 1` for realistic simulation
5. Use standard and simulator commands to control the system

## Development Notes

- The simulator maintains API compatibility with real hardware components
- For creating unit tests, use simulation level 0 (ideal behavior)
- For integration testing, use level 1 or 2 to verify robustness
- To simulate component failures, use simulator commands to trigger events

## Testing Scenarios

Some useful testing scenarios:

1. **Homing Sequence**: Use `HOME` command and watch behavior
2. **Position Accuracy**: Test precision with small moves and check error
3. **Limit Switch Response**: Set `SIM_LIMIT_ON` during motion
4. **PID Tuning**: Adjust PID parameters and observe performance under different simulation levels
5. **Motion Profiles**: Test acceleration/deceleration behavior with different move distances