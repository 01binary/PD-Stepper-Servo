/*
* PD Stepper Servo Controller
* Switchable Control Interface
* Copyright (C) 2025 Ctrl^H Hackerspace
*/

#pragma once

//
// Constants
//

enum COMMAND            // High-level command modes
{
  MANUAL,               // Manual control via buttons
  VELOCITY,             // Direct velocity command
  POSITION              // PID with position command
};

enum CONTROL            // Low-level control modes
{
  VOLTAGE_CONTROL,      // Controlled by voltage, current unregulated
  CURRENT_CONTROL       // Controlled by voltage, current regulated
};

enum VOLTAGE            // Supported USB voltages
{
  VOLTAGE_5V = 5,       // 5V (default)
  VOLTAGE_9V = 9,       // 9V
  VOLTAGE_12V = 12,     // 12V
  VOLTAGE_15V = 15,     // 15V
  VOLTAGE_20V = 20      // 20V
};

enum STANDSTILL         // Standstill operation
{
  NORMAL = 0,           // Same as braking
  FREEWHEELING = 1,     // No braking
  STRONG_BRAKING = 2,   // Strong braking
  BRAKING = 3           // Normal braking
};

enum MICROSTEPS         // Velocity resolutions
{
  MICROSTEPS_1 = 1,     // Full step
  MICROSTEPS_4 = 4,     // Half step
  MICROSTEPS_8 = 8,     // Quarter step
  MICROSTEPS_16 = 16,   // Eighth step
  MICROSTEPS_32 = 32,   // Sixteenth step
  MICROSTEPS_64 = 64,   // Thirty-second step
  MICROSTEPS_128 = 128, // Sixty-fourth step
  MICROSTEPS_256 = 256  // Hundred-twenty-eighth step
};

//
// Structures
//

struct Status
{
  const char* name;         // Controller name for identification

  COMMAND commandMode;      // High-level command mode
  CONTROL controlMode;      // Low-level control mode
  bool enabled;             // Motor enabled
  bool powerGood;           // Voltage negotiated

  int count;                // Microstep counter (relative encoder)
  int rawPosition;          // Absolute encoder position
  int revolutions;          // Absolute encoder revolutions
  double position;          // Scaled encoder position

  int velocity;             // Current velocity

  double voltage;           // Actual voltage
  int current;              // Run at current %

  bool overTemp;            // Over temperature warning
  bool overTempShutdown;    // Shutdown warning
  unsigned int stallGuard;  // Recommended stall guard threshold
  bool stalled;             // Stall detected
};

struct PositionFeedback
{
  double goal;              // Commanded position
  double position;          // Scaled position from absolute encoder
  double error;             // Proportional error
  double tolerance;         // Position command tolerance
  double integralError;     // Integral error
  double derivativeError;   // Derivative error
  double proportional;      // Proportional term
  double integral;          // Integral term
  double derivative;        // Derivative term
  int command;              // Commanded velocity (all terms added)
};

struct Settings
{
  const char* name;               // Controller name for identification

  CONTROL controlMode;            // Low-level control mode
  VOLTAGE voltage;                // USB voltage
  int current;                    // Run at current %
  int holdCurrent;                // Hold at current %
  int holdDelay;                  // Hold delay in ms
  MICROSTEPS microstepsPerStep;   // Velocity resolution
  int stallThreshold;             // Stall detection threshold
  STANDSTILL standstillMode;      // Standstill operation
  int coolStepDurationThreshold;  // CoolStep duration threshold
  int buttonVelocity;             // Button velocity
  bool enableLogging;             // Enable serial monitor/plotter

  int count;                      // Microstep counter (relative encoder)
  int encoderMin;                 // Minimum absolute encoder value
  int encoderMax;                 // Maximum absolute encoder value
  double positionMin;             // Minimum scaled encoder value
  double positionMax;             // Maximum scaled encoder value
  bool positionInvert;            // Whether to invert encoder position
  int velocityMin;                // Minimum velocity
  int velocityMax;                // Maximum velocity

  double Kp;                      // Proportional gain
  double Ki;                      // Integral gain
  double Kd;                      // Derivative gain
  double iMin;                    // Minimum integral term
  double iMax;                    // Maximum integral term
  double tolerance;               // Position command tolerance
};

//
// Types
//

typedef void (*EnableCommandPtr)(bool);
typedef void (*PositionCommandPtr)(double);
typedef void (*VelocityCommandPtr)(int);
typedef void (*SettingsCommandPtr)(const Settings&);
typedef void (*StatusFeedbackPtr)(Status&);
typedef void (*PositionFeedbackPtr)(PositionFeedback&);
typedef void (*VelocityFeedbackPtr)(int&);
typedef void (*SettingsFeedbackPtr)(Settings&);
