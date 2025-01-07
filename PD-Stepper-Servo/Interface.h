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
  VOLTAGE_5V = 5,
  VOLTAGE_9V = 9,
  VOLTAGE_12V = 12,
  VOLTAGE_15V = 15,
  VOLTAGE_20V = 20
};

enum STANDSTILL         // Standstill operation
{
  NORMAL = 0,           // Same as braking
  FREEWHEELING = 1,     // No braking
  STRONG_BRAKING = 2,   // Strong braking
  BRAKING = 3           // Normal braking
};

enum MICROSTEPS         // Supported velocity command resolutions
{
  MICROSTEPS_1 = 1,
  MICROSTEPS_4 = 4,
  MICROSTEPS_8 = 8,
  MICROSTEPS_16 = 16,
  MICROSTEPS_32 = 32,
  MICROSTEPS_64 = 64,
  MICROSTEPS_128 = 128,
  MICROSTEPS_256 = 256
};

//
// Structures
//

struct Status
{
  const char* name;         // Controller name

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
  const char* name;

  CONTROL controlMode;
  VOLTAGE voltage;
  int current;
  int holdCurrent;
  int holdDelay;
  MICROSTEPS microstepsPerStep;
  int stallThreshold;
  STANDSTILL standstillMode;
  int coolStepDurationThreshold;
  int buttonVelocity;

  int count;
  int encoderMin;
  int encoderMax;
  double positionMin;
  double positionMax;
  int velocityMin;
  int velocityMax;

  double Kp;
  double Ki;
  double Kd;
  double iMin;
  double iMax;
  double tolerance;
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
