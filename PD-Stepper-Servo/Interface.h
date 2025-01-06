/*
* PD Stepper Servo Controller
* Switchable Control Interface
* Copyright (C) 2025 Ctrl^H Hackerspace
*/

#pragma once

//
// Constants
//

enum COMMAND_MODE
{
  MANUAL,     // Manual control via buttons
  VELOCITY,   // Velocity command (microsteps)
  POSITION    // PID with position command
};

enum CONTROL_MODE
{
  VOLTAGE_CONTROL,
  CURRENT_CONTROL
};

enum VOLTAGE
{
  VOLTAGE_5V = 5,
  VOLTAGE_9V = 9,
  VOLTAGE_12V = 12,
  VOLTAGE_15V = 15,
  VOLTAGE_20V = 20
};

enum STANDSTILL
{
  NORMAL = 0,
  FREEWHEELING = 1,
  STRONG_BRAKING = 2,
  BRAKING = 3
};

enum MICROSTEPS
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
  const char* name;

  COMMAND_MODE commandMode;
  CONTROL_MODE controlMode;
  bool enabled;
  bool powerGood;

  int count;
  int rawPosition;
  double position;

  int velocity;

  double voltage;
  int current;

  bool overTemp;
  bool overTempShutdown;
  unsigned int stallGuard;
  bool stalled;
};

struct PositionFeedback
{
  double goal;
  double position;
  double error;
  double integralError;
  double derivativeError;
};

struct Settings
{
  const char* name;

  CONTROL_MODE controlMode;
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
