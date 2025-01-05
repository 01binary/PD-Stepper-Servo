/*
* PD Stepper Servo Controller
* Switchable Control Interface
* Copyright (C) 2025 Ctrl^H Hackerspace
*/

#pragma once

//
// Constants
//

enum MODE
{
  MANUAL,     // Manual control via buttons
  VELOCITY,   // Velocity command (microsteps)
  POSITION    // PID with position command
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

  MODE mode;
  bool enabled;
  bool powerGood;

  int rawPosition;
  float position;

  int velocity;

  float voltage;
  int current;

  bool overTemp;
  bool overTempShutdown;
  unsigned int stallGuard;
  bool stalled;
};

struct PositionFeedback
{
  float goal;
  float position;
  float error;
  float integralError;
  float derivativeError;
};

struct Settings
{
  const char* name;

  VOLTAGE voltage;
  int current;
  MICROSTEPS microstepsPerStep;
  int stallThreshold;
  STANDSTILL standstillMode;
  int coolStepDurationThreshold;
  int buttonVelocity;

  int encoderMin;
  int encoderMax;
  float positionMin;
  float positionMax;
  int velocityMin;
  int velocityMax;

  float Kp;
  float Ki;
  float Kd;
  float iMin;
  float iMax;
  float tolerance;
};

//
// Types
//

typedef void (*EnableCommandPtr)(bool);
typedef void (*PositionCommandPtr)(float);
typedef void (*VelocityCommandPtr)(int);
typedef void (*SettingsCommandPtr)(const Settings&);
typedef void (*StatusFeedbackPtr)(Status&);
typedef void (*PositionFeedbackPtr)(PositionFeedback&);
typedef void (*VelocityFeedbackPtr)(int&);
typedef void (*SettingsFeedbackPtr)(Settings&);
