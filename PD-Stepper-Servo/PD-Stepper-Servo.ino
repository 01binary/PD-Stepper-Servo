/*
* PD Stepper Servo Controller
* Copyright (C) 2025 Ctrl^H Hackerspace
*/

//
// Includes
//

// Standard library

#include <stdio.h>
#include <string.h>

// Multi-threading

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// TMC2209 Stepper Motor Driver

#include <TMC2209.h>

// AS5600 Hall Effect Encoder

#include <Wire.h>

// Flash memory

#include <Preferences.h>

// Interface

#include "Interface.h"
#include "RestInterface.h"

//
// Constants
//

// Controller

const int RATE_HZ = 50;
const int TIMESTEP_MS = 1000 / RATE_HZ;
const float TIMESTEP = 1.0 / RATE_HZ;
const int DEBOUNCE_MS = 50;
const int STARTUP_DELAY_MS = 200;

// Encoder

const int AS5600_ADDRESS = 0x36;
const int AS5600_ANGLE_REGISTER = 0x0C;
const int AS5600_MAX = 4096;

// Stepper driver

const long TMC_SERIAL_BAUD_RATE = 115200;
const int TMC_ENABLED = 21;
const int TMC_STEP = 5;
const int TMC_DIR = 6;
const int TMC_MS1 = 1;
const int TMC_MS2 = 2;
const int TMC_SPREAD = 7;
const int TMC_TX = 17;
const int TMC_RX = 18;
const int TMC_OVERCURRENT = 16;
const int TMC_INDEX = 11;

// USB Power Delivery

const int PD_POWERGOOD = 15;
const int PD_VBUS = 4;
const int PD_CFG1 = 38;
const int PD_CFG2 = 48;
const int PD_CFG3 = 47;
const float PD_VREF = 3.3;
const float PD_DIV = 0.1189427313;

// PD Stepper Board

const int BRD_NTC = 7;
const int BRD_LED1 = 10;
const int BRD_LED2 = 12;
const int BRD_SW1 = 35;
const int BRD_SW2 = 36;
const int BRD_SW3 = 37;
const int BRD_AUX1 = 14;
const int BRD_AUX2 = 13;

//
// Variables
//

// State

MODE mode = MANUAL;
bool enabled = 0;
int commandedVelocity = 0;
float commandedPosition = 0;
float proportionalError = 0;
float integralError = 0;
float derivativeError = 0;
bool incrementButtonPushed = false;
bool decrementButtonPushed = false;
bool resetButtonPushed = false;
unsigned long lastDebounceTime = 0;

// Configuration

VOLTAGE voltage = 5V;
int current = 100;
int microsteps = 32;
int stallThreshold = 10;
STANDSTILL standstillMode = NORMAL;
float tolerance = 0.1;
float Kp = 100;
float Ki = 10;
float Kd = 10;
float iMin = -10;
float iMax = 10;
int velocityMin = 0;
int velocityMax = 3000;
int buttonVelocity = 100;

// Encoder

int rawPosition = 0;
int revolutions = 0;
int rawPositionWithRevolutions = 0;
int encoderMin = 0;
int encoderMax = AS5600_MAX;
float positionMin = 0.0;
float positionMax = 1.0;
float position = 0.0;

// Stepper Driver

TMC2209 motorDriver;
HardwareSerial &motorSerial = Serial2;
bool motorState = false;
bool motorEnabled = true;
bool motorOverTemp = false;
bool motorOverTempShutdown = false;
bool motorStalled = false;
unsigned int motorStallGuard = 0;

// Power Delivery

bool powerGood = false;
float busVoltage = 0;

//
// Forward Declarations
//

void initPower();
void readPower();
void initBoard();
void readBoard();
void writeBoard();
void initEncoder();
void readEncoder();
void initMotor();
void readMotor();
void writeMotorEnabled(bool enabled);
void writeMotorVelocity(int velocity);
void writeMotorStepDirection(bool step, bool direction);
void initMotorControl();
void runMotorControl(void *pvParameters);
void enabledCommand(bool enabled);
void positionCommand(float position);
void velocityCommand(int velocity);
void settingsCommand(const Settings& settings);
void statusFeedback(Status& status);
void positionFeedback(PositionFeedback& position);
void velocityFeedback(int& velocity);
void settingsFeedback(Settings& settings);

//
// Interface
//

useRestInterface(
  "ssid", "password", 8080,
  enabledCommand,
  positionCommand,
  velocityCommand,
  settingsCommand,
  statusFeedback,
  positionFeedback,
  velocityFeedback,
  settingsFeedback
);

//
// Entry point
//

void setup()
{
  delay(STARTUP_DELAY_MS);
  Serial.begin(115200);

  readSettings();

  initPower();
  initMotor();
  initEncoder();
  initMotorControl();
  initBoard();
}

void loop()
{
}

//
// RT thread
//

void initMotorControl()
{
  xTaskCreate(
    runMotorControl,
    "MotorControl",
    2048,
    NULL,
    configMAX_PRIORITIES - 1,
    NULL);
}

void runMotorControl(void *pvParameters)
{
  const TickType_t freq = pdMS_TO_TICKS(TIMESTEP_MS);
  TickType_t lastTime = xTaskGetTickCount();

  while (1)
  {
    readEncoder();
    readMotor();
    readPower();
    readBoard();

    if (!powerGood && motorEnabled)
    {
      writeMotorEnabled(false);
      continue;
    }

    if (enabled != motorEnabled)
    {
      writeMotorEnabled(enabled);
    }

    if (!enabled)
    {
      continue;
    }

    if (incrementButtonPushed)
    {
      mode = MANUAL;
      writeMotorVelocity(buttonVelocity);
    }
    else if (decrementButtonPushed)
    {
      mode = MANUAL;
      writeMotorVelocity(-buttonVelocity);
    }
    else if (resetButtonPushed)
    {
      mode = MANUAL;
      writeMotorVelocity(0);
    }

    if (mode == VELOCITY)
    {
      writeMotorVelocity(commandedVelocity);
    }
    else if (mode == POSITION)
    {
      // Calculate proportional error
      float error = commandedPosition - position;

      // Calculate integral error
      integralError += TIMESTEP * error;

      bool limitIntegralError = Ki > 0 && (iMin < 0 || iMax > 0);

      if (limitIntegralError)
      {
        integralError = constrain(integralError, iMin / Ki, iMax / Ki);
      }

      // Calculate derivative error
      derivativeError = (error - proportionalError) / TIMESTEP;
      proportionalError = error;

      // Calculate proportional contribution
      float proportional = Kp * proportionalError;

      // Calculate integral contribution
      float integral = Ki * integralError;

      if (limitIntegralError)
      {
        integral = constrain(integral, iMin, iMax);
      }

      // Calculate derivative contribution
      float derivative = Kd * derivativeError;

      // Calculate command
      int rawCommand = int(proportional + integral + derivative);

      if (rawCommand == 0)
      {
        commandedVelocity = 0;
      }
      else if (rawCommand < velocityMin)
      {
        commandedVelocity = velocityMin;
      }
      else if (rawCommand > velocityMax)
      {
        commandedVelocity = velocityMax;
      }
      else
      {
        commandedVelocity = rawCommand;
      }

      writeMotorVelocity(commandedVelocity);
    }

    writeBoard();

    // Sleep
    vTaskDelayUntil(&lastTime, freq);
  }
}

//
// Helpers
//

void initPower()
{
  pinMode(PD_POWERGOOD, INPUT);
  pinMode(PD_CFG1, OUTPUT);
  pinMode(PD_CFG2, OUTPUT);
  pinMode(PD_CFG3, OUTPUT);

  writeVoltage(voltage);
}

void readPower()
{
  busVoltage = analogRead(PD_VBUS) * (PD_VREF / 4096.0) / PD_DIV;
  powerGood = digitalRead(PD_POWERGOOD) == 0;
}

void initBoard()
{
  pinMode(BRD_SW1, INPUT);
  pinMode(BRD_SW2, INPUT);
  pinMode(BRD_SW3, INPUT);
  pinMode(BRD_LED1, OUTPUT);
  pinMode(BRD_LED2, OUTPUT);

  // Flash LED after setup complete
  digitalWrite(LED1, HIGH);
  delay(STARTUP_DELAY_MS);
  digitalWrite(LED1, LOW);
}

void readBoard()
{
  if ((millis() - lastDebounceTime) > DEBOUNCE_MS)
  {
    lastDebounceTime = millis();
    incrementButtonPushed = digitalRead(BRD_SW3) == HIGH ? false : true;
    decrementButtonPushed = digitalRead(BRD_SW1) == HIGH ? false : true;
    resetButtonPushed = digitalRead(BRD_SW2) == HIGH ? false : true;
  }
}

void writeBoard()
{
  digitalWrite(BRD_LED2, motorStalled ? HIGH : LOW);
}

void initMotor()
{
  pinMode(TMC_STEP, OUTPUT);
  pinMode(TMC_DIR, OUTPUT);
  pinMode(TMC_MS1, OUTPUT);
  pinMode(TMC_MS1, OUTPUT);
  pinMode(TMC_ENABLED, OUTPUT);
  pinMode(TMC_OVERCURRENT, INPUT);

  digitalWrite(TMC_ENABLED, LOW);
  digitalWrite(TMC_MS1, LOW);
  digitalWrite(TMC_MS2, LOW);

  motorDriver.setup(motorSerial, TMC_SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  motorDriver.enableAutomaticCurrentScaling();
  motorDriver.setRunCurrent(current);
  motorDriver.setMicrostepsPerStep(microsteps);
  motorDriver.setStallGuardThreshold(stallThreshold);
  motorDriver.setStandstillMode(standstillMode);
  motorDriver.enableStealthChop();
  motorDriver.setCoolStepDurationThreshold(5000);
  motorDriver.disable();
}

void writeMotorEnabled(bool enabled)
{
  if (enabled)
  {
    motorDriver.enable();
  }
  else
  {
    motorDriver.disable();
  }
}

void writeMotorVelocity(int velocity)
{
  motorDriver.moveAtVelocity(velocity);
}

void writeMotorStepDirection(bool direction)
{
  digitalWrite(TMC_DIR, direction ? HIGH : LOW);
  digitalWrite(TMC_STEP, motorState ? HIGH : LOW);

  motorState = !motorState;
}

void readMotor()
{
  motorEnabled = !motorDriver.hardwareDisabled();
  motorStalled = digitalRead(TMC_OVERCURRENT) == HIGH;
  motorStallGuard = motorDriver.getStallGuardResult();

  TMC2209::Status status = motorDriver.getStatus();
  motorOverTemp = status.over_temperature_warning;
  motorOverTempShutdown = status.over_temperature_shutdown;
}

void writeVoltage(VOLTAGE voltage)
{
  //      | 5V   9V   12V   15V   20V
  // -----+---------------------------
  // CFG1 | 1    0     0     0     0
  // CFG2 | -    0     0     1     1
  // CFG3 | -    0     1     1     0

  switch (voltage)
  {
    case 5V:
      {
        digitalWrite(PD_CFG1, HIGH);
      }
      break;
    case 9V:
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, LOW);
        digitalWrite(PD_CFG3, LOW);
      }
      break;
    case 12V:
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, LOW);
        digitalWrite(PD_CFG3, HIGH);
      }
      break;
    case 15V:
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, HIGH)
        digitalWrite(PD_CFG3, HIGH);
      }
      break;
    case 20V:
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, HIGH)
        digitalWrite(PD_CFG3, LOW);
      }
      break;
  }
}

void initEncoder()
{
  Wire.begin(SDA, SCL);
}

void readEncoder()
{
  // Request raw angle
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_REGISTER);
  Wire.endTransmission(false);

  // Read raw angle
  Wire.requestFrom(AS5600_ADDRESS, 2);

  if (Wire.available() < 2)
    return;

  int reading = Wire.read() << 8 | Wire.read();

  // Keep track of revolutions
  if (rawPosition > 3000 && reading < 1000)
  {
    revolutions++;
  }
  else if (rawPosition < 1000 && reading > 3000)
  {
    revolutions--;
  }

  rawPosition = reading;
  rawPositionWithRevolutions = rawPosition + AS5600_MAX * revolutions;

  // Calculate normalized position
  float norm = 
    (constrain(rawPositionWithRevolutions, encoderMin, encoderMax) - encoderMin)
    / (encoderMax - encoderMin);

  // Scale to final position range
  position = positionMin + norm * (positionMax - positionMin);
}

void readSettings()
{
  preferences.begin("settings", false);
  voltage = (VOLTAGE)preferences.getInt("voltage", voltage);
  current = preferences.getInt("current", current);
  microsteps = preferences.getInt("microsteps", microsteps);
  stallThreshold = preferences.getInt("stallThreshold", stallThreshold);
  standstillMode = (STANDSTILL)preferences.getInt("standstillMode", (int)standstillMode);
  buttonVelocity = preferences.getInt("buttonVelocity", buttonVelocity);
  encoderMin = preferences.getInt("encoderMin", encoderMin);
  encoderMax = preferences.getInt("encoderMax", encoderMax);
  positionMin = preferences.getFloat("positionMin", positionMin);
  positionMax = preferences.getFloat("positionMax", positionMax);
  velocityMin = preferences.getInt("velocityMin", velocityMin);
  velocityMax = preferences.getInt("velocityMax", velocityMax);
  Kp = preferences.getFloat("Kp", Kp);
  Ki = preferences.getFloat("Ki", Ki);
  Kd = preferences.getFloat("Id", Kd);
  iMin = preferences.getFloat("iMin", iMin);
  iMax = preferences.getFloat("iMax", iMax);
  tolerance = preferences.getFloat("tolerance", tolerance);
  preferences.end();
}

void writeSettings()
{
  preferences.begin("settings", false);
  preferences.putInt("voltage", (int)voltage);
  preferences.putInt("current", current);
  preferences.putInt("microsteps", microsteps);
  preferences.putInt("stallThreshold", stallThreshold);
  preferences.putInt("standstillMode", (int)standstillMode);
  preferences.putInt("buttonVelocity", buttonVelocity);
  preferences.putInt("encoderMin", encoderMin);
  preferences.putInt("encoderMax", encoderMax);
  preferences.putFloat("positionMin", positionMin);
  preferences.putFloat("positionMax", positionMax);
  preferences.putInt("velocityMin", velocityMin);
  preferences.putInt("velocityMax", velocityMax);
  preferences.putFloat("Kp", Kp);
  preferences.putFloat("Ki", Ki);
  preferences.putFloat("Id", Kd);
  preferences.putFloat("iMin", iMin);
  preferences.putFloat("iMax", iMax);
  preferences.putFloat("tolerance", tolerance);
  preferences.end();
}

void enabledCommand(bool value)
{
  enabled = value;
}

void positionCommand(float command)
{
  commandedPosition = command;
}

void velocityCommand(int command)
{
  commandedVelocity = command;
}

void settingsCommand(const Settings& settings)
{
  writeMotorVelocity(0);

  if (enabled != settings.enabled)
  {
    enabled = settings.enabled;
    writeMotorEnabled(enabled);
  }

  if (voltage != settings.voltage)
  {
    voltage = settings.voltage;
    writeVoltage(voltage);
  }

  if (current != settings.current)
  {
    current = settings.current;
    motorDriver.setRunCurrent(current);
  }

  if (microsteps != settings.microsteps)
  {
    microsteps = settings.microsteps;
    motorDriver.setMicrostepsPerStep(microsteps);
  }

  if (stallThreshold != settings.stallThreshold)
  {
    stallThreshold = settings.stallThreshold;
    motorDriver.setStallGuardThreshold(stallThreshold);
  }

  if (standstillMode != settings.standstillMode)
  {
    standstillMode = settings.standstillMode;
    motorDriver.setStandstillMode(standstillMode);
  }

  buttonVelocity = settings.buttonVelocity;

  encoderMin = settings.encoderMin;
  encoderMax = settings.encoderMax;
  positionMin = settings.positionMin;
  positionMax = settings.positionMax;
  velocityMin = settings.velocityMin;
  velocityMax = settings.velocityMax;

  Kp = settings.Kp;
  Ki = settings.Ki;
  Kd = settings.Kd;
  iMin = settings.iMin;
  iMax = settings.iMax;
  tolerance = settings.tolerance;

  writeSettings();
}

void statusFeedback(Status& status)
{
  status.mode = mode;
  status.enabled = enabled;
  status.rawPosition = rawPositionWithRevolutions;
  status.position = position;
  status.velocity = commandedVelocity;
  status.voltage = busVoltage;
  status.overTemp = motorOverTemp;
  status.overTempShutdown = motorOverTempShutdown;
  status.stalled = motorStalled;
}

void positionFeedback(PositionFeedback& position)
{
  position.goal = commandedPosition;
  position.position = position;
  position.error = proportionalError;
  position.integralError = integralError;
  position.derivativeError = derivativeError;
}

void velocityFeedback(int& velocity)
{
  velocity = commandedVelocity;
}

void settingsFeedback(Settings& settings)
{
  settings.enabled = enabled;
  settings.voltage = voltage;
  settings.current = current;
  settings.microsteps = microsteps;
  settings.stallThreshold = stallThreshold;
  settings.standstillMode = standstillMode;
  settings.buttonVelocity = buttonVelocity;
  settings.encoderMin = encoderMin;
  settings.encoderMax = encoderMax;
  settings.positionMin = positionMin;
  settings.positionMax = positionMax;
  settings.velocityMin = velocityMin;
  settings.velocityMax = velocityMax;
  settings.Kp = Kp;
  settings.Ki = Ki;
  settings.Kd = Kd;
  settings.iMin = iMin;
  settings.iMax = iMax;
  settings.tolerance = tolerance;
}
