/*
* PD Stepper Servo Controller
* Copyright (C) 2025 Ctrl^H Hackerspace
*/

//
// Includes
//

// Multi-threading

#include "freertos/FreeRTOS.h"

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
const double TIMESTEP = 1.0 / RATE_HZ;
const int BUTTON_DEBOUNCE_MS = 50;
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
const double PD_VREF = 3.3;
const double PD_DIV = 0.1189427313;
const double PD_VOLTAGE_MULTIPLIER = PD_VREF / 4096.0 / PD_DIV;

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

COMMAND_MODE commandMode = MANUAL;
CONTROL_MODE controlMode = CURRENT_CONTROL;
bool enabled = false;
int commandedVelocity = 0;
double commandedPosition = 0;
double proportionalError = 0;
double integralError = 0;
double derivativeError = 0;
bool incrementButtonPushed = false;
bool decrementButtonPushed = false;
bool resetButtonPushed = false;
unsigned long lastDebounceTime = 0;

// Controller

Preferences preferences;
String name = "PD-Stepper";
double tolerance = 0.1;
double Kp = 100;
double Ki = 10;
double Kd = 10;
double iMin = -10;
double iMax = 10;
int velocityMin = 5;
int velocityMax = 1440;
int buttonVelocity = 30;

// Encoder

int rawPosition = -1;
int revolutions = 0;
int rawPositionWithRevolutions = 0;
int encoderMin = 0;
int encoderMax = AS5600_MAX;
double positionMin = 0.0;
double positionMax = 1.0;
double position = 0.0;

// Stepper Driver

TMC2209 motorDriver;
HardwareSerial &motorSerial = Serial2;
bool motorState = false;
bool motorEnabled = true;
bool motorOverTemp = false;
bool motorOverTempShutdown = false;
bool motorStalled = false;
unsigned int motorStallGuard = 0;
unsigned int count = 0;
VOLTAGE voltage = VOLTAGE_5V;
int current = 30;
int holdCurrent = 30;
int holdDelay = 0;
MICROSTEPS microstepsPerStep = MICROSTEPS_32;
STANDSTILL standstillMode = NORMAL;
int stallThreshold = 70;
int coolStepDurationThreshold = 5000;

// Power Delivery

bool powerGood = false;
double busVoltage = 0;

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
void initController();
void controller(void *pvParameters);
void enableCommand(bool enabled);
void positionCommand(double position);
void velocityCommand(int velocity);
void settingsCommand(const Settings& settings);
void statusFeedback(Status& status);
void positionFeedback(PositionFeedback& position);
void velocityFeedback(int& velocity);
void settingsFeedback(Settings& settings);

//
// Functions
//

void setup()
{
  delay(STARTUP_DELAY_MS);

  Serial.begin(115200);

  readSettings();

  initPower();
  initMotor();
  initEncoder();
  initBoard();
  initController();

  initRestInterface(
    "NETWORK",
    "PASSWORD",
    8080,
    enableCommand,
    positionCommand,
    velocityCommand,
    settingsCommand,
    statusFeedback,
    positionFeedback,
    velocityFeedback,
    settingsFeedback
  );
}

void loop()
{
  readPower();
  readBoard();
  writeBoard();
}

void initController()
{
  xTaskCreate(controller, "MotorControl", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}

bool manualControl()
{
  if (incrementButtonPushed)
  {
    commandMode = MANUAL;
    commandedVelocity = buttonVelocity;
    enabled = true;

    writeMotorEnabled(true);
    writeMotorVelocity(commandedVelocity);

    return true;
  }
  else if (decrementButtonPushed)
  {
    commandMode = MANUAL;
    commandedVelocity = -buttonVelocity;
    enabled = true;

    writeMotorEnabled(true);
    writeMotorVelocity(commandedVelocity);

    return true;
  }
  else if (resetButtonPushed)
  {
    commandMode = MANUAL;
    commandedVelocity = 0;
    enabled = true;

    writeMotorEnabled(true);
    writeMotorVelocity(0);

    return true;
  }

  return false;
}

bool powerEnabled()
{
  if (!powerGood && motorEnabled)
  {
    enabled = false;
    writeMotorEnabled(false);
    return false;
  }

  return true;
}

void controller(void *pvParameters)
{
  const TickType_t freq = pdMS_TO_TICKS(TIMESTEP_MS);
  TickType_t lastTime = xTaskGetTickCount();

  while (true)
  {
    readEncoder();
    readMotor();

    if (!powerEnabled() || manualControl() || !enabled)
    {
      continue;
    }

    if (enabled != motorEnabled)
    {
      writeMotorEnabled(enabled);
    }

    if (commandMode == VELOCITY)
    {
      writeMotorVelocity(commandedVelocity);
    }
    else if (commandMode == POSITION)
    {
      // Calculate proportional error
      double error = commandedPosition - position;

      // Stop if within tolerance
      if (abs(error) <= tolerance)
      {
        if (commandedVelocity != 0)
        {
          proportionalError = error;
          commandedVelocity = 0;
          writeMotorVelocity(0);
        }

        continue;
      }

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
      double proportional = Kp * proportionalError;

      // Calculate integral contribution
      double integral = Ki * integralError;

      if (limitIntegralError)
      {
        integral = constrain(integral, iMin, iMax);
      }

      // Calculate derivative contribution
      double derivative = Kd * derivativeError;

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
  busVoltage = analogRead(PD_VBUS) * PD_VOLTAGE_MULTIPLIER;
  powerGood = digitalRead(PD_POWERGOOD) == LOW;
}

void initBoard()
{
  pinMode(BRD_SW1, INPUT);
  pinMode(BRD_SW2, INPUT);
  pinMode(BRD_SW3, INPUT);
  pinMode(BRD_LED1, OUTPUT);
  pinMode(BRD_LED2, OUTPUT);

  // Flash LED after setup complete
  digitalWrite(BRD_LED1, HIGH);
  delay(500);
  digitalWrite(BRD_LED1, LOW);
}

void readBoard()
{
  if ((millis() - lastDebounceTime) > BUTTON_DEBOUNCE_MS)
  {
    lastDebounceTime = millis();
    incrementButtonPushed = !digitalRead(BRD_SW3);
    decrementButtonPushed = !digitalRead(BRD_SW1);
    resetButtonPushed = !digitalRead(BRD_SW2);
  }
}

void writeBoard()
{
  digitalWrite(BRD_LED2, motorStalled);
}

void initMotor()
{
  // https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf
  // https://github.com/janelia-arduino/TMC2209

  // Configure ESP pins used to communicate with motor driver
  pinMode(TMC_STEP, OUTPUT);
  pinMode(TMC_DIR, OUTPUT);
  pinMode(TMC_MS1, OUTPUT);
  pinMode(TMC_MS1, OUTPUT);
  pinMode(TMC_ENABLED, OUTPUT);
  pinMode(TMC_OVERCURRENT, INPUT);

  digitalWrite(TMC_ENABLED, LOW);
  digitalWrite(TMC_MS1, LOW);
  digitalWrite(TMC_MS2, LOW);

  // Initialize motor driver
  motorDriver.setup(motorSerial, TMC_SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  motorDriver.setHardwareEnablePin(TMC_ENABLED);

  if (controlMode == CURRENT_CONTROL)
  {
    motorDriver.enableAutomaticCurrentScaling();
  }
  else
  {
    motorDriver.disableAutomaticCurrentScaling();
  }

  motorDriver.setRunCurrent(current);
  motorDriver.setMicrostepsPerStep((int)microstepsPerStep);
  motorDriver.setStallGuardThreshold(stallThreshold);
  motorDriver.setStandstillMode((TMC2209::StandstillMode)standstillMode);
  motorDriver.enableStealthChop();
  motorDriver.setCoolStepDurationThreshold(coolStepDurationThreshold);
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
  motorDriver.moveAtVelocity(velocity * microstepsPerStep);
}

void readMotor()
{
  motorEnabled = !motorDriver.hardwareDisabled();
  motorStalled = digitalRead(TMC_OVERCURRENT);
  motorStallGuard = motorDriver.getStallGuardResult();
  count = motorDriver.getMicrostepCounter();

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
    case VOLTAGE_5V:
      {
        digitalWrite(PD_CFG1, HIGH);
      }
      break;
    case VOLTAGE_9V:
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, LOW);
        digitalWrite(PD_CFG3, LOW);
      }
      break;
    case VOLTAGE_12V:
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, LOW);
        digitalWrite(PD_CFG3, HIGH);
      }
      break;
    case VOLTAGE_15V:
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, HIGH);
        digitalWrite(PD_CFG3, HIGH);
      }
      break;
    case VOLTAGE_20V:
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, HIGH);
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

  int hi = Wire.read();
  int lo = Wire.read();

  // Convert
  int reading = hi << 8 | lo;

  // Keep track of revolutions
  if (rawPosition != -1 && abs(rawPosition - reading) > AS5600_MAX / 2)
  {
    if (commandedVelocity > 0)
    {
      revolutions++;
    }
    else if (commandedVelocity < 0)
    {
      revolutions--;
    }
  }

  rawPosition = reading;
  rawPositionWithRevolutions = rawPosition + AS5600_MAX * revolutions;

  // Calculate normalized position
  double norm = 
    double(constrain(rawPositionWithRevolutions, encoderMin, encoderMax) - encoderMin)
    / double(encoderMax - encoderMin);

  // Scale to final position range
  position = positionMin + norm * (positionMax - positionMin);
}

void readSettings()
{
  preferences.begin("settings", false);
  name = preferences.getString("name", name);
  controlMode = (CONTROL_MODE)preferences.getInt("controlMode", (int)controlMode);
  voltage = (VOLTAGE)preferences.getInt("voltage", voltage);
  current = preferences.getInt("current", current);
  holdCurrent = preferences.getInt("holdCurrent", holdCurrent);
  holdDelay = preferences.getInt("holdDelay", holdDelay);
  microstepsPerStep = (MICROSTEPS)preferences.getInt("microstepsPerStep", (int)microstepsPerStep);
  stallThreshold = preferences.getInt("stallThreshold", stallThreshold);
  standstillMode = (STANDSTILL)preferences.getInt("standstillMode", (int)standstillMode);
  coolStepDurationThreshold = preferences.getInt("coolStepDurationThreshold", coolStepDurationThreshold);
  buttonVelocity = preferences.getInt("buttonVelocity", buttonVelocity);
  encoderMin = preferences.getInt("encoderMin", encoderMin);
  encoderMax = preferences.getInt("encoderMax", encoderMax);
  positionMin = preferences.getDouble("positionMin", positionMin);
  positionMax = preferences.getDouble("positionMax", positionMax);
  velocityMin = preferences.getInt("velocityMin", velocityMin);
  velocityMax = preferences.getInt("velocityMax", velocityMax);
  Kp = preferences.getDouble("Kp", Kp);
  Ki = preferences.getDouble("Ki", Ki);
  Kd = preferences.getDouble("Id", Kd);
  iMin = preferences.getDouble("iMin", iMin);
  iMax = preferences.getDouble("iMax", iMax);
  tolerance = preferences.getDouble("tolerance", tolerance);
  preferences.end();
}

void writeSettings()
{
  preferences.begin("settings", false);
  preferences.putString("name", name.c_str());
  preferences.putInt("controlMode", (int)controlMode);
  preferences.putInt("voltage", (int)voltage);
  preferences.putInt("current", current);
  preferences.putInt("holdCurrent", holdCurrent);
  preferences.putInt("holdDelay", holdDelay);
  preferences.putInt("microstepsPerStep", microstepsPerStep);
  preferences.putInt("stallThreshold", stallThreshold);
  preferences.putInt("standstillMode", (int)standstillMode);
  preferences.putInt("coolStepDurationThreshold", coolStepDurationThreshold);
  preferences.putInt("buttonVelocity", buttonVelocity);
  preferences.putInt("encoderMin", encoderMin);
  preferences.putInt("encoderMax", encoderMax);
  preferences.putDouble("positionMin", positionMin);
  preferences.putDouble("positionMax", positionMax);
  preferences.putInt("velocityMin", velocityMin);
  preferences.putInt("velocityMax", velocityMax);
  preferences.putDouble("Kp", Kp);
  preferences.putDouble("Ki", Ki);
  preferences.putDouble("Id", Kd);
  preferences.putDouble("iMin", iMin);
  preferences.putDouble("iMax", iMax);
  preferences.putDouble("tolerance", tolerance);
  preferences.end();
}

void enableCommand(bool command)
{
  enabled = command;
}

void positionCommand(double command)
{
  commandMode = POSITION;
  commandedPosition = command;
  enabled = true;
}

void velocityCommand(int command)
{
  commandMode = VELOCITY;
  commandedVelocity = command;
  enabled = true;
}

void settingsCommand(const Settings& settings)
{
  // Reset motor
  commandMode = MANUAL;
  commandedVelocity = 0;
  writeMotorVelocity(0);

  // Apply settings
  if (name != settings.name)
  {
    name = settings.name;
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

  if (controlMode != settings.controlMode)
  {
    controlMode = settings.controlMode;

    if (controlMode == CURRENT_CONTROL)
    {
      motorDriver.enableAutomaticCurrentScaling();
    }
    else
    {
      motorDriver.disableAutomaticCurrentScaling();
    }
  }

  if (holdCurrent != settings.holdCurrent)
  {
    holdCurrent = settings.holdCurrent;
    motorDriver.setHoldCurrent(holdCurrent);
  }

  if (holdDelay != settings.holdDelay)
  {
    holdDelay = settings.holdDelay;
    motorDriver.setHoldDelay(holdDelay);
  }

  if (microstepsPerStep != settings.microstepsPerStep)
  {
    microstepsPerStep = settings.microstepsPerStep;
    motorDriver.setMicrostepsPerStep((int)microstepsPerStep);
  }

  if (stallThreshold != settings.stallThreshold)
  {
    stallThreshold = settings.stallThreshold;
    motorDriver.setStallGuardThreshold(stallThreshold);
  }

  if (standstillMode != settings.standstillMode)
  {
    standstillMode = settings.standstillMode;
    motorDriver.setStandstillMode((TMC2209::StandstillMode)standstillMode);
  }

  if (coolStepDurationThreshold != settings.coolStepDurationThreshold)
  {
    coolStepDurationThreshold = settings.coolStepDurationThreshold;
    motorDriver.setCoolStepDurationThreshold(coolStepDurationThreshold);
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
  status.name = name.c_str();
  status.commandMode = commandMode;
  status.controlMode = controlMode;
  status.enabled = motorEnabled;
  status.powerGood = powerGood;
  status.count = count;
  status.rawPosition = rawPositionWithRevolutions;
  status.position = position;
  status.velocity = commandedVelocity;
  status.voltage = busVoltage;
  status.current = current;
  status.overTemp = motorOverTemp;
  status.overTempShutdown = motorOverTempShutdown;
  status.stalled = motorStalled;
  status.stallGuard = motorStallGuard;
}

void positionFeedback(PositionFeedback& feedback)
{
  feedback.goal = commandedPosition;
  feedback.position = position;
  feedback.error = proportionalError;
  feedback.integralError = integralError;
  feedback.derivativeError = derivativeError;
}

void velocityFeedback(int& velocity)
{
  velocity = commandedVelocity;
}

void settingsFeedback(Settings& settings)
{
  settings.name = name.c_str();
  settings.controlMode = controlMode;
  settings.voltage = voltage;
  settings.current = current;
  settings.holdCurrent = holdCurrent;
  settings.holdDelay = holdDelay;
  settings.microstepsPerStep = microstepsPerStep;
  settings.stallThreshold = stallThreshold;
  settings.standstillMode = standstillMode;
  settings.buttonVelocity = buttonVelocity;
  settings.coolStepDurationThreshold = coolStepDurationThreshold;
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
