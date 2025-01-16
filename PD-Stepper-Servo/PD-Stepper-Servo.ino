/*
* PD Stepper Servo Controller
* Copyright (C) 2025 Ctrl^H Hackerspace
*/

//
// Includes
//

#include "freertos/FreeRTOS.h"            // Multi-threading
#include <TMC2209.h>                      // TMC2209 Stepper Motor Driver
#include <Wire.h>                         // AS5600 Hall Effect Encoder
#include <Preferences.h>                  // Flash memory
#include "Interface.h"                    // Enable switching interfaces
#include "RestInterface.h"                // REST API interface

//
// Constants
//

// Controller

const int RATE_HZ = 50;                   // Frequency
const int TIMESTEP_MS = 1000 / RATE_HZ;   // Time step in ms
const double TIMESTEP = 1.0 / RATE_HZ;    // Time step in seconds
const int BUTTON_DEBOUNCE_MS = 50;        // Button debounce time ms
const int STARTUP_DELAY_MS = 200;         // Startup delay time ms

// Encoder

const int AS5600_ADDRESS = 0x36;          // Encoder I2C address
const int AS5600_ANGLE_REGISTER = 0x0C;   // Encoder angle register
const int AS5600_MAX = 4096;              // Encoder max angle value

// Stepper driver

const long TMC_RATE = 115200;             // Serial baud rate
const int TMC_ENABLED = 21;               // Enabled pin
const int TMC_STEP = 5;                   // Step pin
const int TMC_DIR = 6;                    // Direction pin
const int TMC_MS1 = 1;                    // Address select pin 1
const int TMC_MS2 = 2;                    // Address select pin 2
const int TMC_TX = 17;                    // UART TX pin
const int TMC_RX = 18;                    // UART RX pin
const int TMC_OVERCURRENT = 16;           // Overcurrent pin

// USB Power Delivery

const int PD_POWERGOOD = 15;              // Power good pin
const int PD_VBUS = 4;                    // Voltage analog pin
const int PD_CFG1 = 38;                   // Voltage select pin 1
const int PD_CFG2 = 48;                   // Voltage select pin 2
const int PD_CFG3 = 47;                   // Voltage select pin 3
const double PD_VOLTAGE_MUL = 6.7735e-03; // Voltage conversion multiplier

// PD Stepper Board

const int BRD_LED1 = 10;                  // Info/Warning LED
const int BRD_LED2 = 12;                  // Error LED
const int BRD_SW1 = 35;                   // Decrement button
const int BRD_SW2 = 36;                   // Stop button
const int BRD_SW3 = 37;                   // Increment button

//
// Variables
//

// Controller State

COMMAND commandMode = MANUAL;             // Manual, velocity or position
bool enabled = false;                     // Enable motor
int commandedVelocity = 0;                // Velocity command
double commandedPosition = 0;             // Position command (also sets Velocity command)
double proportionalError = 0;             // PID Proportional error
double integralError = 0;                 // PID Integral error
double derivativeError = 0;               // PID Derivative error
double proportional = 0;                  // PID Proportional component
double integral = 0;                      // PID Integral component
double derivative = 0;                    // PID Derivative component
bool incrementButtonPushed = false;       // Increment button state
bool decrementButtonPushed = false;       // Decrement button state
bool resetButtonPushed = false;           // Stop button state
unsigned long lastDebounceTime = 0;       // Last button debounce time

// Controller Settings

Preferences preferences;                  // Used to save settings to flash
String name = "PD-Stepper";               // Controller name, since several can be connected
double tolerance = 0.05;                  // Position command success tolerance
double Kp = 300;                          // PID Proportional gain
double Ki = 10;                           // PID Integral gain
double Kd = 10;                           // PID Derivative gain
double iMin = -100;                       // PID Integral min
double iMax = 100;                        // PID Integral max
int velocityMin = 10;                     // PID min velocity
int velocityMax = 1440;                   // PID max velocity
int buttonVelocity = 30;                  // Manual control velocity
bool enableLogging = true;                // Log messages over Serial port

// Encoder State

int rawPosition = -1;                     // Raw encoder position
int revolutions = 0;                      // Revolutions from home
double position = 0.0;                    // Scaled encoder position

// Encoder Mapping
// raw           normalized                   scaled
// [0...4096] -> [encoderMin...encoderMax] -> [positionMin...positionMax]

int encoderMin = 0;                       // Normalized position min
int encoderMax = AS5600_MAX;              // Normalized position max
double positionMin = 0.0;                 // Scaled position min
double positionMax = 1.0;                 // Scaled position max
bool positionInvert = true;               // Invert position from encoder

// Stepper State

TMC2209 motorDriver;                      // Driver interface
HardwareSerial &motorSerial = Serial2;    // Serial port for communication
bool motorEnabled = true;                 // Motor enabled
bool motorOverTemp = false;               // Motor overheated
bool motorOverTempShutdown = false;       // Motor shutdown
bool motorStalled = false;                // Motor stalled
unsigned int motorStallGuard = 0;         // Recommended stall threshold
unsigned int count = 0;                   // Microstep counter

// Stepper Settings

CONTROL controlMode = CURRENT_CONTROL;    // Driver PWM control mode
int current = 30;                         // Run current % if current controlled
int holdCurrent = 30;                     // Hold current % if current controlled
int holdDelay = 0;                        // Hold delay cycles
MICROSTEPS microsteps = MICROSTEPS_32;    // Velocity resolution
STANDSTILL standstillMode = NORMAL;       // What happens when velocity set to zero
int stallThreshold = 80;                  // Stall threshold
int coolStepDurationThreshold = 5000;     // CoolStep activation threshold

// Power Delivery

VOLTAGE voltage = VOLTAGE_5V;             // Voltage to negotiate
bool powerGood = false;                   // Configured voltage available
double busVoltage = 0;                    // Actual voltage in volts

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
void resetMotor();
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

  if (enableLogging)
  {
    Serial.begin(115200);
  }

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
    enableLogging,
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

// Manual control via buttons
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

// Enable driver when power delivery configured
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

void logPosition(double error)
{
  Serial.print("goal:");
  Serial.print(commandedPosition);
  Serial.print(",");
  Serial.print("position:");
  Serial.print(position);
  Serial.print(",");
  Serial.print("velocity:");
  Serial.print(double(commandedVelocity) / double(velocityMax));
  Serial.print(",");
  Serial.print("error:");
  Serial.print(error);
  Serial.print(",");
  Serial.print("integralError:");
  Serial.print(integralError);
  Serial.print(",");
  Serial.print("derivativeError:");
  Serial.println(derivativeError);
}

void logVelocity()
{
  Serial.print("position:");
  Serial.print(position);
  Serial.print(",");
  Serial.print("velocity:");
  Serial.println(double(commandedVelocity) / double(velocityMax));
}

// Velocity and Position control
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

      if (enableLogging)
      {
        logVelocity();
      }
    }
    else if (commandMode == POSITION)
    {
      // Calculate proportional error
      double error = commandedPosition - position;

      // Stop if within tolerance
      if (abs(error) <= tolerance)
      {
        if (enableLogging)
        {
          logPosition(error);
        }

        resetMotor();
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
      proportional = Kp * proportionalError;

      // Calculate integral contribution
      integral = Ki * integralError;

      if (limitIntegralError)
      {
        integral = constrain(integral, iMin, iMax);
      }

      // Calculate derivative contribution
      derivative = Kd * derivativeError;

      // Calculate command
      int command = int(proportional + integral + derivative);
      int speed = abs(command);
      int direction = command >= 0 ? 1 : -1;

      // Limit magnitude
      if (speed != 0)
      {
        if (speed < velocityMin)
        {
          speed = velocityMin;
        }
        else if (speed > velocityMax)
        {
          speed = velocityMax;
        }
      }

      int limitedVelocity = direction * speed;

      if (commandedVelocity != limitedVelocity)
      {
        commandedVelocity = limitedVelocity;
        writeMotorVelocity(commandedVelocity);
      }

      if (enableLogging)
      {
        logPosition(error);
      }
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
  busVoltage = analogRead(PD_VBUS) * PD_VOLTAGE_MUL;
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
  motorDriver.setup(motorSerial, TMC_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  motorDriver.setHardwareEnablePin(TMC_ENABLED);

  if (controlMode == CURRENT_CONTROL)
  {
    motorDriver.enableAutomaticCurrentScaling();
    motorDriver.setRunCurrent(current);
  }
  else
  {
    motorDriver.disableAutomaticCurrentScaling();
  }

  motorDriver.setMicrostepsPerStep((int)microsteps);
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
  motorDriver.moveAtVelocity(velocity * microsteps);
}

void resetMotor()
{
  commandMode = MANUAL;
  proportionalError = 0.0;
  integralError = 0.0;
  derivativeError = 0.0;
  proportional = 0.0;
  integral = 0.0;
  derivative = 0.0;
  commandedVelocity = 0;

  writeMotorVelocity(0);
}

void readMotor()
{
  // Motor enabled
  motorEnabled = !motorDriver.hardwareDisabled();

  // Stall detection via current sensing
  motorStalled = digitalRead(TMC_OVERCURRENT);

  // Recommended stall threshold at current velocity
  motorStallGuard = motorDriver.getStallGuardResult();

  // Internal microstep counter (acts like built-in quadrature encoder)
  count = motorDriver.getMicrostepCounter();

  // Motor status
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

  // Calculate normalized position
  int clamped = constrain(rawPosition, encoderMin, encoderMax);
  double normalized = double(clamped - encoderMin) / double(encoderMax - encoderMin);

  // Invert
  if (positionInvert)
  {
    normalized = 1.0 - normalized;
  }

  // Scale to final position range
  position = positionMin + normalized * (positionMax - positionMin);
}

void readSettings()
{
  preferences.begin("settings", false);
  name = preferences.getString("name", name);
  controlMode = (CONTROL)preferences.getInt("controlMode", (int)controlMode);
  voltage = (VOLTAGE)preferences.getInt("voltage", voltage);
  current = preferences.getInt("current", current);
  holdCurrent = preferences.getInt("holdCurrent", holdCurrent);
  holdDelay = preferences.getInt("holdDelay", holdDelay);
  microsteps = (MICROSTEPS)preferences.getInt("microstepsPerStep", (int)microsteps);
  stallThreshold = preferences.getInt("stallThreshold", stallThreshold);
  standstillMode = (STANDSTILL)preferences.getInt("standstillMode", (int)standstillMode);
  coolStepDurationThreshold = preferences.getInt("coolStepDurationThreshold", coolStepDurationThreshold);
  buttonVelocity = preferences.getInt("buttonVelocity", buttonVelocity);
  enableLogging = preferences.getBool("enableLogging", enableLogging);
  encoderMin = preferences.getInt("encoderMin", encoderMin);
  encoderMax = preferences.getInt("encoderMax", encoderMax);
  positionMin = preferences.getDouble("positionMin", positionMin);
  positionMax = preferences.getDouble("positionMax", positionMax);
  positionInvert = preferences.getBool("positionInvert", positionInvert);
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
  preferences.putInt("microstepsPerStep", microsteps);
  preferences.putInt("stallThreshold", stallThreshold);
  preferences.putInt("standstillMode", (int)standstillMode);
  preferences.putInt("coolStepDurationThreshold", coolStepDurationThreshold);
  preferences.putInt("buttonVelocity", buttonVelocity);
  preferences.putBool("enableLogging", enableLogging);
  preferences.putInt("encoderMin", encoderMin);
  preferences.putInt("encoderMax", encoderMax);
  preferences.putDouble("positionMin", positionMin);
  preferences.putDouble("positionMax", positionMax);
  preferences.putBool("positionInvert", positionInvert);
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
  writeMotorEnabled(enabled);

  if (!enabled)
  {
    resetMotor();
  }
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
  // Stop motor
  resetMotor();

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

  if (microsteps != settings.microstepsPerStep)
  {
    microsteps = settings.microstepsPerStep;
    motorDriver.setMicrostepsPerStep((int)microsteps);
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
  enableLogging = settings.enableLogging;
  encoderMin = settings.encoderMin;
  encoderMax = settings.encoderMax;
  positionMin = settings.positionMin;
  positionMax = settings.positionMax;
  positionInvert = settings.positionInvert;
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
  status.rawPosition = rawPosition;
  status.revolutions = revolutions;
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
  feedback.tolerance = tolerance;
  feedback.integralError = integralError;
  feedback.derivativeError = derivativeError;
  feedback.proportional = proportional;
  feedback.integral = integral;
  feedback.derivative = derivative;
  feedback.command = commandedVelocity;
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
  settings.microstepsPerStep = microsteps;
  settings.stallThreshold = stallThreshold;
  settings.standstillMode = standstillMode;
  settings.buttonVelocity = buttonVelocity;
  settings.enableLogging = enableLogging;
  settings.coolStepDurationThreshold = coolStepDurationThreshold;
  settings.encoderMin = encoderMin;
  settings.encoderMax = encoderMax;
  settings.positionMin = positionMin;
  settings.positionMax = positionMax;
  settings.positionInvert = positionInvert;
  settings.velocityMin = velocityMin;
  settings.velocityMax = velocityMax;
  settings.Kp = Kp;
  settings.Ki = Ki;
  settings.Kd = Kd;
  settings.iMin = iMin;
  settings.iMax = iMax;
  settings.tolerance = tolerance;
}
