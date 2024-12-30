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

// Wifi

#include "esp_wifi.h"
#include "esp_event.h"

// Web Server

#include <ESPAsyncWebServer.h> // https://github.com/me-no-dev/ESPAsyncWebServer
#include <AsyncTCP.h>          // https://github.com/me-no-dev/AsyncTCP

// Logging

#include "esp_log.h"

// TMC2209 Stepper Motor Driver

#include <TMC2209.h>           // https://github.com/janelia-arduino/TMC2209/tree/main

// AS5600 Hall Effect Encoder

#include <Wire.h>

// Flash memory

#include <Preferences.h>

//
// Constants
//

// Controller

const char* APPNAME = "PDStepperServo";
const int RATE_HZ = 50;
const int TIMESTEP_MS = 1000 / RATE_HZ;
const float TIMESTEP = 1.0 / RATE_HZ;
const int DEBOUNCE_MS = 50;
const int STARTUP_DELAY_MS = 200;

enum MODE
{
  POSITION,
  VELOCITY,
  MANUAL
}

// Encoder

const int AS5600_ADDRESS = 0x36;
const int AS5600_ANGLE_REGISTER = 0x0C;
const float AS5600_MAX = 4096;

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

enum VOLTAGE
{
  5V = 5,
  9V = 9,
  12V = 12,
  15V = 15,
  20V = 20
};

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

// Wifi

String ssid = "your-ssid";
String password = "your-password";
int port = 8080;
EventGroupHandle_t wifiEventGroup;
AsyncWebServer* pServer;

// State

bool enabled = 0;
VOLTAGE voltage = 5V;
int current = 100;
int microsteps = 32;
int stallThreshold = 10;
TMC2209::StandstillMode standstillMode = NORMAL;
MODE mode = MANUAL;
int velocityCommand = 0;
float positionCommand = 0;
float Kp = 100;
float Ki = 10;
float Kd = 10;
float iMin = -10;
float iMax = 10;
float proportionalError = 0;
float integralError = 0;
float derivativeError = 0;
int buttonVelocity = 100;

// Encoder

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
float reportedVoltage = 0;

// Buttons

bool incrementButtonPushed = false;
bool decrementButtonPushed = false;
bool resetButtonPushed = false;
unsigned long lastDebounceTime = 0;

//
// Forward Declarations
//

void initWifi();
void wifiEvent(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void initServer();
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

//
// Entry point
//

void setup()
{
  // Ensure bootloader finishes
  delay(STARTUP_DELAY_MS);

  // Setup
  readSettings();
  initWifi(ssid, password);
  initBoard();
  initPower();
  initMotor();
  initEncoder();
  initMotorControl();

  // Flash LED after setup complete
  digitalWrite(LED1, HIGH);
  delay(STARTUP_DELAY_MS);
  digitalWrite(LED1, LOW);
}

void loop()
{
  // Server runs itself
}

//
// RT thread
//

void initMotorControl()
{
  xTaskCreate(runMotorControl, "MotorControl", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
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
      writeMotorVelocity(velocityCommand);
    }
    else if (mode == POSITION)
    {
      // Calculate proportional error
      float error = positionCommand - position;

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
      velocityCommand = int(proportional + integral + derivative);
      writeMotorVelocity(velocityCommand);
    }

    writeBoard();

    // Sleep
    vTaskDelayUntil(&lastTime, freq);
  }
}

//
// Helpers
//

static void initWifi()
{
  wifiEventGroup = xEventGroupCreate();

  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  initWifi_config_t cfg = initWifi_CONFIG_DEFAULT();
  esp_initWifi(&cfg);

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;

  esp_event_handler_instance_register(
    WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEvent, NULL, &instance_any_id);
  esp_event_handler_instance_register(
    IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEvent, NULL, &instance_got_ip);

  wifi_config_t wifi_config =
  {
    .sta =
    {
      .ssid = ssid,
      .password = password,
    }
  };

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();

  xEventGroupWaitBits(wifiEventGroup, BIT0, pdFALSE, pdTRUE, portMAX_DELAY);
}

void wifiEvent(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    ESP_LOGI(APPNAME, "Disconnected from Wi-Fi. Reconnecting...");
    esp_wifi_connect();
    xEventGroupClearBits(wifiEventGroup, BIT0);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(APPNAME, "Got IP: %s", ip4addr_ntoa(&event->ip_info.ip));
    xEventGroupSetBits(wifiEventGroup, BIT0);
  }
}

void initServer()
{
  pServer = new AsyncWebServer(port);

  pServer->on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
    // TODO: return settings
    request->send_P(200, "application/json", responseStringTodo);
  });

  pServer->on("/velocity", HTTP_GET, [](AsyncWebServerRequest *request) {
    // TODO: velocity feedback
    request->send_P(200, "application/json", responseStringTodo);
  });

  pServer->on("/position", HTTP_GET, [](AsyncWebServerRequest *request) {
    // TODO: position feedback
    request->send_P(200, "application/json", responseStringTodo);
  });

  pServer->on("/settings", HTTP_POST, [](AsyncWebServerRequest *request) {
    // TODO: change and save settings, reinit
  });

  pServer->on("/enabled", HTTP_POST, [](AsyncWebServerRequest *request) {
    // TODO: enabled command
  });

  pServer->on("/position", HTTP_POST, [](AsyncWebServerRequest *request) {
    // TODO: position command
  });

  pServer->on("/velocity", HTTP_POST, [](AsyncWebServerRequest *request) {
    // TODO: velocity command
  });

  pServer->begin();
}

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
  reportedVoltage = analogRead(PD_VBUS) * (PD_VREF / 4096.0) / PD_DIV;
  powerGood = digitalRead(PD_POWERGOOD) == 0;
}

void initBoard()
{
  pinMode(BRD_SW1, INPUT);
  pinMode(BRD_SW2, INPUT);
  pinMode(BRD_SW3, INPUT);
  pinMode(BRD_LED1, OUTPUT);
  pinMode(BRD_LED2, OUTPUT);
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
    case 9V
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, LOW);
        digitalWrite(PD_CFG3, LOW);
      }
      break;
    case 12V
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, LOW);
        digitalWrite(PD_CFG3, HIGH);
      }
      break;
    case 15V
      {
        digitalWrite(PD_CFG1, LOW);
        digitalWrite(PD_CFG2, HIGH)
        digitalWrite(PD_CFG3, HIGH);
      }
      break;
    case 20V
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
  int angle = 0;
  Wire.requestFrom(AS5600_ADDRESS, 2);

  if (Wire.available() >= 2)
  {
    // Combine two bytes to get the counts value
    angle = Wire.read() << 8 | Wire.read();
  }

  position = angle / AS5600_MAX;
}

void readSettings()
{
  preferences.begin("settings", false);
  ssid = preferences.getString("ssid", ssid);
  password = preferences.getString("password", password);
  port = preferences.getInt("port", port);
  voltage = (VOLTAGE)preferences.getInt("voltage", voltage);
  current = preferences.getInt("current", current);
  microsteps = preferences.getInt("microsteps", microsteps);
  stallThreshold = preferences.getInt("stallThreshold", stallThreshold);
  standstillMode = (TMC2209::StandstillMode)preferences.getInt("standstillMode", (int)standstillMode);
  Kp = preferences.getFloat("Kp", Kp);
  Ki = preferences.getFloat("Ki", Ki);
  Kd = preferences.getFloat("Id", Kd);
  iMin = preferences.getFloat("iMin", iMin);
  iMax = preferences.getFloat("iMax", iMax);
  buttonVelocity = preferences.getInt("buttonVelocity", buttonVelocity);
  preferences.end();
}

void writeSettings()
{
  preferences.begin("settings", false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.putInt("port", port);
  preferences.putInt("voltage", (int)voltage);
  preferences.putInt("current", current);
  preferences.putInt("microsteps", microsteps);
  preferences.putInt("stallThreshold", stallThreshold);
  preferences.putInt("standstillMode", (int)standstillMode);
  preferences.putFloat("Kp", Kp);
  preferences.putFloat("Ki", Ki);
  preferences.putFloat("Id", Kd);
  preferences.putFloat("iMin", iMin);
  preferences.putFloat("iMax", iMax);
  preferences.putInt("buttonVelocity", buttonVelocity);
  preferences.end();
}
