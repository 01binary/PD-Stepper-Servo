/*
* PD Stepper Servo Controller
* REST API interface
* Copyright (C) 2025 Ctrl^H Hackerspace
*/

#pragma once

//
// Includes
//

#include <ESPAsyncWebServer.h> // https://github.com/me-no-dev/ESPAsyncWebServer
#include <AsyncTCP.h>          // https://github.com/me-no-dev/AsyncTCP
#include <ArduinoJson.h>       // https://github.com/bblanchon/ArduinoJson
#include <esp_wifi.h>
#include "Interface.h"

//
// Definitions
//

#define DEFAULT_HANDLER [](AsyncWebServerRequest* request) {}

//
// Constants
//

const char* const COMMAND_DESCRIPTION[] = {
  "manual",
  "velocity",
  "position"
};

const char* const CONTROL_DESCRIPTION[] = {
  "voltage",
  "current"
};

const char* const STANDSTILL_MODE_DESCRIPTION[] = {
  "normal",
  "freewheeling",
  "strongBraking",
  "braking"
};

//
// Variables
//

AsyncWebServer* pServer = NULL;

//
// Forward Declarations
//

void deserializeRequestJson(JsonDocument& doc, uint8_t* data, size_t len);
int deserializeEnum(const char* value, const char* const* values, int count);

//
// Functions
//

void initRestInterface(
  const char* ssid,
  const char* password,
  int port,
  EnableCommandPtr enableCommand,
  PositionCommandPtr positionCommand,
  VelocityCommandPtr velocityCommand,
  SettingsCommandPtr settingsCommand,
  StatusFeedbackPtr statusFeedback,
  PositionFeedbackPtr positionFeedback,
  VelocityFeedbackPtr velocityFeedback,
  SettingsFeedbackPtr settingsFeedback)
{
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(500);
  }

  Serial.print("Connected as ");
  Serial.println(WiFi.localIP());

  // Create server
  pServer = new AsyncWebServer(port);

  // Status feedback
  pServer->on("/status", HTTP_GET, [statusFeedback](AsyncWebServerRequest *request)
  {
    Status status;
    statusFeedback(status);

    JsonDocument doc;
    doc["name"] = status.name;
    doc["commandMode"] = COMMAND_DESCRIPTION[status.commandMode];
    doc["controlMode"] = CONTROL_DESCRIPTION[status.controlMode];
    doc["enabled"] = status.enabled;
    doc["powerGood"] = status.powerGood;
    doc["count"] = status.count;
    doc["rawPosition"] = status.rawPosition;
    doc["revolutions"] = status.revolutions;
    doc["position"] = status.position;
    doc["velocity"] = status.velocity;
    doc["voltage"] = status.voltage;
    doc["current"] = status.current;
    doc["overTemp"] = status.overTemp;
    doc["overTempShutdown"] = status.overTempShutdown;
    doc["stallGuard"] = status.stallGuard;
    doc["stalled"] = status.stalled;

    String res;
    serializeJson(doc, res);

    request->send_P(200, "application/json", res.c_str());
  });

  // Settings feedback
  pServer->on("/settings", HTTP_GET, [settingsFeedback](AsyncWebServerRequest *request)
  {
    Settings settings;
    settingsFeedback(settings);

    JsonDocument doc;
    doc["name"] = settings.name;
    doc["controlMode"] = CONTROL_DESCRIPTION[settings.controlMode];
    doc["standstillMode"] = STANDSTILL_MODE_DESCRIPTION[settings.standstillMode];
    doc["voltage"] = settings.voltage;
    doc["current"] = settings.current;
    doc["holdCurrent"] = settings.holdCurrent;
    doc["holdDelay"] = settings.holdDelay;
    doc["microstepsPerStep"] = settings.microstepsPerStep;
    doc["stallThreshold"] = settings.stallThreshold;
    doc["coolStepDurationThreshold"] = settings.coolStepDurationThreshold;
    doc["buttonVelocity"] = settings.buttonVelocity;
    doc["encoderMin"] = settings.encoderMin;
    doc["encoderMax"] = settings.encoderMax;
    doc["positionMin"] = settings.positionMin;
    doc["positionMax"] = settings.positionMax;
    doc["velocityMin"] = settings.velocityMin;
    doc["velocityMax"] = settings.velocityMax;
    doc["Kp"] = settings.Kp;
    doc["Ki"] = settings.Ki;
    doc["Kd"] = settings.Kd;
    doc["iMin"] = settings.iMin;
    doc["iMax"] = settings.iMax;
    doc["tolerance"] = settings.tolerance;

    String res;
    serializeJson(doc, res);

    request->send_P(200, "application/json", res.c_str());
  });

  // Velocity feedback
  pServer->on("/velocity", HTTP_GET, [velocityFeedback](AsyncWebServerRequest *request)
  {
    int velocity;
    velocityFeedback(velocity);

    JsonDocument doc;
    doc["velocity"] = velocity;

    String res;
    serializeJson(doc, res);

    request->send_P(200, "application/json", res.c_str());
  });

  // Position feedback
  pServer->on("/position", HTTP_GET, [positionFeedback](AsyncWebServerRequest *request)
  {
    PositionFeedback feedback;
    positionFeedback(feedback);

    JsonDocument doc;
    doc["goal"] = feedback.goal;
    doc["position"] = feedback.position;
    doc["error"] = feedback.error;
    doc["tolerance"] = feedback.tolerance;
    doc["integralError"] = feedback.integralError;
    doc["derivativeError"] = feedback.derivativeError;
    doc["proportional"] = feedback.proportional;
    doc["integral"] = feedback.integral;
    doc["derivative"] = feedback.derivative;
    doc["command"] = feedback.command;

    String res;
    serializeJson(doc, res);

    request->send_P(200, "application/json", res.c_str());
  });

  // Enable command
  pServer->on("/enable", HTTP_POST, DEFAULT_HANDLER, NULL, [enableCommand](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
  {
    JsonDocument doc;
    deserializeRequestJson(doc, data, len);

    bool enable = doc["enable"].as<bool>();
    enableCommand(enable);

    request->send(200);
  });

  // Position command
  pServer->on("/position", HTTP_POST, DEFAULT_HANDLER, NULL, [positionCommand](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
  {
    JsonDocument doc;
    deserializeRequestJson(doc, data, len);
  
    double position = doc["position"].as<double>();
    positionCommand(position);

    request->send(200);
  });

  // Velocity command
  pServer->on("/velocity", HTTP_POST, DEFAULT_HANDLER, NULL, [velocityCommand](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
  {
    JsonDocument doc;
    deserializeRequestJson(doc, data, len);
  
    int velocity = doc["velocity"].as<int>();
    velocityCommand(velocity);

    request->send(200);
  });

  // Settings command
  pServer->on("/settings", HTTP_POST, DEFAULT_HANDLER, NULL, [settingsCommand](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
  {
    JsonDocument doc;
    deserializeRequestJson(doc, data, len);

    Settings settings;
    settings.name = doc["name"];
    settings.controlMode = (CONTROL)deserializeEnum(
      doc["controlMode"],
      CONTROL_DESCRIPTION,
      sizeof(CONTROL_DESCRIPTION) / sizeof(char*));
    settings.standstillMode = (STANDSTILL)deserializeEnum(
      doc["standstillMode"],
      STANDSTILL_MODE_DESCRIPTION,
      sizeof(STANDSTILL_MODE_DESCRIPTION) / sizeof(char*));
    settings.voltage = (VOLTAGE)doc["voltage"].as<int>();
    settings.current = doc["current"].as<int>();
    settings.holdCurrent = doc["holdCurrent"].as<int>();
    settings.holdDelay = doc["holdDelay"].as<int>();
    settings.microstepsPerStep = (MICROSTEPS)doc["microstepsPerStep"].as<int>();
    settings.stallThreshold = doc["stallThreshold"].as<int>();
    settings.coolStepDurationThreshold = doc["coolStepDurationThreshold"].as<int>();
    settings.buttonVelocity = doc["buttonVelocity"].as<int>();
    settings.encoderMin = doc["encoderMin"].as<int>();
    settings.encoderMax = doc["encoderMax"].as<int>();
    settings.positionMin = doc["positionMin"].as<double>();
    settings.positionMax = doc["positionMax"].as<double>();
    settings.velocityMin = doc["velocityMin"].as<int>();
    settings.velocityMax = doc["velocityMax"].as<int>();
    settings.Kp = doc["Kp"].as<double>();
    settings.Ki = doc["Ki"].as<double>();
    settings.Kd = doc["Kd"].as<double>();
    settings.iMin = doc["iMin"].as<double>();
    settings.iMax = doc["iMax"].as<double>();
    settings.tolerance = doc["tolerance"].as<double>();

    settingsCommand(settings);

    request->send(200);
  });

  pServer->begin();
}

void deserializeRequestJson(JsonDocument& doc, uint8_t* data, size_t len)
{
  String json = String((char*)data).substring(0, len);
  deserializeJson(doc, json);
}

int deserializeEnum(const char* value, const char* const* values, int count)
{
  for (int n = 0; n < count; n++)
  {
    if (strcmp(value, values[n]) == 0)
    {
      return n;
    }
  }

  return 0;
}
