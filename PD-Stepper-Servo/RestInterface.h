/*
* PD Stepper Servo Controller
* REST API interface
* Copyright (C) 2025 Ctrl^H Hackerspace
*/

//
// Includes
//

#include <ESPAsyncWebServer.h> // https://github.com/me-no-dev/ESPAsyncWebServer
#include <AsyncTCP.h>          // https://github.com/me-no-dev/AsyncTCP
#include <ArduinoJson.h>       // https://github.com/bblanchon/ArduinoJson
#include <AsyncJson.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <functional>
#include "Interface.h"

//
// Variables
//

AsyncWebServer* pServer = NULL;

//
// REST API interface
//

void useRestInterface(
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

  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.print(" with ");
  Serial.println(password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    delay(500);
  }

  Serial.print("Connected with ");
  Serial.println(WiFi.localIP());

  // Create server
  Serial.print("Starting server on port ");
  Serial.println(port);

  pServer = new AsyncWebServer(port);

  // Status feedback
  pServer->on("/status", HTTP_GET, [statusFeedback](AsyncWebServerRequest *request)
  {
    Status status;
    statusFeedback(status);

    JsonDocument doc;
    doc["mode"] = status.mode;
    doc["enabled"] = status.enabled;
    doc["rawPosition"] = status.rawPosition;
    doc["position"] = status.position;
    doc["velocity"] = status.velocity;
    doc["voltage"] = status.voltage;
    doc["overTemp"] = status.overTemp;
    doc["overTempShutdown"] = status.overTempShutdown;
    doc["stalled"] = status.stalled;

    String res;
    serializeJson(doc, res);

    request->send_P(200, "application/json", res);
  });

  // Settings feedback
  pServer->on("/settings", HTTP_GET, [settingsFeedback](AsyncWebServerRequest *request)
  {
    Settings settings;
    settingsFeedback(settings);

    JsonDocument doc;
    doc["voltage"] = settings.voltage;
    doc["current"] = settings.current;
    doc["microsteps"] = settings.microsteps;
    doc["stallThreshold"] = settings.stallThreshold;
    doc["standstillMode"] = settings.standstillMode;
    doc["buttonVelocity"] = settings.buttonVelocity;
    doc["encoderMin"] = settings.encoderMin;
    doc["encoderMax"] = settings.encoderMax;
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

    request->send_P(200, "application/json", res);
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

    request->send_P(200, "application/json", res);
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
    doc["integralError"] = feedback.integralError;
    doc["derivativeError"] = feedback.derivativeError;

    String res;
    serializeJson(doc, res);

    request->send_P(200, "application/json", res);
  });

  // Enable command
  pServer->on("/enable", HTTP_POST, [enableCommand](AsyncWebServerRequest *request)
  {
    deserializeJsonBody(request, [enableCommand](JsonDocument& doc)
    {
      enableCommand(doc["enabled"].as<bool>());
      request->send(200);
    });
  });

  // Position command
  pServer->on("/position", HTTP_POST, [positionCommand](AsyncWebServerRequest *request)
  {
    deserializeJsonBody(request, [positionCommand](JsonDocument& doc)
    {
      positionCommand(doc["position"].as<float>());
      request->send(200);
    });
  });

  // Velocity command
  pServer->on("/velocity", HTTP_POST, [velocityCommand](AsyncWebServerRequest *request)
  {
    deserializeJsonBody(request, [velocityCommand](JsonDocument& doc)
    {
      velocityCommand(doc["velocity"].as<int>());
      request->send(200);
    });
  });

  // Settings command
  pServer->on("/settings", HTTP_POST, [settingsCommand](AsyncWebServerRequest *request)
  {
    deserializeJsonBody(request, [settingsCommand](JsonDocument& doc)
    {
      Settings settings;
      settings.voltage = doc["voltage"].as<int>();
      settings.current = doc["current"].as<int>();
      settings.microsteps = doc["microsteps"].as<int>();
      settings.stallThreshold = doc["stallThreshold"].as<int>();
      settings.standstillMode = doc["standstillMode"].as<STANDSTILL>();
      settings.buttonVelocity = doc["buttonVelocity"].as<int>();
      settings.encoderMin = doc["encoderMin"].as<int>();
      settings.encoderMax = doc["encoderMax"].as<int>();
      settings.velocityMin = doc["velocityMin"].as<int>();
      settings.velocityMax = doc["velocityMax"].as<int>();
      settings.Kp = doc["Kp"].as<float>();
      settings.Ki = doc["Ki"].as<float>();
      settings.Kd = doc["Kd"].as<float>();
      settings.iMin = doc["iMin"].as<float>();
      settings.iMax = doc["iMax"].as<float>();
      settings.tolerance = doc["tolerance"].as<float>();

      settingsCommand(settings);

      request->send(200);
    });
  });

  pServer->begin();
}

void deserializeJsonBody(
  AsyncWebServerRequest *request,
  const std::function <void (JsonDocument&)>& callback)
{
  size_t totalLen = request->contentLength();
  char *buffer = new char[totalLen + 1];

  request->onBody([&](uint8_t *data, size_t len, size_t index, size_t total)
  {
    if (index + len <= totalLen)
    {
      memcpy(buffer + index, data, len);
    }

    if (index + len == total)
    {
      buffer[totalLen] = '\0';

      JsonDocument doc;
      deserializeJson(doc, buffer);

      callback(doc);
    }
  });
}