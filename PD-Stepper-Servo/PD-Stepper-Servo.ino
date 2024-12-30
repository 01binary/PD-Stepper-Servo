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

// Networking

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

// TMC2209 Stepper Motor Driver

#include <TMC2209.h>

// AS5600 Hall Effect Encoder

#include <Wire.h>

// Flash memory

#include <Preferences.h>

//
// Constants
//

// Wifi

const char* WIFI_SSID = "your-ssid";         // TODO: move to flash
const char* WIFI_PASSWORD = "your-password"; // TODO: move to flash
const int PORT = 8080;                       // TODO: move to flash

// Controller

const int RATE_HZ = 50;
const int TIMESTEP_MS = 1000 / RATE_HZ;

// Encoder

const int AS5600_ADDRESS = 0x36;
const int AS5600_ANGLE_REGISTER = 0x0C;
const float AS5600_MAX = 4096;

// Stepper driver

const long TMC_SERIAL_BAUD_RATE = 115200;
const uint8_t TMC_RUN_CURRENT_PERCENT = 100;
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
const int PD_CFG1 = 38;
const int PD_CFG2 = 48;
const int PD_CFG3 = 47;
const float PD_VREF = 3.3;
const float PD_DIV = 0.1189427313;

enum PD_VOLTAGE
{
  5V = 5,
  9V = 9,
  12V = 12,
  15V = 15,
  20V = 20
};

// PD Stepper Board

const int BRD_VBUS = 4;
const int BRD_NTC = 7;
const int BRD_LED1 = 10;
const int BRD_LED2 = 12;
const int BRD_SW1 = 35;
const int BRD_SW2 = 36;
const int BRD_SW3 = 37;
const int BRD_AUX1 = 14;
const int BRD_AUX2 = 13;
const unsigned long DEBOUNCE = 50;

//
// Variables
//

// Wifi

static EventGroupHandle_t wifiEventGroup;

// State

bool enabled = 0;
int velocityCommand = 0;
signed long positionCommand = 0;

// Encoder

float position = 0.0;

// Stepper Driver

TMC2209 motorDriver;
HardwareSerial &motorSerial = Serial2;
bool motorState = false;
bool motorDisabled = true;
bool motorOverTemp = false;
bool motorOverTempShutdown = false;
bool motorStalled = false;
unsigned int motorStallGuard = 0;

// Power Delivery

bool powerGood = false;
float voltage = 0;

// Board

bool incrementButtonPushed = false;
bool decrementButtonPushed = false;
bool resetButtonPushed = false;
int buttonVelocity = 100;
unsigned long lastDebounceTime = 0;

//
// Forward Declarations
//

void initWifi(const char* ssid, const char* password);
void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void initPower();
void readPower();
void initBoard();
void readBoard();
void writeBoard();
void initEncoder();
void readEncoder();
void initMotor();
void readMotor();
void writeMotor(int runCurrent, int microstepsPerStep, int stallThreshold, TMC2209::StandstillMode standstillMode);
void initMotorControl();
void runMotorControl(void *pvParameters);

//
// Entry point
//

void setup()
{
  // Ensure bootloader mode entered correctly
  delay(200);

  initWifi(WIFI_SSID, WIFI_PASSWORD);
  initBoard();
  initPower();
  initMotor();
  initEncoder();
  initMotorControl();
}

//
// Non-RT thread
//

void loop()
{
  int destAddressLen;
  struct sockaddr_in destAddress;
  char buffer[128];
  char address[128];

  int listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

  if (listenSocket < 0)
  {
    ESP_LOGE(APPNAME, "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }

  destAddress.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddress.sin_family = AF_INET;
  destAddress.sin_port = htons(PORT);
  destAddressLen = sizeof(destAddress);

  if (bind(listenSocket, (struct sockaddr *)&destAddress, sizeof(destAddress)) < 0)
  {
    ESP_LOGE(APPNAME, "Socket unable to bind: errno %d", errno);
    close(listenSocket);
    vTaskDelete(NULL);
    return;
  }

  if (listen(listenSocket, 1) < 0)
  {
    ESP_LOGE(APPNAME, "Error occurred during listen: errno %d", errno);
    close(listenSocket);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(APPNAME, "Socket listening on port %d", PORT);

  while (1)
  {
    struct sockaddr_in6 sourceAddress;
    socklen_t sourceAddressLen = sizeof(sourceAddress);
    int socket = accept(listenSocket, (struct sockaddr *)&sourceAddress, &sourceAddressLen);

    if (socket < 0)
    {
      ESP_LOGE(APPNAME, "Unable to accept connection: errno %d", errno);
      break;
    }

    inet_ntoa_r(
      ((struct sockaddr_in *)&sourceAddress)->sin_addr.s_addr,
      address,
      sizeof(address) - 1);

    ESP_LOGI(APPNAME, "Socket accepted connection from %s", address);

    while (1)
    {
      int receivedLen = recv(socket, buffer, sizeof(buffer) - 1, 0);

      if (receivedLen < 0)
      {
        ESP_LOGE(APPNAME, "recv failed: errno %d", errno);
        break;
      }
      else if (receivedLen == 0)
      {
        ESP_LOGI(APPNAME, "Connection closed");
        break;
      }
      else
      {
        buffer[receivedLen] = 0;
        ESP_LOGI(APPNAME, "Received %d bytes: %s", receivedLen, buffer);
      }
    }

    close(socket);
    ESP_LOGI(APPNAME, "Socket closed");
  }

  close(listenSocket);
  vTaskDelete(NULL);
}

//
// RT thread
//

void initMotorControl()
{
  // Start real-time motor controller
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

    if (powerGood && enabled && motorDisabled)
    {
      motorDriver.enable();
    }
    else if (!powerGood || (!enabled && !motorDisabled))
    {
      motorDriver.disable();
    }

    // Sleep
    vTaskDelayUntil(&lastTime, freq);
  }
}

//
// Helpers
//

static void initWifi(const char* ssid, const char* password)
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
    WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, NULL, &instance_any_id);
  esp_event_handler_instance_register(
    IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, NULL, &instance_got_ip);

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

static void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
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

void initPower()
{
  pinMode(PD_POWERGOOD, INPUT);
  pinMode(PD_CFG1, OUTPUT);
  pinMode(PD_CFG2, OUTPUT);
  pinMode(PD_CFG3, OUTPUT);

  writeVoltage(5V);
}

void readPower()
{
  int adc = analogRead(VBUS);
  voltage = adc * (PD_VREF / 4096.0) / PD_DIV;

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
  if ((millis() - lastDebounceTime) > DEBOUNCE)
  {
    lastDebounceTime = millis();
    incrementButtonPushed = digitalRead(SW3) == HIGH ? false : true;
    decrementButtonPushed = digitalRead(SW1) == HIGH ? false : true;
    resetButtonPushed = digitalRead(SW2) == HIGH ? false : true;
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

  motorDriver.setup(motorSerial, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  motorDriver.setRunCurrent(RUN_CURRENT_PERCENT);
  motorDriver.enableAutomaticCurrentScaling();
  motorDriver.enableStealthChop();
  motorDriver.setCoolStepDurationThreshold(5000);
  motorDriver.disable();
}

void writeMotor(int runCurrent, int microstepsPerStep, int stallThreshold, TMC2209::StandstillMode standstillMode)
{
  motorDriver.setRunCurrent(runCurrent);
  motorDriver.setMicrostepsPerStep(microstepsPerStep);
  motorDriver.setStallGuardThreshold(stallThreshold);
  motorDriver.setStandstillMode(standstillMode);
}

void readMotor()
{
  motorDisabled = motorDriver.hardwareDisabled();
  motorStalled = digitalRead(TMC_OVERCURRENT) == HIGH;
  motorStallGuard = motorDriver.getStallGuardResult();

  TMC2209::Status status = motorDriver.getStatus();
  motorOverTemp = status.over_temperature_warning;
  motorOverTempShutdown = status.over_temperature_shutdown;
}

void writeVoltage(PD_VOLTAGE voltage)
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

  position = angle / 4096.0;
}

void readSettings()
{ 
  preferences.begin("settings", false);

  enabled1 = preferences.getString("enable", ""); 
  
  if (enabled1 == "")
  {
    //EEPROM has not been saved to before so save defaults
    preferences.end(); //close to write EEPROM can open again
    enabled1 = "enabled";
    setVoltage = "12";
    microsteps = "32";
    current = "30";
    stallThreshold = "10";
    standstillMode = "NORMAL";
    writeSettings();
  }
  else
  {
    Serial.println("Settings found in EEPROM");
    setVoltage = preferences.getString("voltage", ""); 
    microsteps = preferences.getString("microsteps", ""); 
    current = preferences.getString("current", ""); 
    stallThreshold = preferences.getString("stallThreshold", ""); 
    standstillMode = preferences.getString("standstillMode", ""); 
    preferences.end();
  }
}

void writeSettings()
{
  preferences.begin("settings", false);
  preferences.putString("enable", enabled1); 
  preferences.putString("voltage", setVoltage); 
  preferences.putString("microsteps", microsteps); 
  preferences.putString("current", current); 
  preferences.putString("stallThreshold", stallThreshold); 
  preferences.putString("standstillMode", standstillMode); 
  preferences.end();
  
  configureSettings();
}
