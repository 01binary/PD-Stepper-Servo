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

// TMC2209 Stepper
#include <TMC2209.h>

// AS5600 Hall Effect Encoder
#include <Wire.h>

// Saving settings to flash memory
#include <Preferences.h>

//
// Constants
//

const char *WIFI_SSID = "your-ssid";         // TODO: move to flash
const char *WIFI_PASSWORD = "your-password"; // TODO: move to flash
const int PORT = 8080;                       // TODO: move to flash
const char *APPNAME = "PDStepperServo";
const int RATE_HZ = 50;
const int TIMESTEP_MS = 1000 / RATE_HZ;

// Encoder

const int AS5600_ADDRESS = 0x36;
const int AS5600_ANGLE_REGISTER = 0x0C;
const float AS5600_MAX = 4096;

// Stepper driver

const long SERIAL_BAUD_RATE = 115200;
const uint8_t RUN_CURRENT_PERCENT = 100;
// TMC2209 enabled pin
const int TMC_EN = 21;
// TMC2209 step pin
const int STEP = 5;
// TMC2209 direction pin
const int DIR = 6;
// MS1 ??
const int MS1 = 1;
// MS2 ??
const int MS2 = 2;
// Spread?
const int SPREAD = 7;
// TMC2209 TX pin
const int TMC_TX = 17;
// TMC2209 RX pin
const int TMC_RX = 18;
const int DIAG = 16;
const int INDEX = 11;
// TMC2209 standstill mode
enum STANDSTILL_MODE
{
  STANDSTILL_NORMAL,
  STANDSTILL_FREEWHEELING,
  STANDSTILL_BRAKING,
  STANDSTILL_STRONG_BRAKING
};

// USB Power Delivery

// Power Good pin
const int PG = 15;

// Configuration pins
const int CFG1 = 38;
const int CFG2 = 48;
const int CFG3 = 47;

// Supported voltages
enum VOLTAGE
{
  5V = 5,
  9V = 9,
  12V = 12,
  15V = 15,
  20V = 20
};

// 20k & 2.7K Voltage Divider
const float DIV_RATIO = 0.1189427313;

// PD Stepper Board

const int VBUS = 4;
const int NTC = 7;
const int LED1 = 10;
const int LED2 = 12;
const int SW1 = 35;
const int SW2 = 36;
const int SW3 = 37;
const int AUX1 = 14;
const int AUX2 = 13;

//
// Variables
//

// Controller

bool enabledState = 0;
int set_speed = 0;
signed long setPoint = 0;
signed long CurrentPosition = 0;
unsigned long lastStep = 0;

// Wifi

static EventGroupHandle_t wifi_event_group;

// Encoder

float position = 0.0;

// Stepper Driver

TMC2209 stepperDriver;
HardwareSerial &serial_stream = Serial2;
bool state = 0;

// Power Delivery
bool PGState = 0;
float voltage = 0;
float VREF = 3.3;

// Board

bool incButtonState = HIGH;
bool decButtonState = HIGH;
bool resetButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int buttonSpeed = 0;

//
// Forward Declarations
//

void stepperController(void *pvParameters);
static void initWifi(void);
static void wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) bool isPowerGood();
float readVoltage();
float readEncoder();
bool isDisabled();
bool isOverTempWarning();
bool isOverTempShutdown();
uint32_t getStallGuard();

//
// Entry point
//

void setup(void)
{
  // Initialize Wifi
  initWifi();

  // Create real-time motor control task
  xTaskCreate(stepperController, "stepperController", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}

//
// Non-RT thread
//

void loop()
{
  char rx_buffer[128];
  char addr_str[128];
  int addr_len;
  struct sockaddr_in dest_addr;

  int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

  if (listen_sock < 0)
  {
    ESP_LOGE(APPNAME, "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }

  dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(PORT);
  addr_len = sizeof(dest_addr);

  if (bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) < 0)
  {
    ESP_LOGE(APPNAME, "Socket unable to bind: errno %d", errno);
    close(listen_sock);
    vTaskDelete(NULL);
    return;
  }

  if (listen(listen_sock, 1) < 0)
  {
    ESP_LOGE(APPNAME, "Error occurred during listen: errno %d", errno);
    close(listen_sock);
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(APPNAME, "Socket listening on port %d", PORT);

  while (1)
  {
    struct sockaddr_in6 source_addr;
    socklen_t socklen = sizeof(source_addr);
    int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &socklen);

    if (sock < 0)
    {
      ESP_LOGE(APPNAME, "Unable to accept connection: errno %d", errno);
      break;
    }

    inet_ntoa_r(
        ((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);

    ESP_LOGI(APPNAME, "Socket accepted connection from %s", addr_str);

    while (1)
    {
      int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
      if (len < 0)
      {
        ESP_LOGE(APPNAME, "recv failed: errno %d", errno);
        break;
      }
      else if (len == 0)
      {
        ESP_LOGI(APPNAME, "Connection closed");
        break;
      }
      else
      {
        rx_buffer[len] = 0; // Null-terminate the received data
        ESP_LOGI(APPNAME, "Received %d bytes: %s", len, rx_buffer);
      }
    }

    close(sock);
    ESP_LOGI(APPNAME, "Socket closed");
  }

  close(listen_sock);
  vTaskDelete(NULL);
}

//
// RT thread
//

void stepperController(void *pvParameters)
{
  const TickType_t freq = pdMS_TO_TICKS(TIMESTEP_MS);
  TickType_t lastTime = xTaskGetTickCount();

  while (1)
  {
    // Perform real-time operations here
    printf("Real-time task running at 50Hz\n");

    // Sleep
    vTaskDelayUntil(&lastTime, freq);
  }
}

//
// Helpers
//

static void initWifi(void)
{
  wifi_event_group = xEventGroupCreate();

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
    .sta = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
    }
  };

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();

  xEventGroupWaitBits(wifi_event_group, BIT0, pdFALSE, pdTRUE, portMAX_DELAY);
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
    xEventGroupClearBits(wifi_event_group, BIT0);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(APPNAME, "Got IP: %s", ip4addr_ntoa(&event->ip_info.ip));
    xEventGroupSetBits(wifi_event_group, BIT0);
  }
}

bool isPowerGood()
{
  PGState = digitalRead(PG);
  return PGState == 0;
}

float readVoltage()
{
  int adc = analogRead(VBUS);
  voltage = adc * (VREF / 4096.0) / DIV_RATIO;
  return voltage;
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
        digitalWrite(CFG1, HIGH);
      }
      break;
    case 9V
      {
        digitalWrite(CFG1, LOW);
        digitalWrite(CFG2, LOW);
        digitalWrite(CFG3, LOW);
      }
      break;
    case 12V
      {
        digitalWrite(CFG1, LOW);
        digitalWrite(CFG2, LOW);
        digitalWrite(CFG3, HIGH);
      }
      break;
    case 15V
      {
        digitalWrite(CFG1, LOW);
        digitalWrite(CFG2, HIGH)
        digitalWrite(CFG3, HIGH);
      }
      break;
    case 20V
      {
        digitalWrite(CFG1, LOW);
        digitalWrite(CFG2, HIGH)
        digitalWrite(CFG3, LOW);
      }
      break;
  }
}

void writeMotorDriver()
{
  stepperDriver.setRunCurrent(current.toInt());
  stepperDriver.setMicrostepsPerStep(microsteps.toInt());
  stepperDriver.setStallGuardThreshold(stallThreshold.toInt());

  if (standstillMode == "NORMAL")
  {
    stepperDriver.setStandstillMode(stepperDriver.NORMAL);
  }
  else if (standstillMode == "FREEWHEELING")
  {
    stepperDriver.setStandstillMode(stepperDriver.FREEWHEELING);
  }
  else if (standstillMode == "BRAKING")
  {
    stepperDriver.setStandstillMode(stepperDriver.BRAKING);
  }
  else if (standstillMode == "STRONG_BRAKING")
  {
    stepperDriver.setStandstillMode(stepperDriver.STRONG_BRAKING);
  }
}

float readEncoder()
{
  int count = 0;

  // Request raw angle
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_ANGLE_REGISTER);
  Wire.endTransmission(false);

  // Read raw angle
  Wire.requestFrom(AS5600_ADDRESS, 2);

  if (Wire.available() >= 2)
  {
    // Combine two bytes to get the counts value
    count = Wire.read() << 8 | Wire.read();
  }

  return count / 4096.0;
}

bool isHardwareDisabled()
{
  return stepperDriver.hardwareDisabled();
}

bool isOverTempWarning()
{
  TMC2209::Status status = stepperDriver.getStatus();
  return status.over_temperature_warning;
}

bool isOverTempShutdown()
{
  TMC2209::Status status = stepperDriver.getStatus();
  return status.over_temperature_shutdown;
}

uint32_t getStallGuard()
{
  return stepperDriver.getStallGuardResult();
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
