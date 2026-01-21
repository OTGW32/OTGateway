#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 0

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FileData.h>
#include <LittleFS.h>
#include <ESPTelnetStream.h>

#include "defines.h"
#include "strings.h"
#include "hwdef.h" //

#include <TinyLogger.h>
#include <NetworkMgr.h>
#include "CrashRecorder.h"
#include "Sensors.h"
#include "Settings.h"
#include "utils.h"

#ifdef NODO
#include <EthernetESP32.h>
#include <SPI.h>
SPIClass SPI1(FSPI); // Match your OT-Thing implementation
W5500Driver driver(GPIO_SPI_CS, GPIO_SPI_INT, GPIO_SPI_RST);
bool WIRED_ETHERNET_PRESENT = false;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
QueueHandle_t button_press_queue;

void IRAM_ATTR nodo_boot_button_interrupt() {
    int pin_state = digitalRead(GPIO_BOOT_BUTTON);
    xQueueSendFromISR(button_press_queue, &pin_state, NULL);
}
#endif

#if defined(ARDUINO_ARCH_ESP32)
  #include <ESP32Scheduler.h>
#elif defined(ARDUINO_ARCH_ESP8266)
  #include <Scheduler.h>
#else
  #error Wrong board. Supported boards: esp8266, esp32
#endif

#include <Task.h>
#include <LeanTask.h>
#include "MqttTask.h"
#include "OpenThermTask.h"
#include "SensorsTask.h"
#include "RegulatorTask.h"
#include "PortalTask.h"
#include "MainTask.h"

using namespace NetworkUtils;

// Vars
ESPTelnetStream* telnetStream = nullptr;
NetworkMgr* network = nullptr;
Sensors::Result sensorsResults[SENSORS_AMOUNT];

FileData fsNetworkSettings(&LittleFS, "/network.conf", 'n', &networkSettings, sizeof(networkSettings), 1000);
FileData fsSettings(&LittleFS, "/settings.conf", 's', &settings, sizeof(settings), 60000);
FileData fsSensorsSettings(&LittleFS, "/sensors.conf", 'e', &sensorsSettings, sizeof(sensorsSettings), 60000);

// Tasks
MqttTask* tMqtt;
OpenThermTask* tOt;
SensorsTask* tSensors;
RegulatorTask* tRegulator;
PortalTask* tPortal;
MainTask* tMain;

#ifdef NODO
void manageNetwork() {
  // If Ethernet is plugged in, EthernetESP32 handles the stack switch automatically
  if (WIRED_ETHERNET_PRESENT) {
    // You can add specific logging here if the link status changes
  }
}
#endif

void setup() {
#ifdef NODO
  setLedStatus(true); // Dimmed LEDs via hwdef.h
  Wire.begin(GPIO_I2C_SDA, GPIO_I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  SPI1.begin(GPIO_SPI_SCK, GPIO_SPI_MISO, GPIO_SPI_MOSI, GPIO_SPI_CS);

// We don't need the 'driver' object. 
  // We use a MAC address (either fixed or based on ESP32 MAC)
  uint8_t mac[6];
  WiFi.macAddress(mac); // Use the ESP32's internal MAC for the Ethernet chip
  
  // Tweak the MAC so it's unique from WiFi/BT
  // Usually, we increment the last byte or flip the "locally administered" bit
  mac[5] ^= 0x01; 

  Log.info("Starting Ethernet with MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (Ethernet.begin(mac)) { 
      WIRED_ETHERNET_PRESENT = true;
      Log.info("Ethernet Connected! IP: %s", Ethernet.localIP().toString().c_str());
  } else {
      Log.error("Ethernet hardware failed to initialize.");
  }

  pinMode(GPIO_BOOT_BUTTON, INPUT_PULLUP);
  button_press_queue = xQueueCreate(10, sizeof(int));
  attachInterrupt(GPIO_BOOT_BUTTON, nodo_boot_button_interrupt, CHANGE);
#endif

  CrashRecorder::init();
  Sensors::setMaxSensors(SENSORS_AMOUNT);
  Sensors::settings = sensorsSettings;
  Sensors::results = sensorsResults;
  LittleFS.begin();

  // ... (Original Logging setup remains)
  Log.addStream(&Serial);
  Log.print("\n\n\r");

  // Load Settings
  fsNetworkSettings.read();
  
  if (!strlen(networkSettings.hostname)) {
    strcpy(networkSettings.hostname, getChipId("otgateway-").c_str());
    fsNetworkSettings.update();
  }

  // Network Manager Init
  network = (new NetworkMgr)
    ->setHostname(networkSettings.hostname)
    ->setStaCredentials(
      strlen(networkSettings.sta.ssid) ? networkSettings.sta.ssid : nullptr,
      strlen(networkSettings.sta.password) ? networkSettings.sta.password : nullptr,
      networkSettings.sta.channel
    )
    // ... (rest of standard NetworkMgr config)
    ->setUseDhcp(networkSettings.useDhcp);

  // Load App Settings
  fsSettings.read();
  
  // ... (Original MQTT/Serial/Telnet logic remains)

  // Start Tasks
  tMqtt = new MqttTask(false, 500);
  Scheduler.start(tMqtt);

  tOt = new OpenThermTask(true, 750);
  Scheduler.start(tOt);

  tSensors = new SensorsTask(true, 1000);
  Scheduler.start(tSensors);

  tRegulator = new RegulatorTask(true, 10000);
  Scheduler.start(tRegulator);

  tPortal = new PortalTask(true, 0);
  Scheduler.start(tPortal);

  tMain = new MainTask(true, 100);
  Scheduler.start(tMain);

  Scheduler.begin();
}

void loop() {
#if defined(ARDUINO_ARCH_ESP32)
  // On ESP32, loop() is a task. We can use it for OLED updates
  #ifdef NODO
  while(true) {
    manageNetwork();
    // Insert your OLED page-switching logic from OT-Thing here
    delay(500); 
  }
  #endif
  vTaskDelete(NULL);
#endif
}