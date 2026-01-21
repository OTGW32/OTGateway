#pragma once
#ifdef NODO
/* probably should be here for all versions */
#include <WiFi.h>
// for OLED display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// for W5500 wi-fi
#include <SPI.h>
#include <Ethernet.h>
extern bool WIRED_ETHERNET_PRESENT; 
/* pin defs for Nodo OTGW32 PCB */
#define GPIO_BOOT_BUTTON 0
#define GPIO_CONFIG_BUTTON 9
#define GPIO_STATUS_LED 8
#define GPIO_OTRED_LED 2
#define GPIO_OTGREEN_LED 48
#define GPIO_BYPASS_RELAY 47 /* not connected on Nodo but need for compilation */
#define GPIO_STEPUP_ENABLE 10
#define GPIO_1WIRE_DIO 4
#define GPIO_OTSLAVE_IN 6
#define GPIO_OTSLAVE_OUT 7
#define GPIO_OTMASTER_IN 21
#define GPIO_OTMASTER_OUT 1
/* for I2C OLED display */
#define GPIO_I2C_SCL 17
#define GPIO_I2C_SDA 18
#define GPIO_SPI_CS 14
/* for SPI W5500 Ethernet */
#define GPIO_SPI_MISO 13
#define GPIO_SPI_MOSI 11
#define GPIO_SPI_SCK 12
#define GPIO_SPI_INT 15
#define GPIO_SPI_RST 16
// OLED display width and height
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define LED_BRIGHTNESS 5 // 0-255 
#else
#define GPIO_CONFIG_BUTTON 0
#define GPIO_STATUS_LED 8
#define GPIO_OTRED_LED 2
#define GPIO_OTGREEN_LED 21
#define GPIO_BYPASS_RELAY 20
#define GPIO_STEPUP_ENABLE 10
#define GPIO_1WIRE_DIO 4
#define GPIO_OTSLAVE_IN 6
#define GPIO_OTSLAVE_OUT 7
#define GPIO_OTMASTER_IN 3
#define GPIO_OTMASTER_OUT 1
#endif

inline void setLedOTRed(const bool on) {
#ifdef NODO
    static int init = 0;
    if ( !init ) {
      ledcAttachChannel(GPIO_OTRED_LED, 5000, 8, 2); 
      ledcOutputInvert(GPIO_OTRED_LED, true);
      init = 1;
    }
    if (!on) {
      ledcWrite(GPIO_OTRED_LED, 0);
    } else {
      ledcWrite(GPIO_OTRED_LED, LED_BRIGHTNESS); // dim
    }
#else
    digitalWrite(GPIO_OTRED_LED, !on);
#endif
}

inline void setLedOTGreen(const bool on) {
#ifdef NODO
    static int init = 0;
    if ( !init ) {
      ledcAttachChannel(GPIO_OTGREEN_LED, 5000, 8, 1); 
      ledcOutputInvert(GPIO_OTGREEN_LED, false);
      init = 1;
    }
    if (on) { // off by default
      ledcWrite(GPIO_OTGREEN_LED, 0);
    } else {
      ledcWrite(GPIO_OTGREEN_LED, LED_BRIGHTNESS); // dim
    }
#else
    digitalWrite(GPIO_OTGREEN_LED, !on);
#endif
}

inline void setLedStatus(const bool on) {
#ifdef NODO
    static int init = 0;
    if ( !init ) {
      ledcAttachChannel(GPIO_STATUS_LED, 5000, 8, 0); 
      ledcOutputInvert(GPIO_STATUS_LED, true);
      init = 1;
    }
    if (!on) {
      ledcWrite(GPIO_STATUS_LED, 0); // dim
    } else {
      ledcWrite(GPIO_STATUS_LED, LED_BRIGHTNESS); // dim
    }
#else
    digitalWrite(GPIO_STATUS_LED, !on);
#endif
}
