/*  
 * || Project:          This initial implementation is part of the project: 
 * || "IoT Energy Meter with C/C++/FreeRTOS, Java/Spring, TypeScript/Angular and Dart/Flutter.";
 * || About:            End-to-end implementation of a LoRaWAN network for monitoring electrical quantities;
 * || Version:          1.0;
 * || Backend Mote:     ATmega328P/ESP32/ESP8266/ESP8285/STM32;
 * || Radios:           RFM95w and LoRaWAN EndDevice Radioenge Module: RD49C;
 * || Sensors:          Peacefair PZEM-004T 3.0 Version TTL-RTU kWh Meter;
 * || Backend API:      Java with Framework: Spring Boot;
 * || LoRaWAN Stack:    MCCI Arduino LoRaWAN Library (LMiC: LoRaWAN-MAC-in-C) version 3.0.99;
 * || Activation mode:  Activation by Personalization (ABP) or Over-the-Air Activation (OTAA);
 * || Author:           Adail dos Santos Silva
 * || E-mail:           adail101@hotmail.com
 * || WhatsApp:         +55 89 9 9412-9256
 * || 
 * || WARNINGS:
 * || Permission is hereby granted, free of charge, to any person obtaining a copy of
 * || this software and associated documentation files (the “Software”), to deal in
 * || the Software without restriction, including without limitation the rights to
 * || use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * || the Software, and to permit persons to whom the Software is furnished to do so,
 * || subject to the following conditions:
 * || 
 * || The above copyright notice and this permission notice shall be included in all
 * || copies or substantial portions of the Software.
 * || 
 * || THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * || IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * || FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * || COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * || IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * || CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 */

/********************************************************************
 _____              __ _                       _   _             
/  __ \            / _(_)                     | | (_)            
| /  \/ ___  _ __ | |_ _  __ _ _   _ _ __ __ _| |_ _  ___  _ __  
| |    / _ \| '_ \|  _| |/ _` | | | | '__/ _` | __| |/ _ \| '_ \ 
| \__/\ (_) | | | | | | | (_| | |_| | | | (_| | |_| | (_) | | | |
 \____/\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\__|_|\___/|_| |_|
                          __/ |                                  
                         |___/                                   
********************************************************************/

/* Includes. */
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

/* Definitions. */
#define SCK_GPIO 18
#define MISO_GPIO 19
#define MOSI_GPIO 23
#define NSS_GPIO 5
#define RESET_GPIO 14
#define DIO0_GPIO 34
#define DIO1_GPIO 35 /* Note: not really used on this board. */
#define DIO2_GPIO 39

/* Credentials. */
// CHIRPSTACK - CS (8 at 15 + 65 channels):
/* little-endian - LSB */  // 00 00 00 00 00 00 00 00
/* big-endian - MSB */     // 00 00 00 00 00 00 00 00
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* little-endian - LSB */  // f3 8d f5 e5 a3 1e 63 de (Use this key in registration.)
/* big-endian - MSB */     // de 63 1e a3 e5 f5 8d f3
static const u1_t PROGMEM DEVEUI[8] = { 0xde, 0x63, 0x1e, 0xa3, 0xe5, 0xf5, 0x8d, 0xf3 };

/* little-endian - LSB */  // ae 94 4e 5e 85 b7 bc 02 c5 dd 46 da 4c 7c c5 6f
/* big-endian - MSB */     // 6f c5 7c 4c da 46 dd c5 02 bc b7 85 5e 4e 94 ae (Use this key in registration.)
static const u1_t PROGMEM APPKEY[16] = { 0x6f, 0xc5, 0x7c, 0x4c, 0xda, 0x46, 0xdd, 0xc5, 0x02, 0xbc, 0xb7, 0x85, 0x5e, 0x4e, 0x94, 0xae };

void os_getArtEui(u1_t *buf) {
  memcpy_P(buf, APPEUI, 8);
}
void os_getDevEui(u1_t *buf) {
  memcpy_P(buf, DEVEUI, 8);
}
void os_getDevKey(u1_t *buf) {
  memcpy_P(buf, APPKEY, 16);
}

/* Data to Send. */
static uint8_t mydata[] = "AdailSilva-IoT";

/* Instances. */
/* Jobs: */
static osjob_t sendjob;

/* Schedule TX every this many seconds (Might become longer due to duty cycle limitations). */
const unsigned TX_INTERVAL = 10;

/* Pin mapping. */
const lmic_pinmap lmic_pins = {
  .nss = NSS_GPIO,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RESET_GPIO,
  .dio = { DIO0_GPIO, DIO1_GPIO, DIO2_GPIO },
};

/*****************************
 _____           _      
/  __ \         | |     
| /  \/ ___   __| | ___ 
| |    / _ \ / _` |/ _ \
| \__/\ (_) | (_| |  __/
 \____/\___/ \__,_|\___|
*****************************/

/* Functions. */
void onEvent(ev_t ev) {
  //Serial.print(os_getTime());
  //Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("artKey: ");
        for (int i = 0; i < sizeof(artKey); ++i) {
          Serial.print(artKey[i], HEX);
        }
        Serial.println("");
        Serial.print("nwkKey: ");
        for (int i = 0; i < sizeof(nwkKey); ++i) {
          Serial.print(nwkKey[i], HEX);
        }
        Serial.println("");
      }

      //LMIC_selectSubBand(1);

      /* Channels Control to AU915 (8 at 15 + 65 channels): */
      for (u1_t b = 0; b < 8; ++b) {
        LMIC_disableSubBand(b);
      }
      for (u1_t channel = 0; channel < 72; ++channel) {
        LMIC_disableChannel(channel);
      }

      /* ChirpStack AU915 */
      LMIC_enableChannel(8);
      LMIC_enableChannel(9);
      LMIC_enableChannel(10);
      LMIC_enableChannel(11);
      LMIC_enableChannel(12);
      LMIC_enableChannel(13);
      LMIC_enableChannel(14);
      LMIC_enableChannel(15);
      //LMIC_enableChannel(65); /* Test */

      /* Disable Adaptive Data Rate. */
      LMIC_setAdrMode(0);

      /* Disable link check validation. */
      LMIC_setLinkCheckMode(0);

      /* TTN uses SF9 for its RX2 window. */
      LMIC.dn2Dr = DR_SF9;

      /* Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library). */
      LMIC_setDrTxpow(DR_SF7, 14);

      /* 
       * Use with Arduino Pro Mini ATmega328P 3.3V 8MHz;
       * Let LMIC compensate for +/- 1% clock error.
       */
      LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
      break;
    /*
     * This event is defined but not used in the code.
     * No point in wasting codespace on it.
     * case EV_RFU1:
     *   Serial.println(F("EV_RFU1"));
     *   break;
     */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack;"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" byte(s) of payload."));
      }
      /* Schedule next transmission. */
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      /* Data received in ping slot. */
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
     * This event is defined but not used in the code.
     * No point in wasting codespace on it.
     * case EV_SCAN_FOUND:
     *   Serial.println(F("EV_SCAN_FOUND"));
     *   break;
     */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t *j) {
  /* Check if there is not a current TX/RX job running. */
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    /* Prepare upstream data transmission at the next possible time. */
    LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    Serial.println(F("Packet queued."));
  }
  /* Next TX is scheduled after TX_COMPLETE event. */
}

/*****************************
 _____      _               
/  ___|    | |              
\ `--.  ___| |_ _   _ _ __  
 `--. \/ _ \ __| | | | '_ \ 
/\__/ /  __/ |_| |_| | |_) |
\____/ \___|\__|\__,_| .__/ 
                     | |    
                     |_|    
******************************/

void setup() {

  SPI.begin(SCK_GPIO, MISO_GPIO, MOSI_GPIO, NSS_GPIO);

  Serial.begin(9600);

  delay(100);

  Serial.println(F("Starting..."));

  /* For Pinoccio Scout boards. */
  #ifdef VCC_ENABLE
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  #endif

  /* LMIC init. */
  os_init();

  /* Reset the MAC state. Session and pending data transfers will be discarded. */
  LMIC_reset();

  //LMIC_selectSubBand(1);

  /* Channels Control to AU915 (8 at 15 + 65 channels): */
  for (u1_t b = 0; b < 8; ++b) {
    LMIC_disableSubBand(b);
  }
  for (u1_t channel = 0; channel < 72; ++channel) {
    LMIC_disableChannel(channel);
  }

  /* ChirpStack AU915 */
  LMIC_enableChannel(8);
  LMIC_enableChannel(9);
  LMIC_enableChannel(10);
  LMIC_enableChannel(11);
  LMIC_enableChannel(12);
  LMIC_enableChannel(13);
  LMIC_enableChannel(14);
  LMIC_enableChannel(15);
  //LMIC_enableChannel(65); /* Test */

  /* Disable Adaptive Data Rate. */
  LMIC_setAdrMode(0);

  /* Disable link check validation. */
  LMIC_setLinkCheckMode(0);

  /* TTN uses SF9 for its RX2 window. */
  LMIC.dn2Dr = DR_SF9;

  /* Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library). */
  LMIC_setDrTxpow(DR_SF7, 14);

  /* 
   * Use with Arduino Pro Mini ATmega328P 3.3V 8MHz;
   * Let LMIC compensate for +/- 1% clock error.
   */
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  /* Start job (sending automatically starts OTAA too). */
  do_send(&sendjob);
}

/*****************************
 _                       
| |                      
| |     ___   ___  _ __  
| |    / _ \ / _ \| '_ \ 
| |___| (_) | (_) | |_) |
\_____/\___/ \___/| .__/ 
                  | |    
                  |_|    
*****************************/

void loop() {
  os_runloop_once();
}
