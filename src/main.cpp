/*
  Code Base from RadioLib: https://github.com/jgromes/RadioLib/tree/master/examples/SX126x

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>

#define LoRa_MOSI 10
#define LoRa_MISO 11
#define LoRa_SCK 9

#define LoRa_nss 8
#define LoRa_dio1 14
#define LoRa_nrst 12
#define LoRa_busy 13

SX1262 radio = new Module(LoRa_nss, LoRa_dio1, LoRa_nrst, LoRa_busy);

void setup()
{
  Serial.begin(250000); // Initialize serial communication for UART
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_nss);

  // initialize SX1262 with default settings
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
}

void loop()
{
  if (Serial.available() > 0)
  {
    // Read data from UART
    String uartData = Serial.readStringUntil('\n');

    Serial.print(F("[SX1262] Transmitting packet ... "));

    // Transmit the data over LoRa
    int state = radio.transmit(uartData);

    if (state == RADIOLIB_ERR_NONE)
    {
      // the packet was successfully transmitted
      Serial.println(F("success!"));

      // print measured data rate
      Serial.print(F("[SX1262] Datarate:\t"));
      Serial.print(radio.getDataRate());
      Serial.println(F(" bps"));
    }
    else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
    {
      // the supplied packet was longer than 256 bytes
      Serial.println(F("too long!"));
    }
    else if (state == RADIOLIB_ERR_TX_TIMEOUT)
    {
      // timeout occurred while transmitting packet
      Serial.println(F("timeout!"));
    }
    else
    {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
  }
}