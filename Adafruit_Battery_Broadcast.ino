#include <RTCZero.h>

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#define VBATPIN A7

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEBattery.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
   
                            Enabling this will put your Bluefruit LE module
                            in a 'known good' state and clear any config
                            data set in previous sketches or projects, so
                            running this at least once is a good idea.
   
                            When deploying your project, however, you will
                            want to disable factory reset by setting this
                            value to 0.  If you are making changes to your
                            Bluefruit LE device via AT commands, and those
                            changes aren't persisting across resets, this
                            is the reason why.  Factory reset will erase
                            the non-volatile memory where config data is
                            stored, setting it back to factory default
                            values.
       
                            Some sketches that require you to bond to a
                            central device (HID mouse, keyboard, etc.)
                            won't work at all with this feature enabled
                            since the factory reset will clear all of the
                            bonding data stored on the chip, meaning the
                            central device won't be able to reconnect.
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE      1
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

Adafruit_BLEBattery battery(ble);
RTCZero rtc;


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* Change these values to set the current initial time */
const byte seconds = 00;
const byte minutes = 24;
const byte hours = 8;

/* Change these values to set the current initial date */
const byte day = 19;
const byte month = 11;
const byte year = 16;
bool ledState = false; 

int value = 100;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledState);



  
  while (!Serial);  // required for Flora & Micro
  delay(5000);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit AT Command Example"));
  Serial.println(F("-------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  ble.sendCommandCheckOK( F("AT+BLEPOWERLEVEL=-8") ); //Reduce power of the BLE 

  ble.sendCommandCheckOK( F("AT+HWMODELED=DISABLE") ); //Turn of LED to save power

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW); 

  // Enable Battery service and reset Bluefruit
  battery.begin(true);

  //SetupLow power mode
  rtc.begin();

  // Set the alarm at the 5 second mark
  rtc.setAlarmSeconds(5);

  // Match only seconds (Periodic alarm every minute)
  rtc.enableAlarm(RTCZero::MATCH_SS);

  // Set sleep mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  // Disable USB
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; 
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  
  
  // Disable USB
  //USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; 
  //Enter sleep mode
  __WFI();
  
  // ...Sleep  
  // Enable USB
  //USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
  
  ledState = ! ledState; 
  digitalWrite(LED_BUILTIN, ledState);
  
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  // Should get Battery value from LIPO and update
  //Serial.print("Update battery level = ");
  //Serial.println(measuredvbat);

  int realValue = (int)((float) 100 * (measuredvbat - 3.2));
    
  
  battery.update(realValue);
   value--;
  if (value == 0) value = 100;

  
  rtc.setAlarmSeconds((rtc.getSeconds() +5) % 60);  
}

