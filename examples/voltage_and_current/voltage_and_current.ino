// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3
#include <Arduino.h>
#include <Wire.h>

// using ADS1115 from https://github.com/jrowberg/i2cdevlib.git
// setup to have the alert line to D0 to trigger when conversion
// is done.
#include <ADS1115.h>

#include "EmonLib.h"             // Include Emon Library
EnergyMonitor emon1;             // Create an instance

#define VOLTAGE_PORT 1
#define AMPS_PORT 0
#define ADC_ALERT_READY D0

ADS1115 adc0(ADS1115_DEFAULT_ADDRESS);

uint8_t adcChannel;
void setADCChannel(uint8_t channel){
  if(adcChannel != channel){
    uint8_t mp;
    switch(channel){
      case AMPS_PORT:
        mp = ADS1115_MUX_P0_NG;
        break;
      case VOLTAGE_PORT:
        mp = ADS1115_MUX_P1_NG;
        break;
      case 2:
        mp = ADS1115_MUX_P2_NG;
        break;
      case 3:
        mp = ADS1115_MUX_P3_NG;
        break;
    };
    adc0.setMultiplexer(mp);
    adcChannel = channel;
  }
}

// start a conversion on the given channel, waiting 
// atleast 50 ms for conversion to complete.
// as reading can be < 0, make those = 0
uint16_t getADConversion(uint8_t channel) {
  setADCChannel(channel);
  adc0.triggerConversion();
  uint32_t timestart = millis();
  while(millis() - timestart < 50){
    yield();
    if(digitalRead(ADC_ALERT_READY)){
      break;
    }
  }
  int i = adc0.getConversion(false);
  return i < 0 ? 0 : i;
}

// we use the 4.096 V setting on the ADS
uint16_t vccForAds(){
  return 4096;
}

// My Vcal, for 240v 
double vcal = 486.7;

// increase or decrease value being used for vcal
// by the serial port - makes it easy to configure vcal
void serialEvent(){
  while(Serial.available()){
    switch ((char)Serial.read()){
      case 'a':
        vcal += 1.0;
        break;
      case 'z':
        vcal -= 1.0;
        break;
      case 's':
        vcal += 0.1;
        break;
      case 'x':
        vcal -= 0.1;
        break;
    }
    emon1.voltage(VOLTAGE_PORT, vcal, 1.7);  // Voltage: input pin, calibration, phase_shift
  }
}

void setup()
{  
  Wire.begin();
  Serial.begin(115200);

  // setup the ADS1115, as fast as it can go with PGA_4P096
  // and using D0 as the alert ready pin.
  adc0.initialize();
  Serial.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
  adc0.setMode(ADS1115_MODE_SINGLESHOT);
  adc0.setRate(ADS1115_RATE_860);
  adc0.setGain(ADS1115_PGA_4P096);
  pinMode(ADC_ALERT_READY,INPUT_PULLUP);
  adc0.setConversionReadyPinMode();

  // setup callbacks for reading.
  emon1.inputPinReader = getADConversion; // Replace the default pin reader with the customized ads pin reader
  emon1.readVCC = vccForAds;
  
  // initial configuration.
  emon1.voltage(VOLTAGE_PORT, vcal, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon1.current(AMPS_PORT, 30);       // Current: input pin, calibration.

  // for nothing better, toggle the builtin led.
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
}

uint32_t m = millis();
void loop()
{
  serialEvent();
  if(millis() - m >= 10000){
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));  
    m = millis();
  }
  
  // calculate voltage and the calibration.
  double vrms = emon1.calcVrms(30,1000);         // Calculate all. No.of half wavelengths (crossings), time-out
  double irms = emon1.calcIrms(50);
  emon1.calcRealPowerValues();
  
  Serial.printf("%0.2d %0.2d %0.2d %0.2d %0.2d\n",vcal,vrms,
    emon1.realPower,
    emon1.apparentPower,
    emon1.powerFactor);
  
}
