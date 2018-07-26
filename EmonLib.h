/*
  Emon.h - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

#ifndef EmonLib_h
#define EmonLib_h

#include "Arduino.h"

// to enable 15-bit ADC resolution on the ADS1115
// or to enable 12-bit ADC resolution on Arduino Due,
// otherwise will default to 10 bits, as in regular Arduino-based boards.
// #if defined(ADS1115_CONVERSIONDELAY)
#define ADC_BITS    15
#define ADC_COUNTS  (1<<ADC_BITS)

class EnergyMonitor
{
  public:
    EnergyMonitor(); 
    
    typedef uint16_t (*inputPinReaderMethod) (uint8_t _pin);
    inputPinReaderMethod inputPinReader;

    typedef uint16_t (*readVCCMethod) ();
    readVCCMethod readVCC;
    
    static uint16_t defaultInputPinReader(uint8_t _pin);
    static uint16_t defaultReadVcc();

    void voltage(uint8_t _inPinV, double _VCAL, double _PHASECAL);
    void current(uint8_t _inPinI, double _ICAL);

    double calcVrms(uint8_t crossings, uint16_t timeout);
    double calcIrms(uint16_t NUMBER_OF_SAMPLES);
    void serialprint();

    void calcRealPowerValues();
    //Useful value variables
    double realPower,
      apparentPower,
      powerFactor,
      Vrms,
      Irms;

  private:

    //Set Voltage and current input pins
    uint8_t inPinV;
    uint8_t inPinI;
    //Calibration coefficients
    //These need to be set in order to obtain accurate results
    double VCAL;
    double ICAL;
    double PHASECAL;

    //--------------------------------------------------------------------------------------
    // Variable declaration for emon_calc procedure
    //--------------------------------------------------------------------------------------
    // uint16_t sampleV;                        //sample_ holds the raw analog read value
    uint16_t sampleI;
    uint16_t numberOfSamples;

    double filteredV;          //Filtered_ is the raw analog value minus the DC offset
    double filteredI;
    double offsetV;                          //Low-pass filter output
    double offsetI;                          //Low-pass filter output

    double sumP;

    boolean checkVCross;                  //Used to measure number of times threshold is crossed.

    double I_RATIO;
    double V_RATIO;

};

#endif
