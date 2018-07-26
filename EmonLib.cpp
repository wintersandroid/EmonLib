/*
  Emon.cpp - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

// Proboscide99 10/08/2016 - Added ADMUX settings for ATmega1284 e 1284P (644 / 644P also, but not tested) in readVcc function

#include "EmonLib.h"
#include "Arduino.h"

//--------------------------------------------------------------------------------------
// Constructor. Set the pinReader to the default pin reader method
//--------------------------------------------------------------------------------------
EnergyMonitor::EnergyMonitor()
{
  this->inputPinReader = defaultInputPinReader;
  this->readVCC = defaultReadVcc;
}

//--------------------------------------------------------------------------------------
// By default we just call Arduino's analogRead
//--------------------------------------------------------------------------------------
uint16_t EnergyMonitor::defaultInputPinReader(uint8_t _pin)
{
  return analogRead(_pin);
}

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(uint8_t _inPinV, double _VCAL, double _PHASECAL)
{
  inPinV = _inPinV;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS >> 1;
  filteredV = 0.0;
}

void EnergyMonitor::current(uint8_t _inPinI, double _ICAL)
{
  inPinI = _inPinI;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS >> 1;
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
double EnergyMonitor::calcVrms(uint8_t crossings, uint16_t timeout)
{
  uint16_t SupplyVoltage = readVCC();

  uint8_t crossCount = 0;      //Used to measure number of times threshold is crossed.
  uint8_t numberOfSamples = 0; //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  uint32_t start = millis(); //millis()-start makes sure it doesnt get stuck in the loop if there is an error.
  uint16_t higher = ADC_COUNTS * 0.55;
  uint16_t lower = ADC_COUNTS * 0.45;
  uint16_t startV;

  while (true)
  {
    startV = this->inputPinReader(inPinV); //using the voltage waveform
    if ((startV < higher) && (startV > lower))
    {
      break;
    }
    if ((millis() - start) > timeout)
    {
      break;
    }
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();
  double sumV = 0.0;
  sumP = 0.0;
  while ((crossCount < crossings) && ((millis() - start) < timeout))
  {
    numberOfSamples++;                //Count number of times looped.
    double lastFilteredV = filteredV; //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    uint16_t sampleV = (this->inputPinReader)(inPinV); //Read in raw voltage signal
    // sampleI = (this->inputPinReader)(inPinI);                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV - offsetV) / 1024);
    filteredV = sampleV - offsetV;
    // offsetI = offsetI + ((sampleI-offsetI)/1024);
    // filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    double sqV = filteredV * filteredV; //1) square voltage values
    sumV += sqV;                        //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    // sqI = filteredI * filteredI;                //1) square current values
    // sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    double phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    sumP += (phaseShiftedV * filteredI); //Sum

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    boolean lastVCross = checkVCross;
    checkVCross = (sampleV > startV);
    if (numberOfSamples == 1)
    {
      lastVCross = checkVCross;
    }

    if (lastVCross != checkVCross)
    {
      crossCount++;
    }
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.

  V_RATIO = VCAL * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);
  return Vrms;
}

// Calculation power values, Use after calcVrms and calcIrms
void EnergyMonitor::calcRealPowerValues()
{
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor = realPower / apparentPower;
}

//--------------------------------------------------------------------------------------
double EnergyMonitor::calcIrms(uint16_t samples)
{
  double sumI = 0.0;

  for (uint16_t n = 0; n < samples; n++)
  {
    sampleI = inputPinReader(inPinI);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    //  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI - offsetI) / 1024);
    filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    // 2) sum
    sumI += (filteredI * filteredI);
  }

  I_RATIO = ICAL * ((readVCC() / 1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / samples);
  numberOfSamples = samples;
  return Irms;
}

void EnergyMonitor::serialprint()
{
  Serial.print("Vrms: ");
  Serial.print(Vrms);
  Serial.print(" Irms: ");
  Serial.print(Irms);
  Serial.print(" realPower: ");
  Serial.print(realPower);
  Serial.print(" apparentPower: ");
  Serial.print(apparentPower);
  Serial.print(" powerFactor: ");
  Serial.print(powerFactor);
  Serial.println(' ');
}

uint16_t EnergyMonitor::defaultReadVcc()
{
  return 3300;
}
