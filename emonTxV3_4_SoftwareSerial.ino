/*
  
  emonTxV3.4 SoftwareSerial on RJ45
  
  If AC-AC adapter is detected assume emonTx is also powered from adapter (jumper shorted) and take Real Power Readings and disable sleep mode to keep load on power supply constant
  If AC-AC addapter is not detected assume powering from battereis / USB 5V AC sample is not present so take Apparent Power Readings and enable sleep mode
  
  Transmitt values via RFM69CW radio
  
   -----------------------------------------
  Part of the openenergymonitor.org project
  
  Authors: Diogo Gomes based on original code from Glyn Hudson & Trystan Lea 
  
  Licence: GNU GPL V3

*/

/*

Change Log:
v2.4   14/02/16 Forked to use the RJ45 port as a Software Serial interface to other devices instead of a radio, output is JSON and removed temperature and pulse sensors
v2.3   16/11/15 Change to unsigned long for pulse count and make default node ID 8 to avoid emonHub node decoder conflict & fix counting pulses faster than 110ms, strobed meter LED http://openenergymonitor.org/emon/node/11490 
v2.2   12/11/15 Remove debug timming serial print code
v2.1   24/10/15 Improved timing so that packets are sent just under 10s, reducing resulting data gaps in feeds + default status code for no temp sensors of 3000 which reduces corrupt packets improving data reliability
V2.0   30/09/15 Update number of samples 1480 > 1662 to improve sampling accurancy: 1662 samples take 300 mS, which equates to 15 cycles @ 50 Hz or 18 cycles @ 60 Hz.
V1.9   25/08/15 Fix spurious pulse readings from RJ45 port when DS18B20 but no pulse counter is connected (enable internal pull-up)
V1.8 - 18/06/15 Increase max pulse width to 110ms
V1.7 - 12/06/15 Fix pulse count debounce issue & enable pulse count pulse temperature
V1.6 - Add support for multiple DS18B20 temperature sensors 
V1.5 - Add interrupt pulse counting - simplify serial print debug 
V1.4.1 - Remove filter settle routine as latest emonLib 19/01/15 does not require 
V1.4 - Support for RFM69CW, DIP switches and battery voltage reading on emonTx V3.4
V1.3 - fix filter settle time to eliminate large inital reading
V1.2 - fix bug which caused Vrms to be returned as zero if CT1 was not connected 
V1.1 - fix bug in startup Vrms calculation, startup Vrms startup calculation is now more accuratre

*/

#define emonTxV3                                                                          // Tell emonLib this is the emonTx V3 - don't read Vcc assume Vcc = 3.3V as is always the case on emonTx V3 eliminates bandgap error and need for calibration http://harizanov.com/2013/09/thoughts-on-avr-adc-accuracy/

#include "EmonLib.h"                                                                    // Include EmonLib energy monitoring library https://github.com/openenergymonitor/EmonLib
EnergyMonitor ct1, ct2, ct3, ct4;       

const byte version = 24;         // firmware version divided by 10 e,g 16 = V1.6

//----------------------------emonTx V3 Settings---------------------------------------------------------------------------------------------------------------
const byte Vrms=                  230;                               // Vrms for apparent power readings (when no AC-AC voltage sample is present)
const byte TIME_BETWEEN_READINGS = 5;            //Time between readings   

//http://openenergymonitor.org/emon/buildingblocks/calibration

const float Ical1=                90.9;                                 // (2000 turns / 22 Ohm burden) = 90.9
const float Ical2=                90.9;                                 // (2000 turns / 22 Ohm burden) = 90.9
const float Ical3=                90.9;                                 // (2000 turns / 22 Ohm burden) = 90.9
const float Ical4=                16.67;                               // (2000 turns / 120 Ohm burden) = 16.67

//float Vcal=                       268.97;                             // (230V x 13) / (9V x 1.2) = 276.9 Calibration for UK AC-AC adapter 77DB-06-09 
//float Vcal=276.9; 
float Vcal=                       240;//260;                     //  Calibration for EU AC-AC adapter 77DE-06-09 

const float phase_shift=          1.7;
const int no_of_samples=          1662; 
const int no_of_half_wavelengths= 30;
const int timeout=                2000;                               //emonLib timeout 
const int ACAC_DETECTION_LEVEL=   3000;
//-------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------


//----------------------------emonTx V3 hard-wired connections--------------------------------------------------------------------------------------------------------------- 
const byte LEDpin=                 6;                              // emonTx V3 LED
const byte DIP_switch1=            8;                              // Voltage selection 230 / 110 V AC (default switch off 230V)  - switch off D8 is HIGH from internal pullup
const byte DIP_switch2=            9;                              // RF node ID (default no chance in node ID, switch on for nodeID -1) switch off D9 is HIGH from internal pullup
const byte battery_voltage_pin=    7;                              // Battery Voltage sample from 3 x AA

#include <SoftwareSerial.h>
SoftwareSerial rj45Serial(3, 5); // RX, TX in the RJ45 Connection


//-------------------------------------------------------------------------------------------------------------------------------------------



// Note: Please update emonhub configuration guide on OEM wide packet structure change:
// https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md
typedef struct { 
  int power1, power2, power3, power4, Vrms; 
} PayloadTX;     // create structure - a neat way of packaging data for RF comms

PayloadTX emontx;

//-------------------------------------------------------------------------------------------------------------------------------------------

//Random Variables 
//boolean settled = false;
boolean CT1, CT2, CT3, CT4, ACAC, debug, DS18B20_STATUS; 
byte CT_count=0;

void setup()
{ 
  pinMode(LEDpin, OUTPUT); 

  digitalWrite(LEDpin,HIGH); 

  Serial.begin(57600);

  rj45Serial.begin(9600);

//READ DIP SWITCH POSITIONS 
  pinMode(DIP_switch1, INPUT_PULLUP);
  pinMode(DIP_switch2, INPUT_PULLUP);
  if (digitalRead(DIP_switch1)==LOW) debug = 1; else debug = 0;
 

  if (debug == 1) { 
    Serial.print("emonTx V3.4 Discrete Sampling V"); 
    Serial.print(version*0.1);
    Serial.println(" SERIAL");
  
    Serial.println("OpenEnergyMonitor.org");
    Serial.println("POST.....wait 10s");
  }
  delay(1000);
  
  emontx.power1=0;
  
  if (analogRead(1) > 0) {CT1 = 1; CT_count++;} else CT1=0;              // check to see if CT is connected to CT1 input, if so enable that channel
  if (analogRead(2) > 0) {CT2 = 1; CT_count++;} else CT2=0;              // check to see if CT is connected to CT2 input, if so enable that channel
  if (analogRead(3) > 0) {CT3 = 1; CT_count++;} else CT3=0;              // check to see if CT is connected to CT3 input, if so enable that channel
  if (analogRead(4) > 0) {CT4 = 1; CT_count++;} else CT4=0;              // check to see if CT is connected to CT4 input, if so enable that channel
  
  if ( CT_count == 0) CT1=1;                                             // If no CT's are connect ed CT1-4 then by default read from CT1

  // Quick check to see if there is a voltage waveform present on the ACAC Voltage input
  // Check consists of calculating the RMS from 100 samples of the voltage input.
  delay(10000);
  digitalWrite(LEDpin,LOW); 
  
  // Calculate if there is an ACAC adapter on analog input 0
  //double vrms = calc_rms(0,1780) * (Vcal * (3.3/1024) );
  double vrms = calc_rms(0,1780) * 0.87;
  if (vrms>90) ACAC = 1; else ACAC=0;
 
  if (ACAC) 
  {
    for (int i=0; i<10; i++)                                              // indicate AC has been detected by flashing LED 10 times
    { 
      digitalWrite(LEDpin, HIGH); delay(200);
      digitalWrite(LEDpin, LOW); delay(300);
    }
  }
  else 
  {
    delay(1000);
    digitalWrite(LEDpin, HIGH); delay(2000); digitalWrite(LEDpin, LOW);   // indicate DC power has been detected by turing LED on then off
  }
 
  //################################################################################################################################

  //if (Serial) debug = 1; else debug=0;          // if serial UART to USB is connected show debug O/P. If not then disable serial
  if (debug==1)
  {
    Serial.print("CT 1 Cal "); Serial.println(Ical1);
    Serial.print("CT 2 Cal "); Serial.println(Ical2);
    Serial.print("CT 3 Cal "); Serial.println(Ical3);
    Serial.print("CT 4 Cal "); Serial.println(Ical4);
    delay(1000);

    Serial.print("RMS Voltage on AC-AC  is: ~");
    Serial.print(vrms,0); Serial.println("V");
      
    if (ACAC) {
      Serial.println("AC-AC detected - Real Power measure enabled");
      Serial.println("assuming pwr from AC-AC (jumper closed)");
      Serial.print("Vcal: "); Serial.println(Vcal);
      Serial.print("Phase Shift: "); Serial.println(phase_shift);
    } else {
      Serial.println("AC-AC NOT detected - Apparent Pwr measure enabled");
      Serial.print("Assuming VRMS: "); Serial.print(Vrms); Serial.println("V");
      Serial.println("Assuming power from batt / 5V USB - power save enabled");
    }  

    if (CT_count==0) {
      Serial.println("NO CT's detected");
    } else {
      if (CT1) Serial.println("CT 1 detected");
      if (CT2) Serial.println("CT 2 detected");
      if (CT3) Serial.println("CT 3 detected");
      if (CT4) Serial.println("CT 4 detected");
    }

    Serial.print("CT1 CT2 CT3 CT4 VRMS/BATT");
    Serial.println(" "); 
    delay(500);  

  }
    
  if (CT1) ct1.current(1, Ical1);             // CT ADC channel 1, calibration.  calibration (2000 turns / 22 Ohm burden resistor = 90.909)
  if (CT2) ct2.current(2, Ical2);             // CT ADC channel 2, calibration.
  if (CT3) ct3.current(3, Ical3);             // CT ADC channel 3, calibration. 
  if (CT4) ct4.current(4, Ical4);             // CT ADC channel 4, calibration.  calibration (2000 turns / 120 Ohm burden resistor = 16.66) high accuracy @ low power -  4.5kW Max @ 240V 
  
  if (ACAC)
  {
    if (CT1) ct1.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT2) ct2.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT3) ct3.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
    if (CT4) ct4.voltage(0, Vcal, phase_shift);          // ADC pin, Calibration, phase_shift
  }

} //end SETUP

void loop()
{
  unsigned long start = millis();
  
  if (ACAC) {
    delay(200);                         //if powering from AC-AC allow time for power supply to settle    
    emontx.Vrms=0;                      //Set Vrms to zero, this will be overwirtten by CT 1-4
  }
  
  // emontx.power1 = 1;
  // emontx.power2 = 1;
  // emontx.power3 = 1;
  // emontx.power4 = 1;
  
  if (CT1) {
    if (ACAC) {
      ct1.calcVI(no_of_half_wavelengths,timeout); emontx.power1=ct1.realPower;
      emontx.Vrms=ct1.Vrms*100;
    } else {
      emontx.power1 = ct1.calcIrms(no_of_samples)*Vrms;                               // Calculate Apparent Power 1  1480 is  number of sample
    }
  }
  
  if (CT2) {
    if (ACAC) {
      ct2.calcVI(no_of_half_wavelengths,timeout); emontx.power2=ct2.realPower;
      emontx.Vrms=ct2.Vrms*100;
    } else {
      emontx.power2 = ct2.calcIrms(no_of_samples)*Vrms;                               // Calculate Apparent Power 1  1480 is  number of samples
    }
  }

  if (CT3) {
    if (ACAC) {
      ct3.calcVI(no_of_half_wavelengths,timeout); emontx.power3=ct3.realPower;
      emontx.Vrms=ct3.Vrms*100;
    } else {
      emontx.power3 = ct3.calcIrms(no_of_samples)*Vrms;                               // Calculate Apparent Power 1  1480 is  number of samples
    }
  }
  
  if (CT4) {
    if (ACAC) {
      ct4.calcVI(no_of_half_wavelengths,timeout); emontx.power4=ct4.realPower;
      emontx.Vrms=ct4.Vrms*100;
    } else {
      emontx.power4 = ct4.calcIrms(no_of_samples)*Vrms;                               // Calculate Apparent Power 1  1480 is  number of samples
    }
  }
  
  if (!ACAC){                                                                         // read battery voltage if powered by DC
    int battery_voltage=analogRead(battery_voltage_pin) * 0.681322727;                // 6.6V battery = 3.3V input = 1024 ADC
    emontx.Vrms= battery_voltage;
  }

  rj45Serial.print("{\"ct\": [");
  rj45Serial.print(emontx.power1); rj45Serial.print(", ");
  rj45Serial.print(emontx.power2); rj45Serial.print(", ");
  rj45Serial.print(emontx.power3); rj45Serial.print(", ");
  rj45Serial.print(emontx.power4); rj45Serial.print("], \"Vrms\": ");
  rj45Serial.print(emontx.Vrms);  
  rj45Serial.println(" }");
  delay(50);

  Serial.print("{\"ct\": [");
  Serial.print(emontx.power1); Serial.print(", ");
  Serial.print(emontx.power2); Serial.print(", ");
  Serial.print(emontx.power3); Serial.print(", ");
  Serial.print(emontx.power4); Serial.print("], \"Vrms\": ");
  Serial.print(emontx.Vrms); 
  Serial.println(" }");
  delay(50);

/*
  Serial.print(analogRead(1)); Serial.print("\t");
  Serial.print(analogRead(2)); Serial.print("\t");
  Serial.println(analogRead(3)); Serial.print("\t");
*/
  
  if (ACAC) {digitalWrite(LEDpin, HIGH); delay(200); digitalWrite(LEDpin, LOW);}    // flash LED if powered by AC
    
  unsigned long runtime = millis() - start;
  unsigned long sleeptime = (TIME_BETWEEN_READINGS*1000) - runtime - 100;
  
  delay(sleeptime);
}

double calc_rms(int pin, int samples)
{
  unsigned long sum = 0;
  for (int i=0; i<samples; i++) // 178 samples takes about 20ms
  {
    int raw = (analogRead(0)-512);
    sum += (unsigned long)raw * raw;
  }
  double rms = sqrt((double)sum / samples);
  return rms;
}


