//Include Libraries
#include <Servo.h>
#include <HX711.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_ADS1015.h>
#include <SPI.h>
#include <SD.h>


#if defined(ARDUINO_ARCH_SAMD)
// Used to provide support for Arduino Zero
#define Serial SerialUSB
#endif

// Initialize libraries
Servo Motor;  //create motor object
const byte hx711_data_pin = 8;
const byte hx711_clock_pin = 9;
HX711 scale(hx711_data_pin, hx711_clock_pin);
Adafruit_ADS1115 ads1115(0x4A);  // construct an ads1115 at address 0x49

//RTC stuff
RTC_PCF8523 rtc;

// pin assignments
const int StartSwitch = 2;
const int SafetyOverride = 4;
const int EscCal = 5;
const int MotorPin = 11;
const int LightPin = 13;
const int ChipSelect = 10;

// state tracking.  You shouldn't have to do anything here.
int MotorValuePercent = 0;  // Current value of motor output in % throttle
int MotorValue = 0;  // Current value of motor after converting from % throttle
bool StartState = false;  // This tells void loop() that the test is running.  True = spinning!
bool Safe = false;  // If any errors happen, Safe value is false and test aborts
int DirState = 0;  // 0 is increasing throttle, 1 is decreasing throttle
int SafetyBool = false;
bool EscCalBool = false;
int i = 0;  //Used for ESC calibration routine
unsigned long Hx711Raw = 0;
int16_t RawVolt = 0;
int16_t RawCurrent = 0;
float Volt = 0;
float Current = 0;
bool EscCalStop = false;
unsigned long UnixTime = 0;
bool TestComplete = false;
bool Started = false;  // Used in the start loop for tracking if the test was started already or not
bool FirstStart = false;  // Used to prevent accidental startups on reboot
float TempRaw = 0;
float TempCal = 0;
bool FirstError = false;  // Trips with the first safety check that fails
int AdcFailCount = 0;  //Used to detect if the ADC is at fault or the sensors
int LightState = LOW;
const int CountdownTimer = 3;  // Number of seconds in start countdown timer
unsigned long PreviousIncrement = 0;  // Used to track when the last increment change happened
unsigned long PreviousReading = 0;
unsigned long CurrentMillis = 0;
unsigned long PreviousFlash = 0;
unsigned long PreviousError = 0;  // Tracks time since previous error for LED status indicator
unsigned long SdFileName = 0;  // Names for the SD files are based off the RTC time
String SdFileString;  // format of file and combination of the two for SD logging

// Calibration values and other variables
const int MotorMax = 180;  // Maximum value of motor during test in degrees
const int MotorMin = 0;  // Minimum value of motor during test in degrees
const float VoltMultiplier = 523;  // Multiplier for calibrating voltmeter
const float CurrentMultiplier = 473.6;  // Multiplier for calibrating current sensor
const float TempOffset = -37.47;  // Directly adds or subtracts from TempCal value before outputting to terminal
const float VoltOffset = 0;
const float CurrentOffset = 0;
const uint16_t EscMin = 1100;  // Used to calibrate range of ESC output to the Servo library.  Change if PWM range is innacurate
const uint16_t EscMax = 1750;
const int MinVolt = 5;  // Minimum voltage for calibration routine to start
const int IncrementTime = 1000;  // Time between throttle increments.  Default 1000
const int ReadingTime = 1000;  // Time between readings.
const float LoadCal = -70226.5;  // Calibration value for load cell


void adccheck() {  // Only compiles for Mega boards.  Uno doesn't have enough memory for this safety check.
  //#if defined(__AVR_ATmega1280__) or defined(__AVR_ATmega2560__)
  AdcFailCount = 0;
  int16_t adc0, adc1, adc2, adc3;  // set variables for ADC
  adc0 = ads1115.readADC_SingleEnded(0);  // read ADC
  adc1 = ads1115.readADC_SingleEnded(1);
  adc2 = ads1115.readADC_SingleEnded(2);
  adc3 = ads1115.readADC_SingleEnded(3);
  //Serial.println(adc0);  // For debugging use
  //Serial.println(adc1);
  //Serial.println(adc2);
  //Serial.println(adc3);
  if ((adc0 <= 3000)) {
    Serial.println(F("ERROR: Thermistor disconnected!  Check connections between Thermistor and ADC"));
    Safe = false;
    AdcFailCount = 1;
    PreviousError = millis();
    fastheartbeat();
  }
  if (((adc1 <= 3000) && (adc1 > 2700))) {
    Serial.println(F("WARNING: Voltage sensor might be disconnected!  Check wiring between voltage sensor and ADC"));
    Safe = false;
    AdcFailCount = AdcFailCount + 1;
    PreviousError = millis();
    fastheartbeat();
  }
  if (((adc2 <= 3000) && (adc2 > 2700))) {
    Serial.println(F("WARNING: Current Sensor might be disconnected!  Check wiring between current sensor and ADC"));
    Safe = false;
    AdcFailCount = AdcFailCount + 1;
    PreviousError = millis();
    fastheartbeat();
  }
  if (AdcFailCount >= 3) {
    Serial.println(F("ERROR: Either all sensors between ADC are disconnected or ADC is not communicating with Arduino"));
    Serial.println(F("      Check wiring between ADC and Arduino"));
    Safe = false;
    PreviousError = millis();
    fastheartbeat();
  }
  if ((adc0 == -1) and (adc1 == -1) and (adc2 == -1) and (adc3 == -1)) {
    Serial.println(F("ERROR: ADC is not communicating with Arduino"));
    Serial.println(F("      Check wiring between Arduino and ADC"));
    Serial.println(F("      If wiring is correct, ADC may be malfunctioning"));
    PreviousError = millis();
    fastheartbeat();
  }
  //#endif
}


void start() { //Interrupt code to change start state and throttle value
  delay(20); // Debounce switch
  if (((digitalRead(StartSwitch)) == LOW) && (Started == false) && (FirstStart == true)) {
    Serial.println(F("Begin test"));
    FirstError = false;
    Started = true;
    Safe = true;
    adccheck();
    DateTime now = rtc.now();
    scale.tare(); // reset the scale to 0
    Hx711Raw = scale.read();
    if (Hx711Raw == 0) { //Load cell check
      Serial.println(F("ERROR: Load cell amplifier not responsive.  Thrust data inaccurate!"));
      Serial.println(F("      Check connections to load cell amplifier and restart test."));
      Safe = false;
      PreviousError = millis();
      fastheartbeat();
    }
    else {
      Serial.println("Load cell tare");
      scale.tare(); // reset the scale to 0
    }
    if (UnixTime <= 1536522595) {
      Serial.println(F("ERROR: Clock time invalid!"));
      Serial.println(F("      Please reprogram RTC with current time or use safety override switch"));
      Safe = false;
      PreviousError = millis();
      fastheartbeat();
    }
    else {
      Serial.println(F("Clock time valid"));
    }
    SdFileString = String(now.unixtime());
    SdFileString += ".txt";
    Serial.print(F("SD card file name: "));
    Serial.println(SdFileString);
    File dataFile = SD.open(SdFileString, FILE_WRITE);
    dataFile.println(F("UNIX Time,Motor Thrust %,Thrust Uncalibrated,Thrust Calibrated,Voltage,Current,Temp Raw,Temp Calibrated C"));
    dataFile.close();
    
    if ((Safe == true) or (SafetyBool == true)) {
      StartState = true;
      MotorValuePercent = 0;
      DirState = 0;
      // print warning over serial terminal
      for (int i = CountdownTimer; ((i >= 0) && (StartState == true)); i--) {
        Serial.print(F("MOTOR STARTING IN "));
        Serial.print(i);
        Serial.println(F(" SECONDS!!!  CLEAR PROP!!!"));
        delay(1000);
      }
      if (StartState == true) {
        Serial.println(F("MOTOR STARTING!!!"));
        delay(500);
        Serial.println(F("UNIX Time,Motor Thrust %,Thrust Uncalibrated,Thrust Calibrated,Voltage,Current,Temp Raw,Temp Calibrated C"));
      }
    }
    if ((Safe = false) && (StartState == false)) {
      Serial.println("Something went wrong.  Check above errors for more information.  Aborting test.");
      Started = false;
      PreviousError = millis();
      fastheartbeat();
    }
  }
}


void quit() {  // Interrupt loop used to stop the experiment if the arm switch is flipped
  delay(20);
  if (((digitalRead(StartSwitch)) == HIGH) && (StartState == true)) {
    Motor.write(0);
    StartState = false;
    MotorValuePercent = 0;
    Serial.println(F("Test aborted"));
    TestComplete = false;
    Started = false;
    delay (1000);
  }
  else {
    Motor.write(0);
    StartState = false;
    MotorValuePercent = 0;
    TestComplete = false;
  }
}


void safecheck() { // Failsafes for execution within the test
  if ((Hx711Raw == 0) && (FirstError = false)) { //Load cell check
    Serial.println(F("ERROR: Load cell amplifier not responsive.  Thrust data inaccurate!"));
    Serial.println(F("      Check connections to load cell amplifier and restart test."));
    Safe = false;
    FirstError = true;
    PreviousError = millis();
    fastheartbeat();
  }
  if ((UnixTime <= 1536552595) && (FirstError = false)) {
    Serial.print(F("ERROR: Clock time invalid!  Current time: "));
    Serial.println(UnixTime);
    Serial.println(F("      Please reprogram RTC with current time or use safety override switch"));
    Safe = false;
    PreviousError = millis();
    fastheartbeat();
  }
  adccheck();
}


void calibrate() {
  Serial.println(F("ESC calibration routine started!  Motor may start spinning at full throttle!!!"));
  delay(1000);
  int16_t adc0, adc1, adc2, adc3;  // set variables for ADC
  adc1 = ads1115.readADC_SingleEnded(1);
  Volt = (adc1 / VoltMultiplier) + VoltOffset;
  if ((Volt <= MinVolt)) {
    Serial.println(F("Sending ESC 100% throttle.  Plug in ESC"));
    while (digitalRead(EscCal) == LOW) {
      Motor.write(100);
      delay(50);
      fastheartbeat();
    }
    Motor.write(0);
    Serial.println(F("ESC calibration complete!"));
  }
  else {
    Serial.println(F("ABORT: Batteries connected.  Remove batteries from ESC, restart, and try again"));
    delay(3000);
  }
}

void fastheartbeat() {  // Used to indicate errors in the status LED
  CurrentMillis = millis();
  if (CurrentMillis - PreviousError <= 3000) {  // Only run error flashes for 3 secconds
    if (CurrentMillis - PreviousFlash >= 300) {
      PreviousFlash = CurrentMillis;
      if (LightState == LOW) {
        LightState = HIGH;
      } else {
        LightState = LOW;
      }
      digitalWrite(LightPin, LightState);
    }
  }
}

void heartbeat() {  // Simple flashing LED to show that the Arduino is in fact alive
  CurrentMillis = millis();
  if (CurrentMillis - PreviousFlash >= 1000) {
    PreviousFlash = CurrentMillis;
    if (LightState == LOW) {
      LightState = HIGH;
    } else {
      LightState = LOW;
    }
    digitalWrite(LightPin, LightState);
  }
}

void readsensor() {
  File dataFile = SD.open("Thrust stand v2.0 data.txt", FILE_WRITE);
  //if (dataFile) {
  CurrentMillis = millis();
  if ((CurrentMillis - PreviousReading) >= ReadingTime) {
    PreviousReading = CurrentMillis;
    int16_t adc0, adc1, adc2, adc3;  // set variables for ADC
    //float VoltAdc0, VoltAdc1, VoltAdc2, VoltAdc3;
    adc0 = ads1115.readADC_SingleEnded(0);  // read ADC
    adc1 = ads1115.readADC_SingleEnded(1);
    adc2 = ads1115.readADC_SingleEnded(2);
    adc3 = ads1115.readADC_SingleEnded(3);
    DateTime now = rtc.now();
    Serial.print(now.unixtime());
    dataFile.print(now.unixtime());
    UnixTime = (now.unixtime());
    Serial.print(F(","));
    dataFile.print(F(","));
    Serial.print(MotorValuePercent);
    dataFile.print(MotorValuePercent);
    Serial.print(F(","));
    dataFile.print(F(","));
    Serial.print(scale.read());
    dataFile.print(scale.read());
    Serial.print(F(","));
    dataFile.print(F(","));
    Serial.print(scale.get_units(), 6);
    dataFile.print(scale.get_units(), 6);
    Serial.print(F(","));
    dataFile.print(F(","));
    Volt = (adc1 / VoltMultiplier) + VoltOffset;
    /*RawVolt = analogRead(A0);  // Use onboard ADC
      Volt = RawVolt * VoltMultiplier;*/
    Serial.print(Volt);
    dataFile.print(Volt);
    Serial.print(F(","));
    dataFile.print(F(","));
    //VoltAdc2 = map(adc2, 0, 65535, -6.144, 6.144);
    Current = (adc2 / CurrentMultiplier) + CurrentOffset;
    /*RawCurrent = analogRead(A1);  // Use onboard ADC
      Current = RawCurrent * CurrentMultiplier;*/
    Serial.print(Current);
    dataFile.print(Current);
    Serial.print(F(","));
    dataFile.print(F(","));
    TempRaw = adc0;  //
    Serial.print(TempRaw);
    dataFile.print(TempRaw);
    Serial.print(F(","));
    dataFile.print(F(","));
    TempCal = (65535 / TempRaw) - 1;
    TempCal = 10000 / TempCal;  // TempCal = thermistor resistance hare
    float steinhart;  //Use the Steinhart equation to convert resistance to temperature
    steinhart = TempCal / 10000;       // (R/Ro)
    steinhart = log(steinhart);        // ln(R/Ro)
    steinhart /= 3950;                 // 1/B * ln(R/Ro)
    steinhart += 1.0 / (25 + 273.15);  // + (1/To)
    steinhart = 1.0 / steinhart;       // Invert
    steinhart -= 273.15;               // convert to C
    TempCal = steinhart + TempOffset;
    Serial.println(TempCal);
    dataFile.println(TempCal);
    dataFile.close();
  }
  //}
  /*else {
    Serial.println(F("error opening datalog file"));
    }*/
}


void setup() {
  Motor.attach(MotorPin, EscMin, EscMax);
  Motor.write(0);  // These lines ABSOLUTELY MUST BE the first lines executed during power up or ESC may arm prematurely
#ifndef ESP8266
  while (!Serial); // for Leonardo/Micro/Zero
#endif
  // set pinmodes
  pinMode(StartSwitch, INPUT_PULLUP);
  pinMode(SafetyOverride, INPUT_PULLUP);
  pinMode(EscCal, INPUT_PULLUP);
  pinMode(LightPin, OUTPUT);

  //Motor.setMinimumPulse(EscMin);
  //Motor.setMaximumPulse(EscMax);

  Serial.begin(9600);
  delay (2000);
  Serial.println(F("Purdue Thrust Stand v2.0 by Ryan Ferguson and Evan Hockridge"));

  Serial.println(F("Initializing SD card..."));
  if (!SD.begin(ChipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    //return;
  }
  //Serial.println(F("card initialized."));
  if (! rtc.begin()) {  //Initialize RTC
    Serial.println(F("ERROR: Couldn't find RTC"));
    while (1);
  }
  if (! rtc.initialized()) {
    Serial.println(F("RTC is NOT running!"));
  }
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(2018, 9, 9, 19, 53, 0));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2018 at 3am you would call:
  // rtc.adjust(DateTime(2018, 1, 21, 3, 0, 0));
  if ((digitalRead(EscCal)) == 0) { //ESC calibration routine
    calibrate();
  }
  scale.set_scale(LoadCal);
  scale.tare(); // reset the scale to 0
  ads1115.begin();  // Initialize ads1115
  ads1115.setGain(GAIN_TWOTHIRDS);  //Set gain of ADC to 2/3, or +/- 6.144v.  1 bit = 3mV
  Serial.print(F("Output order: "));
  Serial.println(F("UNIX Time,Motor Thrust %,Thrust Uncalibrated,Thrust Calibrated,Voltage,CurrentTemp Raw,Temp Calibrated"));
  readsensor();
  attachInterrupt(digitalPinToInterrupt(StartSwitch), quit, RISING); // create interrupt for stop tracking.
  Serial.println(F("Ready to test"));
}


void loop() {
  if ((digitalRead(StartSwitch)) == HIGH) { // Detect that the run switch has in fact been off when sketch started running
    FirstStart = true;
  }
  if ((StartState == false) && (CurrentMillis - PreviousError >= 3000)) {
    heartbeat();
  }
  CurrentMillis = millis();
  if (((digitalRead(StartSwitch)) == LOW) && (StartState == false) && (TestComplete == false)) {
    start();
  }
  if (((digitalRead(StartSwitch)) == HIGH) && (TestComplete == true)) {
    quit();
    TestComplete = false;
    Started = false;
  }
  if ((digitalRead(StartSwitch) == HIGH) && (StartState == true)) { //Stop the test if the switch is off
    quit();
    Motor.write(0);
    StartState = false;
    MotorValuePercent = 0;
    Started = false;
  }
  if ((DirState == 0) && (StartState == true)) {  // Increasing throttle
    CurrentMillis = millis();
    digitalWrite(LightPin, HIGH);
    //Serial.println("Increasing");
    if (CurrentMillis - PreviousIncrement >= IncrementTime) {
      PreviousIncrement = CurrentMillis;
      //Serial.println("DirState 0");  // For debugging use
      MotorValuePercent = MotorValuePercent + 1;
      MotorValue = map(MotorValuePercent, 0, 100, EscMin, EscMax);
      Motor.writeMicroseconds(MotorValue);
      //readsensor();
      safecheck();
    }
  }
  if ((DirState == 1) && (StartState == true)) {  // Decreasing throttle
    CurrentMillis = millis();
    digitalWrite(LightPin, HIGH);
    //Serial.println("Dereasing");
    if (CurrentMillis - PreviousIncrement >= IncrementTime) {
      PreviousIncrement = CurrentMillis;
      //Serial.println("DirState 1");  // For debugging use
      MotorValuePercent = MotorValuePercent - 1;
      MotorValue = map(MotorValuePercent, 0, 100, EscMin, EscMax);
      Motor.writeMicroseconds(MotorValue);
      //readsensor();
      safecheck();
    }
  }
  if (StartState == true) {  // Run the readsensor script if the test is running
    readsensor();
  }
  if ((MotorValuePercent == 0) && (StartState == true)) { // End test
    DirState = 0;
    StartState = false;
    TestComplete = true;
    Started = false;
    Serial.println(F("Test complete!"));
  }
  if (StartState == false) { // If quit interrupt has been used during loop, motor may start up unexpectedly
    if (Motor.read() >= 0) {
      Motor.write(15);  // Used to make sure ESC receives a throttle value within its PWM range
      delay(25);
    }
    Motor.writeMicroseconds(EscMin);          // Shut off motor
  }
  if (MotorValuePercent == 100) { // Change from increasing to decreasing throttle when MotorMax reached
    DirState = 1;
  }
  if ((digitalRead(SafetyOverride) == 0) && (SafetyBool == false) && (StartState == false)) { // Enable safety override
    SafetyBool = true;
    Serial.println(F("WARNING: Auto hardware checks disabled!"));
    Started = false;
  }
  if ((digitalRead(SafetyOverride) == 1) && (SafetyBool == true) && (StartState == false)) { // Disable safety override
    SafetyBool = false;
    Serial.println(F("Auto hardware checks re-enabled!"));
  }
}
