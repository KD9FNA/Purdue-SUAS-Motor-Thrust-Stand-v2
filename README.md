# Purdue-SUAS-Motor-Thrust-Stand
  Control code for thrust stand with data logging.  In the test, a brushless motor and prop slowly increase speed
  using the Servo.h library.  After reaching full throttle, the motor will slowly decrease throttle until it
  stops.  Throttle percentage and thrust are output over the serial terminal.  Safety checks are also implemented
  to stop the test if abnormal values are noticed on the sensors.

  Arduino Pinout:
  Digital:
  0 - Terminal RX
  1 - Terminal TX
  2 - Start switch
  3 -
  4 - Safety override switch
  5 - ESC calibration switch
  6 -
  7 -
  8 - Load cell data
  9 - Load cell clock
  10 - SD card Chip Select
  11 - ESC control pin
  12 -
  13 - LED pin
  SDA - I2C
  SCL - I2C
  Analog:
  A0 - Voltage sensor [DEPRECATED FOR EXTERNAL ADC]
  A1 - Current sensor [DEPRECATED FOR EXTERNAL ADC]
  A2 -
  A3 -
  A4 -
  A5 -

  ADC pinout
  VDD - Arduino 5v, 10k Thermistor
  GND - Arduino GND, 10k resistor to A0
  SCL - Arduino SCL
  SDA - Arduino SDA
  ADDR - SDA (Address 0x4A)
  ALRT -
  A0 - Thermistor, 10k resistor to GND
  A1 - Power module voltage sensor
  A2 - Power module current sensor
  A3 -

  Power module pinou
  1 - 5v (disconnected)
  2 - 5v (disconnected)
  3 - Current output (Green wire)
  4 - Voltage output (Blue wire)
  5 - Gnd (Purple wire)
  6 - Gnd (Disconnected)
