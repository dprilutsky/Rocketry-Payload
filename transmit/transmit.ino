#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Madgwick.h>
#include <Mahony.h>

#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h> // GPS
#include <Adafruit_LSM9DS0.h> //accelerometer
#include <Adafruit_MPL3115A2.h> //barometer
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_Simple_AHRS.h>
//#include <SD.h>
//#include "SoftwareSerial.h"
// Use I2C, ID #1000
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000); 
Adafruit_MPL3115A2 bar = Adafruit_MPL3115A2();
TinyGPS gps;
// You can also use software SPI
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(13, 12, 11, 10, 9);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(10, 9);
//Create simple AHRS algorithm using the LSMD9S0 instance's accelerometer and magnetometer
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
//Data array
float data[24];
//SD Card
//File myFile;
//XBee
int led = 13;

//SoftwareSerial XBee(0, 1);
/*
 * Data Key:
 * 0: X acceleration (m/s^2)
 * 1: Y acceleration (m/s^2)
 * 2: Z acceleration (m/s^2)
 * 3: X mag (gauss)
 * 4: Y mag (gauss)
 * 5: Z mag (gauss)
 * 6: Roll (degrees)
 * 7: Pitch (degrees)
 * 8: Heading (degrees)
 * 9: Temperature (C) (from LSM9DS0)
 * 10: Pressure (Pascals)
 * 11: Altitude (m)
 * 12: Temperature (C) (from MPL3115A2)
 * 13: Altitude (m) (From GPS)
 * 14: Speed (m/s)
 * 15: Latitude
 * 16: Longitude
 * 17: Fixage Time (ms since GPS data was encoded)
 * 18: Year
 * 19: Month  
 * 20: Day
 * 21: Hour
 * 22: Minute
 */
void setupSensor()
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}
void setup() 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println("LSM raw read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Unable to initialize the LSM9DS0.");
    while (1);
  }
  Serial.println("Found LSM9DS0 9DOF");
  Serial.println("");
  Serial.println("");
  //Setup the sensor gain and integration time.
  setupSensor();
  //XBee
  Serial1.begin(9600);//Serial1 is the XBee
  pinMode(led, OUTPUT);
}
//XBee
int numberPackets = 0;
void loop() 
{
  
  sensors_vec_t orientation;
  lsm.read();
  
  data[0] = lsm.accelData.x;
  data[1] = lsm.accelData.y;
  data[2] = lsm.accelData.z;
  data[3] = lsm.magData.x;
  data[4] = lsm.magData.y;
  data[5] = lsm.magData.z;
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    data[6] = orientation.roll;
    data[7] = orientation.pitch;
    data[8] = orientation.heading;
  }
  
  data[9] = lsm.temperature;
  data[10] = bar.getPressure();
  data[11] = bar.getAltitude();
  data[12] = bar.getTemperature();
  data[13] = gps.f_altitude();
  data[14] = gps.f_speed_mps();
  data[15] = millis();
  /*float flat, flon, fix_age;
  unsigned long age;
  int year;
  byte month,day, hour,minute, second;
  data[15] = (float)gps.f_get_position(&flat, &flon, &age);
  data[16] = gps.crack_datetime(&year);
  data[17] = gps.crack_datetime(&month);
  data[18] = gps.crack_datetime(&day);
  data[19] = gps.crack_datetime(&hour);
  data[20] = gps.crack_datetime(&minute);
  data[21] = gps.crack_datetime(&second);*/
  String message = "";
  Serial.println("hi");
  for(int i = 0; i < 24; i++)
  {
    /*Serial.print(data[i]);
    Serial.print(" ");*/
    message += (String)data[i] + " // ";
  }
  //Serial.println();
  //SD Card (filename.txt will be data file):
  /*myFile = SD.open("filename.txt", FILE_WRITE);
  if(myFile)  {
    myFile.println(whatever string we want);
    myFile.close();
  } else  {
    Serial.println("Error opening file on SD card");
  }
  */
  //XBee
  
  // put your main code here, to run repeatedly:
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  if (Serial1.available() > 0) {
    String receivedstr = Serial1.readString();
    char received[64];
    receivedstr.toCharArray(received, 64);
    Serial1.write(received); //sending to xbee
    Serial1.write("\n");
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  //In message, we will put the string we are going to send (the same one that went to the SD card) - or can the XBee send an array?
  //Is it possible to do XBee.write(Serial.read()) ?
  //message = String("\nTestPacket Number: ").concat(String(numberPackets).concat(String(" || Time of packet send: ").concat(millis()))); // + numberPackets + " || Time of packet send: " + millis());
  char charArray[128];
  message.toCharArray(charArray, 128);
  Serial1.write(charArray);
    
   
    //digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
  //}
  
 
  delay(1000);
}
