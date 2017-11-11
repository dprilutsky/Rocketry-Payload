#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h> // GPS
#include <Adafruit_LSM9DS0.h> //accelerometer
#include <Adafruit_MPL3115A2.h> //barometer
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_Simple_AHRS.h>
#include <SD.h>
#include <SD_t3.h>
#define XBee Serial2

// Use I2C, ID #1000
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000); 
Adafruit_MPL3115A2 bar = Adafruit_MPL3115A2();
TinyGPS gps;
//Create simple AHRS algorithm using the LSMD9S0 instance's accelerometer and magnetometer
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
//Data array
float data[13];
//SD Card
File myFile;  
char fileName[50];
//XBee
int led = 13;
char rocketNumber; //which flight - sent via ground
long transmissionTime = 0; //time in ms of latest transmission
bool startRecording = false;
float barOffset = 0.0;
long barometerTime = 0; //time since last barometer transmission
String gpsData(TinyGPS &gps1);
String printFloat(double f, int digits = 2);
bool startTransmitting = true;
float xOffset = 0.0;

/*
 * Data Key:
 * 0: X acceleration (m/s^2) *
 * 1: Y acceleration (m/s^2) *
 * 2: Z acceleration (m/s^2) *
 * 3: X mag (gauss)
 * 4: Y mag (gauss)
 * 5: Z mag (gauss)
 * 6: Roll (degrees) *
 * 7: Pitch (degrees) *
 * 8: Heading (degrees) *
 * 9: Temperature (C) (from LSM9DS0)
 * 10: Pressure (Pascals)
 * 11: Altitude (m) *
 * 12: Temperature (C) (from MPL3115A2)
 * From GPS (not in array):
 * 13: Altitude (m) (From GPS) *
 * 14: Speed (m/s) *
 * 15: Latitude *
 * 16: Longitude *
 * 17: Fixage Time (ms since GPS data was encoded)
 * 18: Year
 * 19: Month  
 * 20: Day
 * 21: Hour
 * 22: Minute
 * 23: millis() *
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


  Serial.begin(9600);
  Serial1.begin(9600);
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Unable to initialize the accelerometer.");
    //while (1);
  }
  if (!bar.begin()) {
    Serial.println("Unable to initialize the barometer.");
    //return;
  }
  Serial.println("Found Sensors");
  //Serial.println("");
  //Serial.println("");
  //Setup the sensor gain and integration time.
  setupSensor();
  //XBee
  XBee.begin(9600);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  while (XBee.available() <= 0) {
   //Serial.println("waiting"); 
  }
  
//Wait for rocket number to be sent
  rocketNumber = (char) XBee.read();
  String stringFile = "launch" + (String) rocketNumber + ".txt";
  stringFile.toCharArray(fileName, stringFile.length()+1);
}
int numberPackets = 0;

void loop() 
{
  //start recording when char "r" is sent
  //delay(100);
  unsigned long timeGPS = millis();
  while (millis() - timeGPS < 5) {
  if (Serial1.available()) {
    char c = Serial1.read();
    if (gps.encode(c)) {
      break;
      }
    }
  }
  
  char testChar = 'z';
  if (XBee.available() > 0) {
    testChar = (char) XBee.read();
    Serial.println(testChar);

  }

   if (!startRecording && testChar == 'r') {
    
   if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("initialization failed!");
    }
    Serial.println("initialization done.");
    startRecording = true;
   }

   if (testChar == 's') {
    if (startTransmitting == true) startTransmitting = false;
    else if (startTransmitting == false) startTransmitting = true;
    Serial.println(startTransmitting);
   }

   //GPS code 


  //end program if character 'p' is receivevd
  if (startRecording && testChar == 'p'){
    startRecording = false;
    //exit(1);
  }

  //accelerometer code 
  sensors_vec_t orientation;
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  
  data[0] = accel.acceleration.x;
  data[1] = accel.acceleration.y;
  data[2] = accel.acceleration.z;
  data[3] = mag.magnetic.x;
  data[4] = mag.magnetic.y;
  data[5] = mag.magnetic.z;
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    data[6] = orientation.roll;
    data[7] = orientation.pitch;
    data[8] = orientation.heading;
  }
  //cancel out gravity
  if (xOffset = 0.0) xOffset = -1;
  data[0] = data[0] + 9.81*sin(data[7]*M_PI/180);
  data[1] = data[1] - 9.81*cos(data[7]*M_PI/180)*sin(data[6]*M_PI/180);
  data[2] = data[2] - 9.81*cos(data[7]*M_PI/180)*cos(data[6]*M_PI/180);
  data[9] = temp.temperature;
 // data[10] = bar.getPressure();
 
 //barometer code
 if (millis() - barometerTime >= 1000.0) {
  data[11] = bar.getAltitude();
  if (barOffset == 0.0) barOffset = -data[11];
  data[11] += barOffset;
  barometerTime = millis();
 }
  //data[12] = bar.getTemperature();
  
  // Compile string for sending/recording
  String message = "*#" + (String)data[0] + "#,#" + (String)data[1] + "#,#" + (String)data[2] + "#,#" + (String)data[6] + "#,#" + (String)data[7] + "#,#" + (String)data[8] + "#,#" + (String)data[11] + "#,#" ;
  //String message = "*#" + (String)data[0] + "#,#" + (String)data[1] + "#,#" + (String)data[2] + "#,#" + (String)1 + "#,#" + (String)1 + "#,#" + (String)1 + "#,#" + (String)data[11] + "#,#" ;
 
  message += gpsData(gps);

  message += (String)(millis()/1000.0) + "#&";

  // print message to serial on computer
  //Serial.println(message);


  // Write to SD card if we're recording
  if (startRecording) {
    myFile = SD.open(fileName, FILE_WRITE);
  
    if (myFile) {
      myFile.println(message);
      myFile.close();
    } else {
      Serial.println("SD card error");
    }
  }
    
    // XBee code
    char charArray[128];
    message.toCharArray(charArray, message.length() + 1);
    //Serial.println(millis() - transmissionTime);
    if (startTransmitting && (millis() - transmissionTime >= 150)) {
      XBee.print(charArray);
      transmissionTime = millis();
    }
    
 }

String gpsData(TinyGPS &gps)
{
  String data = "";
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;
  gps.get_position(&lat, &lon, &age);

  data += (String)(gps.f_altitude()) + "#,#" + (String)(gps.f_speed_mps()) + "#,#" + (String)(lat/10000000.0) + "#,#" + (String)(lon/10000000.0) + "#,#";

  //data += (String)(1) + "#,#" + printFloat(1) + "#,#" + (String)1 + "#,#" + (String)1 + "#,#";
  
  return data;
}

String printFloat(double number, int digits)
{
  String toReturn = "";
  // Handle negative numbers
  if (number < 0.0) {
     toReturn = "-";
     number = -number;
  }
  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;
  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  toReturn += (String)int_part;
  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    toReturn += ".";
  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    toReturn += (String)toPrint;
    remainder -= toPrint;
  }

  return toReturn;
}
