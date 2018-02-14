#include <Adafruit_LSM9DS0.h> //accelerometer
#include <Adafruit_MPL3115A2.h> //barometer
#include <Adafruit_Sensor.h>  // not used, but required
#include <Adafruit_Sensor_Set.h>
#include <Adafruit_Simple_AHRS.h> //library to more accurately convert gyro data to roll, pitch, and heading
#include <SD.h> //SD Card
#include <SD_t3.h>
#include <SPI.h>
#include <TinyGPS.h> // GPS
//#include <SoftwareSerial.h>
#include <Wire.h>
#define XBee Serial2 //transceiver

// Use I2C, ID #1000
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000); 
Adafruit_MPL3115A2 bar = Adafruit_MPL3115A2();
TinyGPS gps;
//Create simple AHRS algorithm using the LSMD9S0 instance's accelerometer and magnetometer
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());
//Store data before transmission
float data[7];
//File on the SD Card
File myFile;
int led = 13;
int rocketNumber; //marks which rocket other data corresponds to - sent via ground
float transmissionTime = -250; //time in ms of latest transmission
bool startRecording = false;
float barOffset = 0.0;
float barometerTime = -1000; //time since last barometer transmission
String gpsData(TinyGPS &gps1);
String printFloat(double f, int digits = 2);

//SoftwareSerial XBee(27, 26);

/*
 * Data Key:
 * 0: Roll (degrees) 
 * 1: Pitch (degrees) 
 * 2: Heading (degrees) 
 * 3: X acceleration (m/s^2) 
 * 4: Y acceleration (m/s^2) 
 * 5: Z acceleration (m/s^2) 
 * 6: Altitude (m) 
 * From GPS (not in array):
 * Altitude (m) 
 * Speed (m/s) 
 * Latitude 
 * Longitude 
 * millis() 
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
  Serial.setTimeout(50);
  XBee.setTimeout(50);
  Serial.begin(9600);
  Serial1.begin(9600);
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Unable to initialize the accelerometer.");
    return;
  }
  if (!bar.begin()) {
    Serial.println("Unable to initialize the barometer.");
    return;
  }
  Serial.println("Found accelerometer and barometer");
  //Serial.println("");
  //Serial.println("");
  //Setup the sensor gain and integration time.
  setupSensor();
  //XBee
  XBee.begin(9600);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  while (XBee.peek() == -1); //Wait for rocket number to be sent
  rocketNumber = XBee.read();
}
int numberPackets = 0;

void loop() 
{
  //start recording when char "r" is sent
  char testChar = XBee.read();
   if (testChar == 'r') {
    
   if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("initialization failed!");
    }
    Serial.println("initialization done.");
    myFile = SD.open("filename.txt", FILE_WRITE);
  
    if (myFile) {
      myFile.println("Starting new recording");
      myFile.printf("Rocket Launch: %d", rocketNumber);
      myFile.close();
    } 
    startRecording = true;
   }

   //GPS code 
  while (Serial1.available()) {
    char c = Serial1.read();
    if (gps.encode(c)) {
    // process new gps info here
    }
  }

  //end program if character 'p' is receivevd
  if (testChar == 'p'){
    myFile = SD.open("filename.txt", FILE_WRITE);
  
    if (myFile) {
      myFile.println("End Recording");
      myFile.close();
    } else {
      Serial.println("SD card error");
    }    
    exit(1);
  }

  //accelerometer code 
  sensors_vec_t orientation;
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  
  /* 'orientation' should have valid .roll and .pitch fields;
   *  grab orientation data first since gravity cancellation 
   *  calculations for acceleration depend on these values
   */
  if (ahrs.getOrientation(&orientation))
  {
    data[0] = orientation.roll;
    data[1] = orientation.pitch;
    data[2] = orientation.heading;
  }
  
  //cancel out gravity
  data[3] = accel.acceleration.x + 9.81*sin(data[7]*M_PI/180);
  data[4] = accel.acceleration.y - 9.81*cos(data[7]*M_PI/180)*sin(data[6]*M_PI/180);
  data[5] = accel.acceleration.z - 9.81*cos(data[7]*M_PI/180)*cos(data[6]*M_PI/180);
 
 //barometer code
 if (millis() - barometerTime >= 1000.0) {
  data[6] = bar.getAltitude();
  if (barOffset == 0.0) barOffset = -data[6];
  data[6] += barOffset;
  barometerTime = millis();
 }
  
  // Compile string for sending/recording
  String message = "*#" + (String)data[3] + "#,#" + (String)data[4] + "#,#" + (String)data[5] + "#,#" + (String)data[0] + "#,#" + (String)data[1] + "#,#" + (String)data[2] + "#,#" + (String)data[6] + "#,#" ;
  //String message = "*#" + (String)data[3] + "#,#" + (String)data[4] + "#,#" + (String)data[5] + "#,#" + (String)1 + "#,#" + (String)1 + "#,#" + (String)1 + "#,#" + (String)data[11] + "#,#" ;
 
  message += gpsData(gps);

  message += (String)(millis()/1000.0) + "#&";

  // print message to serial on computer
  Serial.print(message);


  // Write to SD card if we're recording
  if (startRecording) {
    myFile = SD.open("filename.txt", FILE_WRITE);
  
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
    if (millis() - transmissionTime >= 250) {
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

  data += /*printFloat(gps.f_altitude())*/ (String)(gps.altitude() / 100.0) + "#,#" + /*printFloat(gps.f_speed_mph()*0.44704)*/ (String)(gps.speed()*51.44) + "#,#" + (String)lat + "#,#" + (String)lon + "#,#";
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
