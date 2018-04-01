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
//#include <SD_t3.h>
#define XBee Serial2

#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

const int c = 261;
const int d = 294;
const int e = 329;
const int f = 349;
const int g = 391;
const int gS = 415;
const int a = 440;
const int aS = 455;
const int b = 466;
const int cH = 523;
const int cSH = 554;
const int dH = 587;
const int dSH = 622;
const int eH = 659;
const int fH = 698;
const int fSH = 740;
const int gH = 784;
const int gSH = 830;
const int aH = 880;
 
const int buzzerPin = 6;

File root;

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
long noteDuration = 0;
char rocketNumber; //which flight - sent via ground
long transmissionTime = 0; //time in ms of latest transmission
bool startRecording = false;
float barOffset = 0.0;
long barometerTime = 0; //time since last barometer transmission
String gpsData(TinyGPS &gps1);
String printFloat(double f, int digits = 2);
bool startTransmitting = true;
float xOffset = 0.0;
bool startBuzzer = false;

char recordingOutput = 'p';
char transmittingOutput = 't';
char buzzerOutput = 'f';

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


  XBee.begin(9600);
  Serial1.begin(9600);
  //march();
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    XBee.println("Unable to initialize the accelerometer.");
    march();
    //while (1);
  }
  if (!bar.begin()) {
    XBee.println("Unable to initialize the barometer.");
    march();
    //return;
  }
  XBee.println("Found Sensors");
  
  //XBee.println("");
  //XBee.println("");
  //Setup the sensor gain and integration time.
  setupSensor();
  //XBee
  XBee.begin(9600);
  pinMode(led, OUTPUT);

//high pitch
  beep(3671, 150);
  beep(3671, 300);
  beep(3087, 300);
  beep(3671, 150);
  beep(3671, 300);
  beep(3087, 300);
  beep(3671, 150);
  beep(4365, 300);
  beep(4120, 300);
  beep(4365, 70);
  beep(4120, 70);
  beep(3671, 150);
  beep(3087, 500);

//low pitch
//  beep(367, 150);
//  beep(367, 300);
//  beep(308, 300);
//  beep(367, 150);
//  beep(367, 300);
//  beep(308, 300);
//  beep(367, 150);
//  beep(436, 300);
//  beep(412, 300);
//  beep(436, 70);
//  beep(412, 70);
//  beep(367, 150);
//  beep(308, 500);
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)

  while (XBee.available() <= 0) {
   //XBee.println("waiting"); 
  }
  XBee.println("Xbee Ready!");
  
//Wait for rocket number to be sent
  rocketNumber = (char) XBee.read();
  String stringFile = "launch" + (String) rocketNumber + ".txt";
  stringFile.toCharArray(fileName, stringFile.length()+1);
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
  }
  myFile = SD.open(fileName, FILE_WRITE);
  if (SD.exists("example.txt")) {
    SD.remove(fileName);
  }
  XBee.println("Launch " + rocketNumber);
  
  tone(6, 1109, 1000);
  noteDuration = millis();

}
int numberPackets = 0;

void loop() 
{
  //start recording when char "r" is sent
  //delay(100);
  unsigned long timeGPS = millis();
  while (millis() - timeGPS < 5) {
  if (Serial1.available()) {
    Serial1.println("In while (millis() - timeGPS < 5) { while loop");
    char c = Serial1.read();
    Serial1.println("Received char: " + c);
    if (gps.encode(c)) {
      break;
      }
    }
  }
  
  char testChar = 'z';
  if (XBee.available()){// > 0) {
    testChar = (char) XBee.read();
    //Serial1.println(testChar);
    XBee.println("Received char: " + c);
  }

  if (millis() - noteDuration >= 1000){
  noTone(buzzerPin);
  }

  if (startBuzzer) {
    tone(6,5000,100000);
    noteDuration = 10000000000000000000000000000000000;
    //startBuzzer = false;
  }

  // if teensey receives code "a," reboot.
  if(testChar == 'a') {
    XBee.println("restarting . . .");
    WRITE_RESTART(0x5FA0004);
  }

  if (testChar == 'b') {
    XBee.println("buzzing");
    buzzerOutput = 'b';
    startBuzzer = true;
  }
  if (testChar == 'B') {
    buzzerOutput = 'f';
    XBee.println("stopped buzzing");
    startBuzzer = false;
  }


   if (!startRecording && testChar == 'r') {
    
   if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("initialization failed!");
    }
    recordingOutput = 'r';
    tone(6, 5000, 1000);
    noteDuration = millis();
 
  //Stop tone on buzzerPin
    XBee.println("recording");
    startRecording = true;
   }

   if (testChar == 'L') {
      XBee.println("Current SD Files:");
      if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("initalization failed!");
      }
      root = SD.open("/");
      tone(6, 5000, 1000);
      noteDuration = millis();
      printDirectory(root, 0);
   }

   if (testChar == 'T') {
    tone(6, 5000, 1000);
    noteDuration = millis();
    while (XBee.available() <= 0) {
     //XBee.println("waiting"); 
    }    
        tone(6, 5000, 1000);
    noteDuration = millis();
    //Wait for rocket number to be sent
    char requestedNumber = (char) XBee.read();
    XBee.println("Incoming file " + requestedNumber);
    String requestedFile = "launch" + (String) requestedNumber + ".txt";
    char requestedFileName[50];
    requestedFile.toCharArray(requestedFileName, requestedFile.length()+1);
    
    if (!SD.begin(BUILTIN_SDCARD)) {
      Serial.println("initalization failed!");
    }
    File dataFile = SD.open(requestedFileName);

    // if the file is available, write to it:
    if (dataFile) {
    while (dataFile.available()) {
      XBee.write(dataFile.read());
    }
    dataFile.close();
    }
        tone(6, 5000, 1000);
    noteDuration = millis();
   }

   if (testChar == 's') {
    if (startTransmitting == true) {
      startTransmitting = false;
      transmittingOutput = 's';
      XBee.println("transmission stopping");
    }
    else if (startTransmitting == false){
      startTransmitting = true;
      transmittingOutput = 't';
          XBee.println("transmission starting");

    }
    tone(6, 500, 1000);
    noteDuration = millis();
   }

   //GPS code 


  //pause recording
  if (startRecording && testChar == 'p'){
    startRecording = false;
    recordingOutput = 'p';
    tone(6, 500, 1000);
    noteDuration = millis();
    XBee.println("SD paused");
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
  String message = "*#" + (String) rocketNumber + (String) recordingOutput + (String) transmittingOutput + (String) buzzerOutput + "#,#" + (String)data[0] + "#,#" + (String)data[1] + "#,#" + (String)data[2] + "#,#" + (String)data[6] + "#,#" + (String)data[7] + "#,#" + (String)data[8] + "#,#" + (String)data[11] + "#,#" ;
  //String message = "*#" + (String)data[0] + "#,#" + (String)data[1] + "#,#" + (String)data[2] + "#,#" + (String)1 + "#,#" + (String)1 + "#,#" + (String)1 + "#,#" + (String)data[11] + "#,#" ;
 
  message += gpsData(gps);

  message += (String)(millis()/1000.0) + "#&";

  // print message to serial on computer
  Serial.println(message);


  // Write to SD card if we're recording
  if (startRecording) {
    myFile = SD.open(fileName, FILE_WRITE);
  
    if (myFile) {
      myFile.println(message);
      myFile.close();
    } else {
      XBee.println("SD card error");
    }
  }
    
    // XBee code
    char charArray[128];
    message.toCharArray(charArray, message.length() + 1);
    //XBee.println(millis() - transmissionTime);
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

  data += (String)(gps.f_altitude()) + "#,#" + (String)(gps.f_speed_mps()) + "#,#" + String(lat/10000000.0, 5) + "#,#" + String(lon/10000000.0, 5) + "#,#";

  //data += (String)(1) + "#,#" + printFloat(1) + "#,#" + (String)1 + "#,#" + (String)1 + "#,#";
  
  return data;
}

void beep(int note, int duration)
{
  //Play tone on buzzerPin
  tone(buzzerPin, note, duration);
 
  
    delay(duration);
 
  //Stop tone on buzzerPin
  noTone(buzzerPin);
 
  delay(50);
 
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

void firstSection()
{
  beep(a, 500);
  beep(a, 500);    
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);  
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  delay(500);
 
  beep(eH, 500);
  beep(eH, 500);
  beep(eH, 500);  
  beep(fH, 350);
  beep(cH, 150);
  beep(gS, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  delay(500);
}
 
void secondSection()
{
  beep(aH, 500);
  beep(a, 300);
  beep(a, 150);
  beep(aH, 500);
  beep(gSH, 325);
  beep(gH, 175);
  beep(fSH, 125);
  beep(fH, 125);    
  beep(fSH, 250);
 
  delay(325);
 
  beep(aS, 250);
  beep(dSH, 500);
  beep(dH, 325);  
  beep(cSH, 175);  
  beep(cH, 125);  
  beep(b, 125);  
  beep(cH, 250);  
 
  delay(350);
}

void march()
{
 
  //Play first section
  firstSection();
 
  //Play second section
  secondSection();
 
  //Variant 1
  beep(f, 250);  
  beep(gS, 500);  
  beep(f, 350);  
  beep(a, 125);
  beep(cH, 500);
  beep(a, 375);  
  beep(cH, 125);
  beep(eH, 650);
 
  delay(500);
 
  //Repeat second section
  secondSection();
 
  //Variant 2
  beep(f, 250);  
  beep(gS, 500);  
  beep(f, 375);  
  beep(cH, 125);
  beep(a, 500);  
  beep(f, 375);  
  beep(cH, 125);
  beep(a, 650);  
 
  delay(650);
}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    XBee.print(entry.name());
    XBee.print('_');
    entry.close();
  }
}
