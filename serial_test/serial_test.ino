//#include "SoftwareSerial.h"

int led = 13;
String message;
//Serial XBee(0, 1);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(led, OUTPUT);     

}

int numberPackets = 0;

void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  if (Serial1.available() > 0) {
    Serial1.write("received message");
    Serial.println("received message");
    char received[64];
    String receivedstr = Serial1.readString();
    Serial.println(receivedstr); //console when connected to computer
    receivedstr.toCharArray(received, 64);
    Serial1.write(received); //sending to xbee
    Serial1.write("\n");
  }
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    message = String("\nTestPacket Number: ").concat(String(numberPackets).concat(String(" || Time of packet send: ").concat(millis()))); // + numberPackets + " || Time of packet send: " + millis());
    char charArray[64];
    message.toCharArray(charArray, 64);
    Serial1.write(charArray);
    numberPackets = numberPackets + 1;
    delay(1000);
    //digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
  //}
}
