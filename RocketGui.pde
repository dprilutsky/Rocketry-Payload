import processing.serial.*;
import grafica.*;

Serial myPort;  // The serial port

PFont f;

float ax;
float ay;
float az;
float roll;
float pitch;
float heading;
float altitudeBarometer;
float altitudeGPS;
float velocity;
float velocityIntegrated;
float latitude;
float longitude;
float time;

float vx = 0;
float vy = 0;
float vz = 0;

float[] axList = new float[0];
float[] ayList= new float[0];
float[] azList= new float[0];
float[] rollList= new float[0];
float[] pitchList= new float[0];
float[] headingList= new float[0];
float[] altitudeBarometerList= new float[0];
float[] altitudeGPSList= new float[0];
float[] velocityList= new float[0];
float[] velocityIntegratedList= new float[0];
float[] latitudeList= new float[0];
float[] longitudeList= new float[0];
float[] timeList= new float[0];

PImage imgTitle;
PImage imgAlt;
PImage imgVel;
PImage imgAcc;
PImage imgMapRibbon;
int deltatime;
float deltatimeTransmission;

//String inByte = "";
String inByte;
String inByteNew;
String inByteFirst;
String inByteFinal;
String parameters;
String[][] parametersList;

boolean transmissionStarted = false;
String launchNum = "0123456789";
PrintWriter output;
void setup(){
  size(displayWidth,displayHeight);
  imgTitle = loadImage("Rocket Logo.jpg");
  imgAlt = loadImage("Altitude Gauge.jpg");
  imgVel = loadImage("Velocity Gauge.jpg");
  imgAcc = loadImage("Acceleration Gauge.jpg");
  imgMapRibbon = loadImage("Rocket Map.jpg");
  myPort = new Serial(this, Serial.list()[2], 9600);
  inByteNew = "";
  f = createFont("Anita semi-square", 50, true);
  inByteFirst = myPort.readString();
  deltatime = 10;
  velocityIntegrated = 0;
  deltatimeTransmission = .15;
  output = createWriter("LaunchXData.txt");
}
    
void draw(){

  if (keyPressed) {
  println("key is pressed " + key);
  myPort.write(key);
  if (launchNum.indexOf(key) != -1) transmissionStarted = true;
  }
  
  //Draw the intitial items
  //QUESTION: why isn't this just done in setup? Why are we re-drawing this ever single time
  background(#E7E7E7);
  image(imgTitle, displayWidth/2-displayWidth/4.5, 0, displayWidth/2.25, displayWidth/9);
  image(imgAlt, 0, displayWidth/9, (displayHeight-displayWidth/9)/3*638/738,(displayHeight-displayWidth/9)/3);
  image(imgVel, 0, displayWidth/9+(displayHeight-displayWidth/9)/3, (displayHeight-displayWidth/9)/3*638/738,(displayHeight-displayWidth/9)/3);
  image(imgAcc, 0, displayWidth/9+(displayHeight-displayWidth/9)/3*2, (displayHeight-displayWidth/9)/3*638/738,(displayHeight-displayWidth/9)/3);
  image(imgMapRibbon, (displayHeight-displayWidth/9)/3*638/738, displayWidth/9, displayWidth - (displayHeight-displayWidth/9)/3*638/738, (displayWidth - (displayHeight-displayWidth/9)/3*638/738)*73/1480);
  fill(255);
  stroke(255);
  rect((displayHeight-displayWidth/9)/3*638/738+75, 25 + displayWidth/9 + (displayWidth - (displayHeight-displayWidth/9)/3*638/738)*73/1480, -50 + displayWidth - (displayHeight-displayWidth/9)/3*638/738, -50 + displayHeight -displayWidth/9 + (displayWidth - (displayHeight-displayWidth/9)/3*638/738)*73/1480); 
  
  if (transmissionStarted) {
  //Read in one full line  
  char nextChar = ' ';
  boolean lineFilled = false;                                 //This will turn true when we've reached the & character
  String inputLine = "";  //This will be populated with the input characters
  while (nextChar != '*') {
    print(""); // this needs to be here!
     if (myPort.available() > 0) {
       nextChar = (char) myPort.read();
     }
   }

  inputLine += nextChar;
  //Run until line is filled
  while (!lineFilled) {
    print("");
    //Only read if there is something to read
    while (myPort.available() > 0) {    
      //Get the next character and add it to our line. Check that line isn't over
      nextChar = (char) myPort.read();
      inputLine = inputLine + nextChar;
      if (nextChar == '&') {
        lineFilled = true;
        break;
      }
    }
  }

  //Print inputLine and truncate to remove start and end markers
  //I HAVE NOT MODIFIED THIS PORTION --- IT IS AS IT WAS
  println(inputLine);
  parameters = inputLine.substring(inputLine.indexOf("*")+1,inputLine.indexOf("&"));
  parametersList = matchAll(parameters,"#(.*?)#");
  output.println(inputLine);
  if(parametersList.length != 12){
    draw();
  }
  
  ax = float(parametersList[0][1]);
  ay = float(parametersList[1][1]);
  az = float(parametersList[2][1]);
  roll = float(parametersList[3][1]);
  pitch = float(parametersList[4][1]);
  heading = float(parametersList[5][1]);
  altitudeBarometer = float(parametersList[6][1]);
  altitudeGPS = float(parametersList[7][1]);
  velocity = float(parametersList[8][1]);
  latitude = float(parametersList[9][1]);
  longitude = float(parametersList[10][1]);
  time = float(parametersList[11][1]);
  
  axList = (float[])append(axList,ax);
  ayList = (float[])append(ayList,ay);
  azList = (float[])append(azList,az);
  rollList = (float[])append(rollList,roll);
  pitchList = (float[])append(pitchList,pitch);
  headingList = (float[])append(headingList,heading);
  altitudeBarometerList = (float[])append(altitudeBarometerList,altitudeBarometer);
  altitudeGPSList = (float[])append(altitudeGPSList,altitudeGPS);
  velocityList = (float[])append(velocityList,velocity);
  latitudeList = (float[])append(latitudeList,latitude);
  longitudeList = (float[])append(longitudeList,longitude);
  timeList = (float[])append(timeList,time);
  velocityIntegratedList = (float[])append(velocityIntegratedList,velocityIntegrated);

  //Integrate acceleration to get velocity. I HAVE CHANGED THIS TO BE CORRECT
  if (timeList.length > 1) {
    deltatimeTransmission = timeList[timeList.length-1] - timeList[timeList.length-2];
  }
  if (axList.length > 1 && !Float.isNaN(ax) && !Float.isNaN(ay) && !Float.isNaN(az) && !Float.isNaN(axList[axList.length-2]) && !Float.isNaN(ayList[axList.length-2]) && !Float.isNaN(azList[axList.length-2])) {
    vx = vx + (ax+axList[axList.length-2])/2 * deltatimeTransmission;
    vy = vy + (ay+ayList[ayList.length-2])/2 * deltatimeTransmission;
    vz = vz + (az+azList[azList.length-2])/2 * deltatimeTransmission;
    print(vx);
    print(" ");
    print(vy);
    print(" ");
    println(vz);
    velocityIntegrated = sqrt(vx*vx+vy*vy+vz*vz); 
  }
  else {
    vx = 0;
    vy = 0;
    vz = 0;
  }
  velocityIntegratedList = (float[])append(velocityIntegratedList,velocityIntegrated);

  fill(0);
  textFont(f,25);
  textAlign(LEFT);
  text("ax: " + ax, 60, displayWidth/9+(displayHeight-displayWidth/9)/3*2+125);
  text("ay: " + ay, 60, displayWidth/9+(displayHeight-displayWidth/9)/3*2+150);
  text("az: " + az, 60, displayWidth/9+(displayHeight-displayWidth/9)/3*2+175);
  
  
  text("GPS: " + velocity, 60, displayWidth/9 + (displayHeight-displayWidth/9)/3+140);
  text("INT: " + nfc(velocityIntegrated,2), 60, displayWidth/9 + (displayHeight-displayWidth/9)/3+170);
  
  text("GPS: " + altitudeGPS, 60, displayWidth/9 +140);
  text("BAR: " + altitudeBarometer, 60, displayWidth/9 +170);
  fill(255);
  text(nfs(floor(time/60),2,0)+":"+nfs(time%60,2,2),displayWidth*4/5+40,displayWidth/9+50);
  text(nfs(latitude,2,6)+"°N " + nfs(longitude,2,6) + "°W", displayWidth/3+100, displayWidth/9+50);

  

  
  
  //PLOTS DRAWER
  float[] firstPlotPos = new float[] {0, 0};
  float[] panelDim = new float[] {400, 225};
  float[] margins = new float[] {60,displayWidth*1/3, displayHeight/3, 60};
    // Create four plots to represent the 4 panels
  GPlot plot1 = new GPlot(this);
  plot1.setPos(firstPlotPos);
  plot1.setMar(0, margins[1], margins[2], 0);
  plot1.setDim(panelDim);
  plot1.setAxesOffset(0);
  plot1.setTicksLength(-4);
  plot1.getXAxis().setDrawTickLabels(false);

  GPlot plot2 = new GPlot(this);
  plot1.setTitleText("Velocity & Altitude vs. Time");
  plot2.setPos(firstPlotPos[0] + margins[1] + panelDim[0], firstPlotPos[1]);
  plot2.setMar(0, 0, margins[2], margins[3]);
  plot2.setDim(panelDim);
  plot2.setAxesOffset(0);
  plot2.setTicksLength(-4);
  plot2.getXAxis().setDrawTickLabels(false);
  plot2.getYAxis().setDrawTickLabels(true);

  GPlot plot3 = new GPlot(this);
  plot3.setPos(firstPlotPos[0], firstPlotPos[1] + margins[2] + panelDim[1]);
  plot3.setMar(margins[0], margins[1], 0, 0);
  plot3.setDim(panelDim);
  plot3.setAxesOffset(0);
  plot3.setTicksLength(-4);

  GPlot plot4 = new GPlot(this);
  plot4.setPos(firstPlotPos[0] + margins[1] + panelDim[0], firstPlotPos[1] + margins[2] + panelDim[1]);
  plot4.setMar(margins[0], 0, 0, margins[3]);
  plot4.setDim(panelDim);
  plot4.setAxesOffset(0);
  plot4.setTicksLength(-4);
  plot4.getYAxis().setDrawTickLabels(true);

  // Prepare the points for the four plots
  int nPoints = timeList.length;
  GPointsArray points1 = new GPointsArray(nPoints);
  GPointsArray points2 = new GPointsArray(nPoints);
  GPointsArray points3 = new GPointsArray(nPoints);
  GPointsArray points4 = new GPointsArray(nPoints);

  for (int i = 0; i < nPoints; i++) {
    points1.add(timeList[i], velocityList[i]);
    points2.add(timeList[i], velocityIntegratedList[i]);
    points3.add(timeList[i], altitudeGPSList[i]);
    points4.add(timeList[i], altitudeBarometerList[i]);
  }  

  // Set the points, the title and the axis labels
  plot1.setPoints(points1);
  plot2.setTitleText("VELOCITY & ALTITUDE VS. TIME");
  plot2.getTitle().setRelativePos(0.25);
  plot2.getTitle().setTextAlignment(LEFT);
  plot2.getYAxis().setAxisLabelText("Velocity (m/s)");

  plot2.setPoints(points2);
  

  plot3.setPoints(points3);
  plot3.getYAxis().setAxisLabelText("Altitude (m)");
  plot3.setInvertedYScale(true);

  plot4.setPoints(points4);
  plot4.getYAxis().setAxisLabelText("Position (m)");
  plot4.getXAxis().setAxisLabelText("Time (sec)");
  plot4.setInvertedYScale(true);

  // Draw the plots

  plot2.beginDraw();
  plot2.drawTitle();
  plot2.drawBox();
  plot2.drawXAxis();
  plot2.drawYAxis();
  plot2.drawTopAxis();
  plot2.drawRightAxis();
  plot2.drawPoints();
  plot2.drawLines();
  plot1.drawPoints();
  plot1.drawLines();
  plot2.endDraw();

  plot4.beginDraw();
  plot4.drawBox();
  plot4.drawXAxis();
  plot4.drawYAxis();
  plot4.drawTopAxis();
  plot4.drawRightAxis();
  plot4.drawPoints();
  plot3.drawPoints();
  plot3.drawLines();
  plot4.drawLines();
  plot4.endDraw();
  }
  delay(deltatime);
  
  if(keyPressed == true && key == 'w'){
    output.flush(); // Writes the remaining data to the file
    output.close(); // Finishes the file
    exit(); // Stops the program
  }
  //SENDS BYTES
}
