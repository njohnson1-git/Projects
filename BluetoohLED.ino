/*This program is my final project. It is capable of driving via wireless bluetooth. There are two modes,
manual driving mode (default) and line tracking mode. I switch bewteen modes through the button on the 
Arduino Blue app. On line tracking mode it follows a dark, or in our case black line, until coming across
something light in which is scans left and then right, immeaddeiately stopping the scan to continue the 
path once it sees the dark line. On maual mode, there is an auto-break feature as well as the capacity to
store the highest acceleration reached into on board EEPROM. It was challenging, but fun (when it worked).
Have a good code read, it's a lot, but I try to add comments when I can.*/

#include <SoftwareSerial.h>
#include <ArduinoBlue.h>
#include <stdbool.h>
#include <EEPROM.h>
//Libraries and h-bridge definitions
#define enA 5
#define enB 10
#define in1 6
#define in2 7
#define in3 8
#define in4 9

//For accelerometer
const int xInput = A1;     //X-out is in analog pin A1
const int yInput = A2;     //Y-out is in analog pin A2
const int zInput = A3;     //Z-out is in analog pin A3
int xRawMin = 265;
int xRawMax = 405;
int yRawMin = 269;
int yRawMax = 421;
int zRawMin = 279;
int zRawMax = 359;
int xRaw, xNorm;
int yRaw, yNorm;
int zRaw, zNorm;

// The bluetooth tx and rx pins must be supported by software serial.
// Bluetooth TX -> Arduino D12
const int BLUETOOTH_TX = 12;
// Bluetooth RX -> Arduino D13
const int BLUETOOTH_RX = 13;
String str;
int sampleSize = 10;

//IR Sensor and ultrasonic sensor
const int pinIRa = A0;
const int trigPin = (3);  // Add the Trig pin on pin 3.
const int echoPin = (4);  // Add the ECHO pin on pin 4.
int IRvalueA = 0;
int duration, distance;
int maxAccelY, maxAccelX;

//starting point for jostick drive
int prevThrottle = 49;
int prevSteering = 49;
int throttle, steering, sliderVal, button, sliderId;
int spd, turnSpdRight, Rspd, Lspd;

bool switchMode = false;
bool isStop;

//EEPROM address
int addr1 = 5;  // EEPROM address
int addr2 = 6;  // EEPROM address

SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue phone(bluetooth);  // pass reference of bluetooth object to ArduinoBlue constructor.

// Setup code runs once after program starts
void setup() {

  // initializing pins as inputs and outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // set direction once
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Start serial monitor at 9600 bps.
  Serial.begin(9600);

  // Start bluetooth serial at 9600 bps.
  bluetooth.begin(9600);

  pinMode(pinIRa, INPUT);  //ir sensor

  pinMode(trigPin, OUTPUT);  // Same as above, the TRIG pin will send the ultrasonic wave
  pinMode(echoPin, INPUT);   // The ECHO pin will recieve the rebounded wave, so it must be an input type

  // Delay for bluetooth module to "get ready"
  delay(100);

  Serial.println("setup complete"); //Lets me know things are a-ok
}

// Put your main code here, to run repeatedly:
void loop() {

  // ID of the button pressed pressed.
  button = phone.getButton();

  // Returns the text data sent from the phone.
  // After it returns the latest data, empty string "" is sent in subsequent.
  // calls until text data is sent again.
  str = phone.getText();

  // Throttle and steering values go from 0 to 99.
  // When throttle and steering values are at 99/2 = 49, the joystick is at center
  throttle = phone.getThrottle();
  steering = phone.getSteering();

  // If satemnnt checks if the button is pressed and switched mode to tracking
  if (button != -1) {
    Serial.print("Button: ");
    Serial.println(button);
    if (switchMode == false) {
      switchMode = true;
    } else switchMode = false;
    delay(1000);
    button = -1;
  }

  // Display throttle and steering data if steering or throttle value is changed
  if (prevThrottle != throttle || prevSteering != steering) {
    Serial.print("Throttle: ");
    Serial.print(throttle);
    Serial.print("\tSteering: ");
    Serial.println(steering);
    prevThrottle = throttle;
    prevSteering = steering;
  }

  // Send string from serial command line to the phone, left in case I needed it
  if (Serial.available()) {
    Serial.write("send: ");
    String str = Serial.readString();
    phone.sendMessage(str);  // phone.sendMessage(str) sends the text to the phone
    Serial.print(str);
    Serial.write('\n');
  }  

  if(str == "1"){ //If I type 1 in the text box, max X accel recorded
    EEPROM.write(addr1,maxAccelX);
  }
   if(str == "2"){ //If I type 2 in the text box, max Y accel recorded
    EEPROM.write(addr2,maxAccelY);
  }

  isStop = ultrasonic(); //ultrasonic returns a boolean value of true or false if it's within range of
  //a wall. so then it only allows reverse to go through
  if(!isStop){
    reverse(); 
  } else{
  stopMotor(in1, in2, enA, 0, true);      //stop the left motor
  stopMotor(in3, in4, enB, 0, true);     //stop the right motor
}

  stop();
  forward();
  IRsensor();
  while (switchMode == true) { //On a loop and only leaves if button is pressed again
    lineTracking();
  }
  
  if (xRaw > maxAccelX) { //rewrites to get highest accel
    maxAccelX = xRaw;
  }
  if (yRaw > maxAccelY) {
    maxAccelY = yRaw;
  }

}

void forward() {
  if (throttle < 45) {                     //when joystick is in one direction
    spd = map(throttle, 45, 0, 0, 255);    //sets speed based on how far the joystick is from the center
    Rspd = map(steering, 45, 0, 0, 220);   //turn speed for the right wheel, it is inverse to the normal speed, and later gets subtracted
    Lspd = map(steering, 45, 99, 0, 220);  //turn speed for the left wheel, same function as Rspd.
    digitalWrite(in1, LOW);               //sets direction
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    if (steering <= 45) {    //when the joystick is turned right, calculates how much turn is needed
      if (spd - Rspd < 0) {  //ensures that the subtraction of speed on the right wheel is not negative
        Rspd = 90;           //if negative, then set as max
      }
      analogWrite(enA, (spd - Rspd) * 2);  //sets right motor speed to be a lot slower than left, which makes it turn right
      analogWrite(enB, spd);               //sets left motor as usual speed.
    } else if (steering >= 55) {           //when joystick is turned left, calculates how much turn is needed
      if (spd - Lspd < 0) {                //ensures that the subtraction of speed on the left wheel is not negative
        Lspd = 90;                         //if negative, then set as max
      }
      analogWrite(enB, (spd - Lspd) * 2);  //sets left motor speed to be a lot slower than right, which makes it turn left
      analogWrite(enA, spd);               //sets right motor as usual speed.
    } else {
      analogWrite(enB, spd);  //if no turn is seen, then proceed normal, go forward.
      analogWrite(enA, spd);
    }
  }
}
//Hi professor, a lot of the code for forward/backward/stop is taken from project 2 cause if it ain't broke, don't fix it, ya know?

void reverse() {
  if (throttle >= 55) {                    //when joystick is in one direction
    spd = map(throttle, 55, 99, 0, 255);   //sets speed based on how far the joystick is from the center
    Rspd = map(steering, 55, 0, 0, 220);   //turn speed for the right wheel, it is inverse to the normal speed, and later gets subtracted
    Lspd = map(steering, 55, 99, 0, 220);  //turn speed for the left wheel, same function as Rspd.
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    if (steering <= 45) {    //when the joystick is turned right, calculates how much turn is needed
      if (spd - Rspd < 0) {  //ensures that the subtraction of speed on the right wheel is not negative
        Rspd = 90;           //if negative, then set as max
      }
      analogWrite(enA, (spd - Rspd) * 2);  //sets right motor speed to be a lot slower than left, which makes it turn right
      analogWrite(enB, spd);               //sets left motor as usual speed.
    } else if (steering >= 55) {           //when joystick is turned left, calculates how much turn is needed
      if (spd - Lspd < 0) {                //ensures that the subtraction of speed on the left wheel is not negative
        Lspd = 90;                         //if negative, then set as max
      }
      analogWrite(enB, (spd - Lspd) * 2);  //sets left motor speed to be a lot slower than right, which makes it turn left
      analogWrite(enA, spd);               //sets right motor as usual speed.
    } else {
      analogWrite(enB, spd);  //if no turn is seen, then proceed normal, go forward.
      analogWrite(enA, spd);
    }
  }
}

void stop() {  
  if ((throttle >= 45 && throttle <= 55)) {
    digitalWrite(in1, LOW);  //turns off the motor inputs if within the resting joystick position
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enA, 0);  //motors ends
    analogWrite(enB, 0);
  }
}

void IRsensor() { //used for debugging
  Serial.print("Analog Reading=");
  Serial.println(IRvalueA);
  delay(100);

  IRvalueA = analogRead(pinIRa);
}

bool ultrasonic() { //Ultrasonic is interesting cause it's based on a little interval where it shoots out the signal
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin, HIGH);
  delay(5);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1; //The distance is dancluated after the formula
  delay(100);
  Serial.print("cm");
  Serial.println(distance);
  int stopDistance = 30;  //if there is something 30 cm in front of the car then it will stop
  if ((distance > 0) && (distance < stopDistance)) {
    return true;  // Obstacle detected
  } else {
    return false;
  }
}

void lineTracking() {
  delay(20);  //get ultra value

  digitalWrite(in1, HIGH); //set motor the right way cause it can only go forward in this mode
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  IRvalueA = analogRead(pinIRa);  //get IRsensor data
  if (IRvalueA >= 550) { //go forward like normal
    analogWrite(enA, 100);
    analogWrite(enB, 100);

  }
  if (IRvalueA < 550) { //begin scanning for black line
    analogWrite(enA, 0);  //motors ends
    analogWrite(enB, 0);
    for (int i = 0; i < 5; i++) {
      analogWrite(enA, 100);  //motor goes a little in one direction to find the black line
      delay(200);
      analogWrite(enA, 0);
      delay(200);
      IRvalueA = analogRead(pinIRa);
      if (IRvalueA >= 550) i = 5;
    }
  }
  if (IRvalueA < 550) {  //If I didn't find it left, it checks right but does double the amount of turns
    for (int i = 0; i < 10; i++) {
      if (IRvalueA >= 550) i = 10;
      analogWrite(enB, 100);  //motors ends
      delay(200);
      analogWrite(enB, 0);
      delay(200);
      IRvalueA = analogRead(pinIRa);
    }
  }
    button = phone.getButton(); //And of course check to see if it's time to switch modes
  if (button != -1) {
    Serial.print("Button: ");
    Serial.println(button);
    switchMode = false;
  }
}

void stopMotor(int inPin1, int inPin2, int enPin, int speed, bool direction) { //for auto crash detection
  digitalWrite(inPin2, direction);
  digitalWrite(inPin1, !direction);
  analogWrite(enPin, speed);
}

// Read "sampleSize" samples and report the average
int ReadAxis(int axisPin) {
  long reading = 0;
  analogRead(axisPin);
  delay(10);  // I found without this delay, the first set of readings would be inaccurate, low
  for (int i = 0; i < sampleSize; i++) {
    reading += analogRead(axisPin);
  }
  return reading / sampleSize; 
}

void ReadAllAxis(int* xNewptr, int* yNewptr, int* zNewptr) {
  *xNewptr = ReadAxis(xInput);
  *yNewptr = ReadAxis(yInput);
  *zNewptr = ReadAxis(zInput); //unnecessarybecause our car can't fly...
}

/* Function to calibrate the accelerometer only used it once not in final code
void Calibrate() {
  // initialize all mins and maxes with one read
  ReadAllAxis(&xRawMin, &yRawMin, &zRawMin);
  xRawMax = xRawMin;
  yRawMax = yRawMin;
  zRawMax = zRawMin;

  // Start main calibration
  Serial.println("Calibrate");

  // Calibration for x-axis
  Serial.println("Align x-axis up.  When ready press button.");
  while (digitalRead(buttonPin) != LOW) {}
  ReadAllAxis(&xRaw, &yRaw, &zRaw);
  AutoCalibrate(xRaw, yRaw, zRaw);
  delay(1000);
  // Calibration for x-axis down
  Serial.println("Align x-axis down.  When ready press button.");
  while (digitalRead(buttonPin) != LOW) {}
  ReadAllAxis(&xRaw, &yRaw, &zRaw);
  AutoCalibrate(xRaw, yRaw, zRaw);
  delay(1000);
  // Calibration for y-axis
  Serial.println("Align y-axis up.  When ready press button.");
  while (digitalRead(buttonPin) != LOW) {}
  ReadAllAxis(&xRaw, &yRaw, &zRaw);
  AutoCalibrate(xRaw, yRaw, zRaw);
  delay(1000);
  // Calibration for y-axis down
  Serial.println("Align y-axis down.  When ready press button.");
  while (digitalRead(buttonPin) != LOW) {}
  ReadAllAxis(&xRaw, &yRaw, &zRaw);
  AutoCalibrate(xRaw, yRaw, zRaw);
  delay(1000);
  // Calibration for z-axis
  Serial.println("Align z-axis up.  When ready press button.");
  while (digitalRead(buttonPin) != LOW) {}
  ReadAllAxis(&xRaw, &yRaw, &zRaw);
  AutoCalibrate(xRaw, yRaw, zRaw);
  delay(1000);
  // Calibration for z-axis down
  Serial.println("Align z-axis down.  When ready press button.");
  while (digitalRead(buttonPin) != LOW) {}
  ReadAllAxis(&xRaw, &yRaw, &zRaw);
  AutoCalibrate(xRaw, yRaw, zRaw);
  delay(1000);
  Serial.println("Calibration completed. \n");
}*/

// Find the extreme raw readings from each axis
void AutoCalibrate(int xNew, int yNew, int zNew) {
  if (xNew < xRawMin)  // update x Min and Max
  {
    xRawMin = xNew;
  }
  if (xNew > xRawMax) {
    xRawMax = xNew;
  }
  if (yNew < yRawMin)  // update y Min and Max
  {
    yRawMin = yNew;
  }
  if (yNew > yRawMax) {
    yRawMax = yNew;
  }
  if (zNew < zRawMin)  // update z Min and Max
  {
    zRawMin = zNew;  
  }
  if (zNew > zRawMax) {
    zRawMax = zNew;
  }
} //I'm going to sleep