#include <WiFi.h>
#include "html510.h"
#include "website.h"
#include "vive510.h"
#include <esp_now.h>
#include <WiFiUdp.h>

//********* ULTRASONIC SENSOR PIN NUMBERS ***************
#define LEFT_SENSOR_TRIGGER 13 //blue upar black neeche
#define LEFT_SENSOR_ECHO 12 //purple upar orange neeche
#define FRONT_SENSOR_TRIGGER 27 //Grey
#define FRONT_SENSOR_ECHO 33  //purple
#define RIGHT_SENSOR_TRIGGER 15 //White
#define RIGHT_SENSOR_ECHO 32  //Grey
#define NUM_READINGS 5

// Arrays to store ultrasonic sensor readings
int leftReadings[NUM_READINGS];
int frontReadings[NUM_READINGS];
int rightReadings[NUM_READINGS];

// Index for current reading in the arrays
int currentIndex = 0;
//*******************************************************  


//******************* ESP NOW ADDRESS AND CHANNELS *****************************************
#define CHANNEL 6                  // channel can be 1 to 14, channel 0 means current channel.  
#define MAC_RECV  {0xC8,0xF0,0x9E,0xF6,0xE0,0xC8} // receiver MAC address (last digit should be even for STA)
//*****************************************************************************************


//********************* VIVE AND UDP CONSTANTS ******************************************
#define RGBLED 2 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 A3
#define SIGNALPIN2 20 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
#define STUDENTIP 100 // choose a teammembers assigned IP number
#define teamNumber 14
#define FREQ 1 // in Hz

const int UDP_PACKET_SIZE = 100;    // allows packets upto 100 bytes
byte packetBuffer[UDP_PACKET_SIZE]; // can be upto 65535

const int MOVING_AVERAGE_SIZE = 5; //Vive Moving Avg
int x1Values[MOVING_AVERAGE_SIZE];  // Vive moving Avg store
int y1Values[MOVING_AVERAGE_SIZE];
int currentIndex2 = 0;              // Vive moving avg counter

int vive1x;
int vive1y;
//*******************************************************************


//******************* BEACON GLOBAL VARS AND PINS ************************
// #define IR_PIN_1 A4 // Change to the appropriate GPIO pin for IR input on your ESP32

volatile long LastIR1 = 0;
volatile long NewestIR1 = 0;
volatile float IRFreq1 = 0;

long BeaconLastUpdate = 0;
int BeaconTrackFreq = 10;

const int IR_PIN = A4;  // Define the pin connected to the IR sensor

volatile unsigned long pulseStartTime = 0;
volatile unsigned long pulseEndTime = 0;
volatile bool pulseDetected = false;

uint32_t lastIRTime = 0;
uint32_t newestIRTime = 0;
//***************************************************************************


//*********************** OBJECT INITIALIZATIONS *****************************

HTML510Server h(80);            //html510 for website
Vive510 vive1(SIGNALPIN1);      //vive510 for vive1
WiFiUDP UDPTestServer;          //UDP Server to send and get values
IPAddress ipTarget(192, 168, 1, 255); // IP To send UDP Packet 255 => broadcast
//****************************************************************************


//********************  WiFi Setups **********************************
const char* serverName = "Team145574";
const char* pwd = "12340987";

// const char* ssid     = "TP-Link_05AF";
// const char* password = "47543454";

const char* ssid     = "TP-Link_E0C8";
const char* password = "52665134";

// const char* ssid     = "TP-Link_FD24";
// const char* password = "65512111";

//********************************************************************


//************* MOTOR AND ENCODER PINS, GLOBAL VARIABLES AND PID *******************************
// Define pin numbers for motor control
const int m_fwd_rt_PWM = A5;  // PWM pin for motor 1
const int m_fwd_rt_Dir = SCK;  // Direction pin for motor 1
const int m_fwd_lt_PWM = A0; // PWM pin for motor 2
const int m_fwd_lt_Dir = A1; // Direction pin for motor 2 

// Define pin numbers for encoder inputs
const int enc_fwd_lt_PinA = A2; // Encoder 1 pin A
const int enc_fwd_rt_PinA = 21; // Encoder 2 pin A //21

// Define PID parameters
const float Kp = 1.4; // Proportional gain
const float KI = 0; // Integral gain
const float Kd = 0; // Derivative gain

//PID Global Variables
float currentSpeedL = 0; // Current Speed
float currentSpeedR = 0; // Current Speed
int posFL = 0;  // Position of encoder
int posFR = 0;  // Position of encoder
int oldStateFL = 0; // Prev step state
int oldStateFR = 0; // Prev step state

int tick = 0; // counter for time
int integral;
int derivative;
int previousCorrection;
int correction;

int prev_dist_err = 0;    //Wall Follow Task movement variable
//****************************************************************************************


//******************************* ESP NOW SETUP ********************************
esp_now_peer_info_t peer1 = 
{
  .peer_addr = MAC_RECV, 
  .channel = CHANNEL,
  .encrypt = false,
};
//*******************************************************************************


//Variables for command through Website
int command = 0;
int oldCommand = 0;

// int ms;
int ms = 0;
int i = 0;

//**************************************************************************
//*************************FUNCTIONS*******************************************
//***************************************************************************


//*********************** PRIMARY FUNCTIONS FOR TASKS ********************************************

void wallFollow() {
  // Read distances from sensors and update moving averages
  int leftDistance = updateMovingAverage(leftReadings, getDistance(LEFT_SENSOR_TRIGGER, LEFT_SENSOR_ECHO));
  int frontDistance = updateMovingAverage(frontReadings, getDistance(FRONT_SENSOR_TRIGGER, FRONT_SENSOR_ECHO));
  int rightDistance = updateMovingAverage(rightReadings, getDistance(RIGHT_SENSOR_TRIGGER, RIGHT_SENSOR_ECHO));
  if (frontDistance > 50){frontDistance = 50;}
  if (leftDistance > 25) {leftDistance = 25;}
  if (rightDistance > 25) {rightDistance = 25;}

  if (frontDistance > 20){
    int dist_err = (leftDistance - 10);
    int der_dist = dist_err - prev_dist_err;
    float wallCorrection = 0.8 * dist_err ; //+ 0.1 * der_dist; 
    driveStraight(wallCorrection);
    int prev_dist_err = dist_err;
  }  

  if (frontDistance <= 20){
    if (leftDistance > 12){
      stop();
      delay(1);
      turnRight();
      delay(30);
      Serial.println("Wall Ahead");
    }
    if (leftDistance < 12){
      stop();
      delay(1);
      turnRight();
      delay(15);
    }
  }

}


void policeTaskOne(int selfPosX1, int selfPosY1, int* policePos){
  int policePosX = policePos[0];
  int policePosY = policePos[1];
  int selfPosX1Avg = calculateMovingAverage(x1Values, selfPosX1);
  int selfPosY1Avg = calculateMovingAverage(y1Values, selfPosY1);

  if(policePosY - selfPosY1Avg > 100){
    ledcWrite(0, 180); //right=0, left=1
    ledcWrite(1, 180);

    digitalWrite(m_fwd_rt_Dir, HIGH);
    digitalWrite(m_fwd_lt_Dir, HIGH);
  }

  if (selfPosY1Avg - policePosY < 100){
    turnLeft();
    delay(555);
    stop();
    delay(1000);
    driveStraightNoPID();
    delay(10000);
  }
}

// void beaconFollow(){
//   int freqDetected = beaconTrack2();
//   int i = 0;
//   if (i == 0){
//     turnRight();
//     delay(300);
//     stop();
//     driveStraight(0);
//     delay(1000);
//     stop();
//     i = 100;
//   }

//   if (freqDetected > 10 && freqDetected < 36){
//     driveStraight(0);
//     delay(1000);
//     stop();
//   }
//   else{
//     turnLeft();
//     delay(100);
//     stop();
//   }  
// }


void beaconTrack(){
  attachInterrupt(digitalPinToInterrupt(IR_PIN), IRsensorISR, CHANGE);
  delay(60);
  detachInterrupt(digitalPinToInterrupt(IR_PIN));
  if (i == 0){
    turnRight();
    delay(300);
    stop();
    driveStraight(0);
    delay(2001);
    stop();
    i = 100;
  }
  


  if (pulseDetected) {
    unsigned long pulseDuration = pulseEndTime - pulseStartTime;
    unsigned int frequency = 1000000 / (2 * pulseDuration);  // Factor of 2 for complete cycle

    // Check if the frequency is in the specified ranges
    if ((frequency >= 20 && frequency <= 30) || (frequency >= 500 && frequency <= 550)) {
      Serial.print("Detected Frequency: ");
      Serial.println(frequency);
      driveStraight(0);
      delay(2000);
      stop();


    }
  } 
  else {
    Serial.println("Frequency not in the specified ranges.");
    turnLeft();
    delay(70);
    stop();
    delay(30);
    }

  pulseDetected = false;  // Reset the flag
  
}

// void beaconTrack1(){
//   if (i == 0){
//     turnRight();
//     delay(300);
//     stop();
//     driveStraight(0);
//     delay(2001);
//     stop();
//     i = 100;
//   }

//   delay(50); // Adjust the delay based on your application needs
//   float detectedFrequency = detectIRFrequency();

//   if ((detectedFrequency >= 20 && detectedFrequency <= 30) || (detectedFrequency >= 500 && detectedFrequency <= 550)) {
//     Serial.print("Detected Frequency: ");
//     Serial.println(detectedFrequency);
//     driveStraight(0);
//     delay(2000);
//     stop();

//   } 
//   else {
//     Serial.println("Frequency out of range");
//     turnLeft();
//     delay(5);
//     stop();
//     delay(5);
  
//   }
// }
//**************************************************************************************************



//*********************************HELPER FUNCTIONS***********************************


//************************** BEACON ***************************************

// float beacon_track() {
//   long currtime = micros();
//   float freq_1 = 0;

//   freq_1 = 1.0 / ((NewestIR1 - LastIR1) / 1000000.0);
//   Serial.print(freq_1);
//   if (NewestIR1 < micros() - 100000) {
//     freq_1 = -1;
//   }

//   if ((freq_1 >= 10 && freq_1 <= 30) || (freq_1 >= 530 && freq_1 <= 570)) {
//     Serial.print("Frequency: ");
//     Serial.println(freq_1);
//     BeaconLastUpdate = millis();
//     return freq_1;
//   } else {
//     Serial.println("None");
//     BeaconLastUpdate = millis();
//     return -1; // Return a value indicating no valid frequency detected
//   }
// }


// void IRAM_ATTR IR_ISR_1() {
//   LastIR1 = NewestIR1;
//   NewestIR1 = micros();
// }

// void attach_interrupts() {
//   attachInterrupt(digitalPinToInterrupt(IR_PIN_1), IR_ISR_1, RISING);
// }

void IRsensorISR() {
  if (digitalRead(IR_PIN) == HIGH) {
    pulseStartTime = micros();  // Record the start time of the pulse
  } else {
    pulseEndTime = micros();    // Record the end time of the pulse
    pulseDetected = true;       // Indicate that a pulse has been detected
  }
}


//******************************* VIVE ****************************************

//Get Vive values
int get_vive1x(){
  int x1=0;
  int y1=0;
  int vive1Data[2];
  if (vive1.status() == VIVE_RECEIVING) {
    x1 = vive1.xCoord();
    y1 = vive1.yCoord();
    neopixelWrite(RGBLED,0, x1/200, y1/200);  // blue to greenish
  }
  else {
    x1=0;
    y1=0;
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }
  // Serial.print("x1 = ");
  // Serial.print(x1);
  return x1;
}

int get_vive1y(){
  int x1=0;
  int y1=0;
  // int vive1Data[2];
  if (vive1.status() == VIVE_RECEIVING) {
    x1 = vive1.xCoord();
    y1 = vive1.yCoord();
    neopixelWrite(RGBLED,0, x1/200, y1/200);  // blue to greenish
  }
  else {
    x1=0;
    y1=0;
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
    }
  }
  // Serial.print("   y1 = ");
  // Serial.print(y1);
  return y1;
}

// Get Police Car values
int * handleUDPServer() {
  const int UDP_PACKET_SIZE = 14;   // can be upto 65535
  uint8_t packetBuffer[UDP_PACKET_SIZE];

  static int police_coords[2] = {0, 0};

  int cb = UDPTestServer.parsePacket();   // if there is no message, cb = 0
  if(cb) {
    packetBuffer[13] = 0;

    UDPTestServer.read(packetBuffer, UDP_PACKET_SIZE);
    police_coords[0] = atoi((char *)packetBuffer + 3); // ##,####,#### 3d indexed char
    police_coords[1] = atoi((char *)packetBuffer + 8); // ##,####,#### 7th indexed char
    Serial.print("From Team ");
    Serial.println((char *)packetBuffer);
    Serial.print("X coordinate of Police: ");
    Serial.println(police_coords[0]);
    Serial.print("Y coordinate of police car: ");
    Serial.println(police_coords[1]);
  }

  return police_coords;
}

//Moving Avg for VIVE position
int calculateMovingAverage(int* values, int newValue) {
  // Add the new value to the array
  values[currentIndex2] = newValue;
  currentIndex2 = (currentIndex2 + 1) % MOVING_AVERAGE_SIZE;
  int sum = 0;
  for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
    sum += values[i];
  }

  return sum / MOVING_AVERAGE_SIZE;
}

//********************************************************************************************************************



//************************************ WALL FOLLOW SUPPORT ****************************************

long getDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

// Function to update moving average and return the current average
int updateMovingAverage(int readings[], int newValue) {
  readings[currentIndex] = newValue;
  currentIndex = (currentIndex + 1) % NUM_READINGS;

  // Calculate the average
  int sum = 0;
  for (int i = 0; i < NUM_READINGS; i++) {
    sum += readings[i];
  }
  return sum / NUM_READINGS;
}

//*****************************************************************************************************



//****************************************** MOTOR FUNCTIONS *********************************************

// Function to initialize motors and encoders
void setupMotorsAndEncoders() {
  // Initialize motor control pins as outputs
  pinMode(m_fwd_rt_Dir, OUTPUT);
  pinMode(m_fwd_lt_Dir, OUTPUT);

  // Set up LEDC for PWM control
  ledcSetup(0, 1000, 8);
  ledcSetup(1, 1000, 8);

  // Configure LEDC channel 0 with a frequency of 5 kHz and resolution of 8 bits
  ledcAttachPin(m_fwd_rt_PWM, 0);
  ledcAttachPin(m_fwd_lt_PWM, 1);

  // Initialize encoder input pins as inputs
  pinMode(enc_fwd_rt_PinA, INPUT_PULLUP);
  pinMode(enc_fwd_lt_PinA, INPUT_PULLUP);

  //WallFollow Pins
  pinMode(LEFT_SENSOR_TRIGGER, OUTPUT);
  pinMode(LEFT_SENSOR_ECHO,INPUT);
  pinMode(FRONT_SENSOR_TRIGGER, OUTPUT);
  pinMode(FRONT_SENSOR_ECHO, INPUT);
  pinMode(RIGHT_SENSOR_TRIGGER, OUTPUT);
  pinMode(RIGHT_SENSOR_ECHO, INPUT);

}

// Calculate speed of left motor based on encoder reading
int calcCurrentSpeedL(){
  static int oldPosFL;
  int velFL;
  velFL = posFL - oldPosFL;
  oldPosFL = posFL;
  return velFL;
}

// Calculate speed of right motor based on encoder reading
int calcCurrentSpeedR(){
  static int oldPosFR;
  int velFR;
  velFR = posFR - oldPosFR;
  oldPosFR = posFR;
  return velFR;
}

//Update Right Encoder counts moved
void updateEncoderFR(){
  if (digitalRead(enc_fwd_rt_PinA) != oldStateFR){
    oldStateFR = digitalRead(enc_fwd_rt_PinA);
    if (digitalRead(m_fwd_rt_Dir) == 0){
      posFR = posFR + 1;
    }
    else{
      posFR = posFR - 1;
    }
  }
}

//Update Left Encoder counts moved
void updateEncoderFL(){
  if (digitalRead(enc_fwd_lt_PinA) != oldStateFL){
    oldStateFL = digitalRead(enc_fwd_lt_PinA);
    if (digitalRead(m_fwd_lt_Dir) == 0){
      posFL = posFL + 1;
    }
    else{
      posFL = posFL - 1;
    }
  }
}

// Encoders update fnuction
void updateAllEncoders(){
  updateEncoderFR();
  updateEncoderFL();
}

// Function to drive the car without using PID control
void driveStraightNoPID() {
  ledcWrite(0, 250); //right=0, left=1
  ledcWrite(1, 250);

  digitalWrite(m_fwd_rt_Dir, HIGH);
  digitalWrite(m_fwd_lt_Dir, HIGH);
}

void driveStraight(float wall_correction) {

  float PID_correction = calcCorrectionsPID();
  
  // PID control for each wheel
  float pidOutputL = 122 - PID_correction - wall_correction;
  float pidOutputR = 120 + PID_correction + wall_correction;

  // Serial.println("Left - ");
  // Serial.println(pidOutputL);
  // Serial.println("Right - ");
  // Serial.println(pidOutputR);
  // Serial.println("\n");
 
  // Update motor speeds (constrained betweem 0 and max resolution)
  int motorSpeedL = constrain(pidOutputL, 0, 255);
  int motorSpeedR = constrain(pidOutputR, 0, 255);



  ledcWrite(1, abs(motorSpeedL));
  ledcWrite(0, abs(motorSpeedR)); //right=0, left=1

  digitalWrite(m_fwd_rt_Dir, (pidOutputR > 0) ? HIGH : LOW);
  digitalWrite(m_fwd_lt_Dir, (pidOutputR > 0) ? HIGH : LOW);
 
  // Update current speed
  // currentSpeedR = calcCurrentSpeedR();
  // currentSpeedL = calcCurrentSpeedL();

  // Serial.println("Left Speed- ");
  // Serial.println(currentSpeedL);
  // Serial.println("Right Speed - ");
  // Serial.println(currentSpeedR);
  // Serial.println("\n\n");

  // Save error for the next iteration
  // previousErrorL = errorL;
  // previousErrorR = errorR;
  previousCorrection = correction;
  // Serial.println("\n");

}

// Function to turn the car right without PID control
void turnRight() {
  // Set fixed motor speeds for turning right
  ledcWrite(0, 100);
  ledcWrite(1, 100);

  // Set motor directions for turning right
  digitalWrite(m_fwd_rt_Dir, LOW);
  digitalWrite(m_fwd_lt_Dir, HIGH);
}

void turnLeft() {
  // Set fixed motor speeds for turning right
  ledcWrite(0, 100);
  ledcWrite(1, 100);

  // Set motor directions for turning right
  digitalWrite(m_fwd_rt_Dir, HIGH);
  digitalWrite(m_fwd_lt_Dir, LOW);
}

//Fnuction to stop car
void stop() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

void driveBackNoPID() {
  ledcWrite(0, 200); //right=0, left=1
  ledcWrite(1, 200);

  digitalWrite(m_fwd_rt_Dir, LOW);
  digitalWrite(m_fwd_lt_Dir, LOW);
}

float calcCorrectionsPID(){

  float PID_correction;
  currentSpeedR = calcCurrentSpeedR();
  currentSpeedL = calcCurrentSpeedL();

  //compute errors for each wheel
  correction =  (currentSpeedL - currentSpeedR);

  //integral of errors for each wheel
  integral += correction;

  //integral windup
  if (integral > 10){integral = 10;}

  //derivative term

  float derivative = correction - previousCorrection;

  PID_correction = Kp * correction - KI * integral - Kd * derivative;

  return PID_correction;
}

//***********************************************************************************************



//************ HTML CONTROL FUNCTIONS AND SENDING UDP AND ESP NOW ******************************

//HTML handling functions
void handleRoot(){
  h.sendhtml(body);
}

void handleForward(){
  Serial.println("Forward");
  h.sendhtml(body);
  command = 1;  //sets command to 1
};

void handleBackward(){
  Serial.println("Back");
  h.sendhtml(body);
  command = 3;  //sets command to 3
};

void handleRight(){
  Serial.println("Right");
  h.sendhtml(body);
  command = 4;  //sets command to 4
};

void handleLeft(){
  Serial.println("Left");
  h.sendhtml(body);
  command = 2;  //sets command to 2
};

void handleStop(){
  Serial.println("STOP");
  h.sendhtml(body);
  command = 0;  //sets command to 0
};

void handleWallFollow(){
  Serial.println("MODE - WALL FOLLOW");
  h.sendhtml(body);
  command = 5;  //sets command to 5
};

void handleBeacon(){
  Serial.println("MODE - WALL FOLLOW");
  h.sendhtml(body);
  command = 6;  //sets command to 5
};

void handlePoliceTask(){
  Serial.println("MODE - WALL FOLLOW");
  h.sendhtml(body);
  command = 7;  //sets command to 5
};

void commandSender(){
  h.serve();
  if (oldCommand != command){
    pingTA();
  }
  if  (millis()-ms>1000/FREQ) {
    vive1x = get_vive1x();
    vive1y = get_vive1y();
    ms = millis();
    UdpSend(vive1x, vive1y);
  }
  oldCommand = command;
}

void pingTA(){
  static int count;
  uint8_t message[200]; // Max ESPnow packet is 250 byte data

  // put some message together to send
  sprintf((char *) message, "sender %d ", count++);
  
  if (esp_now_send(peer1.peer_addr, message, sizeof(message))==ESP_OK) 
    Serial.printf("Sent '%s' to %x:%x:%x:%x:%x:%x \n",message, peer1.peer_addr[0],peer1.peer_addr[1],peer1.peer_addr[2],peer1.peer_addr[3],peer1.peer_addr[4],peer1.peer_addr[5]);   
  else Serial.println("Send failed");
  
  delay(30); // ESPNow max sending rate (with default speeds) is about 50Hz
}

void UdpSend(int x, int y){
  char udpBuffer[20];
  sprintf(udpBuffer, "%02d:%4d,%4d",teamNumber,x,y);   
                                              
  UDPTestServer.beginPacket(ipTarget, UDPPORT);
  UDPTestServer.println(udpBuffer);
  UDPTestServer.endPacket();
  // Serial.println(udpBuffer);
}
//***************************************************************************************************

void WifiSetupAP(){
  WiFi.softAP(serverName, "");
  WiFi.softAPConfig(IPAddress(192,168,1,100), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  Serial.print("Connect to website: ");
  Serial.print(WiFi.softAPIP()); Serial.println("/");
}

void WifiSetupSTA(){
  WiFi.mode(WIFI_STA);
  WiFi.config(IPAddress(192, 168, 1, 100), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, password);

  Serial.printf("team  #%d ", teamNumber); 
  Serial.print("Connecting to ");
  Serial.println(ssid);
  int i=0;
  while(WiFi.status()!=WL_CONNECTED && i++ < 20){
    delay(500);
    Serial.print(".");
  }
  if (i<19) {
    Serial.print("WiFi connected as ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  } else {
    Serial.printf("Could not connect err: %d ",i); 
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) Serial.println ("Success ");
  else Serial.println("Fail "); 
}

//************************************* MAIN  ***************************************
void setup() {
  Serial.begin(115200);
  Serial.print("hello");
  
  //Motor and Encoder Setup
  setupMotorsAndEncoders();

  //WiFi Setup
  WifiSetupAP();
  // WifiSetupSTA();
  
  h.begin();

  //ESP SETUP 
  if (esp_now_init() != ESP_OK) {
    Serial.println("init failed");
    ESP.restart();
  }

  esp_now_register_send_cb(OnDataSent);

  if (esp_now_add_peer(&peer1) != ESP_OK) {
    Serial.println("Pair failed");     // ERROR  should not happen
  }

  // Attaching handles 
  h.attachHandler("/", handleRoot);
  h.attachHandler("/direction1?value=", handleForward);
  h.attachHandler("/direction2?value=", handleBackward);
  h.attachHandler("/direction3?value=", handleLeft);
  h.attachHandler("/direction4?value=", handleRight);
  h.attachHandler("/direction0?value=", handleStop);
  h.attachHandler("/direction5?value=", handleWallFollow);
  h.attachHandler("/direction6?value=", handleBeacon);
  h.attachHandler("/direction7?value=", handlePoliceTask);

  // Ultrasonic Sensors Pin setup
  pinMode(LEFT_SENSOR_TRIGGER, OUTPUT);
  pinMode(LEFT_SENSOR_ECHO, INPUT);
  pinMode(FRONT_SENSOR_TRIGGER, OUTPUT);
  pinMode(FRONT_SENSOR_ECHO, INPUT);
  pinMode(RIGHT_SENSOR_TRIGGER, OUTPUT);
  pinMode(RIGHT_SENSOR_ECHO, INPUT);

  pinMode(IR_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(IR_PIN), IRsensorISR, CHANGE);

  UDPTestServer.begin(UDPPORT);
  vive1.begin();

}

void loop() {
  h.serve();
  commandSender();
  Serial.println("123");

  // based on value of command 
  // (set by HTML handle functions)

  while (command == 1){
    // Can be used with or without PID (uncomment required part)
    driveStraightNoPID();
    // driveStraight(0);
    h.serve();
    commandSender();
  }

  while (command == 2){
    turnLeft();
    h.serve();
    commandSender();
  }

  while (command == 3){
    driveBackNoPID();
    h.serve();
    commandSender();
  }

  while (command == 4){
    turnRight();
    h.serve();
    commandSender();
  }

  while (command == 0){
    stop();
    h.serve();
    commandSender();
  }

  while (command == 5){
    wallFollow();
    h.serve();
    commandSender();
  }

  while (command == 6){
    // beaconFollow();
    beaconTrack();
    h.serve();
    commandSender();
  }
  
  while (command == 7){
    vive1x = get_vive1x();
    vive1y = get_vive1y();
    int* police_coords = handleUDPServer();
    Serial.println(police_coords[0]);

    // while (vive1x == 0 || vive1y == 0 || police_coords[0] == 0 || police_coords[1] == 0){
    //   vive1x = get_vive1x();
    //   vive1y = get_vive1y();
    //   police_coords = handleUDPServer();
    //   h.serve();
    //   commandSender();
    // }

    policeTaskOne(vive1x, vive1y, police_coords);
    h.serve();
    commandSender();
  }
}
