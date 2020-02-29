/* ========================================================================================================
 * CanSat 2019
 * Mission: Auto Gyro Probe
 * ========================================================================================================*/

/* ========================================================================================================
 * Current version v3d
 * Version differences from v3c
 ** Changes:
 *** blade spin rate 
 **** created new variable:rpmMillis
 **** removed 2*pi multiplication since rotation_counter physically accounts for it
 *** created eepromRecover sub file
 *** stores telemetry to sd
 *** simple detection for descent and landing
 ** Improvements:
 *** EEPROM usage complete and fully functionaly
 ** Fixes:
 *** altitude recover saves and resets when approapriate
 *
 ** Status:
 *** Working
 *** temporarily changed chip select to 20 for testing purposes
 ** Errors:
 *** 
 */

/* ========================================================================================================
 * Libraries
 * ========================================================================================================*/
#include <Wire.h>
#include <LPS.h>    //pressure, altitude, temperature
//#include <SD.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include <math.h>
#include "RTClib.h"
#include "cluster.h"
#include <TimedAction.h>
#include "HardwareSerial.h"
#include <EEPROM.h>
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

/* ========================================================================================================
 * Macros
 * ========================================================================================================*/
#define DATA_BUFFER_LENGTH 10
#define RELEASE_THRESH 450    //release from science payload at 450 meters
#define LAUNCH_ACCEL_THRESH 5 //in g's
#define SOLENOID_PIN 3
#define SERVO_PIN 6
#define SERVO_OFF 0
#define SERVO_ON 180
#define PROPER_PIN 23 //used for proper shut off indication, to check for unexpected restarts

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board
#define M_X_MIN -677
#define M_Y_MIN -307
#define M_Z_MIN -288
#define M_X_MAX +151
#define M_Y_MAX -41
#define M_Z_MAX +569
#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define Xbee Serial3
#define gpsSerial Serial1  //Teensy pin(0,1) to GPS(TX,RX)

/* ========================================================================================================
 * Globals
 * ========================================================================================================*/
LPS PTS;        //pressure, altitude, temperature
Servo SERVO;    //servo
DateTime now;   //RTC
RTC_DS3231 RTC;
SdFat sd;
//HardwareSerial gpsSerial = Serial1; 
Adafruit_GPS GPS(&gpsSerial);

telemetry payloadData[DATA_BUFFER_LENGTH];

//const int chipSelect = 10;  //SD card
const int chipSelect = 20;
//const int chipSelect_cam = 20;  //Camera
int packet_count = 0;   //count of transmitted packets
int pos = 0;    //cycles through 0 to DATA_BUFFER_LENGTH-1
float ground_alt = 0;
int address = 0;
bool isFirstRun = true;
bool sendTelemetry = false;
int software_state = 0;

//Hall Effect
const byte interruptPin = 2;  //digital pin 2
volatile byte state = LOW;
int rotation_counter=0;

//RTC
int startTime;
int elapsedTime = 0;  //time in seconds

//EEPROM data retrieval variables
int addr_flag;  //total address used starting from 0
int last_state, last_alt, last_rtc=0, last_packet;
int resumed_state;

unsigned long startMillis;
unsigned long rpmMillis;
int rtc_time_update;

//functions
void initialize_sensors();
void calibrate_sensors();
void dataSequence();

//altimu globals ============================================================================================
float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;
int magnetom_x, magnetom_y, magnetom_z;
float c_magnetom_x, c_magnetom_y, c_magnetom_z;
float MAG_Heading;
float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};
// Euler angles
float roll, pitch, yaw;
float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
unsigned int counter=0;
byte gyro_sat=0;
float DCM_Matrix[3][3]= {{1,0,0},{0,1,0},{0,0,1}};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3]={{0,0,0},{0,0,0},{0,0,0}};
//end of altimu globals =====================================================================

TimedAction timedAction = TimedAction(1000,dataSequence);

/* ========================================================================================================
 * Setup
 * ========================================================================================================*/

void setup()
{
  Serial.begin(9600);
  GPS.begin(9600);
  gpsSerial.begin(9600);
  Xbee.begin(9600);
  Wire.begin();
  //SPI.setSCK(14);
  SPI.begin();
  delay(250);
  
  pinMode(PROPER_PIN, INPUT);
}

/* ========================================================================================================
 * Loop
 * ========================================================================================================*/

void loop() //Main Loop
{
  payload_stateMachine(pos);
  if(sendTelemetry){
    altimu(); //runs IMU sensor reading
    gps_reading();  //runs gps sensor reading
    timedAction.check();    //handles data at a rate specified by timed action
  }
}

/* ========================================================================================================
 * Functions
 * ========================================================================================================*/

void initialize_sensors(){
  //PTS ----------------------------------------------------
  if (!PTS.init()){
    Serial.println("Error: PTS failed to initialize!");
  }
  else{Serial.println("PTS initialized.");}
  //SD card --------------------------------------------------
  //Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);
  if (!sd.begin(chipSelect, SD_SCK_MHZ(8))) {
    Serial.println("Error: SD card failed to initialize.");
  }
  //if (!SD.begin(chipSelect)) {
  //  Serial.println("Error: SD card failed to initialize, or not present.");
  //}
  else{Serial.println("SD card initialized.");}
  //RTC ----------------------------------------------------
  if (! RTC.begin()){
    Serial.println("Error: RTC failed to initialize!");
  }
  else{Serial.println("RTC initialized.");}
  if (RTC.lostPower()) 
  {
    Serial.println("RTC lost power, lets set the time!");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  //ACC, MAG, GYRO -----------------------------------------
  I2C_Init();
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  //GPS ----------------------------------------------------
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  //1Hz update rate
  //Enable
  PTS.enableDefault();
  //other
  SERVO.attach(SERVO_PIN);
  SERVO.write(SERVO_OFF);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR, RISING);  //interrupts on rising edge of hall effect
  rpmMillis = millis();   //start rpm timer as soon as interrupt begins, will be reset where appropriate
}

void calibrate_sensors(){
  //barometric altitude ------------------------------------
  float pressure, altitude;
  float alt_min, alt_max, alt_0, avg=0;
  unsigned long s_t;
  pressure = PTS.readPressureMillibars();
  alt_0 = PTS.pressureToAltitudeMeters(pressure);
  alt_min = alt_0;
  alt_max = alt_0;
  s_t = millis();
  while((s_t-millis()) < 6000){
    pressure = PTS.readPressureMillibars();
    altitude = PTS.pressureToAltitudeMeters(pressure);
    if(altitude < alt_min){
      alt_min = altitude;
    }
    if(altitude > alt_max){
      alt_max = altitude;
    }
  }
  avg = (alt_max + alt_min)/2; //averages
  ground_alt = avg; //sets ground as reference

  //ACC, MAG, GYRO --------------------------------------
  for(int i=0;i<32;i++){    // We take some readings...
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
  }
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  //Serial.println("Offset:");
  for(int y=0; y<6; y++){
    Serial.println(AN_OFFSET[y]);
  }
}

void begin_timers(){
  Serial.println("starting timers");
  startMillis = millis();
  timer=millis();
  counter=0;
  now = RTC.now();
  startTime = (now.hour()*60*60)+(now.minute()*60)+now.second();    //marks time reference, starts timing
}

void altimu(){
  if(((millis()-timer)>=20) || isFirstRun){  // Main loop runs at 50Hz
    isFirstRun = false;
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old){
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2){
        G_Dt = 0; // ignore integration times over 200 ms
      }
    }
    else{
      G_Dt = 0;
    }
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }
    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    //printdata();
  }
}

void gps_reading(){
  char c = GPS.read();
  if(GPS.newNMEAreceived()){
    GPS.parse(GPS.lastNMEA());
  }
  /*
  char c = GPS.read();
  //if (c) Serial.print(c);
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  */
}

int bladeSpinRate(){
  int rate;
  float t, pi = 3.14;
  unsigned long currentMillis;
  currentMillis = millis();
  t = (currentMillis-rpmMillis)*(1/1000)*(1/60);   //ms*(s/ms)*(min/s) = min
  //rate = (rotation_counter*2*pi)/t;   //counter = 1 full rotation = 2*pi, rate = [rpm]
  rate = (float)rotation_counter/t;    //2*pi is already implied in rotation_counter increments
  //reset
  rotation_counter = 0;
  //startMillis = millis();
  rpmMillis = millis();
  return rate;
}

void deploy_rotor_blades(){
  digitalWrite(SOLENOID_PIN, HIGH); //deploys rotor blades
}

void deploy_parachute(){
  SERVO.write(SERVO_ON);  //deploys parachute
}

int rtc_time(){
  now = RTC.now();    //gets current time stamp, then gets the time difference since initialize()
  elapsedTime = ((now.hour()*60*60)+(now.minute()*60)+now.second())-startTime;    //gets time in seconds
  return elapsedTime;
}

void dataSequence(){
  getData(pos);
  storeData(pos);
  //no need to clear eeprom since total used addresses will always increment, never decrement (unless rtc time rolls over)
  save_to_EEPROM(software_state, ground_alt, rtc_time_update, packet_count); //saves as last data onto EEPROM, note: ground_alt = xx.xx
  sendData(pos);
  printData(pos);
  pos++;    // pos: 0 to 9
  if(pos == DATA_BUFFER_LENGTH){
    pos = 0;
  }
}

void ISR() {
state = !state;
rotation_counter++;
}
