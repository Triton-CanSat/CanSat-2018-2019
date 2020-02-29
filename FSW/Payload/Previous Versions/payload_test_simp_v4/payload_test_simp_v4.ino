/* ========================================================================================================
 * Libraries
 * ========================================================================================================*/
#include <Wire.h>
#include <LPS.h>    //pressure, altitude, temperature
#include <SD.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include <math.h>
#include "RTClib.h"
#include "cluster.h"
#include <TimedAction.h>
#include "HardwareSerial.h"

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
#define SOLENOID_PIN 3
#define SERVO_PIN 6
#define SERVO_OFF 0
#define SERVO_ON 180

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

/* ========================================================================================================
 * Globals
 * ========================================================================================================*/
LPS PTS;        //pressure, altitude, temperature
Servo SERVO;    //servo
DateTime now;   //RTC
RTC_DS3231 RTC;

//HardwareSerial Xbee = Serial3;
HardwareSerial gpsSerial = Serial1;
Adafruit_GPS GPS(&gpsSerial);

telemetry payloadData[DATA_BUFFER_LENGTH];

const int chipSelect = 10;  //SD card
const int chipSelect_cam = 20;  //Camera
int packet_count = 0;   //count of transmitted packets
int pos = 0;    //cycles through 0 to DATA_BUFFER_LENGTH-1
float ground_alt;
int payload_state = 0;

//Hall Effect
const byte interruptPin = 2;  //digital pin 2
volatile byte state = LOW;
int rotation_counter=0;

//RTC
int startTime;
int elapsedTime = 0;  //time in seconds

unsigned long startMillis;

//functions
void initialize_sensors();
void calibrate_sensors();
void dataSequence();

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

TimedAction timedAction = TimedAction(250,dataSequence);

/* ========================================================================================================
 * Setup
 * ========================================================================================================*/

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  Xbee.begin(9600);
  Wire.begin();
  delay(100);
  
  //initialize
  Serial.println("Initializing...");
  initialize_sensors();
  delay(20);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  PTS.enableDefault();
  
  //calibrate
  Serial.println("Calibrating...");
  calibrate_sensors();

  SERVO.attach(SERVO_PIN);
  SERVO.write(SERVO_OFF);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR, RISING);  //interrupts on rising edge of hall effect
  
  startMillis = millis();
  
  timer=millis();
  counter=0;

  now = RTC.now();
  startTime = (now.hour()*60*60)+(now.minute()*60)+now.second();    //marks time reference, starts timing
}

/* ========================================================================================================
 * Loop
 * ========================================================================================================*/

void loop() //Main Loop
{
  timedAction.check();    //handles data at a rate specified by timed action
  
  if((millis()-timer)>=20){  // Main loop runs at 50Hz
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
  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Error: SD card failed to initialize, or not present.");
  }
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
}

void calibrate_sensors(){
  //barometric altitude ------------------------------------
  float pressure, altitude;
  float alt_min=0, alt_max=0, sum=0;
  int i = 0, N=10;  //sensor calibration, averages N samples
  while (i<N){  
    pressure = PTS.readPressureMillibars();
    altitude = PTS.pressureToAltitudeMeters(pressure);
    sum = sum+altitude;
    i++;
  }
  ground_alt = sum/5.0; //sets ground as reference

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

int bladeSpinRate(){
  int rate;
  float t, pi = 3.14;
  unsigned long currentMillis;
  currentMillis = millis();
  t = (currentMillis-startMillis)*(1/1000)*(1/60);   //ms*(s/ms)*(min/s) = min
  rate = (rotation_counter*2*pi)/t;   //counter = 1 full rotation = 2*pi, rate = [rpm]
  rotation_counter = 0;
  startMillis = millis();
  return rate;
}

void triggerSolenoid(){
  digitalWrite(SOLENOID_PIN, HIGH);
  //delay(1000);
  //digitalWrite(solenoidPin, LOW);
}

int rtc_time(){
  now = RTC.now();    //gets current time stamp, then gets the time difference since initialize()
  elapsedTime = ((now.hour()*60*60)+(now.minute()*60)+now.second())-startTime;    //gets time in seconds
  return elapsedTime;
}

void dataSequence(){
  getData(pos);
  //storeData(pos);
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

