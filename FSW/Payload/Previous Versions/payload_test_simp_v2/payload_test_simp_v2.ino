/* ========================================================================================================
 * Libraries
 * ========================================================================================================*/
#include <Wire.h>
#include <LPS.h>    //pressure, altitude, temperature
//#include <L3G.h>    //gyro
//#include <LSM6.h>   //accelerometer and gyro
//#include <LIS3MDL.h>    //magnetometer
//#include <LSM303.h>   //accelerometer and magnetometer
#include <SD.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include <math.h>
#include "RTClib.h"
#include "cluster.h"
#include <TimedAction.h>

/* ========================================================================================================
 * Macros
 * ========================================================================================================*/
#define DATA_BUFFER_LENGTH 10
#define SOLENOID_PIN 3
#define SERVO_PIN 6
#define SERVO_OFF 0
#define SERVO_ON 180

#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
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

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

/* ========================================================================================================
 * Globals
 * ========================================================================================================*/
LPS PTS;        //pressure, altitude, temperature
//LIS3MDL MAG;    //magnometer
//LSM6 ACC;       //accelerometer
//LSM303 ACC_MAG;   //accelerometer and magnetometer
//L3G GYRO;       //gyro
Servo SERVO;    //servo

DateTime now;   //RTC
RTC_DS3231 RTC;

HardwareSerial Xbee = Serial3;
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

void initialize_sensors();
void calibration();
void dataSequence();
TimedAction timedAction = TimedAction(1000,dataSequence);

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

/* ========================================================================================================
 * Functions
 * ========================================================================================================*/
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial3.begin(9600);
  Wire.begin();
  delay(100);
  initialize_sensors();

  PTS.enableDefault();
  //MAG.enableDefault();
  //ACC.enableDefault();
  
  //ACC_MAG.init();
  //ACC_MAG.enableDefault();
  //GYRO.enableDefault();
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  
  I2C_Init();
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  delay(500);
  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  calibration();
  
  SERVO.attach(SERVO_PIN);
  SERVO.write(SERVO_OFF);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR, RISING);  //interrupts on rising edge of hall effect
  startMillis = millis();

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  timer=millis();
  delay(20);
  counter=0;
}

void loop() {
  timedAction.check();

  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
    {
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;
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
    // ***
  }
}

void initialize_sensors(){
  Serial.println("Initializing...");
  if (! PTS.init()){
    Serial.println("Error: Pressure and temperature sensor failed to initialize!");
  }
  else{Serial.println("Connected to PTS.");}
  /*
  if (! ACC_MAG.init()){
    Serial.println("Error: Accelerometer and Magnetometer sensor failed to initialize!");
  }
  else{Serial.println("Connected to ACC_MAG.");}
  if (!GYRO.init())
  {
    Serial.println("Error: Gyro sensor failed to initialize!");
  }
  else{Serial.println("Connected to Gyro.");}
  if (!Serial3.available()){
    Serial.println("Error: Xbee conncetion error!");
  }
  */
  
  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present.");
    // don't do anything more:
    //return;
  }
  else{Serial.println("card initialized.");}
  if (! RTC.begin()) 
  {
    Serial.println("RTC Module failed to initialize!");
  }
  else{Serial.println("RTC Initialized.");}
  if (RTC.lostPower()) 
  {
    Serial.println("RTC lost power, lets set the time!");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  now = RTC.now();
  startTime = (now.hour()*60*60)+(now.minute()*60)+now.second();    //marks time reference, starts timing
}

void calibration(){
  //sensor calibration, averages N samples
  //barometric altitude, roll and pitch
  float pressure, altitude;
  float alt_min=0, alt_max=0, sum=0;
  int i = 0, N=10;
  while (i<N){
    pressure = PTS.readPressureMillibars();
    altitude = PTS.pressureToAltitudeMeters(pressure);
    sum = sum+altitude;
    i++;
  }
  ground_alt = sum/5.0;
}


void getData(int pos){
  payloadData[pos].MISSION_TIME =  rtc_time();
  payloadData[pos].PACKET_COUNT = packet_count;
  payloadData[pos].ALTITUDE = PTS.pressureToAltitudeMeters(payloadData[pos].PRESSURE)-ground_alt;
  payloadData[pos].PRESSURE = PTS.readPressureMillibars();
  payloadData[pos].TEMP = PTS.readTemperatureC();
  //payloadData[pos].VOLTAGE = 
  payloadData[pos].GPS_HOUR = GPS.hour;
  payloadData[pos].GPS_MINUTE = GPS.minute;
  payloadData[pos].GPS_SECOND = GPS.seconds;
  payloadData[pos].GPS_LATITUDE = GPS.lat;
  payloadData[pos].GPS_LONGITUDE = GPS.lon;
  payloadData[pos].GPS_ALTITUDE = GPS.altitude;
  payloadData[pos].GPS_SATS = (int)GPS.satellites;
  //payloadData[pos].PITCH = pitch(ACC_MAG.a.z, ACC_MAG.a.y, ACC_MAG.a.z);
  //payloadData[pos].ROLL = roll(ACC_MAG.a.y, ACC_MAG.a.z);
  payloadData[pos].PITCH = ToDeg(pitch);
  payloadData[pos].ROLL = ToDeg(roll);
  payloadData[pos].BLADE_SPIN_RATE = bladeSpinRate();
  payloadData[pos].SOFTWARE_STATE = payload_state;
  //payloadData[pos].BONUS_DIRECTION = 
/*
  ACC_MAG.read();
  Serial.println(startTime);
  Serial.print(ACC_MAG.m.x);Serial.print(", ");
  Serial.print(ACC_MAG.m.y);Serial.print(", ");
  Serial.print(ACC_MAG.m.z);Serial.print(", ");
  Serial.print(ACC_MAG.a.x);Serial.print(", ");
  Serial.print(ACC_MAG.a.x);Serial.print(", ");
  Serial.print(ACC_MAG.a.x);Serial.println();
  payloadData[pos].MAG_x = ACC_MAG.m.x;
  payloadData[pos].MAG_y = ACC_MAG.m.y;
  payloadData[pos].MAG_z = ACC_MAG.m.z;
  payloadData[pos].ACC_x = ACC_MAG.a.x;
  payloadData[pos].ACC_y = ACC_MAG.a.y;
  payloadData[pos].ACC_z = ACC_MAG.a.z;
  */
  Serial.println(magnetom_x);
  payloadData[pos].MAG_x = magnetom_x;
  payloadData[pos].MAG_y = magnetom_y;
  payloadData[pos].MAG_z = magnetom_z;
  payloadData[pos].ACC_x = accel_x/256;   //units of g's
  payloadData[pos].ACC_y = accel_y/256;
  payloadData[pos].ACC_z = accel_z/256;
  //payloadData[pos].PIC_COUNT =
}

void storeData(int pos){
  File dataFile = SD.open("payload_log.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(payloadData[pos].TEAM_ID); dataFile.print(",");
    dataFile.print(payloadData[pos].MISSION_TIME); dataFile.print(",");
    dataFile.print(payloadData[pos].PACKET_COUNT); dataFile.print(",");
    dataFile.print(payloadData[pos].ALTITUDE);dataFile.print(",");
    dataFile.print(payloadData[pos].PRESSURE);dataFile.print(",");
    dataFile.print(payloadData[pos].TEMP);dataFile.print(",");
    dataFile.print(payloadData[pos].VOLTAGE);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_HOUR);dataFile.print(":");    //h:m:s
    dataFile.print(payloadData[pos].GPS_MINUTE);dataFile.print(":");
    dataFile.print(payloadData[pos].GPS_SECOND);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_LATITUDE);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_LONGITUDE);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_ALTITUDE);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_SATS);dataFile.print(",");
    dataFile.print(payloadData[pos].PITCH);dataFile.print(",");
    dataFile.print(payloadData[pos].ROLL);dataFile.print(",");
    dataFile.print(payloadData[pos].BLADE_SPIN_RATE);dataFile.print(",");
    dataFile.print(payloadData[pos].SOFTWARE_STATE);dataFile.print(",");
    dataFile.print(payloadData[pos].BONUS_DIRECTION);
    dataFile.println();
    dataFile.close();
  }
  else {
    Serial.println("error opening datalog.txt");
  } 
}


//for test purpose ==========================================
void printData(int pos){
  Serial.print(payloadData[pos].TEAM_ID); Serial.print(",");
  Serial.print(payloadData[pos].MISSION_TIME); Serial.print(",");
  Serial.print(payloadData[pos].PACKET_COUNT);Serial.print(",");
  Serial.print(payloadData[pos].ALTITUDE);Serial.print(",");
  Serial.print(payloadData[pos].PRESSURE);Serial.print(",");
  Serial.print(payloadData[pos].TEMP);Serial.print(",");
  Serial.print(payloadData[pos].VOLTAGE);Serial.print(",");
  Serial.print(payloadData[pos].GPS_HOUR);Serial.print(":");    //h:m:s
  Serial.print(payloadData[pos].GPS_MINUTE);Serial.print(":");
  Serial.print(payloadData[pos].GPS_SECOND);Serial.print(",");
  Serial.print(payloadData[pos].GPS_LATITUDE);Serial.print(",");
  Serial.print(payloadData[pos].GPS_LONGITUDE);Serial.print(",");
  Serial.print(payloadData[pos].GPS_ALTITUDE);Serial.print(",");
  Serial.print(payloadData[pos].GPS_SATS);Serial.print(",");
  Serial.print(payloadData[pos].PITCH);Serial.print(",");
  Serial.print(payloadData[pos].ROLL);Serial.print(",");
  Serial.print(payloadData[pos].BLADE_SPIN_RATE);Serial.print(",");
  Serial.print(payloadData[pos].SOFTWARE_STATE);Serial.print(",");
  Serial.print(payloadData[pos].BONUS_DIRECTION);
  Serial.println();
  packet_count++;
}
// ==============================================================

void sendData(int pos){
  Xbee.print(payloadData[pos].TEAM_ID); Xbee.print(",");
  Xbee.print(payloadData[pos].MISSION_TIME); Xbee.print(",");
  Xbee.print(payloadData[pos].PACKET_COUNT);Xbee.print(",");
  Xbee.print(payloadData[pos].ALTITUDE);Xbee.print(",");
  Xbee.print(payloadData[pos].PRESSURE);Xbee.print(",");
  Xbee.print(payloadData[pos].TEMP);Xbee.print(",");
  Xbee.print(payloadData[pos].VOLTAGE);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_HOUR);Xbee.print(":");    //h:m:s
  Xbee.print(payloadData[pos].GPS_MINUTE);Xbee.print(":");
  Xbee.print(payloadData[pos].GPS_SECOND);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_LATITUDE);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_LONGITUDE);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_ALTITUDE);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_SATS);Xbee.print(",");
  Xbee.print(payloadData[pos].PITCH);Xbee.print(",");
  Xbee.print(payloadData[pos].ROLL);Xbee.print(",");
  Xbee.print(payloadData[pos].BLADE_SPIN_RATE);Xbee.print(",");
  Xbee.print(payloadData[pos].SOFTWARE_STATE);Xbee.print(",");
  Xbee.print(payloadData[pos].BONUS_DIRECTION);
  Xbee.println();
  packet_count++;
}
/*
unsigned int roll(float acc_y, float acc_z){
  unsigned int r;
  r = atan2(acc_y, acc_z);
  return r;
}

unsigned int pitch(float acc_x, float acc_y, float acc_z){
  unsigned int p;
  p = atan2(-acc_x, sqrt(sq(acc_y)+sq(acc_z)));
  return p;
}
*/
/*
int bladeSpinRate(int pos){
  int rate, last_t;
  float t;
  if(pos == 0){
    last_t = payloadData[DATA_BUFFER_LENGTH-1].MISSION_TIME;
  }
  else{
    last_t = payloadData[pos-1].MISSION_TIME;
  }
  t = rtc_time_since(last_t)*(1/60);   //sec --> min
  rate = counter/t;   //rpm
  counter = 0;  //reset hall effect counter
  return rate;
}*/

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
/*
int rtc_time_since(int t){
  now = RTC.now();    //gets current time stamp, then gets the time difference since initialize()
  elapsedTime = ((now.hour()*60*60)+(now.minute()*60)+now.second())-t;    //gets time in seconds
  return elapsedTime;
}
*/
int rtc_time(){
  now = RTC.now();    //gets current time stamp, then gets the time difference since initialize()
  elapsedTime = ((now.hour()*60*60)+(now.minute()*60)+now.second())-startTime;    //gets time in seconds
  return elapsedTime;
}

void dataSequence(){
  getData(pos);
  //storeData(pos);
  //sendData(pos);
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
