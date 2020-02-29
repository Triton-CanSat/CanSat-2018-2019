#ifndef cluster_h
#define cluster_h

typedef struct packet{
  int TEAM_ID = 1954;
  unsigned int MISSION_TIME = 0;
  unsigned int PACKET_COUNT = 0;
  float ALTITUDE = 0.0;
  float PRESSURE = 0.0;
  float TEMP = 0.0;
  float VOLTAGE = 0.0;
  int GPS_HOUR = 0;
  int GPS_MINUTE = 0;
  int GPS_SECOND = 0;
  float GPS_LATITUDE = 0.0;
  float GPS_LONGITUDE = 0.0;
  float GPS_ALTITUDE = 0.0;
  int GPS_SATS = 0;
  unsigned int PITCH = 0;
  unsigned int ROLL = 0;
  int BLADE_SPIN_RATE = 0;
  int SOFTWARE_STATE = 0;   //[boot(0), launch_detect(1), descent(2), deploy(3), idle(4)]
  unsigned int BONUS_DIRECTION = 0;

  float ACC_x = 0.0;
  float ACC_y = 0.0;
  float ACC_z = 0.0;
  float MAG_x = 0.0;
  float MAG_y = 0.0;
  float MAG_z = 0.0;
  unsigned long PIC_COUNT = 0;
  
  //float vel = 0;
  //float netforce = 0;
  
} telemetry;

#endif
