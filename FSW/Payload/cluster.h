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
  int PITCH = 0;
  int ROLL = 0;
  int BLADE_SPIN_RATE;
  int SOFTWARE_STATE = 0;   //intialize to boot(0)
  unsigned int BONUS_DIRECTION = 0;

  int ACC_x = 0;
  int ACC_y = 0;
  int ACC_z = 0;
  float MAG_x = 0.0;
  float MAG_y = 0.0;
  float MAG_z = 0.0;
  unsigned long PIC_COUNT = 0;
  
  //float vel = 0;
  float NETFORCE = 0;
  
} telemetry;

#endif

/*
 * Telemetry format:
 * <TEAM ID>,<MISSION TIME>,<PACKET COUNT>,<ALTITUDE>, <PRESSURE>,
 * <TEMP>,<VOLTAGE>,<GPS TIME>,<GPS LATITUDE>,<GPS LONGITUDE>,<GPS ALTITUDE>,
 * <GPS SATS>,<PITCH>,<ROLL>,<BLADE SPIN RATE>,<SOFTWARE STATE>,<BONUS DIRECTION>
 */
 /*
  * <TEAM ID> = TEAM_ID (int)
  * <MISSION TIME> = elapseTime (int)
  * <PACKET COUNT> = PACKET_COUNT (int)
  * <ALTITUDE> = GPS.altitude   *relative to ground level, resolution of 0.1 meters
  * <PRESSURE> = pressure (float)   *resolution must be 1 pascals
  * <TEMP> = temperature (float)    *resolution of 0.1 degrees C
  * <VOLTAGE>   *resolution must be 0.01 volts
  * <GPS TIME> = GPS.hour: GPS.minute: GPS.second   *reported in UTC, resolution of 1 sec
  * <GPS LATITUDE> = GPS.latitude   *resolution of 0.0001 degrees
  * <GPS LONGITUDE> = GPS.longitude   *resolution of 0.0001 degrees
  * <GPS ALTITUDE> = GPS.altitude   *resolution of 0.1 meters
  * <GPS SATS> = GPS.satellites (int)
  * <PITCH> resolution must be 1 degree
  * <ROLL> resolution must be 1 degree
  * <BLADE SPIN RATE> = counter *resolution must be 1 rpm
  * <SOFTWARE STATE> = SOFTWARE_STATE [boot, idle, launch detect, deploy] (enum)
  * <BONUS DIRECTION> *specified in degrees
  */
