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
  //printdata();
  //Serial.println(pos);
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

void sendData(int pos){
  if(Xbee.available()>0){
    Serial.println("Xbee sending data...");
  }
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

