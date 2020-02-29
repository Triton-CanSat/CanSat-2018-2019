void getData(int pos){
  rtc_time_update = rtc_time()+last_rtc;
  payloadData[pos].MISSION_TIME =  rtc_time_update;
  payloadData[pos].PACKET_COUNT = packet_count;
  payloadData[pos].PRESSURE = PTS.readPressureMillibars();
  Serial.print("ground alt: ");Serial.println(ground_alt);
  payloadData[pos].ALTITUDE = PTS.pressureToAltitudeMeters(payloadData[pos].PRESSURE)-ground_alt;
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
  payloadData[pos].SOFTWARE_STATE = software_state;
  //payloadData[pos].BONUS_DIRECTION = 
  //printdata();
  //Serial.println(pos);
  
  int ax = accel_x/256;
  int ay = accel_y/256;
  int az = accel_z/256;
  payloadData[pos].MAG_x = magnetom_x;
  payloadData[pos].MAG_y = magnetom_y;
  payloadData[pos].MAG_z = magnetom_z;
  payloadData[pos].ACC_x = ax;   //units of g's
  payloadData[pos].ACC_y = ay;
  payloadData[pos].ACC_z = az;
  //payloadData[pos].PIC_COUNT =

  payloadData[pos].NETFORCE = sqrt(pow(ax,2)+pow(ay,2)+pow(az,2));
}

void storeData(int pos){
  File dataFile = sd.open("payload_data2.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(payloadData[pos].TEAM_ID); dataFile.print(",");
    dataFile.print(payloadData[pos].MISSION_TIME); dataFile.print(",");
    dataFile.print(payloadData[pos].PACKET_COUNT); dataFile.print(",");
    dataFile.print(payloadData[pos].ALTITUDE,1);dataFile.print(",");
    dataFile.print(payloadData[pos].PRESSURE);dataFile.print(",");
    dataFile.print(payloadData[pos].TEMP,1);dataFile.print(",");
    dataFile.print(payloadData[pos].VOLTAGE,2);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_HOUR);dataFile.print(":");    //h:m:s
    dataFile.print(payloadData[pos].GPS_MINUTE);dataFile.print(":");
    dataFile.print(payloadData[pos].GPS_SECOND);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_LATITUDE,4);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_LONGITUDE,4);dataFile.print(",");
    dataFile.print(payloadData[pos].GPS_ALTITUDE,1);dataFile.print(",");
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
  Xbee.print(payloadData[pos].ALTITUDE,1);Xbee.print(",");
  Xbee.print(payloadData[pos].PRESSURE);Xbee.print(",");
  Xbee.print(payloadData[pos].TEMP,1);Xbee.print(",");
  Xbee.print(payloadData[pos].VOLTAGE,2);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_HOUR);Xbee.print(":");    //h:m:s
  Xbee.print(payloadData[pos].GPS_MINUTE);Xbee.print(":");
  Xbee.print(payloadData[pos].GPS_SECOND);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_LATITUDE,4);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_LONGITUDE,4);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_ALTITUDE,1);Xbee.print(",");
  Xbee.print(payloadData[pos].GPS_SATS);Xbee.print(",");
  Xbee.print(payloadData[pos].PITCH);Xbee.print(",");
  Xbee.print(payloadData[pos].ROLL);Xbee.print(",");
  Xbee.print(payloadData[pos].BLADE_SPIN_RATE);Xbee.print(",");
  Xbee.print(payloadData[pos].SOFTWARE_STATE);Xbee.print(",");
  Xbee.print(payloadData[pos].BONUS_DIRECTION);
  Xbee.print("\n");
  packet_count++;
}

//for test purpose ==========================================
void printData(int pos){
  Serial.print(payloadData[pos].TEAM_ID); Serial.print(",");
  Serial.print(payloadData[pos].MISSION_TIME); Serial.print(",");
  Serial.print(payloadData[pos].PACKET_COUNT);Serial.print(",");
  Serial.print(payloadData[pos].ALTITUDE,1);Serial.print(",");
  Serial.print(payloadData[pos].PRESSURE);Serial.print(",");
  Serial.print(payloadData[pos].TEMP,1);Serial.print(",");
  Serial.print(payloadData[pos].VOLTAGE,2);Serial.print(",");
  Serial.print(payloadData[pos].GPS_HOUR);Serial.print(":");    //h:m:s
  Serial.print(payloadData[pos].GPS_MINUTE);Serial.print(":");
  Serial.print(payloadData[pos].GPS_SECOND);Serial.print(",");
  Serial.print(payloadData[pos].GPS_LATITUDE,4);Serial.print(",");
  Serial.print(payloadData[pos].GPS_LONGITUDE,4);Serial.print(",");
  Serial.print(payloadData[pos].GPS_ALTITUDE,1);Serial.print(",");
  Serial.print(payloadData[pos].GPS_SATS);Serial.print(",");
  Serial.print(payloadData[pos].PITCH);Serial.print(",");
  Serial.print(payloadData[pos].ROLL);Serial.print(",");
  Serial.print(payloadData[pos].BLADE_SPIN_RATE);Serial.print(",");
  Serial.print(payloadData[pos].SOFTWARE_STATE);Serial.print(",");
  Serial.print(payloadData[pos].BONUS_DIRECTION);
  Serial.println();
  //test prints ---------------
  Serial.print("Net force: ");Serial.println(payloadData[pos].NETFORCE);
  /*
      Serial.print("ANG:");
      Serial.print(ToDeg(roll));
      Serial.print(",");
      Serial.print(ToDeg(pitch));
      Serial.print(",");
      Serial.print(ToDeg(yaw));
      Serial.print(", ACC:");
      Serial.print(accel_x/256);Serial.print(",");
      Serial.print(accel_y/256);Serial.print(",");
      Serial.println(accel_z/256);
      
  Serial.print(payloadData[pos].ACC_x);
  Serial.print(payloadData[pos].ACC_y);
  Serial.println(payloadData[pos].ACC_z);
  */
  //end of test prints ---------
}
// ==============================================================

