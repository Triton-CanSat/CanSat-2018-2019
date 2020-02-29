void payload_stateMachine(int pos){
  if(software_state == 0){ //boot ================================================
    if(!digitalRead(PROPER_PIN)){ //normal startup, if pin is off
      //1. initialize sensors
      //2. calibrate sensors
      //3. continue to next software state
      Serial.println("Normal startup mode.");
      Serial.println("Clearing EEPROM.");
      EEPROM_clear();
      Serial.println("Initializing...");
      initialize_sensors();
      Serial.println("Calibrating...");
      calibrate_sensors();
      Serial.println("calibration complete!");
      begin_timers();
      sendTelemetry = true; //begins telemetry
      nextState(pos, 1);
    }
    else{  //unexpected power reset startup
      //1. retrieve last data (rtc time, calibrated altitude, last software state)
      //2. initialize sensors
      //3. resume operation from last software state (skip calibration)
      Serial.println("Unexpected restart startup mode.");
      retrieve_last_data();
      initialize_sensors();
      begin_timers();
      set_last_data(last_state, last_alt/100.00, last_rtc, last_packet);  //sets retrieved data to current data
      Serial.print("retrieved data: ");
      Serial.println(resumed_state);
      Serial.println(last_rtc);
      sendTelemetry = true; //begins telemetry
      nextState(pos, resumed_state);
    }
  }
  else if(software_state == 1){  //idle ===========================================
    //1. listen for next software state
    if(payloadData[pos-1].NETFORCE >= LAUNCH_ACCEL_THRESH){
      nextState(pos, 2);
    }
  }
  else if(software_state == 2){  //launch_detect ==================================
    //1. listen for deploying altitude
    if(descent_detect(pos)){
      nextState(pos, 3);
    }
  }
  else if(software_state == 3){  //descent_detect & deploy_parachute ========================================
    //1. imediately deploy parachute
    //2. continues to next state
    deploy_parachute();
    nextState(pos, 4);
  }
  else if(software_state == 4){  //deployed_parachute ========================================
    //1. listen for deploying altitude
    if(payloadData[pos-1].ALTITUDE <= RELEASE_THRESH){
      nextState(pos, 5);
    }
  }
  else if(software_state == 5){  //deploy_rotor_blades =========================================
    //1. deploy rotor blades
    //2. continue to next software state (if successful?)
    deploy_rotor_blades();
    nextState(pos, 6);
  }
  else if(software_state == 6){  //deployed_rotor_blades =======================================
    //1. start camera recording (?)
    //2. continue to next state when done
    camera_recording();
    if(landing_detect()){
      nextState(pos, 7);
    }
  }
  else if(software_state == 7){  //landed =========================================
    //when payload lands, all telemetry transmission shall stop and a located audio beacon shall activate.
    //1. stop telemetry transmission
    sendTelemetry = false;  //stops telemetry
  }
}

//functions
bool descent_detect(int pos){    //may need to poll real time
  //quick check: altitude less than altitude 1 second ago
  //thorough check: sample altitudes to see a trend
  float diff;
  diff = payloadData[pos-1].ALTITUDE - payloadData[pos-2].ALTITUDE;   //assuming fall rate will be greater than 5 meters per second
  if(diff < -5.0){  //find the sensitivity (or min and max fluctuation) of the altitude sensor
  //if(payloadData[pos-1].ALTITUDE < payloadData[pos-2].ALTITUDE){ //quick comparision
    //sample for accuracy
    
    return true;
  }
  return false;
}
void camera_recording(){}
bool landing_detect(){
  float diff;
  diff = abs(payloadData[pos-1].ALTITUDE) - abs(payloadData[pos-2].ALTITUDE);
  if(abs(diff) < 2){
    return true;
  }
  return false;
}

void nextState(int pos, int s){
  if(pos == DATA_BUFFER_LENGTH-1){
    software_state = s;
  }
  else{
    software_state = s;
  }
}
/* Key of changes:
 * accounts for unexpected start ups
 * directly updates software state in payload_stateMachine() rather than in getData()
 * added additional states
 * runs intialize and calibrate in boot sequence rather than setup()
 *  
 * Updated software states:
 * state = [boot, idle, launch_detect, descent_detect, deployed_parachute, deploy_rotor_blades, deployed_rotor_blades, landed]
 * [0] boot: initialize, calibrate, begins telemetry
 * [1] idle: detects launch acceleration
 * [2] launch_detect: detects falling occurence
 * [3] descent_detect: deploys parachute
 * [4] deployed_parachute: detect threshold altitude
 * [5] deploy_rotor_blades: deploys rotor blades
 * [6] deployed_rotor_blades: listends for landing
 * [6] landed: stops telemetry transmission
 * 
 * Note: separating deployed from deployed incase of unexpected restart during deploying mechanism.
 * break up into more states for higher state recovery accuracy.
*/
