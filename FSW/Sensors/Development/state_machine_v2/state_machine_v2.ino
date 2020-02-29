void payload_stateMachine(int pos){
  if(payloadData[pos].SOFTWARE_STATE == 0){ //boot ================================================
    if(){ //normal startup
      //1. initialize sensors
      //2. calibrate sensors
      //3. continue to next software state
      initialize_sensors();
      calibrate_sensors();
      //start timers now(?) and start sending telemetry
      begin_timers();
      begin_telemetry();
      payloadData[pos].SOFTWARE_STATE = 1;
    }
    else if(){  //unexpected power reset startup
      //1. retrieve last data (rtc time, calibrated altitude, last software state)
      //2. initialize sensors
      //3. resume operation from last software state (skip calibration)
      retrieve_last_data();
      intialize_sensors();
      set_last_data();  //sets retrieved data to current data
      payloadData[pos].SOFTWARE_STATE = resumed_state;
    }
  }
  else if(payloadData[pos].SOFTWARE_STATE == 1){  //idle ===========================================
    //1. listen for next software state
    if(payloadData[pos].NETFORCE >= LAUNCH_ACCEL_THRESH){
      //payload_state = 1;  //Problem: there is a delay in the update
      payloadData[pos].SOFTWARE_STATE = 2;
    }
  }
  else if(payloadData[pos].SOFTWARE_STATE == 2){  //launch_detect ==================================
    //1. listen for deploying altitude
    if(descent_detect()){
      payloadData[pos].SOFTWARE_STATE = 3;
    }
  }
  else if(payloadData[pos].SOFTWARE_STATE == 3){  //descent_detect & deploy_parachute ========================================
    //1. imediately deploy parachute
    //2. continues to next state
    deploy_parachute();
    payloadData[pos].SOFTWARE_STATE = 4;
  }
  else if(payloadData[pos].SOFTWARE_STATE == 4){  //deployed_parachute ========================================
    //1. listen for deploying altitude
    if(payloadData[pos].ALTITUDE <= RELEASE_THRESH){
      payloadData[pos].SOFTWARE_STATE = 5;
    }
  }
  else if(payloadData[pos].SOFTWARE_STATE == 5){  //deploy_rotor_blades =========================================
    //1. deploy rotor blades
    //2. continue to next software state (if successful?)
    deploy_rotor_blades();
    payloadData[pos].SOFTWARE_STATE = 6;
  }
  else if(payloadData[pos].SOFTWARE_STATE == 6){  //deployed_rotor_blades =======================================
    //1. start camera recording (?)
    //2. continue to next state when done
    camera_recording();
    if(landing_detect()){
      payloadData[pos].SOFTWARE_STATE = 7;
    }
  }
  else if(payloadData[pos].SOFTWARE_STATE == 7){  //landed =========================================
    //when payload lands, all telemetry transmission shall stop and a located audio beacon shall activate.
    //1. stop telemetry transmission
  }
}

//functions
void initialize_sensors(){}
void calibrate_sensors(){}
void retrieve_last_data(){}
void set_last_data(){}
void begin_timers(){}
void being_telemetry(){}
bool descent_detect(){
  //take numerous samples
  if((payloadData[pos].ALTITUDE < payloadData[pos-1].ALTITUDE) && (payloadData[pos].GPS_ALTITUDE < payloadData[pos].GPS_ALTITUDE)){
    
  }
}
void camera_recording(){}
bool landing_detect(){
  
}

/* Key of changes:
 * accounts for unexpected start ups
 * directly updates software state in payload_stateMachine() rather than in getData()
 * added additional states
 * runs intialize and calibrate in boot sequence rather than setup()
 *  
 * Updated software states:
 * state = [boot, idle, launch_detect, descent, deploy, deployed, landed]
 * [0] boot: initialize, calibrate
 * [1] idle: send telemetry
 * [2] launch_detect: detects falling occurence
 * [3] descent: detect threshold altitude
 * [4] deploy: deploy rotor blades
 * [5] deployed: confirms deployments
 * [6] landed: verifies landing and stops telemetry
 * 
 * Note: separating deployed from deployed incase of unexpected restart during deploying mechanism.
 * break up into more states for higher state recovery accuracy.
*/
