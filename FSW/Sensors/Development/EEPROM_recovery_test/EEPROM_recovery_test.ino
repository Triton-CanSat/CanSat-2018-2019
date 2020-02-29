#include <EEPROM.h>
#include <math.h>

int addr = 0;

float ground_alt = 00908.22;
int software_state = 0;
int rtc = 62;
int packet = 102;

int addr_flag;  //total address used starting from 0

int last_state, last_alt, last_rtc, last_packet;

void setup() {
  Serial.begin(9600);
  delay(250);
  set_last_data(software_state, ground_alt, rtc, packet); //note: ground_alt = xx.xx
}

void loop() {
  int u=0;
  Serial.println("saved data: ");
  while(u<addr_flag){
    Serial.println(EEPROM_read(u));
    u++;
  }
  Serial.println("end");
  Serial.println("retrieved data: ");
  retrieve_last_data(addr_flag);
  Serial.println("last state, alt, time, packet: ");
  Serial.println(last_state);
  Serial.println((float)last_alt/100);
  Serial.println(last_rtc);
  Serial.println(last_packet);
  /*
  int i;
  i = ground_alt*100;
  Serial.print("initial i value: ");Serial.println(i);
  delay(250);
  int j;
  while(i){
    j = i%10;
    i = i/10;
    Serial.println(j);  //gets each integer digits
  }
  */
  delay(1000);
}

void retrieve_last_data(int n){    //n = number of used addresses = address_flag
  //[software_state][255][calibrated alt][255][rtc time][255][packet count]
  //1,255,98.22,255,60 > [1/255/2/2/8/9/255/0/6 > 1/255/9822/255/60] > 1/255/98.22/255/60
  int i,j,k;
  int addr=0, sum=0, break_count=0;
  for(i=0; i<n; i++){ //runs through all used addresses
    k = EEPROM_read(addr); addr++;
    if(i==0){
      last_state = k;
    }
    else if(k==255){
      break_count++;
      j=0;
    }
    else{
      if(break_count==1){
        //calibrated altitude
        last_alt = last_alt+(k*pow(10,j));
        j++;
      }
      else if (break_count==2){
        //last rtc time
        last_rtc = last_rtc+(k*pow(10,j));
        j++;
      }
      else if(break_count==3){
        //last packet count
        last_packet = last_packet+(k*pow(10,j));
        j++;
      }
    }
  }
}

void set_last_data(int state, float alt, int rtc_t, int packet){   //return last address starting from 0
  //addr value of 255 can be a divider, data will be entered as single integers
  //example: [software_state][255][calibrated alt][255][rtc time]
  //[1,255,98.22,255,60 > 1/255/2/2/8/9/255/0/6] > 1/255/9822/255/60 > 1/255/98.22/255/60
  int addr = 0; //overwrites previous
  EEPROM_single_write(addr, state); addr++;
  EEPROM_single_write(addr, 255); addr++;    //divider
  int i = alt*100;
  int j;
  while(i){   //gets each integer digits in reverse order
    j = i%10;
    i = i/10;
    EEPROM_single_write(addr,j); addr++;
  }
  EEPROM_single_write(addr, 255); addr++;    //divider
  i = rtc_t;
  while(i){   //gets each integer digits in reverse order
    j = i%10;
    i = i/10;
    EEPROM_single_write(addr,j); addr++;
  }
  EEPROM_single_write(addr, 255); addr++;    //divider
  i = packet;
  while(i){   //gets each integer digits in reverse order
    j = i%10;
    i = i/10;
    EEPROM_single_write(addr,j); addr++;
  }
  addr_flag = addr;
}

void EEPROM_clear(){
  int i;
  for (i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void EEPROM_logger(int address, int t){
  EEPROM.write(address, t);
  address++;
  if(address == EEPROM.length()){
    address = 0;    //overwrite, reset address
  }
}

void EEPROM_single_write(int address, int t){
  EEPROM.write(address, t);
}

int EEPROM_read(int address){
  int val;
  val = EEPROM.read(address);
  return val;
}
