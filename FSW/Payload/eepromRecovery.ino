void retrieve_last_data(){
  //[software_state][255][calibrated alt][255][rtc time][255][packet count]
  //1,255,98.22,255,60 > [1/255/2/2/8/9/255/0/6 > 1/255/9822/255/60] > 1/255/98.22/255/60
  int i,j,k;
  int addr=0, sum=0, break_count=0, n;
  n = EEPROM_read(addr); addr++;   //n = number of used addresses = address_flag
  for(i=1; i<n; i++){ //runs through all used addresses
    k = EEPROM_read(addr); addr++;
    if(i==1){
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

void save_to_EEPROM(int state, float alt, int rtc_t, int packet){   //return last address starting from 0
  //addr value of 255 can be a divider, data will be entered as single integers
  //example: [software_state][255][calibrated alt][255][rtc time]
  //[1,255,98.22,255,60 > 1/255/2/2/8/9/255/0/6] > 1/255/9822/255/60 > 1/255/98.22/255/60
  int addr = 0; //overwrites previous
  addr++;   //shifts address by one, addr[0] reserved for used address count
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
  EEPROM_single_write(0,addr_flag);
}

void EEPROM_single_write(int address, int t){
  EEPROM.write(address, t);
}

void set_last_data(int s, float a, int t, int p){
  resumed_state = s;
  ground_alt = a;
  //set rtc time
  packet_count = p;
}

