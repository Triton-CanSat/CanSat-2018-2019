void EEPROM_clear(){
  int i;
  for (i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  Serial.println("EEPROM Cleared!");
}

void EEPROM_logger(int address, int t){
  EEPROM.write(address, t);
  address++;
  if(address == EEPROM.length()){
    address = 0;    //overwrite, reset address
  }
}

int EEPROM_read(int address){
  int val;
  val = EEPROM.read(address);
  return val;
}
