//working
#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

// SD chip select pin
const uint8_t chipSelect = 10;

// file system
SdFat sd;

//------------------------------------------------------------------------------
void setup(void) {
  Serial.begin(9600);
  //SPI.setMOSI(7);
  //SPI.setMISO(8);
  SPI.setSCK(14);
  SPI.begin();
}
//------------------------------------------------------------------------------
void loop(void) {
  if (!sd.begin(chipSelect, SD_SCK_MHZ(10))) {
    sd.initErrorHalt();
    Serial.println("Error: failed to connect to sd card!");
  }
  File file = sd.open("video.txt", FILE_WRITE);

  file.print("all good?"); file.println();
  file.close();
  
  Serial.println("complete!");
}
