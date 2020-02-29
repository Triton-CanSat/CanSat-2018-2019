#include <Wire.h>
#include "RTClib.h"

DateTime now;
RTC_DS3231 rtc;

int startTime;
int elapsedTime = 0;  //time in seconds

void setup ()
{
  Serial.begin(9600);
  //delay(2000);
  if (! rtc.begin()) 
  {
    Serial.println("Couldn't find RTC Module");
    while (1);
  }
  if (rtc.lostPower()) 
  {
    Serial.println("RTC lost power, lets set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  now = rtc.now();
  startTime = (now.hour()*60*60)+(now.minute()*60)+now.second();
}

void loop () 
{
  now = rtc.now();
  elapsedTime = ((now.hour()*60*60)+(now.minute()*60)+now.second())-startTime;
  Serial.print("start time: ");
  Serial.println(startTime);
  Serial.println("elapsed time: ");
  Serial.println(elapsedTime);
  delay(250);
}
