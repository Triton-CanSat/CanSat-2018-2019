const byte ledPin = 13; //teensy on chip led
const byte interruptPin = 2;  //digital pin 2
volatile byte state = LOW;
int counter=0, spin_rate=0;

unsigned long startMillis;
unsigned long timer;

void setup() {
pinMode(ledPin, OUTPUT);
pinMode(interruptPin, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(interruptPin), ISR, RISING);  //interrupts when pin goes from LOW to HIGH.
Serial.begin(9600);
startMillis = millis();
Serial.println(startMillis);
}

void loop() {
  Serial.println(millis());
  if((millis()-startMillis)>=3000){      //per second
    spin_rate=rpm();
    //reset
    counter = 0;
    startMillis = millis();
    Serial.println("reset!");
  }
  
digitalWrite(ledPin, state);
Serial.print(counter);
Serial.print(",");
Serial.print(state);
Serial.print(",");
Serial.println(spin_rate);
delay(500);
}

int rpm(){
  int rate;
  int r;
  counter = 233;
  Serial.print("test: ");
  Serial.println((float)counter/60);
  Serial.print("to int: ");
  r=(float)counter/60;
  Serial.println(r);
  rate = (float)counter;
  return rate;
}

//Interrupt Service Routine, function must take no params and return nothing.
void ISR() {
state = !state;
counter++;
}
