#define HWSERIAL Serial3

void setup() {
  Serial.begin(9600);
        HWSERIAL.begin(9600);
}

void loop() {
        int incomingByte;
        
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("USB received: ");
    Serial.println(incomingByte, DEC);
                HWSERIAL.print("USB received:");
                HWSERIAL.println(incomingByte, DEC);
  }
  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
                HWSERIAL.print("UART received:");
                HWSERIAL.println(incomingByte, DEC);
  }
}
