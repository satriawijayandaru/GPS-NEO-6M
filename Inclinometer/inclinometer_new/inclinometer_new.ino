#define internalSerial Serial
#define sensorSerial Serial3
int sensordata;
int incomingbyte = 0;

const byte numChars = 28;
word receivedChars[numChars];
boolean newData = false;

void setup() {
  sensorSerial.begin(115200);
  internalSerial.begin(115200);
  delay(1500);
  internalSerial.println("ready");
}

void loop() {
  //  sensorSerial1.write(0x77);
  //  delay(30);
  //  sensorSerial1.write(0x04);
  //  delay(30);
  //  sensorSerial1.write(0x00);
  //  delay(30);
  //  sensorSerial1.write(0x04);
  //  delay(30);
  //  sensorSerial1.write(0x08);
  //  delay(30);
  //  sensordata = sensorSerial1.read();
  //
  //  Serial1.print("Sensor data = ");
  //  Serial1.println(sensordata);
  //  delay(200);

//  uint8_t message[] = {0x77, 0x04, 0x00, 0x04, 0x08};
//  char buf[4];
//  for (uint8_t i = 0; i < sizeof(message); i++) {
//    itoa(message[i], buf, 16);
//    sensorSerial.write(buf, sizeof(buf));
//  }
//  delay(500);
  recvWithEndMarker();
  showNewData();
}


#define END_MARKER '\n'
void recvWithEndMarker() {
  static byte ndx = 0;
  char rc;

  while (sensorSerial.available() > 0 && !newData) {
    rc = sensorSerial.read();

    if (rc != END_MARKER) {
      receivedChars[ndx++] = rc;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData() {
  if (newData) {
    Serial.print("data ");
    for (unsigned int i = 0; i < numChars; i++) {
      Serial.print(receivedChars[i], HEX);
    }
    Serial.println();
    newData = false;
  }
}
