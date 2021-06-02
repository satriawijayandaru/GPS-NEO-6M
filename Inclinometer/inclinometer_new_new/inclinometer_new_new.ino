const byte numChars = 28;
word receivedChars[numChars];   // an array to store the received data

boolean newData = false;

void setup() {
    Serial2.begin(115200);
    Serial.begin(115200);
    delay(200);
    Serial.println("<Arduino is ready>");
}

void loop() {   
//    uint8_t message[] = {0x77, 0x04, 0x00, 0x04, 0x08 };
//
//    char buf[4];
//    for (uint8_t i = 0; i < sizeof(message); i++) {
//        itoa(message[i], buf, 16);
//        Serial2.write(buf, sizeof(buf));
//    }
//    delay(500);

//    recvWithEndMarker();
//    showNewData();
//Serial2.println("test");
delay(200);
}

#define END_MARKER '\n'
void recvWithEndMarker() {
    static byte ndx = 0;
    char rc;

    while (Serial2.available() > 0 && !newData) {
        rc = Serial2.read();

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
