#define inclinometer  Serial2
#define outputSerial Serial
char terima[28];

void setup() {
  outputSerial.begin(115200);
  inclinometer.begin(9600);

}

void loop() {
  inclinometerangle();
  delay(500);
}

void inclinometerangle() {
  inclinometer.write(0x77);
  inclinometer.write(0x04);
  inclinometer.write(0x00);
  inclinometer.write(0x04);
  inclinometer.write(0x08);
  int i;
  if (inclinometer.available()) {
    terima[i] = inclinometer.read();
    i++;
  }
  outputSerial.print("from inclinometer : ");
  outputSerial.println(terima);
}
