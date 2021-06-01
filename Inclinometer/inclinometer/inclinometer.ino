#define inclinometer  Serial1
String terima;

void setup() {
  inclinometer.begin(9600);
  
}

void loop() {
  inclinometerangle();
  delay(500);
}

void inclinometerangle(){
  inclinometer.write(0x77);
  inclinometer.write(0x04);
  inclinometer.write(0x00);
  inclinometer.write(0x04);
  inclinometer.write(0x08);
  terima = inclinometer.read();
  Serial.println(terima);
  }
