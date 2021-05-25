
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

static const uint32_t GPSBaud = 9600;

char coordinateLat[10];
char coordinateLng[10];
//char coordinate[20];
String coordinateCSV;
String coordinateGMaps;
String formatted;

char latStr[10];
String plusminLat;
int degLat, bilionthsLat;
String plusminLng;
int degLng, bilionthsLng;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
#define ss Serial3


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println("GPS START");
  Serial2.println("GPS Positioning");
  Serial.println("GPS Positioning");
  ss.begin(GPSBaud);
  pinMode(13, OUTPUT);
  //  changeFrequency();
  delay(100);
  ss.flush();

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
}

void loop() {
  GPSgetData();
}

void GPSgetData() {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      digitalWrite(13, HIGH);
      // Latitude in degrees (double)
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      // Longitude in degrees (double)
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);

      // Raw latitude in whole degrees
      Serial.print("Raw latitude = ");
      Serial.print(gps.location.rawLat().negative ? "-" : "+");
      Serial.println(gps.location.rawLat().deg);
      // ... and billionths (u16/u32)
      Serial.println(gps.location.rawLat().billionths);

      // Raw longitude in whole degrees
      Serial.print("Raw longitude = ");
      Serial.print(gps.location.rawLng().negative ? "-" : "+");
      Serial.println(gps.location.rawLng().deg);
      // ... and billionths (u16/u32)
      Serial.println(gps.location.rawLng().billionths);

      // Raw date in DDMMYY format (u32)
      Serial.print("Raw date DDMMYY = ");
      Serial.println(gps.date.value());

      // Year (2000+) (u16)
      Serial.print("Year = ");
      Serial.println(gps.date.year());
      // Month (1-12) (u8)
      Serial.print("Month = ");
      Serial.println(gps.date.month());
      // Day (1-31) (u8)
      Serial.print("Day = ");
      Serial.println(gps.date.day());

      // Raw time in HHMMSSCC format (u32)
      Serial.print("Raw time in HHMMSSCC = ");
      Serial.println(gps.time.value());

      // Hour (0-23) (u8)
      Serial.print("Hour = ");
      Serial.println(gps.time.hour());
      // Minute (0-59) (u8)
      Serial.print("Minute = ");
      Serial.println(gps.time.minute());
      // Second (0-59) (u8)
      Serial.print("Second = ");
      Serial.println(gps.time.second());
      // 100ths of a second (0-99) (u8)
      Serial.print("Centisecond = ");
      Serial.println(gps.time.centisecond());

      // Raw speed in 100ths of a knot (i32)
      Serial.print("Raw speed in 100ths/knot = ");
      Serial.println(gps.speed.value());
      // Speed in knots (double)
      Serial.print("Speed in knots/h = ");
      Serial.println(gps.speed.knots());
      // Speed in miles per hour (double)
      Serial.print("Speed in miles/h = ");
      Serial.println(gps.speed.mph());
      // Speed in meters per second (double)
      Serial.print("Speed in m/s = ");
      Serial.println(gps.speed.mps());
      // Speed in kilometers per hour (double)
      Serial.print("Speed in km/h = ");
      Serial.println(gps.speed.kmph());

      // Raw course in 100ths of a degree (i32)
      Serial.print("Raw course in degrees = ");
      Serial.println(gps.course.value());
      // Course in degrees (double)
      Serial.print("Course in degrees = ");
      Serial.println(gps.course.deg());

      // Raw altitude in centimeters (i32)
      Serial.print("Raw altitude in centimeters = ");
      Serial.println(gps.altitude.value());
      // Altitude in meters (double)
      Serial.print("Altitude in meters = ");
      Serial.println(gps.altitude.meters());
      // Altitude in miles (double)
      Serial.print("Altitude in miles = ");
      Serial.println(gps.altitude.miles());
      // Altitude in kilometers (double)
      Serial.print("Altitude in kilometers = ");
      Serial.println(gps.altitude.kilometers());
      // Altitude in feet (double)
      Serial.print("Altitude in feet = ");
      Serial.println(gps.altitude.feet());

      // Number of satellites in use (u32)
      Serial.print("Number os satellites in use = ");
      Serial.println(gps.satellites.value());

      // Horizontal Dim. of Precision (100ths-i32)
      Serial.print("HDOP = ");
      Serial.println(gps.hdop.value());
      Serial.println();
      dataPreparation();
      digitalWrite(13, LOW);
    }
  }
}

void dataPreparation() {
  //CONVERT DOUBLE > CHAR > CONCATE STRING
  plusminLat = (gps.location.rawLat().negative ? "-" : "");
  degLat = (gps.location.rawLat().deg);
  bilionthsLat = (gps.location.rawLat().billionths);

  plusminLng = (gps.location.rawLng().negative ? "-" : "");
  degLng = (gps.location.rawLng().deg);
  bilionthsLng = (gps.location.rawLng().billionths);

  dtostrf(gps.location.lat(), 6, 6, coordinateLat);
  dtostrf(gps.location.lng(), 6, 6, coordinateLng);

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  coordinateCSV += plusminLat;
  coordinateCSV += degLat;
  coordinateCSV += ",";
  coordinateCSV += bilionthsLat;
  coordinateCSV += ",";
  coordinateCSV += plusminLng;
  coordinateCSV += degLng;
  coordinateCSV += ",";
  coordinateCSV += bilionthsLng;


  coordinateGMaps += plusminLat;
  coordinateGMaps += degLat;
  coordinateGMaps += ".";
  coordinateGMaps += bilionthsLat;
  coordinateGMaps += ",";
  coordinateGMaps += plusminLng;
  coordinateGMaps += degLng;
  coordinateGMaps += ".";
  coordinateGMaps += bilionthsLng;

  formatted += coordinateCSV;
  formatted += ",";
  formatted += gps.satellites.value();
  formatted += ",";
  formatted += (pressure_event.pressure);
  //  coordinate += coordinateLng;




  sendSerialData();

  //CLEAR COORDINATE BUFFER
  coordinateCSV = "";
  coordinateGMaps = "";
  formatted = "";
}

void sendSerialData() {
  Serial2.print("Latitude        = ");
  Serial2.println(coordinateLat);
  Serial2.print("Longitude       = ");
  Serial2.println(coordinateLng);
  Serial2.print("Satelite        = ");
  Serial2.println(gps.satellites.value());
  Serial2.print("Coordinate CSV  = ");
  Serial2.println(coordinateCSV);
  Serial2.print("Coordinate Maps = ");
  Serial2.println(coordinateGMaps);
  Serial2.print("CSV Formatted   = ");
  Serial2.println(formatted);
  Serial2.println();
}


void sendPacket(byte *packet, byte len) {
  for (byte i = 0; i < len; i++)
  {
    ss.write(packet[i]); // GPS is HardwareSerial
  }
}

void bmp280(){
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
}
void changeFrequency() {
  byte packet[] = {
    0xB5, //
    0x62, //
    0x06, //
    0x08, //
    0x06, // length
    0x00, //
    0x64, // measRate, hex 64 = dec 100 ms
    0x00, //
    0x01, // navRate, always =1
    0x00, //
    0x01, // timeRef, stick to GPS time (=1)
    0x00, //
    0x7A, // CK_A
    0x12, // CK_B
  };
  sendPacket(packet, sizeof(packet));
}
