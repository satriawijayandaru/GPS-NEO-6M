
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include "MPU9250.h"
#include "eeprom_utils.h"

MPU9250 mpu;


Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

static const uint32_t GPSBaud = 9600;
int SerialBaud = 2000000;

char coordinateLat[10];
char coordinateLng[10];
char gpsAlt[5];
char latStr[10];
char speedGPS[10];

String coordinateCSV;
String coordinateGMaps;
String formatted;
String plusminLat;
String plusminLng;

int degLat, bilionthsLat;
int degLng, bilionthsLng;


String pressureBMP;
double altitudeBMP;
double tempBMP;
double altitudeAVG;

int mpuRoll, mpuPitch, mpuYaw;

// The TinyGPS++ object
TinyGPSPlus gps;

// Define Serial Connection
#define internalSerial Serial
#define externalSerial Serial3
#define externalSerialDebug Serial4
#define gpsSerial Serial2

void setup() {
  internalSerial.begin(SerialBaud);
  externalSerial.begin(SerialBaud);
  externalSerialDebug.begin(SerialBaud);
  gpsSerial.begin(GPSBaud);
  internalSerial.println("GPS START");


  
  Wire.begin();
  mpu.setup(0x68);
  pinMode(13, OUTPUT);
  changeFrequency();
  delay(100);
  gpsSerial.flush();

  if (!bmp.begin()) {
    internalSerial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

  delay(500);
  //  internalSerial.println("MPU Calibrating");
  //  mpu.calibrateAccelGyro();
  //  mpu.calibrateMag();
  //  mpu.printCalibration();
  loadCalibration();
  internalSerial.println("GPS Positioning");
  externalSerial.println("GPS Positioning");
  //  delay(2000);
}

void loop() {
  GPSgetData();
}

void GPSgetData() {
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      digitalWrite(13, HIGH);
//      // Latitude in degrees (double)
//      internalSerial.print("Latitude= ");
//      internalSerial.print(gps.location.lat(), 6);
//      // Longitude in degrees (double)
//      internalSerial.print(" Longitude= ");
//      internalSerial.println(gps.location.lng(), 6);
//
//      // Raw latitude in whole degrees
//      internalSerial.print("Raw latitude = ");
//      internalSerial.print(gps.location.rawLat().negative ? "-" : "+");
//      internalSerial.println(gps.location.rawLat().deg);
//      // ... and billionths (u16/u32)
//      internalSerial.println(gps.location.rawLat().billionths);
//
//      // Raw longitude in whole degrees
//      internalSerial.print("Raw longitude = ");
//      internalSerial.print(gps.location.rawLng().negative ? "-" : "+");
//      internalSerial.println(gps.location.rawLng().deg);
//      // ... and billionths (u16/u32)
//      internalSerial.println(gps.location.rawLng().billionths);
//
//      // Raw date in DDMMYY format (u32)
//      internalSerial.print("Raw date DDMMYY = ");
//      internalSerial.println(gps.date.value());
//
//      // Year (2000+) (u16)
//      internalSerial.print("Year = ");
//      internalSerial.println(gps.date.year());
//      // Month (1-12) (u8)
//      internalSerial.print("Month = ");
//      internalSerial.println(gps.date.month());
//      // Day (1-31) (u8)
//      internalSerial.print("Day = ");
//      internalSerial.println(gps.date.day());
//
//      // Raw time in HHMMSSCC format (u32)
//      internalSerial.print("Raw time in HHMMSSCC = ");
//      internalSerial.println(gps.time.value());
//
//      // Hour (0-23) (u8)
//      internalSerial.print("Hour = ");
//      internalSerial.println(gps.time.hour());
//      // Minute (0-59) (u8)
//      internalSerial.print("Minute = ");
//      internalSerial.println(gps.time.minute());
//      // Second (0-59) (u8)
//      internalSerial.print("Second = ");
//      internalSerial.println(gps.time.second());
//      // 100ths of a second (0-99) (u8)
//      internalSerial.print("Centisecond = ");
//      internalSerial.println(gps.time.centisecond());
//
//      // Raw speed in 100ths of a knot (i32)
//      internalSerial.print("Raw speed in 100ths/knot = ");
//      internalSerial.println(gps.speed.value());
//      // Speed in knots (double)
//      internalSerial.print("Speed in knots/h = ");
//      internalSerial.println(gps.speed.knots());
//      // Speed in miles per hour (double)
//      internalSerial.print("Speed in miles/h = ");
//      internalSerial.println(gps.speed.mph());
//      // Speed in meters per second (double)
//      internalSerial.print("Speed in m/s = ");
//      internalSerial.println(gps.speed.mps());
//      // Speed in kilometers per hour (double)
//      internalSerial.print("Speed in km/h = ");
//      internalSerial.println(gps.speed.kmph());
//
//      // Raw course in 100ths of a degree (i32)
//      internalSerial.print("Raw course in degrees = ");
//      internalSerial.println(gps.course.value());
//      // Course in degrees (double)
//      internalSerial.print("Course in degrees = ");
//      internalSerial.println(gps.course.deg());
//
//      // Raw altitude in centimeters (i32)
//      internalSerial.print("Raw altitude in centimeters = ");
//      internalSerial.println(gps.altitude.value());
//      // Altitude in meters (double)
//      internalSerial.print("Altitude in meters = ");
//      internalSerial.println(gps.altitude.meters());
//      // Altitude in miles (double)
//      internalSerial.print("Altitude in miles = ");
//      internalSerial.println(gps.altitude.miles());
//      // Altitude in kilometers (double)
//      internalSerial.print("Altitude in kilometers = ");
//      internalSerial.println(gps.altitude.kilometers());
//      // Altitude in feet (double)
//      internalSerial.print("Altitude in feet = ");
//      internalSerial.println(gps.altitude.feet());
//
//      // Number of satellites in use (u32)
//      internalSerial.print("Number os satellites in use = ");
//      internalSerial.println(gps.satellites.value());
//
//      // Horizontal Dim. of Precision (100ths-i32)
//      internalSerial.print("HDOP = ");
//      internalSerial.println(gps.hdop.value());
//      internalSerial.println();
      dataPreparation();
      digitalWrite(13, LOW);
    }
  }
}

void dataPreparation() {
  //CONVERT DOUBLE > CHAR > CONCATE STRING
//  plusminLat = (gps.location.rawLat().negative ? "-" : "");
//  degLat = (gps.location.rawLat().deg);
//  bilionthsLat = (gps.location.rawLat().billionths);
//
//  plusminLng = (gps.location.rawLng().negative ? "-" : "");
//  degLng = (gps.location.rawLng().deg);
//  bilionthsLng = (gps.location.rawLng().billionths);

  dtostrf(gps.location.lat(), 7, 7, coordinateLat);
  dtostrf(gps.location.lng(), 7, 7, coordinateLng);
  dtostrf(gps.altitude.meters(), 2, 2, gpsAlt);
  dtostrf(gps.speed.mps(), 5, 5, speedGPS);

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  altitudeBMP = (bmp.readAltitude(1013.25));
  tempBMP = (bmp.readTemperature());

  altitudeAVG = (((bmp.readAltitude(1013.25)) + gps.altitude.meters()) / 2);

  if (mpu.update()) {
    mpuRoll = (mpu.getRoll());
    mpuPitch = (mpu.getPitch());
    mpuYaw = (mpu.getYaw());
  }


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

  //  formatted += coordinateCSV;
//  formatted += coordinateGMaps;
  formatted += coordinateLat;
  formatted += ",";
  formatted += coordinateLng;
  formatted += ",";
  formatted += (pressure_event.pressure);   //Pressure value from BMP280
  formatted += ",";
  formatted += altitudeBMP;                 //Altitude from Altimeter BMP280
  formatted += ",";
  formatted += gpsAlt;                      //Altitude from GPS data
  formatted += ",";
  formatted += altitudeAVG;                 //Average Altitude from GPS and Altimeter sensor
  formatted += ",";
  formatted += gps.satellites.value();      //Connected GPS Satellite 
  formatted += ",";
  formatted += tempBMP;                     //Temperature from BMP280
  formatted += ",";
  formatted += speedGPS;                    //GPS Speed in meter per second
  formatted += ",";
  formatted += gps.hdop.value();
  formatted += ",";
  formatted += mpuRoll;
  formatted += ",";
  formatted += mpuPitch;
  formatted += ",";
  formatted += mpuYaw;

  //  coordinate += coordinateLng;




  sendSerialData();

  //CLEAR COORDINATE BUFFER
  coordinateCSV = "";
  coordinateGMaps = "";
  formatted = "";
}

void sendSerialData() {
  internalSerial.println();
  internalSerial.println();
  internalSerial.println();
  internalSerial.println();
  internalSerial.print("Time GMT 0          = ");
  internalSerial.print(gps.time.hour());
  internalSerial.print(":");
  internalSerial.print(gps.time.minute());
  internalSerial.print(":");
  internalSerial.println(gps.time.second());
  internalSerial.print("Time GMT +7         = ");
  internalSerial.print(gps.time.hour() + 7);
  internalSerial.print(":");
  internalSerial.print(gps.time.minute());
  internalSerial.print(":");
  internalSerial.println(gps.time.second());
  internalSerial.print("Latitude            = ");
  internalSerial.println(coordinateLat);
  internalSerial.print("Longitude           = ");
  internalSerial.println(coordinateLng);
  internalSerial.print("Satelite            = ");
  internalSerial.println(gps.satellites.value());
  internalSerial.print("Coordinate Maps     = ");
  internalSerial.println(coordinateGMaps);
  internalSerial.print("Altitude Sensor     = ");
  internalSerial.println(altitudeBMP);
  internalSerial.print("Altitude GPS        = ");
  internalSerial.println(gpsAlt);
  internalSerial.print("Altitude Average    = ");
  internalSerial.println(altitudeAVG);
  internalSerial.print("Speed in m/s        = ");
  internalSerial.print(speedGPS);
  internalSerial.println(" m/s");
  internalSerial.print("Horizontal Accuracy = ");
  internalSerial.println(gps.hdop.value());  
  internalSerial.print("MPU ROLL            = ");
  internalSerial.println(mpuRoll);
  internalSerial.print("MPU PITCH           = ");
  internalSerial.println(mpuPitch);
  internalSerial.print("MPU YAW             = ");
  internalSerial.println(mpuYaw);
  internalSerial.println();
  internalSerial.println();
  internalSerial.println("===================================================================");
  //  internalSerial.print("                  ");
  internalSerial.println("LATITUDE   LONGITUDE   PRESS  ALT-s  ALT-g  ALT-a  S Temp  Speed HDOP  R  P  Y");
  //internalSerial.priln("-6.5808473,106.8907700,981.40,268.60,295.20,281.90,8,30.28,0.24179,120,0,1,58");
  //  internalSerial.print("CSV Formatted   = ");
  internalSerial.println(formatted);
  externalSerial.println(formatted);
  externalSerialDebug.println(formatted);
  internalSerial.println();
}


void sendPacket(byte *packet, byte len) {
  for (byte i = 0; i < len; i++)
  {
    gpsSerial.write(packet[i]); // GPS is HardwareSerial
  }
}

void bmp280() {
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
