#include <TinyGPS++.h>

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

struct GPS_Time{
  int day;
  int month;
  int year;
  int second;
  int minute;
  int hour;
};
struct ContainerTelemetry{
  int team_ID = 1002;
  String MISSION_TIME;
  int packet_count;
  char packet_type;
  int mode;
  int tp_released;
  double altitude;
  double temp;
  double voltage;
  GPS_Time gps_time;
  double gps_latitude;
  double gps_longtitude;
  double gps_altitude;
  int gps_sats;
  int software_state;
  String cmd_echo;
};

Adafruit_BME280 bme;
TinyGPSPlus gps;

//EEPROM DATA
uint8_t softwareState = 0;
uint8_t cxFlag = 0;
uint8_t stFlag = 0;
uint8_t secondParachuteFlag = 0;
uint8_t tetheredFlag = 0; // Flag for tethering payload
uint8_t cameraRecordingFlag = 0;
uint8_t simEnableFlag = 0; 
uint8_t simActivateFlag = 0;
uint8_t simpFlag = 0;
int trig_camera = 2;
bool cameraRecording = false;
bool tethered;

unsigned long cameraStartTime;
unsigned long currentTime;

unsigned long interval5Secs; // variable to check if 5 seconds passed
ContainerTelemetry ct;

double groundAltitude;
double relAltitude;
double altitude5secBefore;

static const int RXPin = 7, TXPin = 8;
static const uint32_t GPSBaud = 9600;
SoftwareSerial gpsSerial(RXPin, TXPin);
#define SEALEVELPRESSURE_HPA (1013.25)
#define TETHER_ALTITUDE (300)


void bmeSetup(){
  
  Serial.begin(9600);
  while(!Serial);    // time to get serial running
  Serial.println(F("BME280 test"));

  unsigned status_bme;
  // default settings
  status_bme = bme.begin();  
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status_bme) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }

  groundAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  altitude5secBefore = groundAltitude;
  
}

void bmeTask(){
  

  ct.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  ct.temp = bme.readTemperature();
  relAltitude = bme.readAltitude(917);

}

void gpsSetup(){
  Serial.begin(9600);
  gpsSerial.begin(GPSBaud);
}
void gpsTask(){
  if(gps.satellites.isValid()){
    ct.gps_sats =(int) gps.satellites.value();
  }
  if(gps.location.isValid()){
    ct.gps_latitude = gps.location.lat();
    ct.gps_longtitude = gps.location.lng();
  }
  if(gps.altitude.isValid()){
    ct.gps_altitude = gps.altitude.meters();
  }
  ct.gps_time.day = gps.date.day();
  ct.gps_time.month = gps.date.month();
  ct.gps_time.year = gps.date.year();
  ct.gps_time.second = gps.time.second();
  ct.gps_time.minute = gps.time.minute();
  ct.gps_time.hour = gps.time.hour();
}

void containerCameraSetup(){
    pinMode(trig_camera, OUTPUT);         
    digitalWrite(trig_camera, HIGH); 
}

void startRecording(){
    
    if(cameraRecording==false){
      cameraRecording = true;
      cameraStartTime = millis();
      // NEED WORK
    }
    else{

    }
}
void stopRecording(){
    // NEED WORK
    cameraRecordingFlag = 0;
}
//Acilip kapanmaya karsi cameraRecordingFlag e bakarak kaydi acicaz
void cameraTask(){
  if(cameraRecordingFlag==1){
    startRecording();
  }
  currentTime = millis();
  if(currentTime-cameraStartTime>60000){
    stopRecording();
  }
}

void commandTask(){
  //NEED WORK
}

void tetherDown(){
  if(tethered==false){
    tethered = true;
    // Payloadi indiren motoru dondur
    //NEED WORK
  }
}
void tetherUp(){
  // Payloadi indiren motoru terse dondur
  //NEED WORK
  
}
//Payload indirme taski
void tetherTask(){
  //NEED WORK
  tetherDown();
  if(relAltitude<140 && tethered){
    tetherUp();
  }
}

void buzzerTask(){
  //NEED WORK
  if(softwareState!=0 && relAltitude <15){
    //buzzeri calistir
    clearEeprom();
    //EEPROMU temizleyip sonsuz donguye sokuyoz ve power loss olmasin diye dua ediyoruz
    while(1){

    }
  }
}
// EEPROM byte 0: softwareState
void clearEeprom(){
  for(int i=0;i<EEPROM.length();i++){
    EEPROM.write(i,0);
  }
}
void writeEeprom(){
  EEPROM.write(0,softwareState);
  EEPROM.write(1,cxFlag);
  EEPROM.write(2,stFlag);
  EEPROM.write(3,secondParachuteFlag);
  EEPROM.write(4,tetheredFlag);
  EEPROM.write(5,cameraRecordingFlag);
  EEPROM.write(6,simEnableFlag);
  EEPROM.write(7,simActivateFlag);
  EEPROM.write(8,simpFlag);
}
void setup() {
  
  bmeSetup();
  gpsSetup();
  containerCameraSetup();
  interval5Secs = millis();
  //Check EEPROM byte 0 
  if(EEPROM.read(0)!=0){
    softwareState = EEPROM.read(0);
    cxFlag = EEPROM.read(1);
    stFlag = EEPROM.read(2);
    secondParachuteFlag = EEPROM.read(3);
    tetheredFlag = EEPROM.read(4);
    cameraRecordingFlag = EEPROM.read(5);
    simEnableFlag = EEPROM.read(6);
    simActivateFlag = EEPROM.read(7);
    simpFlag = EEPROM.read(8);
  }
  
  

}

void loop() {
  
  bmeTask();
  gpsTask();
  cameraTask();
  if(millis()-interval5Secs> 5000){
    altitude5secBefore = ct.altitude;
  }
  //CANSAT 700e ulaştı mı? ulaşmadıysa düşüyor mu 
  if(relAltitude>670 || altitude5secBefore>ct.altitude+2){
    softwareState = 1;
    secondParachuteFlag = 1;
    //DEPLOY PARACHUTE
  }
  if(softwareState == 1 && relAltitude < TETHER_ALTITUDE){
    tetheredFlag = 1;
  }
  tetherTask();
  commandTask(); // 
  writeEeprom();
  buzzerTask();

}
