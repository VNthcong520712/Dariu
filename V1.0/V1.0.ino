// Mo hinh he thong do luong - canh bao ve nong do oxy va nhiet do
// Nhom tac gia: TCong, KToan, PAn, TPhung


#include <Wire.h> // Thu vien giao tiep I2C
#include <LiquidCrystal_I2C.h> // Thu vien dieu khien man hinh LCD I2C
#include <OneWire.h> // Thu vien giao tiep 1 wire
#include <DallasTemperature.h> // Thu vien cam bien nhiet do DB18B20
#include "DFRobot_OxygenSensor.h" // Thu vien cam bien nong do O2
#include <Servo.h> // Thu vien dieu khien servo

#define ONE_WIRE_BUS 7 // chan ket noi cam bien nhiet do
#define TEMPERATURE_PRECISION 9 // do chinh xac thang do nhiet do
#define HU_DETE 8 // chan ket noi cam bien chuyen dong
#define WAR 4 // chan canh bao 
#define SET_V 3 // chan set do lon am thanh
#define SER1 5 // cua so xe
#define SER2 6 // cua so xe
#define ADD_O2 ADDRESS_3 // dia chi cam bien o2
#define COLLECT_NUMBER  10 // collect number, the collection range is 1-100.
#define ADD_LCD 0x27 // dia chi man hinh LCD

int numberOfDevices; // so luong cam bien nhiet do
DeviceAddress tempDeviceAddress;

Servo ser1; // khoi tao servo
Servo ser2; // khoi tao servo
OneWire oneWire(ONE_WIRE_BUS); // setup chan doc 1 wire
DallasTemperature tempSensors(&oneWire); // khoi tao cam bien nhiet do
LiquidCrystal_I2C monitor(ADD_LCD, 16, 2); // khoi tao man hinh LCD 1602
DFRobot_OxygenSensor oxygen; // khoi tao cam bien oxy

// ham doc gia tri nhiet do
float Temperature(DeviceAddress deviceAddress) {
  float tempC = tempSensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    return;
  }
  return tempC;
}

// ham hien thi len man hinh LCD
void write_string(int x, int y, String mess) {
  Serial.println(mess);
  monitor.setCursor(0, y);
  monitor.print("                ");
  monitor.setCursor(x, y);
  monitor.print(mess);
}

// setup cac thong so
void setup() {
  Serial.begin(9600);
  tempSensors.begin(); // bat dau nhiet do
  monitor.init(); // bat dau man hinh LCD

  // khai bao chan dieu khien 
  ser1.attach(SER1);
  ser2.attach(SER2);

  // kiem tra cam bien oxy
  while(!oxygen.begin(ADD_O2)){
    Serial.println("I2c device number error !");
    delay(1000);
  }

  numberOfDevices = tempSensors.getDeviceCount(); // lay so luong cam bien nhiet do
  monitor.backlight(); // bat den nen LCD
  write_string(0, 0, "Hello");

  pinMode(HU_DETE, INPUT); // setup nhan du lieu chuyen dong
  pinMode(WAR, OUTPUT); // setup xuat du lieu canh bao
  pinMode(SET_V, OUTPUT); // setup xuat du lieu do lon am thanh

  // tim so luong cam bien nhiet do
  for (int i = 0; i < numberOfDevices; i++) {
    if (tempSensors.getAddress(tempDeviceAddress, i)) {
      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      tempSensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    }
  }
  delay(2000);

  monitor.noBacklight(); // tat den nen
}

void loop() {

  // phat hien co chuyen dong trong xe hay khong
  if (digitalRead(HU_DETE)) {
    monitor.backlight(); // bat man hinh khi co nguoi trong xe
    
    tempSensors.requestTemperatures(); // yeu cau nhiet do

    float temp_; // bien nhan nhiet do
    tempSensors.getAddress(tempDeviceAddress, 0); // setup doc cam bien nhiet do so 1
    temp_ = tempSensors.getTempC(tempDeviceAddress); // nhan nhiet do
    float oxygenData = oxygen.getOxygenData(COLLECT_NUMBER); // nhan nong do oxy

    write_string(0, 0, "%O:" + String(oxygenData) + " *C:" + String(temp_)); // show du lieu len LCD

    // so sanh so lieu
    if ((20 < oxygenData && oxygenData <= 21) || (32 > temp_ && temp_ >= 31)) {
      // canh bao nhe
      digitalWrite(SET_V, 0);
      digitalWrite(WAR, 1);
      write_string(0, 1, "chi so nguy hiem");
    } else if (oxygenData <= 20 || temp_ > 32) {
      // mo cua xe
      ser1.write(45);
      ser2.write(45);
      // canh bao to
      digitalWrite(SET_V, 1);
      monitor.noBacklight();
      digitalWrite(WAR, 1);
      delay(500);
      monitor.backlight();
      digitalWrite(WAR, 0);
      write_string(0, 1, "ha kinh xuong");
      delay(500);
    } else {
      // dung canh bao
      digitalWrite(WAR, 0);
      write_string(0, 1, "");
      // dong cua
      ser1.write(0);
      ser1.write(0);
    }

  } else { // tat dem LCD
    monitor.noBacklight();
  }
}
