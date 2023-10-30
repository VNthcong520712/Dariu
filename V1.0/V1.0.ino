// Mo hinh he thong do luong - canh bao ve nong do oxy va nhiet do
// Nhom tac gia: TCong, KToan, PAn, TPhung


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9
#define HU_DETE 3
#define WAR 4  // pin 4 connected with relay which turn on/off alarm
#define MQ1 A2
#define MQ2 A3

int numberOfDevices;
DeviceAddress tempDeviceAddress;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C monitor(0x27, 16, 2);


float Temperature(DeviceAddress deviceAddress) {
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    return;
  }
  return tempC;
}

void write_string(int x, int y, String mess) {
  monitor.setCursor(0, y);
  monitor.print("                ");
  monitor.setCursor(x, y);
  monitor.print(mess);
}

void setup() {
  Serial.begin(9600);
  sensors.begin();
  monitor.init();

  numberOfDevices = sensors.getDeviceCount();
  monitor.backlight();
  write_string(0, 0, "Hello");

  pinMode(HU_DETE, INPUT);  //pir, instead HLK-LD2410B
  pinMode(WAR, OUTPUT);     //led, instead warning horn

  for (int i = 0; i < numberOfDevices; i++) {
    if (sensors.getAddress(tempDeviceAddress, i)) {
      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    }
  }
  delay(2000);
  monitor.noBacklight();
}

void loop() {
  if (digitalRead(HU_DETE)) {
    monitor.backlight();
    sensors.requestTemperatures();

    float temp_;
    int quali_1 = map(analogRead(MQ1), 200, 60, 21, 16.5), quali_2 = analogRead(MQ2);
    sensors.getAddress(tempDeviceAddress, 0);
    temp_ = sensors.getTempC(tempDeviceAddress);

    write_string(0, 0, "*C:" + String(temp_) + " %O:" + String(quali_1));

    if (17 < quali_1 && quali_1 <= 18) {
      digitalWrite(WAR, 1);
      write_string(0, 1, "muc oxy thap");
    } else if (quali_1 <= 17 || temp_ > 37) {
      monitor.noBacklight();
      digitalWrite(WAR, 1);
      delay(500);
      monitor.backlight();
      digitalWrite(WAR, 0);
      write_string(0, 1, "ha kinh xuong");
    } else {
      write_string(0, 1, "");
    }

  } else {
    monitor.noBacklight();
  }
}
