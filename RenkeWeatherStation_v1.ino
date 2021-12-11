#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"
#include "DeviceConfig.h"
#include <ModbusMaster.h>
#include <HardwareSerial.h>

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


#define battPIN  34
#define donePIN  25


HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

ModbusMaster node;
// thingcontrol.io setup
String deviceToken = "";
String serverIP = "147.50.151.130";
String serverPort = "9956";


BluetoothSerial SerialBT;

 
struct Device
{
  String windSpeed;
  String windForce;
  String windDir;
  String windDirDeg;
  String hum;
  String temp;
  String pm2_5;
  String pm10;
  String pres;
  String lux;
  String light;
  String rain;
  String uv;
  
};
Device device;
signal meta ;

unsigned long currentMillis;
unsigned long previousMillis;
int interval = 15; // Interval Time
int intervalSleep = 30; // Interval Time
//unsigned int previous_check = 0;
boolean waitSleep = 0;
unsigned long previousMillisSleep;


float Batt = 0.0;
int countSend = 0;



uint16_t dataWeather[14];




float hexTofloat(uint32_t x) {
  return (*(float*)&x);
}


uint16_t readSunLight(char addr , uint16_t  REG) {
  node.begin(ID_RADIATION, modbus);

  Serial.print("readSunLight:");
  unsigned int i = 0;
  uint32_t j, result;
  uint16_t data[1];
  uint32_t value = 0;

  result = node.readInputRegisters (REG, 1); ///< Modbus function 0x04 Read Input Registers
  if (result == node.ku8MBSuccess)   {
    for (j = 0; j < 1; j++)
    {

      data[j] = node.getResponseBuffer(j);
      Serial.print(data[j]); Serial.print(",");

    }
    Serial.println("");
    device.uv = data[0];


  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug

  }
  return value;

}

void readWeather(char addr , uint16_t  REG) {

  node.begin(ID_WEATHER, modbus);

  Serial.print("readWeather:");
  unsigned int i = 0;
  uint32_t j, result;

  uint32_t value = 0;

  result = node.readHoldingRegisters (REG, 14); ///< Modbus function 0x04 Read Input Registers
  if (result == node.ku8MBSuccess)   {
    for (j = 0; j < 14; j++)
    {

      dataWeather[j] = node.getResponseBuffer(j);
      Serial.print(dataWeather[j]); Serial.print(",");


    }



    Serial.println(dataWeather[12]);
    Serial.println(dataWeather[11]);



    device.windSpeed = dataWeather[0];
    device.windForce = dataWeather[1];
    device.windDir = dataWeather[2];
    device.windDirDeg = dataWeather[3] ;
    device.hum = dataWeather[4];
    device.temp = dataWeather[5];   // the number 6th for omiting the Noise 
    device.pm2_5 = dataWeather[7];
    device.pm10 = dataWeather[8];
    device.pres = dataWeather[9];
   

    device.lux = dataWeather[10] + dataWeather[11];

    device.light = dataWeather[12];
    device.rain = dataWeather[13];



    Serial.println("");
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
  }

}


//**************************************************************************************************************
void setup() {

  Serial.begin(115200);

  modbus.begin(9600, SERIAL_8N1, 16, 17);


  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  pingRESP pingR = AISnb.pingIP(serverIP);
  deviceToken = AISnb.getNCCID();
  SerialBT.begin(deviceToken);
  previousMillis = millis() / 1000;
  previousMillisSleep = millis() / 1000;

}

void sendViaNBIOT() {
  meta = AISnb.getSignal();
  String json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken.c_str());
  json.concat("\",\"windSp\":");
  json.concat(device.windSpeed);
  json.concat(",\"windForce\":");
  json.concat(device.windForce);
  json.concat(",\"windDir\":");
  json.concat(device.windDir);
  json.concat(",\"windDirDeg\":");
  json.concat(device.windDirDeg);
  json.concat(",\"hum\":");
  json.concat(device.hum);
  json.concat(",\"temp\":");
  json.concat(device.temp);
  json.concat(",\"pm2_5\":");
  json.concat(device.pm2_5);
  json.concat(",\"pm10\":");
  json.concat(device.pm10);
  json.concat(",\"pres\":");
  json.concat(device.pres);
  json.concat(",\"lux\":");
  json.concat(device.lux);
  json.concat(",\"light\":");
  json.concat(device.light);
  json.concat(",\"uv\":");
  json.concat(device.uv);
  json.concat(",\"batt\":");
  json.concat(String(Batt, 3));
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
}
void loop() {

  unsigned long currentMillis = millis();



  currentMillis = millis() / 1000;

  if ((currentMillis - previousMillis >= interval) && (waitSleep == 0))
  {

    readWeather(ID_WEATHER, _WEATHER);
    delay(1000);
    readSunLight(ID_RADIATION, _RADIATION);



    sendViaNBIOT();
    previousMillis = millis() / 1000;

    countSend++;

    if (countSend >= 4)
    {
      waitSleep = 1;
      previousMillisSleep = millis() / 1000;
    }
  }

  if (waitSleep == 1)
  {
    if (currentMillis - previousMillisSleep >= intervalSleep)
    {
      delay(2000);
      countSend = 0;
      waitSleep = 0;
      doneProcess();
    }
  }


}


float Read_Batt()
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  float R1 = 15000.0;
  float R2 = 3900.0;
  int bitRes = 4096;
  int16_t adc = 0;

  for (int a = 0; a < 20; a++)
  {
    adc  += analogRead(battPIN);
    delay(1);
  }

  vRAW = adc / 20;
  Vout = (vRAW * 3.3) / bitRes;
  Vin = Vout / (R2 / (R1 + R2));
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  return Vin;
}



void doneProcess()
{
  Serial.println("!!!!!! Done ready to Sleep ~10 Min (TPL5110) !!!!!!");
  pinMode(donePIN, OUTPUT);
  digitalWrite(donePIN, HIGH);
  delay(100);
}
