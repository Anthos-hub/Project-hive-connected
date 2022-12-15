#include <MKRWAN.h>
#include <HX711.h>
#include <DHT.h>
#include <DS18B20.h>
#include <Wire.h>
#include "DFRobot_INA219.h"
#include "ArduinoLowPower.h"

//#define DHTTYPE     // Here we use DHT22
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht1(A0, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
DHT dht2(A1, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 1;
const int LOADCELL_SCK_PIN = 2;
const float TARE = 133024;

//definition for the weight
HX711 scale;

//Pour la communication lora 
LoRaModem modem;
char output[64]; 
String appEui = "0000000000000000";
String appKey = "4D740505194EA0BEE9F6EF3A786F5A64";

//Variable pour le lora
bool connected;
int err_count;
int con;
short data[7]; // short type because the higher value is 12001 for the weigh * 100


//Initialisation of the battery
int PinReadBat = A2; //Lecture de la battery
float TensionBat = 0; //Tension de la battery

// Sensor of the temperature DS18B20
uint8_t address_DS18B20[8];
DS18B20 ds(3);
int nbrCapteurTempDS18B20 = 2;
float tabTempDS18B20[2];

/**
 * @fn DFRobot_INA219_IIC
 * @brief pWire I2C controller pointer
 * @param i2caddr  I2C address
 * @n INA219_I2C_ADDRESS1  0x40   A0 = 0  A1 = 0
 * @n INA219_I2C_ADDRESS2  0x41   A0 = 1  A1 = 0
 * @n INA219_I2C_ADDRESS3  0x44   A0 = 0  A1 = 1
 * @n INA219_I2C_ADDRESS4  0x45   A0 = 1  A1 = 1   
  */
DFRobot_INA219_IIC     ina219(&Wire, INA219_I2C_ADDRESS4);


void setup() {

    //Allumage de la LED au demarage
    pinMode(LED_BUILTIN,OUTPUT); //LED start
    digitalWrite(LED_BUILTIN,HIGH); //allumer led au demarage pendant 3 secondes
    delay(3000);
    digitalWrite(LED_BUILTIN,LOW);
  
    //Initialisation de l'UART
    Serial.begin(115200);
    //while (!Serial);
    Serial.println("Welcome to MKR WAN 1300/1310 first configuration sketch");
    Serial.println("Register to your favourite LoRa network and we are ready to go!");

    //Initialisation DHT22
    dht1.begin();
    dht2.begin();

    
    //Initialisaion Lora
    modem.begin(EU868);

    //Initialisation of the sensor of the weight
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  /* Commented to test the code without the sensor
    //Initialiser SEN0291
    while(ina219.begin() != true) { 
        Serial.println("INA219 begin faild");
        delay(2000);
    }*/

    delay(1000);      // apparently the murata dislike if this tempo is removed...

}

void loop() {

//  ---------------------------------- (battery, precision de 1%, resolution de 1% -> 7 bits)
  data[0] = (short) round(battery()*100);
//  ----------------------------------- (poids en kg, precision de 0,2mv/V, resolution de 0.01kg , 120.01 kg -> 14 bits)
  data[1] = (short) round(poids()*100);
//   ---------------------------------- (temp DS18B20 *2 en °C, precision de 0,5°C, resolution de 0.1°C, -10.1° à 85.1°C -> 10 bits + 1 bit)
  tempDS18B20();
  for(int i = 0; i < nbrCapteurTempDS18B20; i++){
    data[2+i] = (short) round(tabTempDS18B20[i]*100);
  }  
//   ---------------------------------- (temp DHT22 en °C, precision de 0,5°C, resolution de 0.1°C, -10.1° à 85.1°C -> 10 bits + 1 bit)
  data[4] = (short) round(tempDHT22(1)*100);
//   ---------------------------------- (Humidite DHT22 en %, precision de 2%, resolution de 0,1% -> 10 bits)
  data[5] = (short) round(humDHT22(1)*100);
//   ---------------------------------- (temp DHT22 en °C, precision de 0,5°C, resolution de 0.1°C, -10.1° à 85.1°C -> 10 bits + 1 bit)
  data[6] = (short) round(tempDHT22(2)*100);
//   ---------------------------------- (Humidite DHT22 en %, precision de 2%, resolution de ...)
  data[7] = (short) round(humDHT22(2)*100);
//   ---------------------------------- (Courrant ...)
  //data.concat(courrantSEN0291());
  //data.concat(";");
//------------------------------------- vérification de connection à LPWAN
   sendData();
   
   // go in low power mode
   //LowPower.deepSleep(10000); // 10s for the test  || 1000 * 60 * 60 * 24 = 86400000 for 1 measurement / day
   delay(100); // small delay after the deepsleep mode to give time to the ESP32 to wake up completely
   delay(10000); // 10s for the test
}

float battery (void){
  float TensionBat = analogRead(PinReadBat)*0.0062 - 0.0644; //Voltage of the battery
  Serial.print("BAT : ");
  Serial.println(TensionBat);
  if(TensionBat < 0){
    TensionBat = 0;
  }
  return TensionBat;
}

float poids (void){
  scale.power_up(); // wake up the sensor
  float reading = (scale.read()- TARE)/28413;
  if(reading < 0) reading = 0;
  Serial.print("poids reading : ");
  Serial.println(reading);
  scale.power_down(); // power down to save power
  return reading;
}

void tempDS18B20(void){
  int j = 0;
  for(int i = 0; i < 2; i++){
    tabTempDS18B20[nbrCapteurTempDS18B20] = -1;
  }
 
  while(ds.selectNext()){
    
    ds.getAddress(address_DS18B20);
    Serial.print("Address:");
    
    for (uint8_t i = 0; i < 8; i++) {
      Serial.print(" ");
      Serial.print(address_DS18B20[i]); 
    }
    Serial.print("\n");
    tabTempDS18B20[j] = ds.getTempC();
    Serial.print("Temperature: ");
    Serial.println(tabTempDS18B20[j]);
    j=j+1;
  }
}

double tempDHT22(int n){
  double temp;
  if(n == 1)      temp = dht1.readTemperature();
  else if(n == 2) temp = dht2.readTemperature();
  Serial.print("Temp DHT22 ");
  Serial.print(n);
  Serial.print(" :");
  Serial.println(temp);
  return temp;
}

double humDHT22(int n){
  double hum;
  if(n == 1)      hum = dht1.readHumidity();
  else if(n == 2) hum = dht2.readHumidity();
  Serial.print("Hum DHT22 ");
  Serial.print(n);
  Serial.print(" :");
  Serial.println(hum);
  return hum;
}


float courrantSEN0291(void){
  float courrant = ina219.getCurrent_mA();
  Serial.print("Couurent SEN0291 : ");
  Serial.println(courrant);
}

int sendData(void){
  if ( !connected ) {
    int ret=modem.joinOTAA(appEui, appKey);
    if ( ret ) {
      connected=true;
      modem.minPollInterval(60);
      Serial.println("Connected");
      modem.dataRate(5);   // switch to SF7
      delay(100);          // because ... more stable
      err_count=0;
    }
  }
  con++; 
  Serial.print("Join test : ");
  Serial.println(con);
  
  if ( connected ) {
    int err=0;
    modem.beginPacket();
    
    for(int i = 0 ; i<8 ; i++){
      modem.write(data[i]);
      Serial.println(data[i]);
    }
    err = modem.endPacket(true);
    if ( err <= 0 ) {
      // Confirmation not received - jam or coverage fault
      err_count++;
      if ( err_count > 50 ) {
        connected = false;
      }
      // wait for 2min for duty cycle with SF12 - 1.5s frame
      for ( int i = 0 ; i < 120 ; i++ ) {
        delay(1000);
      }
    } else {
      err_count = 0;
      //Mode LowPower during 1 minute
      //delay(15000);
      
    }
  }
}
