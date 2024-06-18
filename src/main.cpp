#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_INA219.h> 
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <MQUnifiedsensor.h>
#include "ScioSense_ENS160.h"
#include <MPU6500_WE.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>

// --------------------------------------
#define SAMPLE_TIME 250
//#define ENS160_I2CADDR_1 0x53
//#define AHTXX_ADDRESS_X38 0x38
#define AHT10_I2CADDR AHTX0_I2CADDR_ALTERNATE // AHTX0_I2CADDR_DEFAULT
#define MPU6500_ADDR 0x68
#define INA219_ADDR 0x40
#define BMP280_ADDR 0x76
//#define GPS_BAUDRATE 57600
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPS_BAUDRATE = 9600;
// ------------------ Activate actions --------------------------
#define SERIALPRINT 1 // defined is ON and commented is OFF
#define AHT 1 // defined is ON and commented is OFF
//#define AHT21 1 // defined is ON and commented is OFF
#define ENS 1 // defined is ON and commented is OFF
#define GPS 1 // defined is ON and commented is OFF
#define MPU 1 // defined is ON and commented is OFF
#define BMP 1 // defined is ON and commented is OFF
#define INA 1 // defined is ON and commented is OFF
#define MQ 1 // defined is ON and commented is OFF
#define Tellemetry_send 1 // defined is ON and commented is OFF
#define SD_init 1 // defined is ON and commented is OFF

ScioSense_ENS160 ens160(ENS160_I2CADDR_1); // I2C address of the ENS160 sensor
unsigned long lastMeasurementTime = 1000;
const unsigned long measurementInterval = 4000;
#ifdef AHT21
  #include <AHTxx.h>
  AHTxx aht21(AHTXX_ADDRESS_X38  , AHT2x_SENSOR); // I2C address and type of the AHT21 sensor
#endif
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
Adafruit_INA219 ina219_0 (INA219_ADDR);
TinyGPSPlus gps;
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
SPIClass spi = SPIClass(HSPI);
sensors_event_t humidityEvent, temperatureEvent;

float current_mA = 0;
float power_mW = 0;
float shuntvoltage = 0;
float busvoltage = 0;
float lat = 0;
float lon = 0;
float temperatura_bmp = 0;
float pressao_bmp = 0;
float altitude_bmp = 0;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float temp = 0; 
float resultantG = 0;
String jsonStr;
String jsonStrFull;

//  ------------------ ENS160 and AHT21 definitions --------------------------
float aqi = 0;
float tvoc = 0;
float CO2_ENS = 0;
float hp0 = 0;
float hp1 = 0;
float hp2 = 0;
float hp3 = 0;
float temperature = 0;
float humidity = 0;

//  ------------------  MQ135 Definitions --------------------------
#define placa "ESP 32"
#define Voltage_Resolution 3.3
#define pin 34 //Analog input 0 of your esp32
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm
#define DEFAULT_C02 423.16 //may 2024 default ppm of CO2 for calibration
float calcR0 = 0;
float R0 = 0;
float RL = 0;
float CO = 0;
float Alcohol = 0;
float cFactor = 0;
float CO2_MQ = 0;
float Toluen = 0;
float NH4 = 0;
float Aceton = 0;
//#define calibration_button 13 //Pin to calibrate your sensor
//Declare Sensor
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
#define MQ135_DEFAULTPPM 418.82 //default ppm of CO2 for calibration
#define MQ135_DEFAULTRO 68550 //default Ro for MQ135_DEFAULTPPM ppm of CO2
// #define MQ135_DEFAULTRO 1726.5
#define MQ135_SCALINGFACTOR 116.6020682 //CO2 gas value
#define MQ135_EXPONENT -2.769034857 //CO2 gas value
#define MQ135_MAXRSRO 2.428 //for CO2
#define MQ135_MINRSRO 0.358 //for CO2
/// Parameters for calculating ppm of CO2 from sensor resistance
#define PARA 116.6020682
#define PARB 2.769034857
/// Parameters to model temperature and humidity dependence
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018

// -------------- Configuracoes WiFi -----------------------
//Definindo as informações da rede Wi-Fi
const char* ssid = "Embauba"; //Define o nome do ponto de acesso
const char* password = "satelitaos2"; //Define a senha
WiFiUDP udp;
IPAddress raspberryPiIP(255, 255, 255, 255);  // Replace with the Raspberry Pi's IP address (192, 168, 4, 255)
const int udpPort = 1234;

// -------------- Configuracoes SD -----------------------
#define SCK 14
#define MISO 12
#define MOSI 13
#define CS 15
// File dataFile;

// ------------------------ Leituras ------------------------

void readINA(){
    shuntvoltage = ina219_0.getShuntVoltage_mV(); //Adicionar
    busvoltage = ina219_0.getBusVoltage_V();
    current_mA = ina219_0.getCurrent_mA(); /* comando para chamar a corrente */
    power_mW = ina219_0.getPower_mW(); /*comando para chamar a potência */
    Serial.print("Corrente: "); 
    Serial.print(current_mA); 
    Serial.println(" mA"); /*printa a corrente */
    Serial.print("Potência: "); 
    Serial.print(power_mW); 
    Serial.println(" mW"); /* printa a potência */
    Serial.print("BusVoltage: "); 
    Serial.print(busvoltage); 
    Serial.println(" V"); /* printa a potência */
    Serial.print("Shunt Voltage: "); 
    Serial.print(shuntvoltage); 
    Serial.println(" V"); /* printa a potência */
}

// -------------- GPS ----------------
void displayInfo() {
  Serial.print(F("Location: ")); 
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    lat = gps.location.lat();
    lon = gps.location.lng();
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void readGPS()
{
  while(Serial2.available() > 0) {
    if(gps.encode(Serial2.read())) {
      displayInfo();
    }
  }

  if(millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}

// -------------- BMP ----------------

void readBMP(){
    temperatura_bmp = bmp.readTemperature();
    pressao_bmp = bmp.readPressure();
    altitude_bmp = bmp.readAltitude(1013.25);
    Serial.print(F("Temperatura = "));
    Serial.print(temperatura_bmp);
    Serial.println(" *C");
    //Imprimindo os valores de Pressão
    Serial.print(F("Pressão = "));
    Serial.print(pressao_bmp);
    Serial.println(" Pa");
    //Imprimindo os valores de Altitude Aproximada
    Serial.print(F("Altitude Aprox = "));
    Serial.print(altitude_bmp); /* Ajustar a pressão de nível do mar de acordo com o local!*/
    Serial.println(" m");

}

// -------------- MQ135 ----------------

float getCorrectionFactor(float t, float h) {
  return CORA * t * t - CORB * t + CORC - (h-33.)*CORD;
}

float getCorrectedResistance(long resvalue, float t, float h) {
  return resvalue/getCorrectionFactor(t, h);
}

float getCorrectedPPM(long resvalue,float t, float h, long ro) {
  return PARA * pow((getCorrectedResistance(resvalue, t, h)/ro), -PARB);
}

// -------------- AHT ----------------
void readAHT(){
  aht.getEvent(&humidityEvent, &temperatureEvent);
  Serial.print("Humidity: ");
  Serial.print(humidityEvent.relative_humidity);
  Serial.println("% rH");
}

void readMQ135(){
  // if( isinf(MQ135.getR0()) && millis() < 60000 ){
  //   for(int i = 1; i<=10; i ++)
  //   { MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  //     calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  //     Serial.print(".");}
  //   MQ135.setR0(calcR0/10);
  //   Serial.print("  done! RO value is:");
  //   Serial.println(calcR0/10);
  // }
  R0 = MQ135.getR0(); 
  RL = MQ135.getRL();
  MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  MQ135.setA(605.18); MQ135.setB(-3.937); // Configure the equation to calculate CO concentration value
  CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  MQ135.setA(77.255); MQ135.setB(-3.18); //Configure the equation to calculate Alcohol concentration value
  Alcohol = MQ135.readSensor(); // SSensor will read PPM concentration using the model, a and b values set previously or from the setup
  cFactor = 0;
  if (!isnan(temperatureEvent.temperature) && !isnan(humidityEvent.relative_humidity)) cFactor = getCorrectionFactor(temperatureEvent.temperature, humidityEvent.relative_humidity);
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configure the equation to calculate CO2 concentration value
  CO2_MQ = MQ135.readSensor(false, cFactor); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  MQ135.setA(44.947); MQ135.setB(-3.445); // Configure the equation to calculate Toluen concentration value
  Toluen = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  MQ135.setA(102.2); MQ135.setB(-2.473); // Configure the equation to calculate NH4 concentration value
  NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  MQ135.setA(34.668); MQ135.setB(-3.369); // Configure the equation to calculate Aceton concentration value
  Aceton = MQ135.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
  #ifdef SERIALPRINT
    Serial.print("| CO:  "); Serial.print(CO); 
    Serial.print("   | Alcohol:  "); Serial.print(Alcohol);
    Serial.print("   | CO2:  "); Serial.print(CO2_MQ + DEFAULT_C02); 
    Serial.print("   | Toluen:  "); Serial.print(Toluen); 
    Serial.print("   | NH4:  "); Serial.print(NH4); 
    Serial.print("   | Aceton:  "); Serial.print(Aceton);
    Serial.println("   |"); 
  #endif
}

// -------------- MPU----------------
void readMPU6500(){
// Read accelerometer data
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  //temp = myMPU6500.getTemperature(); // Read temperature data
  //resultantG = myMPU6500.getResultantG(gValue);
  ax = gValue.x;
  ay = gValue.y;
  az = gValue.z;
  // Read gyroscope data
  gx = gyr.x;
  gy = gyr.y;
  gz = gyr.z;
}

// -------------- ENS160 ----------------
void readENS(){
  #ifdef AHT21
  float temperature = aht21.readTemperature();
  float humidity = aht21.readHumidity();
  Serial.println("Temperature: " + String(temperature));
  Serial.println("Humidity: " + String(humidity));
  #endif
  //ens160.measureRaw(0);
  float aqi = ens160.getAQI();
  float tvoc = ens160.getTVOC();
  float CO2_ENS = ens160.geteCO2();
  Serial.println("AQI: " + String(aqi));
  Serial.println("TVOC: " + String(tvoc));
  Serial.println("eCO2: " + String(CO2_ENS));
  ////////////////////
  
  ////////////////////
  // ens160.measure(true);
  // ens160.measureRaw(true);
  // Serial.print("AQI: ");Serial.print(ens160.getAQI());Serial.print("\t");
  // Serial.print("TVOC: ");Serial.print(ens160.getTVOC());Serial.print("ppb\t");
  // Serial.print("eCO2: ");Serial.print(ens160.geteCO2());Serial.print("ppm\t");
  // CO2_ENS = ens160.geteCO2();
  // Serial.print("R HP0: ");Serial.print(ens160.getHP0());Serial.print("Ohm\t");
  // Serial.print("R HP1: ");Serial.print(ens160.getHP1());Serial.print("Ohm\t");
  // Serial.print("R HP2: ");Serial.print(ens160.getHP2());Serial.print("Ohm\t");
  // Serial.print("R HP3: ");Serial.print(ens160.getHP3());Serial.print("Ohm\t");
  // Serial.println();
}

// -------------- SAVE SD CARD ----------------
void saveSD(fs ::FS &fs, const char *path, const char *message){
  Serial.printf("SD saving: Appending to file : %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println(" Failed to open file for appending ");
    return;
  }
  if (file.print(message))
  {
    Serial.println(" Message appended ");
  }
  else
  {
    Serial.println(" Append failed ");
  }
  file.print("\n");
  file.close();
}// saveSD(SD, "/ hello . txt ", "teste");

// -------------- Current time ----------------
String getCurrentTime() {
      unsigned long currentTime = millis(); // Get the current time
      unsigned long hours = (currentTime / 3600000) % 24, minutes = (currentTime / 60000) % 60, seconds = (currentTime / 1000) % 60;
      String timeString = String(hours) + ":" + String(minutes) + ":" + String(seconds); // Create the time string
      return timeString;
    }

// -------------- JSON STRING ----------------

String Json(){
  DynamicJsonDocument sensores(256);
  //Adicionando os valores dos sensores ao JsonObject
  sensores["sat"] = "Embauba";
  sensores["time"] = millis();
  sensores["cur"] = current_mA;
  sensores["temp"] = temperatura_bmp;
  sensores["press"] = pressao_bmp;
  sensores["alt"] = altitude_bmp;
  sensores["lat"] = lat;
  sensores["long"] = lon;
  sensores["aX"] = ax;
  sensores["aY"] = ay;
  sensores["aZ"] = az;
  sensores["gX"] = gx;
  sensores["gY"] = gy;
  sensores["gZ"] = gz;
  sensores["co"] = CO;
  sensores["co2"] = CO2_MQ + DEFAULT_C02;
  String jsonString; //Convertendo o JsonDocument em uma string JSON
  serializeJson(sensores, jsonString); // Imprimindo a string JSON no monitor serial
  return jsonString;
}

String FullJson(){
  DynamicJsonDocument sensores2(512);
  //Adicionando os valores dos sensores ao JsonObject
  sensores2["sat"] = "Embauba";
  sensores2["time"] = millis();
  sensores2["cur"] = current_mA;
  sensores2["pow"] = power_mW;
  sensores2["busV"] = busvoltage;
  sensores2["temp"] = temperatura_bmp;
  sensores2["press"] = pressao_bmp;
  sensores2["alt"] = altitude_bmp;
  sensores2["hum"] = humidityEvent.relative_humidity;
  sensores2["lat"] = lat;
  sensores2["long"] = lon;
  sensores2["aX"] = ax;
  sensores2["aY"] = ay;
  sensores2["aZ"] = az;
  sensores2["gX"] = gx;
  sensores2["gY"] = gy;
  sensores2["gZ"] = gz;
  sensores2["alc"] = Alcohol;
  sensores2["tol"] = Toluen;
  sensores2["NH4"] = NH4;
  sensores2["ace"] = Aceton;
  sensores2["co"] = CO;
  sensores2["eco2"] = CO2_ENS;
  sensores2["co2"] = CO2_MQ + DEFAULT_C02;
  String jsonStringFull; //Convertendo o JsonDocument em uma string JSON
  serializeJson(sensores2, jsonStringFull); // Imprimindo a string JSON no monitor serial
  return jsonStringFull;
}

// -------------- SEND ESP32 JSON TO BASE STATION ----------------
void espToRasp(String jsonStr){
  // StaticJsonDocument<240> jsonBuffer; //Cada par de valores utiliza aproximadamente 16 bytes
  //                                     //Cada par nome-vetor utiliza aproximadamente 16*(1+N) bytes, em que N é o comprimento do vetor 
  // //Criando um objeto JsonObject para armazenar os valores dos sensores
  // JsonObject sensores = jsonBuffer.to<JsonObject>();
  // Send the JSON packet to the Raspberry Pi's IP address
  udp.beginPacket(raspberryPiIP, udpPort);
  udp.print(jsonStr);
  udp.endPacket();
  Serial.println(jsonStr);
}

//-------------- writeFile on SD ----------------

void writeFile(fs ::FS &fs, const char *path, const char *message)
{
  Serial.printf(" Writing file : %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println(" Failed to open file for writing ");
    return;
  }
  if (file.print(message))
  {
    Serial.println(" File written ");
  }
  else
  {
    Serial.println(" Write failed ");
  }
  file.close();
}

//  ---------------------------------------------- void setup -----------------------------------------------------------------
void setup (){
  Serial.begin(115200);
  Serial.println("Hello, SACup!");
  Serial.println();
  //  ----------------------- Initializing satellite ------------------------------------------
  Serial.println("Initializing EmbaubaSAT...");

  // Initialize the WiFi SOFT AP
  WiFi.softAP(ssid, password); //Inicia o ponto de acesso
  Serial.print("Connecting to: "); //Imprime mensagem sobre o nome do ponto de acesso
  Serial.println(ssid);
  Serial.print("IP address: "); //Imprime o endereço de IP
  Serial.println(WiFi.softAPIP()); //Endereço de IP
  udp.begin(udpPort); // Choose a port number

// Initialize GPS
  #ifdef GPS
    Serial2.begin(GPS_BAUDRATE, SERIAL_8N1, RXPin, TXPin);
    Serial.println("GPS initialization done..."); 
  #endif

// Initialize MQ135
  #ifdef MQ
    float calcR0 = 0;
    MQ135.setRL(1); //Set Rl as 1kOhm
    MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b //Set math model to calculate the PPM concentration and the value of constants
    MQ135.init(); 
    Serial.print("Calibrating gas sensor, please wait.");
    for(int i = 1; i<=100; i ++)
    { MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
      calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
      Serial.print(".");}
    MQ135.setR0(calcR0/100);
    Serial.print("  done! RO value is:");
    Serial.println(calcR0/100);
    if(isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");}
    if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");}//while(1);
  #endif

  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000); // Set the I2C clock speed to 100 kHz

  // Initialize MPU6500
  #ifdef MPU
    if(!myMPU6500.init()){
      Serial.println("MPU6500 does not respond");
    }
    else{
      Serial.println("MPU6500 is connected");
    }
    Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
    myMPU6500.autoOffsets();
    Serial.println("Done!");
    myMPU6500.enableGyrDLPF();
    myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
    myMPU6500.setSampleRateDivider(5);
    myMPU6500.setAccRange(MPU6500_ACC_RANGE_16G);
    myMPU6500.enableAccDLPF(true);
    myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  #endif

// Initialize AHT
  #ifdef AHT
    if (!aht.begin(&Wire, 0, AHTX0_I2CADDR_ALTERNATE)) {
      Serial.println("Failed to find AHT10 chip with 0x39 address");
      if (!aht.begin(&Wire, 0, AHTX0_I2CADDR_DEFAULT)) {
      Serial.println("Failed to find AHT10 chip with 0x38 address");
      }
      else{Serial.println("AHT10 initialized with default address");}
      }
   else{Serial.println("AHT10 initialized with alternate address");}
  #endif

  #ifdef AHT21
    if (!aht21.begin()) {
    Serial.println("Could not find a valid ATH21 sensor, check wiring!");
    }
    else{
      Serial.println("ATH21 sensor found");
      // aht21.setResolution(AHTXX_14BIT_RESOLUTION); // Set 14-bit resolution
      Serial.println("ATH21 sensor set to 14-bit resolution");
    }
  #endif
  
  // Initialize BMP
  #ifdef BMP
    if (!bmp.begin(BMP280_ADDR)) { /*Definindo o endereço I2C como 0x76. Mudar, se necessário, para (0x77)*/
    //Imprime mensagem de erro no caso de endereço invalido ou não localizado. Modifique o valor 
    Serial.println(F(" BMP280 not found, try another address!"));
    // while (1) delay(10);
    }
    else{ 
      Serial.println("BMP initialized...");
    }
  #endif
  //Initialize SD
  #ifdef SD_init
    Serial.println("SD initializing...");
    spi.begin(SCK, MISO, MOSI, CS);
    if (!SD.begin(CS, spi, 80000000))
    {
      Serial.println(" Card Mount Failed ");
      return;
    }
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
      Serial.println("No SD card attached ");
      return;
    }
    writeFile(SD, "/ hello . txt ", "");
  #endif

  #ifdef INA
    // Initialize INA219
    if (! ina219_0.begin()) 
    {
      //while (1) {
        Serial.println("Failed to find INA219"); 
      //delay(10); 
      //} 
    }
    else{
      Serial.println("INA initialized...");
    }
  #endif

  // Initialize ENS160
  #ifdef ENS
    Serial.println("ENS160 - Digital air quality sensor");
    if (!ens160.begin()) {
      Serial.println("Could not find a valid ENS160 sensor, check wiring!");
      //while (1);
    }
    else{
    Serial.println("ENS160 sensor found");
    ens160.setMode(ENS160_OPMODE_STD);  // Set standard mode of operation
    Serial.println("ENS160 sensor set to standard mode");
    ens160.measure(1);
    lastMeasurementTime = millis();
    }
    
    /////////////////
    // ens160.begin();
    // Serial.println(ens160.available() ? "done." : "failed!");
    // if (ens160.available()) {
    //   // Print ENS160 versions
    //   Serial.print("\tRev: "); Serial.print(ens160.getMajorRev());
    //   Serial.print("."); Serial.print(ens160.getMinorRev());
    //   Serial.print("."); Serial.println(ens160.getBuild());
    //   Serial.print("\tStandard mode ");
    //   Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!" );
    // }
    // else{
    //   Serial.println("Could not find a valid ENS160 sensor, check wiring!");
    // }
  #endif

  #ifdef MQ
  int maxCalibrationAttempts = 4;
  for (int i = 0; i < maxCalibrationAttempts; i++) {
    float calcR0 = 0;
    Serial.print("Calibrating gas sensor, please wait.");
    for(int j = 1; j<=100; j ++)
    { MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
      calcR0 += MQ135.calibrate(RatioMQ135CleanAir); //Serial.print(".");
    }
    MQ135.setR0(calcR0/100);
    Serial.print("  done! RO value is:");
    Serial.println(calcR0/100);
    if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");}
    if (!isinf(calcR0)) {
      break;
    } else {
      Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    }
  }
  // if(isinf(calcR0)){

  //   float calcR0 = 0;
  //   Serial.print("Calibrating gas sensor, please wait.");
  //   for(int i = 1; i<=100; i ++)
  //   { MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
  //     calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  //     Serial.print(".");}
  //   MQ135.setR0(calcR0/100);
  //   Serial.print("  done! RO value is:");
  //   Serial.println(calcR0/100);
  //   if(isinf(calcR0)) {
  //   Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
  //   }
    // if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");}//while(1);
    // cont++;
    // while(isinf(MQ135.getR0()) && cont < 3);

    // }
  #endif

  Serial.println("All startup programming codes done. Default code running!");

}

//  ---------------------------------------------- void loop -----------------------------------------------------------------
void loop() {
    #ifdef INA
      readINA();
    #endif
    
    #ifdef GPS
      readGPS();
    #endif

    #ifdef BMP
      readBMP();
    #endif

    #ifdef AHT
      readAHT();
    #endif

    #ifdef MQ
      readMQ135();
    #endif

    #ifdef MPU
      readMPU6500();
    #endif

    #ifdef ENS
    if (millis() - lastMeasurementTime >= measurementInterval) {
      readENS();
      lastMeasurementTime = millis();
      ens160.measure(0);
      }
    #endif

    jsonStr = Json(); 
    //Serial.println(jsonStr);
    #ifdef Tellemetry_send
      espToRasp(jsonStr);
    #endif

    jsonStrFull = FullJson();
    #ifdef SD_init
      saveSD(SD, "/ hello . txt ", jsonStrFull.c_str());
    #endif   

    delay(SAMPLE_TIME);
}