#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_INA219.h> 
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <SD.h>
// --------------------------------------


#define SAMPLE_TIME 1500
#define GPS_BAUDRATE 9600

// ------------------ Activate SD and Tellemetry --------------------------
#define SD_init 0 // 0 is OFF, 1 is ON
#define Tellemetry_send 0 // 0 is OFF, 1 is ON

Adafruit_INA219 ina219_0 (0x68);
TinyGPSPlus gps;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
Adafruit_AHTX0 aht;
Adafruit_Sensor *aht_humidity, *aht_temp;
SPIClass spi = SPIClass(HSPI);

float current_mA = 0;
float power_mW = 0;
float shuntvoltage = 0;
float busvoltage = 0;
float lat = 0;
float lon = 0;
float temperatura_bmp = 0;
float pressao_bmp = 0;
float altitude_bmp = 0;
double ozonio = 0;
double carbono = 0;
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
String jsonStr;
sensors_event_t humidity;
sensors_event_t temp;


//Definição dos parâmetros do sensor de O3
#define RL_o3 10     // Resistência ao lado do DOUT_LED
#define APin_o3 34   // Pino analógico utilizado
float curve_o3[2] = {0.05775, 0.2647};  // Curva do gráfico em log do MQ131 para O3 (a, b)
//Definição dos parâmetros do sensor de CO2
#define RL_co2 10     // Resistência ao lado do DOUT_LED
#define APin_co2 33   // Pino analógico utilizado
float curve_co2[2] = {-0.32372, 0.648};  // Curva do gráfico em log do MQ135 para CO2 (a, b)
float R0_co2 = 0;
float R0_o3 = 0;

// -------------- Configuracoes WiFi -----------------------
//Definindo as informações da rede Wi-Fi
const char* ssid = "Embauba"; //Define o nome do ponto de acesso
const char* password = "satelitaos2"; //Define a senha
// Definindo as informações da servidor HTTP
// const char* serverAddress = "https://obsat.org.br";
// const char* endpoint = "/teste_post/envio.php";
WiFiUDP udp;
IPAddress raspberryPiIP(255, 255, 255, 255);  // Replace with the Raspberry Pi's IP address (192, 168, 4, 255)
const int udpPort = 1234;

// -------------- Configuracoes SD -----------------------
#define SCK 14
#define MISO 12
#define MOSI 13
#define CS 15

// File dataFile;
// const int CS = 15;

// ----------------------------------------------------------

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

void readGPS()
{

  Serial.print(F("Location: "));
  if (gps.location.isValid()){
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    lat = gps.location.lat();
    lon = gps.location.lng();
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print(F("DATE: "));
  if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      } else {
        Serial.println(F("INVALID date-time"));
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

// -------------- MQ131 ----------------

void readMQ131(){
    double ADCread=0;
    double RS, RSR0, Y, X;

    //5 Leituras e tira a media
    for (int count=0;count<5;count++) {
                    ADCread += analogRead(APin_o3);
                    delay(50);
    }
    ADCread = ADCread/5;
    //Calcula RS
    RS = (float)RL_o3 * (4095-ADCread) / ADCread;
    //Calcula RS/R0
    RSR0 = RS/R0_o3;
    //Tira o Log de RSR0 para utilizar na curva log-log (Y)
    Y = log10(RSR0);
    //Calcula o X
    X = (Y - curve_o3[1])/curve_o3[0];
    ozonio =  pow10(X);
    Serial.print("Ozonio: ");
    Serial.println(ozonio);
}

// -------------- MQ135 ----------------

void readMQ135(){
    double ADCread=0;
    double RS, RSR0, Y, X;
    //5 Leituras e tira a media
    for (int count=0;count<5;count++) {
                    ADCread += analogRead(APin_co2);
                    delay(50);
    }
    ADCread = ADCread/5;
    //Calcula RS
    RS = (float)RL_co2 * (4095-ADCread) / ADCread;
    //Calcula RS/R0
    RSR0 = RS/R0_co2;
    //Tira o Log de RSR0 para utilizar na curva log-log (Y)
    Y = log10(RSR0);
    //Calcula o X
    X = (Y - curve_co2[1])/curve_co2[0];
    //Retorna 10^X = PPM
    carbono = pow10(X);
    Serial.print("Carbono: ");
    Serial.println(carbono);
}

// -------------- MPU6050 ----------------

void readMPU(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(accX);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accY);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.println(accZ);
  Serial.print("GyroX:");
  Serial.print(gyroX);
  Serial.print(",");
  Serial.print("GyroY:");
  Serial.print(gyroY);
  Serial.print(",");
  Serial.print("GyroZ:");
  Serial.println(gyroZ);
}

// -------------- AHT ----------------

void readAHT(){
  // Serial.print("AHT temperatura: ");
  // Serial.print(aht_temp);
  // Serial.print("\nAHT Umidade: ");
  // Serial.print(aht_humidity);
  aht_humidity->getEvent(&humidity);
  aht_temp->getEvent(&temp);
  Serial.print("AHT temperatura: ");
  Serial.println(temp.temperature);
  Serial.print("Umidade: ");
  Serial.println(humidity.relative_humidity);
}

// -------------- SAVE SD CARD ----------------

void saveSD(fs ::FS &fs, const char *path, const char *message){
  Serial.println("Salvando no SD...");
  Serial.printf(" Appending to file : %s\n", path);
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
  // sensores["cur"] = current_mA;
  // sensores["pot"] = power_mW;
  sensores["lat"] = lat;
  sensores["long"] = lon;
  sensores["temp"] = temperatura_bmp;
  sensores["press"] = pressao_bmp;
  sensores["alt"] = altitude_bmp;
  sensores["gX"] = gyroX;
  sensores["gY"] = gyroY;
  sensores["gZ"] = gyroZ;
  sensores["aX"] = accX;
  sensores["aY"] = accY;
  sensores["aZ"] = accZ;
  sensores["o3"] = ozonio;
  sensores["co2"] = carbono;
  //Convertendo o JsonDocument em uma string JSON
  String jsonString;
  serializeJson(sensores, jsonString);
  // Imprimindo a string JSON no monitor serial
  return jsonString;
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

// -------------- writeFile on SD ----------------

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

  // Initialize MPU
  if (!mpu.begin(0x68)) {
  Serial.println("Failed to find MPU6050 chip");
  while (1) {
    delay(10);
    }
  }
  Serial.println("MPU initialized...");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  // Initialize INA219
  if (! ina219_0.begin()) 
	{ 
		while (1) {
            Serial.println("Failed to find INA219"); 
            delay(10); 
	    } 
    }
    Serial.println("INA initialized...");

    // Initialize GPS
    Serial2.begin(GPS_BAUDRATE);
    Serial.println("GPS initialized..."); 

    // Initialize BMP
    if (!bmp.begin(0x76)) { /*Definindo o endereço I2C como 0x76. Mudar, se necessário, para (0x77)*/
    //Imprime mensagem de erro no caso de endereço invalido ou não localizado. Modifique o valor 
    Serial.println(F(" BMP280 not found, try another address!"));

    while (1) delay(10);
    }
    Serial.println("BMP initialized...");

    // Initialize SD
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
      writeFile(SD, "/ hello . txt ", " teste ");
    #endif

  Serial.println("All programming codes initialize. Default code running!");   
}

//  ---------------------------------------------- void loop -----------------------------------------------------------------
void loop() {

    readINA();
    readGPS();
    readBMP();
    readMQ131();
    readMQ135();
    readMPU();
    readAHT();
    jsonStr = Json(); //jsonStr = ManualJson();
    Serial.println(jsonStr);
    
    if(SD_init)
    {
      String timeString = getCurrentTime();
      saveSD(SD, "/ hello . txt ", timeString.c_str());
      saveSD(SD, "/ hello . txt ", jsonStr.c_str());
    }    

    if(Tellemetry_send)
    {
      espToRasp(jsonStr);
    }
    
    delay(SAMPLE_TIME);
}