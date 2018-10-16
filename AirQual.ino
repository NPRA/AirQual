/**
 * This is an interactive console for the uBlox SARA N2 breakout from
 * Exploratory Engineering. The module is connected to pin 10 and 11 on the
 * Arduino. Note that the module uses the uBlox convention of RX and TX markings
 * so TX on the module must be connected to pin 11 and RX on pin 10.
 *
 * Connect the GND and VCC pins on the module to the corresponding pins on the
 * Arduino (GND and 5V or GND and 3.3V) and you are ready to go.
 *
 * This example uses the Horde backend to receive data
 * (at https://nbiot.engineering/) but the send() function can be replaced with
 * sendTo() if you want to send data somewhere else.
 *
 * Use the Serial Monitor to send commands to the module.
 *
 *
 * This example is in the public domain.
 *
 * Read more on the Exploratory Engineering team at
 * https://exploratory.engineering/
 */
#include "TelenorNBIoT.h"
#include <SHT1x.h>
int datapin=18,clkpin=19; //blue, yellow
#define PMS_RX 4 //green
#define PMS_TX 5 //blue
#define RX_PIN 16
#define TX_PIN 17
#define DONEPIN 27
#define AUTO true
#define I2C_ADDRESS 0x3C
#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

//PMS sensor (no, not that kind of pms...)
HardwareSerial pmsSerial(2);
int setpin = 14; //white
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data data;
struct pms5003data tempdata;

TelenorNBIoT nbiot(RX_PIN, TX_PIN, 7000);
SSD1306AsciiWire oled;

SHT1x sht1x(datapin, clkpin);
float tempValC;
float humVal;

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&tempdata, (void *)buffer_u16, 30);
 
  if (sum != tempdata.checksum) {
    Serial.println("Checksum failure");
    s->flush();
    return false;
  }

  data = tempdata;
  
  // success!
  return true;
}

void readSense()
{
  tempValC = sht1x.readTemperatureC();
  humVal = sht1x.readHumidity();
  // Prepare upstream data transmission at the next possible time.
  oled.clear();
  oled.println("Waiting for pms5003");
  digitalWrite(setpin, HIGH); //wake up pms sensor
  long warmTime = 30000;
  long warmStart = millis();
  long printTimer = 0;
  long printTimerStart = millis();
  while(millis() < warmStart+warmTime)
  {
    yield();
    printTimer = millis()-printTimerStart;
    if(printTimer > 1000)
    {
      Serial.print((warmTime-(warmTime+warmStart-millis()))/1000);
      Serial.print(" PMS Data: ");
      if (readPMSdata(&pmsSerial))
      {
        //Serial.print("Part>0.3um/0.1L:"); Serial.println(data.particles_03um);
        if(data.particles_03um == 0) oled.print(".");
        else oled.print("|");
      }
      printTimerStart = millis();
    }
  }
  
  //yieldDelay(20000); //warmup time
  
  
  Serial.print("Temperature in celcius: "); 
  Serial.println(tempValC,DEC);
  Serial.print("Humidity:");
  Serial.println(humVal,DEC);
  char result[80] = "";
  dtostrf(tempValC, 4, 1, result);
  result[strlen(result)] = ',';
  dtostrf(humVal, 4, 1, &result[strlen(result)]);
  result[strlen(result)] = ',';

  Serial.println("PMS Data:");
  if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 50 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");
  }
  
  itoa(data.particles_03um, &result[strlen(result)],10);
  result[strlen(result)] = ',';
  itoa(data.particles_05um, &result[strlen(result)],10);
  result[strlen(result)] = ',';
  itoa(data.particles_10um, &result[strlen(result)],10);
  result[strlen(result)] = ',';
  itoa(data.particles_25um, &result[strlen(result)],10);
  result[strlen(result)] = ',';
  itoa(data.particles_50um, &result[strlen(result)],10);
  result[strlen(result)] = ',';
  itoa(data.particles_100um, &result[strlen(result)],10);
  result[strlen(result)] = ',';
  itoa(data.pm10_env, &result[strlen(result)],10);
  result[strlen(result)] = ',';
  itoa(data.pm25_env, &result[strlen(result)],10);
  result[strlen(result)] = ',';
  itoa(data.pm100_env, &result[strlen(result)],10);
  result[strlen(result)] = '\0';
  
  //SEND DATA HERE
  int timeOut = 0;
  while( !nbiot.sendTo("212.125.231.179",50120,result, strlen(result)) && timeOut < 20)
  {
    oled.clear();
    oled.println(F("Sending message"));
    delay(500);
    timeOut++;
  }
  Serial.println(result);
  
  digitalWrite(setpin, LOW); //put pms sensor to sleep
}

void setup() {
  pinMode(DONEPIN,OUTPUT);
  digitalWrite(DONEPIN, LOW);
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX, PMS_TX);
  pinMode(setpin,OUTPUT);
  digitalWrite(setpin, LOW);
  Serial.begin(9600);
  while (!Serial) ;

  nbiot.begin();
  Wire.begin();                
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  oled.println("Welcome to");
  oled.println("TranTek Air Qual");
}

unsigned long long tmp;
char buf[16];
char databuf[32];
uint16_t tmplen = 32;
uint16_t port = 50120;

void loop() 
{
  int timeOut = 0;
  if(AUTO)
  {
    while( !nbiot.connected() && timeOut < 20)
    {
      oled.clear();
      oled.println(F("Waiting for module"));
      delay(500);
      timeOut++;
    }
    timeOut = 0;
    oled.println(F("Module is online"));
    delay(500);
    
    while( !nbiot.createSocket() && timeOut < 20)
    {
      oled.clear();
      oled.println(F("Opening socket"));
      delay(500);
      timeOut++;
    }
    timeOut = 0;
    oled.println(F("Socket created"));
    delay(500);
    
    /*while( !nbiot.sendTo("212.125.231.179",50120,"Hello", 5) && timeOut < 20)
    {
      oled.clear();
      oled.println(F("Sending message"));
      delay(500);
      timeOut++;
    }*/
    readSense();
    oled.println(F("Data sent"));
    delay(500);
    oled.clear();
    oled.println(F("Shutting down"));
    delay(500);
    while (1) {
      digitalWrite(DONEPIN, HIGH);
      delay(2000);
      digitalWrite(DONEPIN, LOW);
      delay(2000);
    }
  }
}

