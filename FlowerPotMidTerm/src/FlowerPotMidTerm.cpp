/* 
 * Project Flower Pot Mid Term
 * Author: Dillon Davis
 * Date: 3/19/25  
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Air_Quality_Sensor.h"
#include "neopixel.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "IoTClassroom_CNM.h"

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
// Let Device OS manage the connection to the Particle Cloud

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

// Run the application and system concurrently in separate threads
SYSTEM_THREAD(ENABLED);

//Variables
const int OLED_RESET = -1;
String dateTime, timeOnly;
unsigned int lastTime;
int moisture, pubValue, lightOn;
const int MPIN = A2;
const int PIXELCOUNT = 12;
const int AQPIN = A0;
const int DUSTPIN = A5;
const int PHOTOPIN = A1;
const int SWITCHPIN = D16;
const int FANPIN = D17;
int lightIn;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
const int BMEADDRESS = 0x76;
float tempC, pressPA, humidRH, tempF, pressInHg, lastSecond, lastMinute; 
unsigned int currentTime;
int color, value, quality, moist;
char degree = 248;
char percent = 37;
bool timeEnd;
byte status;
String pollution;


/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();
void pixelFill(int startP, int endP, int hex);


// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
SerialLogHandler logHandler(LOG_LEVEL_INFO);
AirQualitySensor airSensor(AQPIN);
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B);
Adafruit_BME280 bme;
IoTTimer timer;
IoTTimer timer2;

//Feeds
Adafruit_MQTT_Subscribe lightSub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttononoff");
Adafruit_MQTT_Publish tempPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temp");
Adafruit_MQTT_Publish aqPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality");
Adafruit_MQTT_Publish dustPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Dust");
Adafruit_MQTT_Publish humidPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish moistPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Moisture");
Adafruit_MQTT_Subscribe waterSub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Button");
Adafruit_MQTT_Subscribe colorSub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Color");

// setup() runs once, when the device is first turned on
void setup() {
  pixel.begin();
  pixel.setBrightness(255);
  pixel.show();
  Serial.begin(9600);
  pinMode(MPIN, INPUT);
  pinMode(DUSTPIN,INPUT);
  pinMode(PHOTOPIN, INPUT);
  pinMode(SWITCHPIN, OUTPUT);
  pinMode(FANPIN, OUTPUT);
  status = bme.begin(BMEADDRESS);
  if ( status == false ) {
    Serial.printf("BME280 at address 0x%02X failed to start ", BMEADDRESS);
  }
  starttime = millis();//get the current time;
  Time.zone( -7); // MST = -7, MDT = -6
  Particle.syncTime(); // Sync time with Particle Cloud
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.clearDisplay();
  display.display();
  mqtt.subscribe(&lightSub);
  mqtt.subscribe(&waterSub);
  mqtt.subscribe(&colorSub);
  digitalWrite(FANPIN, LOW);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  MQTT_connect();
  MQTT_ping();
  //digitalWrite(FANPIN, HIGH);
  
  //Moisture
  moist = analogRead(MPIN);
  if(moist > 2500){
    timer2.startTimer(100);
    while(!timer2.isTimerReady()){
      digitalWrite(SWITCHPIN, HIGH);
    }
    digitalWrite(SWITCHPIN, LOW);
  }

  //Photodiode
  lightIn = analogRead(PHOTOPIN);
  if(lightIn < 25){
    pixelFill(0, PIXELCOUNT, white);
  }
  if(lightIn >= 75){
    pixel.clear();
    pixel.show();
  }
  
  //BME Code
  currentTime = millis();
  tempC = bme.readTemperature (); // deg C
  pressPA = bme.readPressure (); // pascals
  humidRH = bme.readHumidity (); //%RH

  tempF = tempC*(9.0/5)+32.0; 
  pressInHg = pressPA / 3386.39;

  display.setCursor(0,0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.printf("%0.0f%c || %0.0f%c",tempF, degree, humidRH, percent);
  display.setTextSize(1);
  display.printf("Moisture Level: %i\n", moist);

  //Fan Code
  if((millis() - lastMinute) > 60000){
      digitalWrite(FANPIN, HIGH);
      timer.startTimer(10000);
      while(!timer.isTimerReady()){
        digitalWrite(FANPIN, HIGH);
        Serial.printf("\nFAN ON");
      }
      lastMinute = millis();
  }
  digitalWrite(FANPIN, LOW);
  timeEnd = false;
  //Dust Sensor Code
  // duration = pulseIn(DUSTPIN, LOW);
  // lowpulseoccupancy = lowpulseoccupancy+duration;

  // if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
  // {
  //     ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
  //     concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
  //     Serial.print(lowpulseoccupancy);
  //     Serial.print(",");
  //     Serial.print(ratio);
  //     Serial.print(",");
  //     Serial.println(concentration);
  //     lowpulseoccupancy = 0;
  //     display.printf("Dust Level: %f", concentration);
  //     starttime = millis();
  // }

  //Air Sensor Code
  quality = airSensor.slope();
  value = airSensor.getValue();
  display.printf("Air Quality: %i", value);

  if (quality == AirQualitySensor::FORCE_SIGNAL) {
        //Serial.printf("\nHigh pollution! Force signal active.\n");
        display.printf("\nHigh pollution! Force signal active.\n");
        pollution = "High pollution! Force signal active.";
  } 
   if (quality == AirQualitySensor::FRESH_AIR) {
    //Serial.printf("\nFresh air.\n");
    display.printf("\nFresh air.");
    pollution = "Fresh Air!";
  }
   if (quality == AirQualitySensor::LOW_POLLUTION) {
    //Serial.printf("\nLow pollution!\n");
    display.printf("\nLow pollution!");
    pollution = "Low pollution!";
  } 
   if (quality == AirQualitySensor::HIGH_POLLUTION) {
        //Serial.printf("\nHigh pollution!\n");
        display.printf("\nHigh pollution!");
        pollution = "High pollution!";
  } 
  
  

  //delay(1000);
  display.display();
  display.clearDisplay();
  if((millis() - lastSecond)> 1000){
    lastSecond = millis();
    //Serial.printf("\nSensor value: %i", value);
    Serial.printf("\nLight IN: %i", lightIn);
    //Serial.printf("\n%i", quality);
  }

  //Subscribe from Adafruit
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if(subscription == &colorSub){
      color = atoi((char *)colorSub.lastread);
      pixelFill(0,PIXELCOUNT,color);
    }
    //Turns on or off light
    if(subscription == &lightSub){
      lightOn = atoi((char *)lightSub.lastread);
      if(lightOn == 1){
        pixelFill(0,PIXELCOUNT,white);
      }
      else{
        pixel.clear();
        pixel.show();
      }
    }
    if (subscription == &waterSub) {
      int subValue = atoi((char *)waterSub.lastread);
      Serial.printf("%f\n", subValue);
      if(subValue == 1.0){
        digitalWrite(SWITCHPIN, HIGH);
      }
      else{
        digitalWrite(SWITCHPIN, LOW);
      }
    }
    
  }

  //Publish to Adafruit
  if((millis()-lastTime > 10000)) {
    if(mqtt.Update()) {
      aqPub.publish(value);
      aqPub.publish(pollution);
      tempPub.publish(tempF);
      //dustPub.publish(concentration);
      humidPub.publish(humidRH);
      moistPub.publish(moist);
      Serial.printf("Publishing AQ: %0.2f \n",pubValue);
      Serial.printf("Publishing TempF: %0.2f \n",tempF); 
      //Serial.printf("Publishing Dust: %0.2f \n",concentration);  
      Serial.printf("Publishing Humidity: %0.2f \n",humidRH);
      } 
    lastTime = millis();
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

void pixelFill(int startP, int endP, int hex){
  for(int i = startP; i <= endP; i++){
    pixel.setPixelColor(i, hex);
    pixel.show();
  }
}
