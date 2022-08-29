/*************************************************************************************************
 * This Example sends harcoded data to Ubidots using a ESP32. The code sends a distance value 
 * between a device and its opposite endpoint to Ubidots, then the value will be managed in 
 * Ubidots to calculate the volume of a tank with the characteristics of your tank.
 * 
 * This example is given AS IT IS without any warranty.
 * 
 * Made by María Carlina Hernandez.
 *************************************************************************************************/

/****************************************
 * Include Libraries
 ****************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>


/****************************************
 * Define Constants
 ****************************************/
namespace {
  const char * WIFISSID = "iPhone"; // Put your WifiSSID here
  const char *  PASSWORD = "ivangd93"; // Put your wifi password here
  
  const char * TOKEN = "BBFF-jZCPaW6axeL5PCZQo4sTpHxzNrNrjr"; // Put your Ubidots' TOKEN
  const char * MQTT_CLIENT_NAME = "maceta"; // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
  const char * VARIABLE_LABEL_1 = "level"; // Assign the variable label
  const char * VARIABLE_LABEL_2 = "humidity"; // Assign the variable label
  const char * VARIABLE_LABEL_3 = "temperature"; // Assign the variable label
  const char * VARIABLE_LABEL_4 = "tierra"; // Assign the variable label
  const char * DEVICE_LABEL = "Maceta"; // Assign the device label
  const char * MQTT_BROKER = "industrial.api.ubidots.com"; 
   
  const int DHTPIN = 33; // Pin where is connected the DHT11
  const int DHTTYPE = DHT11; // Type of DHT
  const int trigPin = 5; // Triger pin of the HC-SR04
  const int echoPin = 18; // Echo pin of the HC-SR04 
  const int bomba = 22; // Bomba agua
  const int hum_tierra = 21; //Entrada digital higrometro

}
#define SOUND_SPEED 0.034

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

/* Sensor's declarations */
RTC_DATA_ATTR int riego = 0;
long duration;
float distance;
float level;
/* Space to store the request */
char payload[400];
char topic[200];
/* Space to store values to send */
char str_sensor[10];
char str_TempSensor[10];
char str_HumSensor[10];
char str_Tierra[2];

/****************************************
 * Auxiliar Functions
 ****************************************/
WiFiClient ubidots;
PubSubClient client(ubidots);
DHT dht(DHTPIN, DHTTYPE);

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

/****************************************
 * Sensor Functions
 ****************************************/
float readLevel() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = (pulseIn(echoPin, HIGH)); 
  distance = float(duration*SOUND_SPEED/2);
  level = (distance*(-6.25))+131.25;
  return level; 
}

/****************************************
 * Main Functions
 ****************************************/
void setup() {
  
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor

  //Increment boot number and print it every reboot
  ++bootCount;
  ++riego;
  Serial.println("Boot number: " + String(bootCount));
  Serial.println("Bomba: " + String(riego));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  WiFi.begin(WIFISSID, PASSWORD);

  /* Initializing the DHT11 */
  dht.begin();

  /* Assign the PINS as INPUT/OUTPUT */
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(bomba, OUTPUT);
  pinMode(hum_tierra, INPUT);

  Serial.println();
  Serial.print("Wait for WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(MQTT_BROKER, 1883);
  client.setCallback(callback);  


  if (!client.connected()) {
    reconnect();
  }
  
  /* Reading temperature and humidity */
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int tierra = digitalRead(hum_tierra);  

  /* call the funtion readLevel() */
  level = readLevel();
  
  /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  dtostrf(level, 4, 2, str_sensor);
  dtostrf(humidity, 4, 2, str_HumSensor);
  dtostrf(temperature, 4, 2, str_TempSensor);

  if(tierra == HIGH){
    str_Tierra[0]='1';
    }
  if(tierra == LOW){
    str_Tierra[0]='0';
    }
  
  /* Building the Ubidots request */
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  sprintf(payload, "%s", ""); // Cleans the payload
  sprintf(payload, "{\"%s\": %s,", VARIABLE_LABEL_1, str_sensor); // Adds the variable label
  sprintf(payload, "%s\"%s\": %s,", payload, VARIABLE_LABEL_2, str_HumSensor); // Adds the variable label
  sprintf(payload, "%s\"%s\": %s,", payload, VARIABLE_LABEL_3, str_TempSensor); // Adds the variable label
  sprintf(payload, "%s\"%s\": %s}", payload, VARIABLE_LABEL_4, str_Tierra); // Adds the variable label
    
  //sprintf(payload, "%s {\"value\": %s}}", payload, str_sensor); 

  /* Print the sensor reading to the Serial Monitor */
  Serial.println("Publishing values to Ubidots Cloud");
  Serial.print("Nivel = ");
  Serial.println(level);
  Serial.print("Humidity = ");
  Serial.println(humidity);
  Serial.print("Temperature = ");
  Serial.println(temperature);
  Serial.print("Tierra Humedad = ");
  Serial.println(str_Tierra);
  
  /* Publish the request to Ubidots */
   client.publish(topic, payload);
  client.loop();

  //activa riego cada 12 levantadas
  if (riego == 3){
    digitalWrite(bomba, HIGH);
    delay(6000);
    digitalWrite(bomba, LOW);
    riego = 0;
    }


  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");


  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  Serial.println("Going to sleep now");
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}


void loop() {
  
}
