#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>

#define ONE_WIRE_BUS 2 // D4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define RELAY_PIN 14 // D5

#define WIFI_AP "EGAT-IoT"
#define WIFI_PASSWORD ""
//#define WIFI_AP "ZAB"
//#define WIFI_PASSWORD "Gearman1"
//#define TOKEN "8eGVqAwnnDleorNl7ssn"

#define TOKEN "6KXCSIYTvT9mYquGOWmy"

char thingsboardServer[] = "mqtt.egat.co.th";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

int status = WL_IDLE_STATUS;

unsigned long previousMillis = 0;      
const long interval = 5000;            // 5 second             
const long delayToStart = 3*60*1000;   // 1 minute
const long delayToStop  = 3*60*1000;   // 1 minute
unsigned long previousStop = -delayToStart;
unsigned long previousStart = -delayToStop;
bool stopenable = true;
bool startenable = true;

float t, h, temperature_setpoint = 0;
String responseTopic;
char default_settemp[] = "7"; // default set point

void relayAction (int relay, int cmd) {
  unsigned long currentMillis = millis();
  if (cmd == 1 && startenable && (currentMillis -  previousStop >= delayToStart)) {
    previousStart = currentMillis;
    digitalWrite(relay, HIGH);
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    Serial.println("On relay");
    stopenable =  true;
    startenable = false;    
  }
  else if (cmd == 0 && stopenable && (currentMillis -  previousStart >= delayToStop)) {
    previousStop = currentMillis;
    digitalWrite(relay, LOW);
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
    Serial.println("OFF Relay");
    stopenable = false;
    startenable = true;
  }
}

String makePayload() {
      String payload = "{";
      payload += "\"temperature\":"; payload += t; payload += ",";
      payload += "\"humidity\":"; payload += h; payload += ",";
      payload += "\"setpoint\":"; payload += temperature_setpoint; payload += ",";
      payload += "\"relay\":"; payload += digitalRead(RELAY_PIN);
      payload += "}";

      return payload;
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  delay(2000);
  InitWiFi();
  client.setServer( thingsboardServer, 1883 );
  client.setCallback(on_message);
  
  //Initialize File System
  //bool formatted = SPIFFS.format();   // format spiffs at the fitst time
  if(SPIFFS.begin()){
    Serial.println("SPIFFS Initialize....ok");
  }
  else{
    Serial.println("SPIFFS Initialization...failed");
  }
  
  Serial.println("Getting configuration");
  temperature_setpoint = getSPIFF("/temperature.txt", default_settemp); 
    // Start up the library
  sensors.begin();
  
  delay(2000);
}

void loop() {
   unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
//    h=55;      t=10;
      sensors.requestTemperatures(); // Send the command to get temperatures
      t = sensors.getTempCByIndex(0); 

     if (isnan(t)) {
        Serial.println(F("Failed to read from DHT sensor!"));
      }
      else {
        char attributes[100];
        String payload = makePayload();
        payload.toCharArray( attributes, 100 );
        client.publish( "v1/devices/me/telemetry", attributes );
        //Serial.println( attributes );Serial.print("Setpoint = ");Serial.println(temperature_setpoint);
        if (t > temperature_setpoint) {
          relayAction(RELAY_PIN, 1);
        }
        else {
          relayAction(RELAY_PIN, 0);        
        }
      }
    }
  
    if ( !client.connected() ) {
      reconnect();
    }
  
    client.loop();
}

// The callback for when a PUBLISH message is received from the server.
void on_message(const char* topic, byte* payload, unsigned int length) {
  //Serial.println("On message");
  char json[length + 1];
  strncpy (json, (char*)payload, length);
  json[length] = '\0';

  Serial.print("Topic: ");Serial.println(topic);
  Serial.print("Message: ");Serial.println(json);
    
  // Decode JSON request
  //StaticJsonBuffer<200> jsonBuffer;
  //JsonObject& data = jsonBuffer.parseObject((char*)json);
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc,json);
  
  if (error){
    Serial.print("parseObject() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // Check request method
  String methodName = String((const char*)doc["method"]);
  String paramsName = String((const char*)doc["params"]);
  responseTopic = String(topic);
  responseTopic.replace("request", "response");
  
  char attributes[255];
  
  if (methodName.equals("setRelay")) {
    if (paramsName.equals("false")) {
      relayAction(RELAY_PIN,0);
    }
    else if (paramsName.equals("true")){
      relayAction(RELAY_PIN,1);
    }
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
  }
  else if (methodName.equals("getRelay")) {    
    client.publish(responseTopic.c_str(), get_gpio_status().c_str());
  }
  else if (methodName.equals("setValue")) {    
    temperature_setpoint = paramsName.toFloat();
    client.publish(responseTopic.c_str(), String(temperature_setpoint).c_str()); 
     // write to spiffs
    char buf[100];
    paramsName.toCharArray(buf,paramsName.length() + 1);
    writeFile("/temperature.txt", buf);
  }
  else if (methodName.equals("getValue")) {
    client.publish(responseTopic.c_str(), String(temperature_setpoint).c_str());
  }
}

String get_gpio_status() {  
  //DeserializationError error = deserializeJson(doc,json);
  //JsonObject& data = doc.createObject();
  //data[String(RELAY_PIN)] = digitalRead(RELAY_PIN) ? true : false;
  //char payload[256];
  //data.printTo(payload, sizeof(payload));
  //String strPayload = String(payload);
  StaticJsonDocument<200> doc;
  String strPayload;
  
  doc[String(RELAY_PIN)] = digitalRead(RELAY_PIN) ? true : false;
  serializeJson(doc, strPayload);
  Serial.print("Relay status: ");Serial.println(strPayload);
  
  return strPayload;
}

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_AP, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  while (!client.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_AP, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to ThingsBoard node ...");

    if ( client.connect("ESP8266 Device", TOKEN, NULL) ) {
      client.subscribe("v1/devices/me/rpc/request/+");
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      delay( 5000 );
    }
  }
}

// SPIFFS ---------------------------------------------
float readFile(const char * path){
    String line;
    Serial.printf("Reading file: %s\r\n", path);

    File file = SPIFFS.open(path,"r");
    if(!file || file.isDirectory()){
        Serial.println("- failed to open file for reading");
        return 0;
    }

    while(file.available()) {
      line = file.readStringUntil('\n');
    }
    
    file.close();
    
    return line.toFloat();
}

void writeFile( const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = SPIFFS.open(path, "w");
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.println(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- frite failed");
    }
    file.close();
}

void deleteFile(const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(SPIFFS.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

float getSPIFF(char* file, char* val) {
  float d = readFile(file); 
  if ( d == 0) {
    Serial.print("data init = ");
    Serial.println(val);
    writeFile(file, val);
    return 0;
  }
  else {
    return d;
  }
}
