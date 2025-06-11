#include <HardwareSerial.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <Wire.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <WebSocketsClient.h>
WebSocketsClient webSocket; 
const char* ssid = "AndroidAP5916";  //replace
const char* password =  "caaf59io67"; //replace
unsigned long messageInterval = 1000;
bool connected = false;
bool autopilot = false;

float p = 0;
float t = 0;
float h = 0;
AsyncWebServer server(80);

HardwareSerial SerialPort(2); // use UART2
char cmd = 's';
// Motor A
int motor1Pin1 = 26; 
int motor1Pin2 = 27; 
// Motor A
int motor2Pin1 = 25; 
int motor2Pin2 = 33; 
Adafruit_BMP085 bmp;
//Pines 21 y 22

#define DHTPIN 18


#define DHTTYPE DHT11 

DHT dht(DHTPIN, DHTTYPE);
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            connected = false;
            break;
        case WStype_CONNECTED: {
            connected = true;
            webSocket.sendTXT("Connected");
        }
            break;
        case WStype_TEXT:
            break;
        case WStype_BIN:
            break;
                case WStype_PING:
                        // pong will be send automatically
                      
                        break;
                case WStype_PONG:
                        // answer to a ping we send
                        break;
    }
 
}

void forward(){
  cmd = 'f';
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 
}
void backward(){
  cmd = 'b';
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW); 
}
void stop(){
  cmd = 's';
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 
}
void right(){
  cmd = 'r';
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW); 
}
void left(){
  cmd = 'l';
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH); 
}

void setup() {
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  SerialPort.begin(115200, SERIAL_8N1, 16, 17); 
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }

  bmp.begin();
  dht.begin();
  webSocket.begin("192.168.48.15", 1880, "/ws/rover");
  webSocket.onEvent(webSocketEvent);
  stop();
  SerialPort.print(cmd);

  server.on("/forward", HTTP_GET   , [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ok");
    forward();
    SerialPort.print(cmd);
  });
  server.on("/backward", HTTP_GET   , [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ok");
    backward();
    SerialPort.print(cmd);
  });
  server.on("/left", HTTP_GET   , [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ok");
    left();
    SerialPort.print(cmd);
  });
  server.on("/right", HTTP_GET   , [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ok");
    right();
    SerialPort.print(cmd);
  });
  server.on("/stop", HTTP_GET   , [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ok");
    stop();
    SerialPort.print(cmd);
  });
  server.on("/auto", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain","Auto");
    autopilot = true;
    cmd = 'A';
    SerialPort.print(cmd);
  });

  server.on("/normal", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain","Normal");
    autopilot = false;
    cmd = 'N';
    SerialPort.print(cmd);
  });
  
  
  server.begin();
}
unsigned long lastUpdate = millis();
void loop() {
    webSocket.loop();
    if (connected && lastUpdate+messageInterval<millis()){
      p = bmp.readPressure();
      t = bmp.readTemperature();
      //h = dht.getHumidity();
      delay(1000);
      String jsonString = "{";
      jsonString += "\"p\":";
      jsonString += String(p);
      jsonString += ",";
      jsonString += "\"t\":";
      jsonString += String(t);
      jsonString += ",";
      jsonString += "\"h\":";
      jsonString += String(h);
      jsonString += "}";
      webSocket.sendTXT(jsonString);
      lastUpdate = millis();
    }
}
