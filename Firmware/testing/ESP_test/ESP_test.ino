#include <WiFi.h>
#include <WebServer.h>
#include <string>
#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define VR_PIN 34
#define THERMISTOR_PIN 35
#define HEAT_PIN 25
#define RESET_PIN 26
#define RELAY_PIN 12
#define UPDATE_TIME 1500
#define BUFFER_LEN 64
#define MAX_TEMP 350
#define MIN_TEMP 50


const char * ssid = "MakerSpace_2.4G";
const char * password = "ntueesaad";

LiquidCrystal_I2C lcd(0x27, 20, 4);
uint32_t last_Time, curr_Time = 0;

// PID control
double input_Temp, output_Time, target_Temp, VR_target_Temp, VR_pre_target_Temp;
double Kp = 40, Ki = 0, Kd = 3;
int WindowSize = 1000;
PID myPID(&input_Temp, &output_Time, &target_Temp, Kp, Ki, Kd, DIRECT);
uint32_t window_Start_Time = 0;

float avg_Buffer[BUFFER_LEN];
int avg_ptr;
bool active = false;

// handle server
WebServer server(80);
void handle_OnConnect() {
  server.send(200, "text/html", SendHTML()); 
}
void handleP5() {
  if (target_Temp <= MAX_TEMP - 5) target_Temp += 5;
  server.send(200, "text/html", SendHTML()); 
}
void handleM5() {
  if (target_Temp >= MIN_TEMP + 5) target_Temp -= 5;
  server.send(200, "text/html", SendHTML()); 
}
void handleP10() {
  if (target_Temp <= MAX_TEMP - 10) target_Temp += 10;
  server.send(200, "text/html", SendHTML()); 
}
void handleM10() {
  if (target_Temp >= MIN_TEMP + 10) target_Temp -= 10;
  server.send(200, "text/html", SendHTML()); 
}
void handleP50() {
  if (target_Temp <= MAX_TEMP - 50) target_Temp += 50;
  server.send(200, "text/html", SendHTML()); 
}
void handleM50() {
  if (target_Temp >= MIN_TEMP + 50) target_Temp -= 50;
  server.send(200, "text/html", SendHTML()); 
}
void handleHeat() {
  // TODO
  active = true;
  input_Temp = MIN_TEMP;
  server.send(200, "text/html", SendHTML()); 
}
void handleReset() {
  target_Temp = -1.0;
  active = false;
  server.send(200, "text/html", SendHTML()); 
}
void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}
String SendHTML(){
  String ptr ="<!DOCTYPE html>\n";
  ptr +="<html>\n";
  ptr +="<head>\n";
  ptr +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\" charset=\"utf-8\">\n";
  ptr +="<title>Heater Control</title>\n";
  ptr +="<style>\n";
  ptr +="html, body {position:fixed; top:0; bottom:0; left:0; right:0;}\n";
  ptr +="html{font-family: Helvetica; text-align: center;}\n";
  ptr +="body{justify-content: center; align-items: center;} \n";
  ptr +="h1 {color: #444444;margin: 20px auto 10px;} \n";
  ptr +="h3 {color: #444444;margin-bottom: 0px;}\n";
  ptr +=".button {display: inline-block;width: 80px;background-color: #c0899b;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button2 {display: inline-block;width: 60px;background-color: #a35664;border: none;color: white;padding: 20px 20px;text-decoration: none;font-size: 25px;margin: 0px 20px 5px;cursor: pointer;border-radius: 60px;}\n";
  ptr +=".button:hover {background-color: #9a6d7c}\n";
  ptr +=".button2:hover {background-color: #864753}\n";

  ptr +="p {font-size: 14px;color: #888;margin-bottom: 10px;}\n";
  ptr +="#gauge { position: relative; width: 233px; height: 233px; background: #fff; border-radius: 10px; display: flex; justify-content: center; align-items: center;}\n";
  ptr +="#bottom-circle { position: absolute; width: 90%; height: 90%; background: linear-gradient(#efefef, #ffffff); border-radius: 50%; box-shadow: inset 0 5px 5px #e7e7e7;}\n";
  ptr +="svg { position: absolute; }\n";
  ptr +="svg path { transition: 1s; }\n";
  ptr +="#center-circle { position: absolute; width: 170px; height: 170px; background: linear-gradient(180deg, #ffffff 0%, #e7ecf1 100%); border-radius: 50%; display: flex; justify-content: center; align-items: center; box-shadow: 0px 10px 10px rgba(0,0,0,0.1);}\n";
  ptr +="#center-circle::before { content: ''; position: absolute; width: 145px; height: 145px; background: linear-gradient(0deg, #ffffff 0%, #e7ecf1 100%); border-radius: 50%;}\n";
  ptr +="#name { position: absolute; font-size: 1em; color: #7f7f7f; font-weight: 700; top: 40px;}\n";
  ptr +="#center-circle img { position: absolute; width: 30px; height: 30px; bottom: 20px;}\n";
  ptr +="#temperature { position: absolute; font-size: 3em; color: #afafaf;}\n";
  ptr +="#range { position: absolute; width: 80%; bottom: 10px;}\n";
  ptr +=".gauge-wrapper { display: flex; justify-content: center;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>Heater Bed Web Server</h1>\n";
  // ptr +="<h3>Using Station(STA) Mode</h3>\n";

  ptr +="<div class=\"gauge-wrapper\">\n";
  ptr +="<div id=\"gauge\">\n";
  ptr +="<div id=\"minor-ticks-bottom-mask\"></div>\n";
  ptr +="<div id=\"bottom-circle\"></div>\n";
  ptr +="<svg version=\"1.1\" baseProfile=\"full\" width=\"190\" height=\"190\" xmlns=\"http://www.w3.org/2000/svg\">\n";
  ptr +="<linearGradient id=\"gradient\" x1=\"0\" x2=\"1\" y1=\"0\" y2=\"0\">\n";
  ptr +="<stop offset=\"0%\" stop-color=\"#b96e85\"/>\n";
  ptr +="<stop offset=\"100%\" stop-color=\"#ae69bb\"/>\n";
  ptr +="</linearGradient>\n";
  ptr +="<path d=\"M5 95 A80 80 0 0 1 185 95\" stroke=url(#gradient) fill=\"none\" stroke-width=\"10\"\n";
  
  ptr +="<stroke-linecap=\"round\" stroke-dasharray=\"";
  ptr +=String(input_Temp * 360 / 350);
  ptr +=" 282.78\"/>\n";
  
  ptr +="</svg>\n";
  ptr +="<div id=\"center-circle\">\n";
  ptr +="<span id=\"name\" style=\"padding-top: 10px\">Current Temp:</span>\n";
  ptr +="<span id=\"temperature\" style=\"padding-top: 20px\">";
  ptr +=String(input_Temp);
  ptr +="</span>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  ptr +="</div>\n";
  ptr +="<h3>Target temperature: ";
  ptr +=String(target_Temp);
  ptr +="</h3>";

  ptr +="<br>\n";
  ptr +="<br>\n";
  ptr +="<div>\n";
  ptr +="<a class=\"button\" href=\"/-50\">-50</a>\n";
  ptr +="<a class=\"button\" href=\"/-10\">-10</a>\n";
  ptr +="<a class=\"button\" href=\"/-5\">-5</a>\n";
  ptr +="<a class=\"button\" href=\"/+5\">+5</a>\n";
  ptr +="<a class=\"button\" href=\"/+10\">+10</a>\n";
  ptr +="<a class=\"button\" href=\"/+50\">+50</a>\n";
  ptr +="</div>\n";
  ptr +="<div>\n";
  ptr +="<a class=\"button2\" href=\"/heat\">Heat</a>\n";
  ptr +="<a class=\"button2\" href=\"/reset\">Reset</a>\n";
  ptr +="</div>\n";
  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}

// handle PID
void startPID() {
  input_Temp = readTemp();
  curr_Time = millis();
  if (curr_Time - last_Time >= UPDATE_TIME){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Current Temperature:");
    lcd.setCursor(7, 1);
    lcd.print(input_Temp);
    lcd.setCursor(0, 2);
    lcd.print("Target Temperature:");
    lcd.setCursor(7, 3);
    lcd.print(target_Temp);
    last_Time = curr_Time;
  }

  // data averaging
  avg_Buffer[avg_ptr ++] = input_Temp;
  if (avg_ptr >= BUFFER_LEN) avg_ptr = 0;
  
  float avg_Temp = 0;
  for (int i = 0; i < BUFFER_LEN; i++) avg_Temp += avg_Buffer[i];
  avg_Temp /= BUFFER_LEN;

  myPID.Compute();

  if (millis() - window_Start_Time > WindowSize) window_Start_Time += WindowSize; // time to shift the Relay Window
  
  if (output_Time < millis() - window_Start_Time) { digitalWrite(RELAY_PIN, LOW); }
  else { digitalWrite(RELAY_PIN, HIGH); }
}
void stopPID() {
  digitalWrite(RELAY_PIN, LOW);
  input_Temp = readTemp();

  curr_Time = millis();
  if (curr_Time - last_Time >= UPDATE_TIME){
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("(Not activate)");
    lcd.setCursor(0, 2);
    lcd.print("Current Temperature:");
    lcd.setCursor(7, 3);
    lcd.print(input_Temp);
    last_Time = curr_Time;
  }
}

// read temperature
float readTemp() {
  // Variables for Temp reading
  int Vo;
  float log_R, c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  
  Vo = analogRead(THERMISTOR_PIN);
  log_R = log(10000 * (4095.0 / (float)Vo - 1.0));
  return ((1.0 / (c1 + c2 * log_R + c3 * log_R * log_R * log_R)) - 273.15);
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // LCD
  lcd.init();
  lcd.backlight();

  // Pins
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(HEAT_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  digitalWrite(RELAY_PIN, LOW);
  
  // Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP: ");  Serial.println(WiFi.localIP());

  // Server handling
  server.on("/", handle_OnConnect);
  server.on("/+5", handleP5);
  server.on("/-5", handleM5);
  server.on("/+10", handleP10);
  server.on("/-10", handleM10);
  server.on("/+50", handleP50);
  server.on("/-50", handleM50);
  server.on("/heat", handleHeat);
  server.on("/reset", handleReset);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  // PID
  target_Temp = -1.0; // initialize the variables we're linked to
  myPID.SetOutputLimits(0, WindowSize); // tell the PID to range between 0 and the full window size
  myPID.SetMode(AUTOMATIC);
  window_Start_Time = millis();
}
void loop() {
  server.handleClient();
  VR_target_Temp = map(analogRead(VR_PIN), 0, 4095, 50, 350);
  if (active) {
    if (VR_pre_target_Temp != VR_target_Temp) {
      target_Temp = VR_target_Temp;
      VR_pre_target_Temp = VR_target_Temp;
    }
    Serial.println(VR_target_Temp);
    startPID();
  } else {
    stopPID();
  }
  if (digitalRead(HEAT_PIN) == LOW) handleHeat();
  if (digitalRead(RESET_PIN) == LOW) handleReset();  
}
