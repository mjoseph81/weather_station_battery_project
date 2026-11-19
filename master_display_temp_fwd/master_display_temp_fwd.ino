#include <WiFi.h>
#include <PicoMQTT.h>
#include <SPI.h>             // SPI library for comm to LCD interface
#include <TFT_eSPI.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Update.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>


//Pins for LCD
#define TFT_CS      5
#define TFT_RST     4 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC      17
#define TFT_DIN     23
#define TFT_CLK     18

 
const String DISPLAY_FW_VER = "v1.0.8";
String SENSOR_FW_VER = "v0.0.0";

RTC_DATA_ATTR float rtcMinTemp = 70;
RTC_DATA_ATTR float rtcMaxTemp = -40;

Preferences prefs;

bool minMaxStored = false;

const char* ssid = "ESP32-MyAP";
const char* password = "password123!";

//const char* home_ssid = "Ragnarok";
const char* home_ssid = "Estes_Park";
const char* home_pass = "skywalker1138#";

const int x_offset = 20;
const int y_offset = 80;

// Set your desired IP addresses
IPAddress local_IP(10, 0, 0, 1);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);

bool updateLcd = false; 
float tempC = 0;
float tempF = 0;
float battVolts = 0;
unsigned long timeOfLastUpdate = 0;
const unsigned long commLossTime = 610 * 1000;
bool commLoss = true;
bool firstDataRx = false;

//create instance of MQTT server
PicoMQTT::Server mqtt;

//create instance of LCD
TFT_eSPI tft = TFT_eSPI();       

const int TEMP_UNIT_PIN = 12;
const int FW_DISP_PIN = 27;
bool deg_units = 1;
bool deg_units_last = 1;

//Create instance of the HTTP server
AsyncWebServer server(80);

bool ota_pending = false;
bool ota_reboot_flag = false;
unsigned long ota_reboot_time = 0;

const int SCREEN_BTN_PIN = 32;
const int TEMP_RESET_BTN_PIN = 33;

bool lastButtonState = HIGH; // For edge detection
bool updateScreen = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 500;

bool lastResetButtonState = HIGH; // For edge detection
unsigned long lastResetDebounceTime = 0;
bool pendingReset = false;

bool lastFwButtonState = HIGH; // For edge detection
unsigned long lastFwDebounceTime = 0;
bool showVersion = false;   // Our "State" variable

int screenStateCurr = 0;
const int screenStateMain = 0;
const int screenStateMax  = 1;
const int screenStateMin  = 2;
const int screenStateROC  = 3;
const int totalScreens = 4; // Total number of screens you have

float rocHistory[6];        // Stores last 6 readings (30 mins)
int rocCount = 0;           // Track how many readings we have
float currentROC = 0.0;     // The calculated degrees per hour

int configScreenStateCurr = -1;
const int configScreenStateFW = 0;
const int configScreenStateWIFI = 1; 
const int configScreenStateMain = 2;
const int configTotalScreens = 3;

// Outbound Client targeting the ESP32-S3
WiFiClient espClient;
PubSubClient outboundMqtt(espClient);

//const char* masterS3_IP = "10.117.3.76"; // IP address of your ESP32-S3
const char* masterS3_IP = "192.168.86.45"; // IP address of your ESP32-S3
const int masterS3_Port = 1883;
const char* ds1820_fwd_temp_topic = "weather/master/ds1820_temp";

// Non-blocking MQTT bridge reconnect tracking
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL_MS = 5000; // Wait 5 seconds between reconnect attempts

void setup() {
  Serial.begin(115200);

  pinMode(TEMP_UNIT_PIN, INPUT_PULLUP);
  pinMode(SCREEN_BTN_PIN, INPUT_PULLUP);
  pinMode(TEMP_RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(FW_DISP_PIN, INPUT_PULLUP);
  

  Serial.println("Display FW version: " + DISPLAY_FW_VER);

  //get saved data from flash
  loadMinMax();
  loadROC();

  if (!LittleFS.begin(true)) { // Adding 'true' allows auto-formatting
    Serial.println("LittleFS Mount Failed and could not be formatted");
    return;
  }

   
  WiFi.mode(WIFI_AP_STA);

  // MAC Connection Event
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.print("New Client Physical Connection! MAC: ");
    for (int i = 0; i < 6; i++) {
      Serial.printf("%02X%s", info.wifi_ap_staconnected.mac[i], (i < 5) ? ":" : "");
    }
    Serial.println();
  }, ARDUINO_EVENT_WIFI_AP_STACONNECTED);

  // IP Assignment Event 
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.print("IP Assigned to Client: ");
    Serial.println(IPAddress(info.wifi_ap_staipassigned.ip.addr)); // FIXED: .addr
  }, ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED);

  
  // Setup the Private Network for the Sensor
  if (WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("SoftAP Config Success");
  }
  
  WiFi.softAP(ssid, password); // This is the "Sensor" network
  Serial.print("Master's Private IP (for Sensor): ");
  Serial.println(WiFi.softAPIP());
  
  // Setup the Home Connection for your Dashboard
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE); // Helps some routers see the name
  WiFi.setHostname("weatherstation");
  WiFi.begin(home_ssid, home_pass); 
  Serial.print("Connecting to Home WiFi...");
  
  // Note: Don't use a blocking 'while' loop if you want the AP to start immediately.
  // But for setup, it's usually okay to wait a few seconds.
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter < 20) {
    delay(500);
    Serial.print(".");
    counter++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Home WiFi!");
    Serial.print("Dashboard IP (Use this on your phone): ");
    Serial.println(WiFi.localIP()); 

    if (!MDNS.begin("weatherstation")) { // Set your desired hostname here
      Serial.println("Error setting up MDNS responder!");
    } else {
      Serial.println("mDNS responder started: http://weatherstation.local");
    }
  } else {
    Serial.println("\nHome WiFi connection failed. Working in AP mode only.");
  }

  
  //MQTT subscription and callback for message received
  mqtt.subscribe("esp32/#", [](const char * topic, const char * payload) {
    Serial.printf("Broker received on '%s': %s\n", topic, payload);

    //set the flag when first data message is received
    if(firstDataRx == false){
      firstDataRx = true;
    }
    
    if(payload) {

      //if OTA update pending, send nofication to sensor
      if (ota_pending) {
          mqtt.publish("sensor/ota", "START");
          // Important: Clear the flag after sending so we don't spam the sensor
          ota_pending = false; 
          Serial.println("Handshake complete: START sent to sensor.");
      }

      
      // Create a buffer for the JSON object
      // 200 bytes is plenty for this small payload
      StaticJsonDocument<200> doc;

      // Deserialize (parse) the JSON
      DeserializationError error = deserializeJson(doc, payload);

      // Check if parsing succeeded
      if (error) {
          Serial.print("deserializeJson() failed: ");
          Serial.println(error.f_str());
          return;
      }

      // Extract values by their keys ("t" and "v")
      // We use the keys we defined in your sending code
      if (doc.containsKey("t")) {
          tempC = doc["t"]; // Automatically converts to float

          saveROC(tempC);

          // FORWARDING: Send the DS18B20 temperature to the WS-MN-S3-R1 Master Broker
          forwardTempToMaster(tempC);
      }
      
      // Extract the voltage if you have a variable for it
      if (doc.containsKey("v")) {
          battVolts = doc["v"];
          Serial.printf("Battery Voltage received: %.2fV\n", battVolts);
      }

      // Extract the sensor FW version
      if (doc.containsKey("fw")) {
          SENSOR_FW_VER = doc["fw"].as<String>(); 
          Serial.printf("Sensor Version: %s\n", SENSOR_FW_VER.c_str());
      }

      updateLcd = true;
      timeOfLastUpdate = millis();
    }
  });

  //Start MQTT broker
  mqtt.begin();
  Serial.println("PicoMQTT Broker started!");

  //Create HTTP handles for GET/POST requests
  createHttpHandles();

  //Create HTTP server
  server.begin();

  //Initialize the LCD
  lcdInit();

  // Configure outbound MQTT client target (WS-MN-S3-R1)
  outboundMqtt.setServer(masterS3_IP, masterS3_Port);
  Serial.println("Outbound MQTT client initialized for ESP32-S3 bridging.");
    
}

void loop() {
  mqtt.loop(); // Required to process client connections and messages

  //Read display units button
  deg_units = digitalRead(TEMP_UNIT_PIN);
  if(deg_units != deg_units_last){
    selectScreen();
    
    deg_units_last = deg_units;
  }

  //If LCD update flag is set then update the LCD
  if (updateLcd) {
    updateTemp(tempC);
    updateLcd = false; // Reset the flag
  }

  //If OTA reboot flag is set then trigger a reboot.
  if (ota_reboot_flag && (millis() - ota_reboot_time > 2000)) {
    ESP.restart();
  }

  //Read the screen button
  bool screenBtn = digitalRead(SCREEN_BTN_PIN);

  // Detect the moment the button is pressed (Falling Edge)
  if (screenBtn == LOW && lastButtonState == HIGH) {
      
      if ((millis() - lastDebounceTime) > debounceDelay) {
        // Increment state and wrap around using the modulo operator (%)
        screenStateCurr = (screenStateCurr + 1) % totalScreens;
        
        updateScreen = true; 
        lastDebounceTime = millis();
        Serial.printf("Screen changed to state: %d\n", screenStateCurr);

        //clear pending reset flag
        pendingReset = false;
    }
  }
  
  lastButtonState = screenBtn;

  // Decide what to display
  if (updateScreen) {
    selectScreen();
    updateScreen = false;
  }


  //logic for temp reset button
  //Read the screen button
  bool resetBtn = digitalRead(TEMP_RESET_BTN_PIN);
  
  // Detect the moment the button is pressed (Falling Edge)
  if (resetBtn == LOW && lastResetButtonState == HIGH) {
    
    if ((millis() - lastResetDebounceTime) > debounceDelay) {
      // reset pressed, display warning
      //Only trigger if on MAX or MIN screen
      if(screenStateCurr == screenStateMax || screenStateCurr == screenStateMin || pendingReset ==true){
        if(pendingReset == true){
          //reset stored MIN/MAX temps to default values.
          rtcMinTemp = 70;
          rtcMaxTemp = -40;
          minMaxStored = false;
          saveMinMax();
          clearROCHistory(tempC);
          selectScreen();
        }else{
          displayResetWarning();
          pendingReset = true;
        }
        
        lastResetDebounceTime = millis();
        Serial.printf("Screen changed to Reset Warning screen");
      }
    }
  }


  //logic for FW display button
  //Read the screen button
  bool fwSreenBtn = digitalRead(FW_DISP_PIN);

  // Detect the moment the button is pressed (Falling Edge)
  if (fwSreenBtn == LOW && lastFwButtonState == HIGH) {
      
      if ((millis() - lastFwDebounceTime) > debounceDelay) {
         configScreenStateCurr = (configScreenStateCurr + 1) % configTotalScreens;
        //FW button pressed, display FW sreen
        selectConfigScreen();
        /*
        if(showVersion==false){
          
        }else{
          selectScreen();
        }
        */
        
        lastFwDebounceTime = millis();
        Serial.printf("Screen changed to Firmware version display");

    }
  }
  
  lastButtonState = screenBtn;

  //check if sensor comm loss timer has expired 
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 2000) {
    lastCheck = millis();

    if(millis() - timeOfLastUpdate > commLossTime ){
      if(commLoss == false && firstDataRx == true){
        commLoss = true;
        displayCommLoss();
      }
    }else{
      if(commLoss == true && firstDataRx == true){
        commLoss = false;
        Serial.println("Comm with sensor reestablished");
      }
    }
  }

  // Keep outbound client connection alive if connected
  if (outboundMqtt.connected()) {
    outboundMqtt.loop();
  } else {
    // Non-blocking reconnect handler
    unsigned long now = millis();
    if (now - lastReconnectAttempt > RECONNECT_INTERVAL_MS) {
      lastReconnectAttempt = now;
      connectToMasterBroker();
    }
  }
}

bool connectToMasterBroker() {
  Serial.print("[BRIDGE] Attempting connection to Master S3 Broker at ");
  Serial.println(masterS3_IP);

  // Unique client identifier for the ESP32-S3 master broker
  const char* clientId = "WS-MN-D1-Bridge";

  if (outboundMqtt.connect(clientId)) {
    Serial.println("[BRIDGE] SUCCESS: Connected to Master ESP32-S3 Broker!");
    return true;
  } else {
    Serial.printf("[BRIDGE] ERROR: Connection failed, rc=%d. Retrying in 5 seconds...\n", outboundMqtt.state());
    return false;
  }
}


void forwardTempToMaster(float tempInC) {
  // 1. Verify Home Wi-Fi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[BRIDGE] Skip forward: Home Wi-Fi disconnected.");
    return;
  }

  // 2. Connect to WS-MN-S3-R1 broker if not currently connected
  if (!outboundMqtt.connected()) {
    Serial.print("[BRIDGE] Connecting to WS-MN-S3-R1 Master at ");
    Serial.println(masterS3_IP);
    
    // Attempt connection with a unique Client ID
    if (outboundMqtt.connect("WS-MN-D1-Bridge")) {
      Serial.println("[BRIDGE] Connected to Master ESP32-S3 Broker!");
    } else {
      Serial.printf("[BRIDGE] Connection failed, rc=%d\n", outboundMqtt.state());
      return; // Could not connect, exit
    }
  }

  // 3. Convert value and publish
  char payload[8];
  dtostrf(tempInC, 4, 2, payload); // Formats float (e.g., "13.88")

  bool success = outboundMqtt.publish(ds1820_fwd_temp_topic, payload);
  if (success) {
    Serial.printf("[BRIDGE] Successfully forwarded temp (%.2f °C) to S3 Master!\n", tempInC);
  } else {
    Serial.println("[BRIDGE] Publish to S3 Master failed.");
  }
}


//Function to determine which screen to display based on screenStateCurr
void selectScreen(void){
  tft.fillScreen(inv(TFT_BLACK)); // Wipe once before drawing new state

  switch (screenStateCurr) {
      case screenStateMain:
          if(commLoss) {
              displayCommLoss();
          } else {
              clearCommLoss(); // Your main temp screen
          }
          break;

      case screenStateMin:
          displayMinTemp(); // Show the RTC Min Temp
          break;

      case screenStateMax:
          displayMaxTemp(); // Show the RTC Max Temp
          break;

      case screenStateROC: // Show the ROC screen
          displayTrend();
          break;

  }
}


//Function to determine which config screen to display based
void selectConfigScreen(void){
  tft.fillScreen(inv(TFT_BLACK)); // Wipe once before drawing new state

  switch (configScreenStateCurr) {
      case configScreenStateFW:
          displayFwVersion();  //show FW screen
          break;

      case configScreenStateWIFI:
          displayWifiInfo(); // Show the WIFI screen
          break;

      case configScreenStateMain:
          selectScreen(); // Show the main screen
          break;
          

  }
}

//Function to initialize the TFT display
void lcdInit(void){
  // use this initializer if using a 2.0" 320x240 TFT:
  tft.init();           // Init ST7789 320x240
  tft.setRotation(3);           // set rotation horizontal, connector on left side
  
  tft.writecommand(TFT_MADCTL); 
  tft.writedata(TFT_MAD_MX | TFT_MAD_MV | TFT_MAD_COLOR_ORDER); // Flips X-axis / Mirrors back
  
  // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
  // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
  // may end up with a black screen some times, or all the time.
  //tft.setSPISpeed(40000000);
  Serial.println("Display Initializing");

  delay(500);

  
  tft.fillScreen(inv(TFT_BLACK));
  tft.setTextSize(1);
  tft.setTextColor(inv(TFT_WHITE));
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.drawString("Initializing", 40, 100);
  
  delay(2000);

  Serial.println("Display Initialized");
  
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.fillScreen(inv(TFT_BLACK));
  tft.setTextSize(2);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextPadding(214);
  drawMsg("??.?", 0, 0);

  drawDegree(220, 20, TFT_WHITE);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextPadding(0);
  drawMsg("F", 230, 10);
  
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  drawMsg("B:??%", 215, 60);

}

//Function that updates the current Temp and Battery level from the sensor
void updateTemp(float tempV) {
  char charTemp[10];
  char charBatt[8];
  char battDis[10];
  int battPerc;

  
  Serial.print("Temp C: ");
  Serial.println(tempV);

  //convert to get temp in F
  tempF = (tempV * 1.8) + 32;

  //convert C and F temps to string for display
  if(deg_units){
    dtostrf(tempF, -5, 1, charTemp);
  }else{
    dtostrf(tempC, -5, 1, charTemp);
  }

  //check for new MIN temp and save if detected
  if(tempV < rtcMinTemp){
    rtcMinTemp = tempV;
    saveMinMax();
    minMaxStored = true;
    String minTempMsg = "New MIN temp: " + String(tempV, 1) + "C";
    Serial.println(minTempMsg);
  }

  //check for new MAX temp and save if detected
  if(tempV > rtcMaxTemp){
    rtcMaxTemp = tempV;
    saveMinMax();
    minMaxStored = true;
    String maxTempMsg = "New MAX temp: " + String(tempV, 1) + "C";
    Serial.println(maxTempMsg);
  }
  
  //display battery %
  battPerc = getBatteryPercentage(battVolts);
  snprintf(battDis, sizeof(battDis), "B:%d%%", battPerc);


  Serial.print("Temp F: ");
  Serial.println(tempF);

  Serial.print("Batt Volts: ");
  Serial.println(battVolts);

  //set display parameters for temp
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextSize(2);
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(214);
  
  // Safety: If the string is somehow empty, print "??.?" instead of nothing
  if (strlen(charTemp) == 0) {
      drawMsg("??.?", 0, 0);
  } else {
      drawMsg(charTemp, 0, 0);
  }

  //set display parameters for deg unit label
  drawDegree(220, 20, TFT_WHITE);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextPadding(80);
  if(deg_units){
    drawMsg("F", 230, 10);
  }else{
    drawMsg("C", 230, 10);
  }

  //set display for sensor battery %
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  drawMsg(battDis, 215, 60);
  
}


//Function that displays comm loss on display
void displayCommLoss(void) {
  //diplay ?? for temp and battery when in comm loss with sensor
  Serial.println("Comm loss with sensor.");

  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextSize(2);
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(80);
  drawMsg("??.?", 0, 0);

  drawDegree(220, 20, TFT_WHITE);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextPadding(80);
  if(deg_units){
    drawMsg("F", 230, 10);
  }else{
    drawMsg("C", 230, 10);
  }
  
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSans9pt7b);
  drawMsg("B:??%", 215, 60);
  
}


//Function that clear the comm loss on the display and displays current reading
void clearCommLoss(void){
  //display sensor values once comm loss has cleared

  char charTemp[10];
  char battDis[10];
  int battPerc = 0;
  
  tempF = (tempC * 1.8) + 32;

  if(deg_units){
    dtostrf(tempF, -5, 1, charTemp);
  }else{
    dtostrf(tempC, -5, 1, charTemp);
  }

  //display battery %
  battPerc = getBatteryPercentage(battVolts);
  snprintf(battDis, sizeof(battDis), "B:%d%%", battPerc);
  
  //set display parameters for temp
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextSize(2);
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(214);
  
  // Safety: If the string is somehow empty, print "??.?" instead of nothing
  if (strlen(charTemp) == 0) {
      drawMsg("??.?", 0, 0);
  } else {
      drawMsg(charTemp, 0, 0);
  }
  
  drawDegree(220, 20, TFT_WHITE);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextPadding(80);
  if(deg_units){
    drawMsg("F", 230, 10);
  }else{
    drawMsg("C", 230, 10);
  }
  
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextPadding(80);
  drawMsg(battDis, 215, 60);
}


//Function that displays FW versions on TFT display
void displayFwVersion(void) {
  Serial.println("Displaying FW version screen.");
  tft.fillScreen(inv(TFT_BLACK));
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(80);

  String display_fw_msg = "Display FW version: " + DISPLAY_FW_VER;
  drawMsg(display_fw_msg.c_str(), 20, 10);

  String sensor_fw_msg = "Sensor FW version: " + SENSOR_FW_VER;
  drawMsg(sensor_fw_msg.c_str(), 20, 50);
  
}


//Function that displays WIFI info
void displayWifiInfo(void) {
  Serial.println("Displaying WIFI info screen.");
  tft.fillScreen(inv(TFT_BLACK));
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(80);

  String ssid_msg = "SSID: " + String(home_ssid);
  drawMsg(ssid_msg.c_str(), 20, 10);

  String wifi_ip_msg = "WIFI IP: " + WiFi.localIP().toString();
  drawMsg(wifi_ip_msg.c_str(), 20, 50);
  
}

//Function that displays MIN Temp on TFT display
void displayMinTemp(void){
  //display MIN temp reading on screen
  char charTemp[10];
  float minTempC = rtcMinTemp;
  float minTempF = (minTempC * 1.8) + 32;

  if(deg_units){
    dtostrf(minTempF, -5, 1, charTemp);
  }else{
    dtostrf(minTempC, -5, 1, charTemp);
  }

  //set display parameters for temp
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextSize(2);
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(214);
  
  // Safety: If the string is somehow empty, print "??.?" instead of nothing
  if (strlen(charTemp) == 0 || (minTempC >= 70)) {
      drawMsg("??.?", 0, 0);
  } else {
      drawMsg(charTemp, 0, 0);
  }
  
  drawDegree(220, 20, TFT_WHITE);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextPadding(80);
  if(deg_units){
    drawMsg("F", 230, 10);
  }else{
    drawMsg("C", 230, 10);
  }
  
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextPadding(80);
  drawMsg("LOW", 215, 60);
}


//Function that displays MAX Temp on TFT display
void displayMaxTemp(void){
  //display MAX temp reading on screen
  char charTemp[10];
  float maxTempC = rtcMaxTemp;
  float maxTempF = (maxTempC * 1.8) + 32;

  if(deg_units){
    dtostrf(maxTempF, -5, 1, charTemp);
  }else{
    dtostrf(maxTempC, -5, 1, charTemp);
  }

  //set display parameters for temp
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextSize(2);
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(214);
  
  // Safety: If the string is somehow empty, print "??.?" instead of nothing
  if (strlen(charTemp) == 0 || (maxTempC <= -40)) {
      drawMsg("??.?", 0, 0);
  } else {
      drawMsg(charTemp, 0, 0);
  }
  
  drawDegree(220, 20, TFT_WHITE);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextPadding(80);
  if(deg_units){
    drawMsg("F", 230, 10);
  }else{
    drawMsg("C", 230, 10);
  }
  
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextPadding(80);
  drawMsg("HIGH", 215, 60);
}


//Function that displays reset warning and confirmation
void displayResetWarning(void) {
  //display reset warning on screen.
  Serial.println("Displaying temp reset warning screen.");
  tft.fillScreen(inv(TFT_BLACK));
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(80);

  String display_fw_msg = "Reset HIGH/LOW values?";
  drawMsg(display_fw_msg.c_str(), 5, 5);

  String confirm_msg = "<-- Confirm";
  drawMsg(confirm_msg.c_str(), 5, 32);

  String cancel_msg = "--> Cancel";
  drawMsg(cancel_msg.c_str(), 5, 60);
  
}


//diplay Trend for Rate Of Change (ROC)
void displayTrend(void) {
  tft.fillScreen(inv(TFT_BLACK));
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextSize(2); // Match the main temp size
  tft.setTextColor(inv(TFT_WHITE), inv(TFT_BLACK));
  tft.setTextPadding(214);

  char rocStr[10];
  float displayROC = currentROC;
  
  // Convert to Fahrenheit if that's the current setting
  if(deg_units) {
    displayROC = currentROC * 1.8; 
  }

  // Format with a + or - sign
  snprintf(rocStr, sizeof(rocStr), "%+.1f", displayROC);
  drawMsg(rocStr, 0, 0);

  // Labels
  drawDegree(220, 20, TFT_WHITE);
  tft.setTextSize(1);
  tft.setFreeFont(&FreeSansBold24pt7b);
  if(deg_units){
    drawMsg("F", 230, 10);
  }else{
    drawMsg("C", 230, 10);
  }

  tft.setFreeFont(&FreeSans9pt7b);
  drawMsg("per hour", 205, 60);
}



//Function taht display Display FW update progress
void displayFwUpdate(void) {
  //display status of FW update for the Display
  tft.fillScreen(inv(TFT_BLACK));
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(inv(TFT_WHITE));
  tft.setTextDatum(MC_DATUM); // Center middle
  drawMsg("Update In Prog...", tft.width()/2, 20);
  
  // Draw an empty progress bar frame
  tft.drawRect(20, 120, tft.width()-40, 20, inv(TFT_WHITE));
}


//Function that updates the progress bar during FW update
void updateProgressBar(int percent) {
  int barWidth = tft.width() - 44; // Subtracting padding/borders
  int fillWidth = (percent * barWidth) / 100;
  
  // Fill the progress bar
  tft.fillRect(22, 122, fillWidth, 16, inv(TFT_WHITE));

  /*
  // Show the number below
  tft.setFreeFont(&FreeSansBold12pt7b);
  tft.setTextPadding(100); // Prevents "ghosting" of old numbers
  tft.drawNumber(percent, tft.width()/2, 150);
  tft.drawString("%", (tft.width()/2) + 30, 150);
  */
}


//Function that calculates batter percentage from voltage
int getBatteryPercentage(float v) {
  // 6.2V = Brand New (1.55V/cell)
  // 5.2V = Nominal (1.3V/cell)
  // 4.4V = Near Empty (1.1V/cell)
  // 4.1V = Critical/Shutdown (1.0V/cell)
  
  static const float voltSteps[] =    {6.2, 6.0, 5.6, 5.2, 4.8, 4.5, 4.2, 4.0};
  static const int percentSteps[] =   {100, 95,  85,  70,  50,  30,  15,  0};
  //static const int percentSteps[] =  {100,  85,   60,   40,   25,   10,   3,    0};    //non-linear curve
  int numSteps = 8;

  if (v >= voltSteps[0]) return 100;
  if (v <= voltSteps[numSteps - 1]) return 0;

  for (int i = 0; i < numSteps - 1; i++) {
    if (v <= voltSteps[i] && v > voltSteps[i + 1]) {
      float factor = (v - voltSteps[i + 1]) / (voltSteps[i] - voltSteps[i + 1]);
      float result = percentSteps[i + 1] + (factor * (percentSteps[i] - percentSteps[i + 1]));
      return (int)(result + 0.5); // Proper rounding
    }
  }
  return 0;
}



// This function flips all bits in a 16-bit color
// The 76x284 TFT display inverts standars 16-bit colors so this handles that issue
uint16_t inv(uint16_t color) {
  return ~color;
}


// Custom Draw Function
// The 76x284 does't work correctly with eSPI library so it has to be treated as a 240x320 display
// This function handles "drawString" with proper offset (global vars) to work with the 76x284 display
void drawMsg(const char* msg, int x, int y) {
  tft.drawString(msg, x + x_offset, y + y_offset);
}


//customer function to draw the degree symbol
void drawDegree(int x, int y, uint16_t color) {
  int cx = x + x_offset;
  int cy = y + y_offset;
  
  tft.fillCircle(cx, cy, 8, inv(color));
  tft.fillCircle(cx, cy, 5, inv(TFT_BLACK));
}


// Function that creates and handles the HTTP routes (GET / POST) 
// 1. Admin Web Screen creation 
// 2. Sensor FW upload to display LittleFS
// 3. Sensor FW download (install triggered with MQTT message)
// 4. Display FW upload and installation
void createHttpHandles(void){
  // 1. The Admin Page (The Form)
  server.on("/admin", HTTP_GET, [&](AsyncWebServerRequest *request){
    if(!request->authenticate("matt", "thx1138")) 
      return request->requestAuthentication();

      String html = "<!DOCTYPE html><html><head>";
      html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
      html += "<style>";
      html += "body { font-family: 'Segoe UI', sans-serif; background: #0f172a; color: #f8fafc; text-align: center; padding: 20px; }";
      html += ".container { display: flex; flex-wrap: wrap; justify-content: center; gap: 15px; max-width: 900px; margin: 20px auto; }";
      html += ".card { background: #1e293b; border-radius: 12px; padding: 25px; border: 1px solid #334155; flex: 1 1 300px; max-width: 400px; text-align: left; }";
      html += "h4 { color: #94a3b8; text-transform: uppercase; letter-spacing: 1px; margin-bottom: 10px; font-size: 0.9rem; }";
      
      // Version Info Box Styling
      html += ".version-info { background: #0f172a; padding: 8px 12px; border-radius: 6px; font-family: monospace; color: #38bdf8; margin-bottom: 20px; font-size: 0.9rem; border: 1px solid #1e293b; }";
      
      html += ".progress-wrapper { background: #0f172a; border-radius: 8px; margin: 15px 0; display: none; overflow: hidden; border: 1px solid #334155; height: 14px; }";
      html += ".progress-bar { background: #3b82f6; height: 100%; width: 0%; transition: width 0.15s linear; }";
      html += ".m-bar { background: #8b5cf6; }"; 
      
      html += "input[type=file] { background: #334155; padding: 10px; border-radius: 6px; width: 100%; margin-bottom: 15px; color: white; }";
      html += ".btn-upload { background: #3b82f6; color: white; border: none; padding: 12px 20px; border-radius: 8px; font-weight: bold; cursor: pointer; width: 100%; }";
      html += ".btn-master { background: #8b5cf6; }";
      html += "</style></head><body>";
  
      html += "<h1>Firmware Hub</h1>";
      html += "<div class='container'>";
  
      
  
      // MASTER CARD
      html += "<div class='card' style='border-color: #8b5cf6;'><h4>Master Display Update</h4>";
      html += "<div class='version-info'>Active Version: " + DISPLAY_FW_VER + "</div>"; 
      html += "<input type='file' id='masterFile'>";
      html += "<div id='mWrapper' class='progress-wrapper'><div id='mBar' class='progress-bar m-bar'></div></div>";
      html += "<div id='mStatus' style='font-size:0.8rem; text-align:center;'></div>";
      html += "<button type='button' class='btn-upload btn-master' onclick='uploadFile(\"master\")'>Flash Master</button></div>";

      // SENSOR CARD
      html += "<div class='card'><h4>Sensor Node Update</h4>";
      html += "<div class='version-info'>Active Version: " + SENSOR_FW_VER + "</div>"; 
      html += "<input type='file' id='sensorFile'>";
      html += "<div id='sWrapper' class='progress-wrapper'><div id='sBar' class='progress-bar'></div></div>";
      html += "<div id='sStatus' style='font-size:0.8rem; text-align:center;'></div>";
      html += "<button type='button' class='btn-upload' onclick='uploadFile(\"sensor\")'>Upload to OTA Server</button></div>";
  
      html += "</div>";
  
      // JavaScript to handle the upload progress bar
      html += "<script>";
      html += "function uploadFile(type) {";
      html += "  var fileId = (type === 'sensor') ? 'sensorFile' : 'masterFile';";
      html += "  var url = (type === 'sensor') ? '/upload' : '/update-master';";
      html += "  var barId = (type === 'sensor') ? 'sBar' : 'mBar';";
      html += "  var wrapId = (type === 'sensor') ? 'sWrapper' : 'mWrapper';";
      html += "  var statId = (type === 'sensor') ? 'sStatus' : 'mStatus';";
      html += "  var file = document.getElementById(fileId).files[0];";
      html += "  if(!file) { alert('Select file!'); return; }";
      html += "  var xhr = new XMLHttpRequest();";
      html += "  var formData = new FormData();";
      html += "  formData.append((type === 'sensor' ? 'data' : 'update'), file);";
      html += "  xhr.upload.addEventListener('progress', function(e) {";
      html += "    if (e.lengthComputable) {";
      html += "      var pc = Math.round((e.loaded / e.total) * 100);";
      html += "      document.getElementById(wrapId).style.display = 'block';";
      html += "      document.getElementById(barId).style.width = pc + '%';";
      html += "      document.getElementById(statId).innerHTML = pc + '% Complete';";
      html += "    }";
      html += "  });";
      html += "  xhr.onreadystatechange = function() {";
      html += "    if (xhr.readyState == 4 && xhr.status == 200) {";
      html += "      document.body.innerHTML = xhr.responseText;";
      html += "    } else if (xhr.readyState == 4) { alert('Upload Failed'); }";
      html += "  };";
      html += "  xhr.open('POST', url, true);";
      html += "  xhr.send(formData);";
      html += "}";
      html += "</script>";
      html += "<a href='/' style='display:block; margin-top:20px; color:#94a3b8; text-decoration:none;'>&larr; Return to Dashboard</a></body></html>";
  
      request->send(200, "text/html", html);
  });
  
  // 2. The Upload Handler (Saves sensor files to LittleFS)
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){
    // 1. Construct a success page that redirects after 3 seconds
    String html = "<html><head>";
    html += "<meta http-equiv='refresh' content='3;url=/admin'>";
    html += "<style>body{background:#0f172a;color:#f8fafc;font-family:'Segoe UI',sans-serif;text-align:center;padding-top:100px;}";
    html += ".loader{border:4px solid #334155;border-top:4px solid #3b82f6;border-radius:50%;width:30px;height:30px;animation:spin 1s linear infinite;margin:20px auto;}";
    html += "@keyframes spin{0%{transform:rotate(0deg);}100%{transform:rotate(360deg);}}</style>";
    html += "</head><body>";
    html += "<h2>Update Prepared!</h2>";
    html += "<p>Sensor firmware saved to Master storage.</p>";
    html += "<div class='loader'></div>";
    html += "<p>Redirecting to Admin Hub...</p>";
    html += "</body></html>";

    request->send(200, "text/html", html);

  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    
    if(!index){
      // We force this name so the sensor's URL (/update) always works
      request->_tempFile = LittleFS.open("/sensor_firmware.bin", "w");
    }
    
    if(len && request->_tempFile){
      request->_tempFile.write(data, len);
    }
    
    if(final && request->_tempFile){
      request->_tempFile.close();
      ota_pending = true; // Signal the MQTT loop to tell the sensor to wake up
      Serial.println("Sensor firmware is ready in LittleFS.");
    }
  });


  // 3. Route for sensor to download binary (Matches filename you'll upload)
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/sensor_firmware.bin", "application/octet-stream");
    Serial.println("Sent /sensor_firmware.bin to sensor");
  });


  // 4. Route for Master Display update
  server.on("/update-master", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool updateFailed = Update.hasError();
    
    // Create a styled HTML response instead of plain text
    String html = "<html><head>";
    if (!updateFailed) {
      // Redirect to the Dashboard (/) after 10 seconds to allow for reboot
      html += "<meta http-equiv='refresh' content='10;url=/'>";
    }
    html += "<style>body{background:#0f172a;color:#f8fafc;font-family:'Segoe UI',sans-serif;text-align:center;padding-top:100px;}";
    html += ".status-box{background:#1e293b;border:1px solid #334155;border-radius:12px;padding:30px;display:inline-block;max-width:400px;}";
    html += "h2{color:" + String(updateFailed ? "#ef4444" : "#8b5cf6") + ";}";
    html += ".loader{border:4px solid #334155;border-top:4px solid #8b5cf6;border-radius:50%;width:30px;height:30px;animation:spin 1s linear infinite;margin:20px auto;}";
    html += "@keyframes spin{0%{transform:rotate(0deg);}100%{transform:rotate(360deg);}}</style>";
    html += "</head><body><div class='status-box'>";
    
    if (updateFailed) {
      html += "<h2>Update Failed!</h2><p>Please check the Serial Monitor for errors.</p>";
      html += "<a href='/admin' style='color:#3b82f6;text-decoration:none;'>&larr; Back to Admin</a>";
    } else {
      html += "<h2>Flash Successful!</h2>";
      html += "<div class='loader'></div>";
      html += "<p>Master is rebooting...<br>Returning to Dashboard in 10 seconds.</p>";
    }
    
    html += "</div></body></html>";

    // Send the response
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", html);
    response->addHeader("Connection", "close");
    request->send(response);
    
    // Handle the reboot flag
    if (!updateFailed) {
        Serial.println("Update finished. Rebooting in 2 seconds...");
        ota_reboot_flag = true; 
        ota_reboot_time = millis();
    }
  }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (!index) {
          displayFwUpdate();
          Serial.printf("Master Update Start: %s\n", filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
              Update.printError(Serial);
          }
      }
      if (len) {
          if (Update.write(data, len) != len) {
              Update.printError(Serial);
          } else {
              int progress = (index + len) * 100 / request->contentLength();
              static int lastPercent = -1;
              if (progress != lastPercent) {
                  updateProgressBar(progress);
                  lastPercent = progress;
              }
          }
      }
      if (final) {
          if (Update.end(true)) {
              Serial.printf("Update Success: %u bytes\n", index + len);
          }
      }
  });

  //5. Route for dashboard
  server.on("/", HTTP_GET, [&](AsyncWebServerRequest *request){
    // Default to Fahrenheit locally for the web
    bool webFahrenheit = true; 
  
    // If the URL specifically asks for Celsius (?unit=C), switch it
    if (request->hasParam("unit")) {
      if (request->getParam("unit")->value() == "C") {
        webFahrenheit = false;
      }
    }
  
    // Prepare Display Values (Local to this web request only)
    String unitLabel = webFahrenheit ? "F" : "C";
    float dTemp = webFahrenheit ? (tempC * 1.8 + 32) : tempC;
    float dMin  = webFahrenheit ? (rtcMinTemp * 1.8 + 32) : rtcMinTemp;
    float dMax  = webFahrenheit ? (rtcMaxTemp * 1.8 + 32) : rtcMaxTemp;
    float dROC  = webFahrenheit ? (currentROC * 1.8) : currentROC;
    int battPercent = getBatteryPercentage(battVolts);

    // Determine a color for the battery percentage
    String battColor = "#22c55e"; // Default Green
    if (battPercent < 55) battColor = "#eab308"; // Yellow
    if (battPercent < 35) battColor = "#ef4444"; // Red
  
    // Heartbeat Calculation
    unsigned long secondsAgo = (millis() - timeOfLastUpdate) / 1000;
    String statusColor = (secondsAgo < 600) ? "#22c55e" : "#ef4444"; 
  
    // Construct the HTML
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    
    // Update the refresh tag to stay on the chosen unit
    html += "<meta http-equiv='refresh' content='15;url=/?unit=" + unitLabel + "'>";
    
    html += "<style>";
    html += "body { font-family: 'Segoe UI', sans-serif; background: #0f172a; color: #f8fafc; margin: 0; padding: 20px; text-align: center; }";
    html += ".container { display: flex; flex-wrap: wrap; justify-content: center; gap: 15px; max-width: 1000px; margin: 20px auto; }";
    html += ".card { background: #1e293b; border-radius: 12px; padding: 20px; border: 1px solid #334155; ";
    html += "flex: 1 1 240px; max-width: 350px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1); }";
    html += ".label { font-size: 0.85rem; color: #94a3b8; margin-bottom: 8px; text-transform: uppercase; }";
    html += ".value { font-size: 2.2rem; font-weight: bold; }";
    html += ".batt-value { font-size: 2.2rem; font-weight: bold; }";
    html += ".unit { font-size: 1.2rem; color: #64748b; vertical-align: middle; }"; 
    html += ".unit-large { font-size: 2.2rem; font-weight: bold; vertical-align: middle; color: #f8fafc; }"; 
    html += ".trend-up { color: #fb7185; } .trend-down { color: #38bdf8; }";
    html += ".status-dot { height: 10px; width: 10px; background-color: " + statusColor + "; border-radius: 50%; display: inline-block; margin-right: 5px; }";
    html += ".btn { display: inline-block; padding: 12px 24px; background: #334155; color: white; text-decoration: none; border-radius: 8px; margin: 10px; font-weight: bold; border: 1px solid #475569; }";
    html += ".btn:hover { background: #475569; }"; // Added a nice hover effect
    html += "</style></head><body>";
  
    html += "<h1>Weather Dashboard</h1>";
    html += "<p><span class='status-dot'></span> Sensor Last Seen: " + String(secondsAgo) + "s ago</p>";
  
    html += "<div class='container'>";
    
    // Temperature Card
    if(commLoss || firstDataRx==false){
      html += "<div class='card'><div class='label'>Current</div><div class='value'>??.?<span class='unit-large'>&deg;" + unitLabel + "</span></div></div>";
    }else{
      html += "<div class='card'><div class='label'>Current</div><div class='value'>" + String(dTemp, 1) + "<span class='unit-large'>&deg;" + unitLabel + "</span></div></div>";
    }
    
    
    // Mix Temp Card
    if(dMax <= -40){
      html += "<div class='card'><div class='label'>High</div><div class='value' style='color:#fda4af'>??.?&deg;" + unitLabel + "</div></div>";
    }else{
      html += "<div class='card'><div class='label'>High</div><div class='value' style='color:#fda4af'>" + String(dMax, 1) + "&deg;" + unitLabel + "</div></div>";
    }

    //Min Temp Card
    if(dMin >= 70){
      html += "<div class='card'><div class='label'>Low</div><div class='value' style='color:#7dd3fc'>??.?&deg;" + unitLabel + "</div></div>";
    }else{
      html += "<div class='card'><div class='label'>Low</div><div class='value' style='color:#7dd3fc'>" + String(dMin, 1) + "&deg;" + unitLabel + "</div></div>";
    }
    
    
    // Trend (ROC) Card
    String rocClass = (dROC >= 0) ? "trend-up" : "trend-down";
    String sign = (dROC > 0) ? "+" : "";
    html += "<div class='card'><div class='label'>Trend</div><div class='value " + rocClass + "'>" + sign + String(dROC, 2) + "</div></div>";
  

    // Battery Percentage Card
    html += "<div class='card'><div class='label'>Sensor Battery</div><div class='batt-value' style='color:" + battColor + "'>" + String(battPercent) + "<span class='unit-small'>%</span></div></div>";
  
    html += "</div>";
  
    // Local Unit Toggle Button (Doesn't affect Master Display)
    String nextUnit = webFahrenheit ? "C" : "F";
    String btnText = webFahrenheit ? "View in Celsius" : "View in Fahrenheit";
    html += "<a href='/?unit=" + nextUnit + "' class='btn'>" + btnText + "</a>";
  
    // Global Reset Button
    html += "<a href='/reset' class='btn' style='background:#475569' onclick='return confirm(\"Reset Daily Records?\")'>Reset Min/Max</a>";
  
    html += "</body></html>";
  
    request->send(200, "text/html", html);
  });


  // 6. Route to handle the Reset action
  server.on("/reset", HTTP_GET, [&](AsyncWebServerRequest *request){
    // Reset High/Low
    rtcMinTemp = 99.0;
    rtcMaxTemp = -99.0;
    
    prefs.begin("temp-data", false);
    prefs.putFloat("min", 99.0);
    prefs.putFloat("max", -99.0);
    prefs.end();

    clearROCHistory(tempC);
    
    Serial.println("Global Reset: Min/Max and ROC cleared.");

    request->redirect("/");
  });
}


//save MIN and MAX temp values to "preferences" so that they are retained through power loss
void saveMinMax() {
  prefs.begin("temp-data", false); // "false" means read/write mode
  prefs.putFloat("min", rtcMinTemp);
  prefs.putFloat("max", rtcMaxTemp);
  prefs.putBool("minMaxStored", minMaxStored);
  prefs.end();
}


//load stored MIN and MAX temp values from flash memory
void loadMinMax() {
  prefs.begin("temp-data", true); // "true" means read-only
  rtcMinTemp = prefs.getFloat("min", 99.0); // 99.0 is the default if not found
  rtcMaxTemp = prefs.getFloat("max", -99.0);
  minMaxStored = prefs.getBool("minMaxStored", false);
  prefs.end();
}


// Save Rate of Change data to NVS
void saveROC(float newTemp) {
  prefs.begin("roc-data", false);
  
  // Shift the history array and save each index
  for (int i = 0; i < 5; i++) {
    rocHistory[i] = rocHistory[i + 1];
    prefs.putFloat(String(i).c_str(), rocHistory[i]);
  }
  
  // Add newest reading to the end
  rocHistory[5] = newTemp;
  prefs.putFloat("5", newTemp);
  
  // Handle the counter
  if (rocCount < 6) {
    rocCount++;
    prefs.putInt("count", rocCount);
  }
  prefs.end();

  // Update the global currentROC variable (Degrees per Hour)
  if (rocCount >= 6) {
    currentROC = (rocHistory[5] - rocHistory[0]) * 2.0; 
  }
}



// Load Rate of Change data from NVS
void loadROC() {
  prefs.begin("roc-data", true);
  rocCount = prefs.getInt("count", 0);
  for (int i = 0; i < 6; i++) {
    rocHistory[i] = prefs.getFloat(String(i).c_str(), 0.0);
  }
  
  // Recalculate the trend immediately upon loading
  if (rocCount >= 6) {
    currentROC = (rocHistory[5] - rocHistory[0]) * 2.0;
  }
  prefs.end();
  Serial.printf("ROC History loaded. Current Trend: %.1f deg/hr\n", currentROC);
}


//Function that clears the ROC from NVS
void clearROCHistory(float currentTemp) {
  prefs.begin("roc-data", false);
  
  // 1. Reset the counter so we don't calculate ROC until we have 6 new readings
  rocCount = 0;
  prefs.putInt("count", 0);
  
  // 2. Fill the array and Flash with the current temperature to "flatten" the trend
  for (int i = 0; i < 6; i++) {
    rocHistory[i] = currentTemp;
    prefs.putFloat(String(i).c_str(), currentTemp);
  }
  
  // 3. Reset the global trend variable
  currentROC = 0.0;
  
  prefs.end();
  Serial.println("ROC History cleared and flattened.");
}
