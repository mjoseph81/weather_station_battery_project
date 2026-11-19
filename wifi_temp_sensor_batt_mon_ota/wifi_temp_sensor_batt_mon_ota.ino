#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_bt.h" // Required for hardware-level BT disable
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HTTPClient.h>
#include <Update.h>


// --- DEBUG TOGGLE ---
#define DEBUG 0  // Set to 1 for Serial output, 0 for absolute minimum power usage

#if DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif


const String SENSOR_FW_VER = "v1.0.3";

// --- Configuration ---
const char* ssid = "ESP32-MyAP";
const char* password = "password123!";
const char* mqtt_broker = "10.0.0.1"; 
const char* pub_topic = "esp32/pub";
const char* sub_topic = "sensor/ota";
const int mqtt_port = 1883;

// --- FAST CONNECT SETTINGS ---
IPAddress local_IP(10, 0, 0, 50); 
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);
int wifi_channel = 1;              
uint8_t bssid[] = {0x70, 0xB8, 0xF6, 0x5B, 0x8F, 0x09}; 


const int mqttMaxRetries = 3;
const int wifi_conn_attempts = 3;
const unsigned long wifiTimeout = 5000; 

#define uS_TO_S_FACTOR 1000000ULL  
#define TIME_TO_SLEEP  300          

const int oneWireBus = 10; // GPIO where the DS18B20 is connected
const int SENSOR_POWER_PIN = 6; // GPIO where the DS18B20 power control is connected
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

const int BAT_READ_PIN = 2;   // A0 (GPIO 2)
const int BAT_GROUND_PIN = 3; // D1 (GPIO 3) - Our switchable ground

WiFiClient espClient;
PubSubClient client(espClient);

volatile bool otaUpdate = false;

bool setup_wifi() {
  unsigned long startAttemptTime = millis();
  const unsigned long fastTimeout = 2000; // 2 seconds is plenty for Static IP

  DEBUG_PRINT("\nConnecting to ");
  DEBUG_PRINTLN(ssid);

  WiFi.config(local_IP, gateway, subnet); 
  //WiFi.begin(ssid, password, wifi_channel, bssid);
  WiFi.begin(ssid, password, wifi_channel);
  
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startAttemptTime > fastTimeout) {
      DEBUG_PRINTLN("\nWatchdog Triggered: WiFi taking too long. Aborting to save battery.");
      return false;
    }
    delay(50); // Small delay to prevent CPU hogging
  }

  DEBUG_PRINTLN("\nWiFi connected.");
  return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  
  DEBUG_PRINT("Message arrived [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("] ");
  for (int i = 0; i < length; i++){
    message += (char)payload[i];  ;
  }
  DEBUG_PRINTLN(message);

  if (String(topic) == sub_topic && message == "START") {
    otaUpdate = true;
    DEBUG_PRINTLN("OTA Trigger Received!");
  }
}

// Updated to return success/fail and handle retries
bool mqtt_connect() {
  int retries = 0;
  while (!client.connected() && (retries < mqttMaxRetries) && (WiFi.status() == WL_CONNECTED)) {
    Serial.printf("Attempting MQTT connection (Attempt %d/%d)...\n", retries + 1, mqttMaxRetries);
    String clientId = "ESP32Client-" + String(WiFi.macAddress());
    
    if (client.connect(clientId.c_str())) {
      DEBUG_PRINTLN("MQTT connected!");
      client.subscribe(sub_topic);
      return true;
    } else {
      DEBUG_PRINT("MQTT failed, rc=");
      DEBUG_PRINT(client.state());
      DEBUG_PRINTLN(" - Retrying in 500 milliseconds...");
      delay(500);
      retries++;
    }
  }
  return false;
}



float getBatteryVoltage() {
  //Enable the ground path
  pinMode(BAT_GROUND_PIN, OUTPUT);
  digitalWrite(BAT_GROUND_PIN, HIGH); 
  delay(10); // Let voltage stabilize

  // Take the reading
  // The XIAO ADC works best with a bit of averaging
  int raw = 0;
  for(int i = 0; i < 10; i++) {
    raw += analogRead(BAT_READ_PIN);
    delay(2);
  }
  raw /= 10;

  //Disable ground path to save power
  pinMode(BAT_GROUND_PIN, INPUT); // Set to high-impedance

  // Step 1: Calculate the voltage AT THE PIN (0 to 6.3V)
  float vPin = (raw / 4095.0) * 3.3 * 2.78;

  return vPin;
}



void setup() {
  Serial.begin(115200);
  // Wait up to 5 seconds for the Serial Monitor to be opened
  while (!Serial && millis() < 5000) { 
    delay(10); 
  }

  DEBUG_PRINTLN("Sensor FW version: " + SENSOR_FW_VER);
  
  // Kill Bluetooth immediately at the stack and hardware level
  btStop();
  esp_bt_controller_disable(); 

  // Lower CPU frequency to 80MHz (lowest for Wi-Fi)
  setCpuFrequencyMhz(80); 
  
  // --- SENSOR POWER UP ---
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, LOW);

  // Extra safety: make sure internal pullups are off
  gpio_pullup_dis((gpio_num_t)SENSOR_POWER_PIN);
  
  // Give the sensor and pull-up resistor time to stabilize
  delay(100); 

  sensors.begin();
  sensors.setResolution(10); // 10-bit is faster and more reliable at 3.3V
  
  // Declare tempC here so it is available for the rest of setup()
  float tempC = DEVICE_DISCONNECTED_C;

  // Request temperature
  sensors.requestTemperatures(); 
  
  // Wait for conversion (10-bit needs ~188ms)
  delay(200); 

  tempC = sensors.getTempCByIndex(0);

  // If it's reading 85 (power-on reset value) or disconnected, retry once
  if (tempC == 85.0 || tempC == DEVICE_DISCONNECTED_C) {
      DEBUG_PRINTLN("Initial read failed or returned 85. Retrying...");
      delay(500); 
      sensors.requestTemperatures();
      delay(200);
      tempC = sensors.getTempCByIndex(0);
  }
  
  if(tempC != DEVICE_DISCONNECTED_C) {
    DEBUG_PRINT("Temperature: ");
    DEBUG_PRINTLN(tempC);
  } else {
    DEBUG_PRINTLN("Error: Could not read temperature data");
  }

  float battV = 0;
  
  battV = getBatteryVoltage();

  
  // Convert float to string for MQTT
  char battString[8]; 
  dtostrf(battV, 1, 3, battString);
  
  char tempString[8]; 
  dtostrf(tempC, 1, 2, tempString); 

  String payload = "{\"t\":" + String(tempC, 1) + 
                 ",\"v\":" + String(battV, 2) + 
                 ",\"fw\":\"" + SENSOR_FW_VER + "\"}";

  // --- WIFI & MQTT SECTION ---
  bool connected = false;
  int cnt_attempts = 0;

  while(cnt_attempts < wifi_conn_attempts) {
    if(setup_wifi()) {
      client.setServer(mqtt_broker, mqtt_port);
      client.setCallback(callback);
  
      if (mqtt_connect()) {
        client.loop(); 
        client.publish(pub_topic, payload.c_str());
        
        DEBUG_PRINTLN("Data Sent! Listening for OTA handshake...");

        // --- NEW HANDSHAKE WINDOW ---
        unsigned long startWait = millis();
        while (millis() - startWait < 600) { // Wait 600ms for Master to reply
          client.loop(); 
          if (otaUpdate) break; 
          delay(10);
        }

        if (otaUpdate) {
          DEBUG_PRINTLN("Starting OTA Update Process...");
           
          // Give the MQTT client a moment to actually push the packet out
          client.loop(); 
          delay(100); 
      
          if (battV < 4.4) {
              DEBUG_PRINTLN("OTA Aborted: Battery too low for safe flashing!");
              client.publish("esp32/sensor1/status", "LOW_BATT_CANCEL");
              otaUpdate = false; // Reset the flag and go back to sleep
          } else {
              DEBUG_PRINTLN("Starting OTA Update Process...");
              client.publish("esp32/sensor1/status", "UPDATING");
              
              client.loop(); 
              delay(100); 
       
              UpdateFirmware(); 
          }
        }
        // --- END HANDSHAKE ---
        connected = true;
      }
      break; 
    } else {
      DEBUG_PRINTLN("Retrying wifi connection...");
      cnt_attempts++;
    }
  }

  if(!connected) {
    DEBUG_PRINTLN("Critical Failure: WiFi or MQTT could not connect.");
  }
  
  // Power down sequence
  client.disconnect();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF); 
  
  // Turn off sensor power to save battery during sleep
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  // disable sensor data line to prevent ghost leak
  pinMode(oneWireBus, INPUT);
  gpio_pullup_dis((gpio_num_t)oneWireBus);

  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  DEBUG_PRINTLN("Going to sleep now");
  esp_deep_sleep_start();
}

/*
void UpdateFirmware() {
  DEBUG_PRINTLN("Starting OTA Update...");
  
  // Disable sleep timers so it doesn't die mid-update
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  // The httpUpdate.update call handles the GET request, 
  // the flashing, and the REBOOT if successful.
  t_httpUpdate_return ret = httpUpdate.update(espClient, "http://10.0.0.1/update");

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      DEBUG_PRINTF("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      DEBUG_PRINTLN("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      DEBUG_PRINTLN("HTTP_UPDATE_OK"); // This usually won't print because it reboots
      break;
  }
}
*/  

void loop() {}


void UpdateFirmware() {
    HTTPClient http;

    String url = "http://10.0.0.1/update"; 
    
    DEBUG_PRINTLN("Connecting to Master for binary download...");
    http.begin(url);
    
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        int contentLength = http.getSize();
        DEBUG_PRINTF("Update size: %d bytes\n", contentLength);

        if (contentLength > 0) {
            // Check if there is enough space in the OTA partition
            if (Update.begin(contentLength)) {
                DEBUG_PRINTLN("Flashing... Please do not power off.");
                
                // Stream the data from the web server directly to the Flash
                WiFiClient* stream = http.getStreamPtr();
                size_t written = Update.writeStream(*stream);

                if (written == contentLength) {
                    DEBUG_PRINTLN("Written : " + String(written) + " successfully");
                } else {
                    DEBUG_PRINTLN("Written only : " + String(written) + "/" + String(contentLength) + ". Failed!");
                }

                if (Update.end()) {
                    if (Update.isFinished()) {
                        DEBUG_PRINTLN("Update successfully completed. Rebooting...");
                        delay(1000);
                        ESP.restart();
                    } else {
                        DEBUG_PRINTLN("Update not finished? Something went wrong.");
                    }
                } else {
                    DEBUG_PRINTF("Update Error Occurred. Error #: %d\n", Update.getError());
                }
            } else {
                DEBUG_PRINTLN("Not enough space to begin OTA");
            }
        }
    } else {
        DEBUG_PRINTF("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
}
