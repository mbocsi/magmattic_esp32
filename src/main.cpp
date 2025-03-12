#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

const char * ssid = (char *) "UWNet";
const char * password = (char *) "";
const char * websocket_addr = "ws://10.140.250.213:44444";

WebsocketsClient client;

// put function declarations here:
void initWifi();
void scanWifi();

void setup() {
  Serial.begin(115200);
  delay(1000);
  initWifi(); 
  client.connect(websocket_addr);
  Serial.println("Connected to WS server");
  client.send("{\"type\":\"esp32_init\"}");
  Serial.println("Setup done");
}

void loop() {
  // scanWifi();
  // client.send("Sending voltages!");
  delay(5000);
}

// put function definitions here:
void initWifi() {
  delay(5000);
  uint8_t baseMac[6];
  
  // Get MAC address of the WiFi station interface
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  Serial.print("Station MAC: ");
  for (int i = 0; i < 5; i++) {
    Serial.printf("%02X:", baseMac[i]);
  }
  Serial.printf("%02X\n", baseMac[5]);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void scanWifi() {
  Serial.println("scan start");
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0) {
    Serial.println("no networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10);
    }
  }
  Serial.println("");
}