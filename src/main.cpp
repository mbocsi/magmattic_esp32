#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ads1262.h>
#include <SPI.h>
#include <ArduinoJson.h>

#define PGA 1                     // Programmable Gain = 1
#define VREF 2.50                 // Internal reference of 2.5V
#define VFSR VREF/PGA             
#define FSR (((long int)1<<23)-1) 

using namespace websockets;

// Wifi
const char * SSID = "UWNet";
const char * PASSWORD = "";

// Websockets
const char * WEBSOCKET_ADDR = "192.168.1.58";
const uint32_t WEBSOCKET_PORT = 44444;

// ADC
const double ADC_RESOLUTION = (double)(VFSR/pow(2,31));
volatile char * spi_rx_buff_ptr;
volatile int32_t adc_reading = 0;
double adc_voltage;
volatile size_t voltage_buf_size = 32; // Samples
double voltage_buf[32];
volatile size_t voltage_buf_idx = 0;
uint16_t sample_rate = 1200; // Hz

// Data sending
volatile bool sendReady = false; // Flag
char outJson[512];

WebsocketsClient client;
ads1262 adc;

// function definitions (should probably be in a header file)
void initWifi();
// void scanWifi();
void recv(WebsocketsMessage message);
void send(double * buf, short buf_size);
void readVoltage();

void setup() {
  Serial.begin(115200);
  delay(200);

  // Connect Wifi
  initWifi(); 
  delay(100);

  // Attach recv handler
  client.onMessage([&](WebsocketsMessage message) {
    recv(message);
  });

  // Connect WS client 
  if(client.connect(WEBSOCKET_ADDR, WEBSOCKET_PORT, "/")) {
    Serial.println("Connected to WS server");
    client.send("{\"topic\":\"subscribe\",\"payload\":{\"topics\":[\"adc/command\"]}}"); // Subscriber message
  } else {
    Serial.println("Failed to connect to WS server");
  }

  // Initialize pins
  pinMode(ADS1262_DRDY_PIN, INPUT);                  //data ready input line
  pinMode(ADS1262_CS_PIN, OUTPUT);                   //chip enable output line
  pinMode(ADS1262_START_PIN, OUTPUT);               // start 
  pinMode(ADS1262_PWDN_PIN, OUTPUT);                // Power down output  
  attachInterrupt(digitalPinToInterrupt(ADS1262_DRDY_PIN), readVoltage, FALLING); // readVoltage when ready
  Serial.println("Pins and interrupts configured");

  // Initialize ADC
  adc.ads1262_Init();
  Serial.println("ADC initialized");

  Serial.println("Setup done");
  Serial.println("--------------------------------------");
}

void loop() {
  if (client.available()) {
    client.poll();
  } else {
    Serial.println("ws server disconnected - retrying connection...");
    if(client.connect(WEBSOCKET_ADDR, WEBSOCKET_PORT, "/")) {
      client.send("{\"topic\":\"subscribe\",\"payload\":{\"topics\":[\"adc/command\"]}}"); 
      Serial.println("ws server connected!");
    }
  }
  if(sendReady) {
    sendReady = false;
    send(voltage_buf, voltage_buf_size);
  }
  client.ping();
  delay(5);
}

void readVoltage() {
  /*
  SPI Message Format (6 Bytes Total)
  -----------------------------------
  | Byte 1  | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6  |
  |---------|--------|--------|--------|--------|---------|
  | Status  | Data 1 | Data 2 | Data 3 | Data 4 | CRC/CHK |

  - Byte 1: Status byte (optional, controlled by STATUS bit 2 in INTERFACE register at address 0x02)
  - Bytes 2-5: 32-bit ADC data value
  - Byte 6: Checksum or CRC (optional, controlled by CRC[1:0] bits in INTERFACE register at address 0x02)
  - If CRC/CHK is disabled, the message is only 5 bytes (Status + Data 1-4)

  Note:
  - Data is transmitted MSB first.
  - If "Repeat Data" mode is enabled, additional repeated bytes may follow.
  */
  spi_rx_buff_ptr = adc.ads1262_Read_Data();      // read 6 bytes conversion register
  adc_reading = (int32_t) ((uint32_t)spi_rx_buff_ptr[1]) << 24  // Extract value from buffer
    | ((uint32_t)spi_rx_buff_ptr[2]) << 16 
    | ((uint32_t)spi_rx_buff_ptr[3]) << 8 
    | ((uint32_t)spi_rx_buff_ptr[4]);
  adc_voltage = ADC_RESOLUTION * adc_reading; // Convert to volts
  voltage_buf[voltage_buf_idx++] = adc_voltage;
  if(voltage_buf_idx >= voltage_buf_size) {
    voltage_buf_idx = 0;
    sendReady = true;
  }
};

void recv(WebsocketsMessage message) {
  Serial.print("recv - ");
  Serial.println(message.data());
};

void send(double * buf, short buf_size) {
  JsonDocument doc;
  JsonArray payload = doc["payload"].to<JsonArray>();
  doc["topic"] = "voltage/data";
  for(int i = 0; i < buf_size; i++) {
    payload.add(buf[i]);
  }
  serializeJson(doc, outJson);
  client.send(outJson);
}

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
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.print(" connected! IP: ");
  Serial.println(WiFi.localIP());
}

// void scanWifi() {
//   int n = WiFi.scanNetworks();
//   if (n == 0) {
//     Serial.println("no network found");
//   } else {
//     Serial.print("networks found: ");
//     Serial.println(n);
//     for (int i = 0; i < n; i++) {
//       Serial.print(i + 1);
//       Serial.print(": ");
//       Serial.print(WiFi.SSID(i));
//       Serial.print(" (");
//       Serial.print(WiFi.RSSI(i));
//       Serial.print("dBm)");
//       Serial.print(" channel ");
//       Serial.print(WiFi.channel(i));
//       Serial.print(" encryption ");
//       Serial.println(WiFi.encryptionType(i));
//     }
//   }
// }
