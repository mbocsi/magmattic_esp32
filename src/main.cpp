#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <ads1262.h>
#include <SPI.h>

using namespace websockets;

const char * SSID = (char *) "UWNet";
const char * PASSWORD = (char *) "";
const char * WEBSOCKET_ADDR = "10.141.235.101";
const int WEBSOCKET_PORT = 44444;
const double VREF = 2.5;
const double ADC_RESOLUTION = (double)(VREF/pow(2,31));
const uint8_t VOLTAGE_RESOLUTION = 9;
char * spi_rx_buff_ptr;
signed long adc_reading = 0;
double adc_voltage;
int buf_size = 32; // Samples
double voltage_buf[32];
short voltage_buf_idx = 0;
int sample_rate = 1000; // Hz

WebsocketsClient client;
ads1262 adc;

void initWifi();
void recv(WebsocketsMessage message);
void send_voltage(double * buf, short buf_size);

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
  bool connected = client.connect(WEBSOCKET_ADDR, WEBSOCKET_PORT, "/");
  if(connected) {
    Serial.println("connected to WS server");
  } else {
    Serial.println("failed to connect to WS server");
  }
  // Send subscription message to server for commands to ESP32
  client.send("{\"topic\":\"subscribe\",\"payload\":{\"topics\":[\"adc/command\"]}}");

  // Initialize pins
  pinMode(ADS1262_DRDY_PIN, INPUT);                  //data ready input line
  pinMode(ADS1262_CS_PIN, OUTPUT);                   //chip enable output line
  pinMode(ADS1262_START_PIN, OUTPUT);               // start 
  pinMode(ADS1262_PWDN_PIN, OUTPUT);                // Power down output  

  // Initialize ADC
  adc.ads1262_Init();

  Serial.println("Setup done");
}

void loop() {
  if (client.available()) {
    client.poll();
  } else {
    Serial.println("ws server disconnected - retrying connection...");
    client.connect(WEBSOCKET_ADDR, WEBSOCKET_PORT, "/");
    client.send("{\"topic\":\"subscribe\",\"payload\":{\"topics\":[\"adc/command\"]}}"); 
  }
  for(int i = 0; i < buf_size; i++){

  }
  if((digitalRead(ADS1262_DRDY_PIN)) == LOW){
    spi_rx_buff_ptr = adc.ads1262_Read_Data();      // read 6 bytes conversion register
    adc_reading = (signed long) ((unsigned long)spi_rx_buff_ptr[1]) << 24  // Extract value from buffer
      | ((unsigned long)spi_rx_buff_ptr[2]) << 16 
      | ((unsigned long)spi_rx_buff_ptr[3]) << 8 
      | ((unsigned long)spi_rx_buff_ptr[4]);
    adc_voltage = ADC_RESOLUTION * adc_reading; // Convert to volts
    voltage_buf[voltage_buf_idx++] = adc_voltage;
    if(voltage_buf_idx >= buf_size) {
      voltage_buf_idx = 0;
      send_voltage(voltage_buf, buf_size);
    }
  }
  // Serial.printf("Voltage Reading: %lfV\n", adc_voltage);
}

void recv(WebsocketsMessage message) {
  Serial.print("recv - ");
  Serial.println(message.data());
};

void send_voltage(double * buf, short buf_size) {
  char val[16];
  Serial.println("sending buffer!");
  for(int i = 0; i < buf_size; i++) {
    int size = sprintf(val, "%.*lf", VOLTAGE_RESOLUTION, buf[i]);
    Serial.printf("val: %s size: %i\n", val, size);
  }
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
  Serial.println(WiFi.localIP());
}

