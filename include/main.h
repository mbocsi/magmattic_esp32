#include <ArduinoWebsockets.h>

using namespace websockets;

void initWifi();
void scanWifi();
void recv(WebsocketsMessage message);
void send(double * buf, short buf_size);
void readVoltage();