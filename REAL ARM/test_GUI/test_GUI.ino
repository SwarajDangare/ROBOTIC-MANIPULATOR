#include <WiFi.h>
const char *ssid = "Swaraj Laptop";
const char *password = "12345678";
WiFiServer wifiServer(27339);
char rxData;

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    //Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
  wifiServer.begin();
}

void loop() {
  WiFiClient client = wifiServer.available();
  if (client)
  {
    //    Serial.println("Client Found");
    while (client.connected())
    {
      while (client.available() > 0)
      {
        rxData = client.read();
        Serial.println(rxData);
        client.write(rxData);

        Serial.println(rxData);
        delay(10);
      }
    }
  }
}
