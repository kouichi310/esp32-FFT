#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "GL-AR750S-14e"; 
const char *password = "goodlife";

// MQTTブローカー  
const char *mqtt_broker = "192.168.8.228";
String topic = "mqtt-topics/esp32-" + String(WiFi.macAddress());
const char *mqtt_username = "emqx-user";
const char *mqtt_password = "emqx-user-password";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
int tempratureValue;

void init_wifi(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());
}

void init_mqtt(){
  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.println("Connecting to MQTT...");
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void setup(){
  Serial.begin(115200);
  init_wifi();
  init_mqtt();
}

void loop(){
  tempratureValue = temperatureRead();
  Serial.println("Temperature: " + String(tempratureValue) + "°C");
  client.publish(topic.c_str(), ("temp cpu=" + String(tempratureValue)).c_str());
  sleep(5);
}