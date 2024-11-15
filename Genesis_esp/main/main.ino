#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

const char* ssid = "WI-FIzgoraj_2G";
const char* password = "lovrenchertis";
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;

const char* mqtt_user = "zanhertis@gmail.com";
const char* mqtt_password = "Cokolada123";
bool conctd = true;

WiFiClient espClient;
PubSubClient client(espClient);

HardwareSerial SerialUART(1);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_mqtt() {
  client.setServer(mqtt_server, mqtt_port);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    if(conctd)
    {
      Serial.print("Attempting MQTT connection...");
    } 
    if (client.connect("ESP32Client")) {
      if(conctd)
      {
        Serial.println("connected");
        conctd = false;
      }
      
      client.subscribe("test150900");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      conctd = true;
      delay(5000);
    }
  }
}

void publish_message() {
  char message[50];
  snprintf(message, 50, "Hello from ESP32");

  // Publish message to the "test150900" topic
  if (client.publish("test150900", message)) {
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Error sending message");
  }
}

void setup() {
  Serial.begin(115200);
  //setup_wifi();
  //setup_mqtt();
  //client.setCallback(callback);
  SerialUART.begin(115200, SERIAL_8N1, 23, 22); // 22 -> tx, 23 -> rx
  //Serial0.begin(115200, SERIAL_8N1, 23, 22);
  //pinMode(16, OUTPUT);
}

void loop() {
  //if (!client.connected()) {
    //reconnect();
  //}
  //client.loop();
  //publish_message();
  //delay(1000);
  static unsigned long lastSendTime = 0;
  //SerialUART.write("4");
  //delay(2000);
  if(SerialUART.available())
  {
    while(SerialUART.available())
    {
      char receiver = SerialUART.read();
      Serial.println(receiver);
    }
  }
  
  if (millis() - lastSendTime > 1000) {
    
    lastSendTime = millis();
  }
  
}