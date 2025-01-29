#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

///////////////////////////////////////////////////////////////
// Defines
///////////////////////////////////////////////////////////////

#define     DEBUG_MODE

///////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////

const char* ssid = "WI-FIzgoraj_2G";
const char* password = "lovrenchertis";
//const char* mqtt_server = "test.mosquitto.org";
//const char* mqtt_server = "192.168.1.237"; // PC IP address running local broker
const char* mqtt_server = "192.168.1.118"; // PC IP address running local broker
const int mqtt_port = 1883;

bool conctd = true;

char stmMessage[10] = "000000000";
bool stmRxComplete = false;
bool mqttRxComplete = false;
char mqttMessage[50];

static unsigned long lastSendTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);

HardwareSerial SerialUART(1);

///////////////////////////////////////////////////////////////
// Function prototipes
///////////////////////////////////////////////////////////////
void checkUart();
template <typename T>
void debugPrint(T msg);

///////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////

void setup_wifi() {
  delay(10);

  debugPrint("Connecting to ");
  debugPrint(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(200);
    Serial.print(".");
  }
  
  WiFi.setSleep(false);

  debugPrint("");
  debugPrint("WiFi connected");
  debugPrint("IP address: ");
  debugPrint(WiFi.localIP());
}

void setup_mqtt() {
  client.setServer(mqtt_server, mqtt_port);
}

void callback(char* topic, byte* payload, unsigned int length) 
{
  for (int i = 0; i < length; i++) {
    mqttMessage[i] = (char)payload[i];
    if( i >= 49 )
    {
      break;
    }
  }
  mqttRxComplete = true;
}

void reconnect() 
{
  while (!client.connected()) {
    if(conctd)
    {
      debugPrint("Attempting MQTT connection...");
    } 
    if (client.connect("ESP32Client")) 
    {
      if(conctd)
      {
        debugPrint("connected");
        conctd = false;
      }
      
      client.subscribe("zs", 0);
    } else 
    {
      debugPrint("failed, rc=");
      debugPrint(client.state());
      debugPrint(" try again in 5 seconds");
      conctd = true;
      delay(1000);
    }
  }
}

void publish_message(const char* message) 
{  
  if (client.publish("zr", message))
  {
    //debugPrint("Msg sent");
  } else 
  {
    debugPrint("Error sending message");
  }
}

void setup() 
{
#ifdef DEBUG_MODE
  Serial.begin(115200);
#endif /* DEBUG_MODE */
  setup_wifi();
  setup_mqtt();
  client.setCallback(callback);
  client.setKeepAlive(60);
  SerialUART.begin(115200, SERIAL_8N1, 23, 22); // 22 -> tx, 23 -> rx
  //SerialUART.setTimeout(20);
  //pinMode(16, OUTPUT);
}

void loop() 
{
  if ( !client.connected() ) 
  {
    reconnect();
  }
  client.loop();

  if( mqttRxComplete )
  {
    debugPrint(mqttMessage);
    SerialUART.write(mqttMessage);
    mqttRxComplete = false;
  }

  if( stmRxComplete )
  {
    //debugPrint();
    //debugPrint(stmMessage);
    publish_message(stmMessage);
    debugPrint(stmMessage);
    stmRxComplete = false;
  }

  // if (millis() - lastSendTime > 2000) 
  // {
  //   lastSendTime = millis();
  //   publish_message(stmMessage);
  //   debugPrint("RSSI: ");
  //   //debugPrint(WiFi.RSSI());  // doesn't work
  //   //Serial.print(WiFi.RSSI())
  // }

  checkUart();
}

void checkUart()
{
  while( SerialUART.available() )
  {
    char chr = (char)SerialUART.read();

    if( chr == 'Z' )
    {
      int result = SerialUART.readBytesUntil(chr, stmMessage, sizeof(stmMessage));
      if( result == 10 )
      {
        stmRxComplete = true;
      }
    }

  } 
}

template <typename T>
void debugPrint(T msg)
{
#ifdef DEBUG_MODE
  Serial.println(msg);
#endif /* DEBUG_MODE */
}
