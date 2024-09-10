
#include <SimpleFOC.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <queue> // Bibliothek für die Queue (Puffer)

// WiFi und MQTT Einstellungen
const char *ssid = "";
const char *password = "";
const char *mqtt_server = "";
const char *mqtt_username = "";
const char *mqtt_password = "";
const int mqtt_port = 8883;
float incrAngle = 0.0;
bool afterFirstMessage = false;
float prevAngle = 0.0;
float currAngle = 0.0;
const float epsilon = 0.1f; // Precision for an angle to be recognized as new input
const float minAngle = epsilon * 2.0f;
byte *buffer;               // To safe the incoming message from the subscribed topic and use it elsewhere in the code
boolean Rflag = false;      // true, if new message has arrived
int r_len;                  // Buffer length
float inverseAngle = 3.14;
float motorShaftAngle = 0.0;

std::queue<float> angleQueue; // Queue für die Zwischenspeicherung der Winkel

// MQTT Topics
const char *topic_publish = "y";
const char *topic_subscribe = "x";

// Motor und Sensor Setup für Motor 1
const int AS5048A_CS_PIN1 = 5; // Setze diesen Wert auf den tatsächlichen CS-Pin
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048A_CS_PIN1, 14, 0x3FFF);
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver6PWM driver1 = BLDCDriver6PWM(27, 26, 33, 25, 14, 32);

WiFiClientSecure espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
char msg[50];

// Root CA Zertifikat
static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

// Callback-Funktion für eingehende MQTT-Nachrichten
void callback(char *topic, byte *payload, unsigned int length)
{
  String incomingMessage = "";
  for (unsigned int i = 0; i < length; i++)
  {
    incomingMessage += (char)payload[i];
  }
  Serial.println("Message arrived: [" + String(topic) + "] " + incomingMessage);

  if (strcmp(topic, topic_subscribe) == 0)
  {
    float receivedAngle;
    if (sscanf(incomingMessage.c_str(), "%f", &receivedAngle) == 1)
    {
      if (abs(motorShaftAngle - receivedAngle) >= minAngle)
      {
        angleQueue.push(receivedAngle); // Winkel in die Queue einfügen
        Serial.printf("Angle queued: %f\n", receivedAngle);
      }
    }
    else
    {
      Serial.println("Failed to parse angle");
    }
  }
}

void setupWiFi()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA); // <-- Hier ist die Korrektur für den WLAN-Modus
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("connected");
      client.subscribe(topic_subscribe);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup()
{

  // Configure the velocity PID controller
  motor1.PID_velocity.P = 0.2;
  motor1.PID_velocity.I = 20;
  motor1.PID_velocity.D = 0.001;
  motor1.PID_velocity.output_ramp = 300; // Reduced ramp to avoid sudden changes

  // Configure the velocity low-pass filter
  motor1.LPF_velocity.Tf = 0.01;

  // Configure the angle P controller
  motor1.P_angle.P = 20;

  // Set velocity and voltage limits
  motor1.velocity_limit = 4;
  motor1.voltage_limit = 8;

  Serial.begin(9600);
  setupWiFi();

  // Initialize sensor and motor for Motor 1
  sensor1.init();
  motor1.linkSensor(&sensor1); // Sensor mit Motor verlinken
  driver1.init();
  motor1.linkDriver(&driver1); // Treiber mit Motor verlinken
  motor1.controller = MotionControlType::angle;
  motor1.init();
  motor1.initFOC();
  delay(100);
}

// Set point variable for target angle
float target_angle = 3.14;

// Timestamp for changing direction
long timestamp_us = _micros();

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // Check for next angle every 0.1 second in queue
  if (_micros() - timestamp_us > 1e5)
  {
    timestamp_us = _micros();
    // Verarbeite den nächsten Winkel aus der Queue, falls vorhanden
    if (!angleQueue.empty())
    {
      float newAngle = angleQueue.front(); // Nächsten Winkel aus der Queue holen
      // Überprüfe, ob der Motor nicht bereits zum neuen Winkel bewegt hat
      if (abs(motor1.shaft_angle - currAngle) <= epsilon) // Wenn der Winkel fast erreicht ist
      {
        angleQueue.pop();     // Entferne den Winkel aus der Queue
        currAngle = newAngle; // Setze den aktuellen Winkel auf den neuen Winkel
        Serial.printf("Moving to angle: %f\n", currAngle);
      }
      else
      {
        currAngle = motor1.shaftAngle();
      }
    }
  }

  motor1.loopFOC();
  //motor1.move(currAngle); // Bewege den Motor zum neuen Winkel

  motorShaftAngle = (motor1.shaftAngle()) * -1.0;

  // Only publish angle every 0.1 second
  // TODO: And only if it has a certain deviation from the current angle (Alternatively: In callback)
  if (_micros() - timestamp_us > 1e5)
  {
    timestamp_us = _micros();
    char message[50];
    snprintf(message, 50, "%f", motorShaftAngle);
    client.publish(topic_publish, message);
    delay(10); // Adjust delay as needed
  }
}
