#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Servo.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// WiFi information
const char* ssid = "quocnguyn";  
const char* password = "20110304"; 

// MQTT info
#define MQTT_SERV "io.adafruit.com"
#define MQTT_PORT 1883
#define MQTT_NAME "QUYNH22"
#define MQTT_PASS "aio_KkEs71DUY8mODr4oyCOZaLfgKrkv"

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, MQTT_SERV, MQTT_PORT, MQTT_NAME, MQTT_PASS);

// Register MQTT topics
Adafruit_MQTT_Subscribe EntryGate = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/EntryGate");
Adafruit_MQTT_Subscribe ExitGate = Adafruit_MQTT_Subscribe(&mqtt, MQTT_NAME "/f/ExitGate");
Adafruit_MQTT_Publish CarsParked = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/CarsParked");

// Assuming you already have MQTT publish objects for each slot like this:
Adafruit_MQTT_Publish EntrySlot1_Publish = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/EntrySlot1");
Adafruit_MQTT_Publish ExitSlot1_Publish = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/ExitSlot1");

Adafruit_MQTT_Publish EntrySlot2_Publish = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/EntrySlot2");
Adafruit_MQTT_Publish ExitSlot2_Publish = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/ExitSlot2");

Adafruit_MQTT_Publish EntrySlot3_Publish = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/EntrySlot3");
Adafruit_MQTT_Publish ExitSlot3_Publish = Adafruit_MQTT_Publish(&mqtt, MQTT_NAME "/f/ExitSlot3");

// Sensor and servo pins
const int IR1_PIN = D0;  // Pin D0 for vehicle entry sensor
const int IR2_PIN = D1;  // Pin D1 for vehicle exit sensor
const int SLOT1_PIN = D6;
const int SLOT2_PIN = D7;
const int SLOT3_PIN = D8;
const int SERVO_PIN = D5; 
const int DEBOUNCE_DELAY = 200;  // Debounce delay is 200ms
unsigned long lastIR1Time = 0;
unsigned long lastIR2Time = 0;
Servo myservo;
int count = 0; 
const int CLOSE_ANGLE = 0;  // Close angle for servo
const int OPEN_ANGLE = 80;  // Open angle for servo

bool s1_occupied = false;
bool s2_occupied = false;
bool s3_occupied = false;
unsigned long lastDetectedTime = 0; // Time when the vehicle stops

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // UTC+7

// Function prototypes
void handleSlot(bool slotState, bool& slotOccupied, const char* slotName, String formattedTime,
                Adafruit_MQTT_Publish& entrySlotPublish, Adafruit_MQTT_Publish& exitSlotPublish);
void handleGateCommand(const char* command, const char* gateName);

void setup() {
  Serial.begin(57600);

  // Connect to WiFi
  Serial.print("Connecting to WiFi");
WiFi.begin((char*)ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to WiFi!");

  // Setup MQTT
  mqtt.subscribe(&EntryGate);
  mqtt.subscribe(&ExitGate);

  // Setup sensors and servo
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(SLOT1_PIN, INPUT);
  pinMode(SLOT2_PIN, INPUT);
  pinMode(SLOT3_PIN, INPUT);
  myservo.attach(SERVO_PIN);
  myservo.write(CLOSE_ANGLE);  // Initially, close the gate

  // Update time client
  timeClient.update();
}

void loop() {
  MQTT_connect();
  timeClient.update();
  
  String currentTime = timeClient.getFormattedTime(); // Get time as HH:MM:SS
  Serial.println("Current Time: " + currentTime);

  // Read slot sensor states
  bool s1 = digitalRead(SLOT1_PIN);
  bool s2 = digitalRead(SLOT2_PIN);
  bool s3 = digitalRead(SLOT3_PIN);
  
  // Handle slot occupancy
  handleSlot(s1, s1_occupied, "Slot 1", timeClient.getFormattedTime(), EntrySlot1_Publish, ExitSlot1_Publish);
  handleSlot(s2, s2_occupied, "Slot 2", timeClient.getFormattedTime(), EntrySlot2_Publish, ExitSlot2_Publish);
  handleSlot(s3, s3_occupied, "Slot 3", timeClient.getFormattedTime(), EntrySlot3_Publish, ExitSlot3_Publish);

  // Check if IR1 is triggered to open the gate (when the vehicle enters)
  if (digitalRead(IR1_PIN) == HIGH) { // Vehicle enters (IR1 triggered)
    openCloseGate(true);  // Open gate
    lastDetectedTime = millis(); // Update time when vehicle enters
    delay(1000); // Optional: add a delay to avoid rapid repeated triggers (debouncing)
  }

  // Check if IR2 is triggered to close the gate (when the vehicle exits)
  if (digitalRead(IR2_PIN) == HIGH) { // Vehicle exits (IR2 triggered)
    openCloseGate(false); // Close gate (servo set to 0 degrees)
    lastDetectedTime = millis(); // Update time when vehicle exits
    delay(1000); // Optional: add a delay to avoid rapid repeated triggers (debouncing)
  }

  // Handle commands from MQTT
  Adafruit_MQTT_Subscribe* subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &EntryGate) {
      handleGateCommand((char*)EntryGate.lastread, "Entry Gate");
    } else if (subscription == &ExitGate) {
      handleGateCommand((char*)ExitGate.lastread, "Exit Gate");
    }
  }

  // Update the car count
  int carsParkedCount = 0;  // Reset counter each loop cycle
  if (s1_occupied) carsParkedCount++;
  if (s2_occupied) carsParkedCount++;
  if (s3_occupied) carsParkedCount++;

  Serial.print("Cars parked: ");
  Serial.println(carsParkedCount);  // Debugging the car count

  // Publish the car count to MQTT
  if (!CarsParked.publish(String(carsParkedCount).c_str())) {
    Serial.println("Failed to publish CarsParked value.");
  } else {
    Serial.println("CarsParked value published.");
  }
}

void handleSlot(bool slotState, bool& slotOccupied, const char* slotName, String formattedTime,
                Adafruit_MQTT_Publish& entrySlotPublish, Adafruit_MQTT_Publish& exitSlotPublish) {
  if (slotState && !slotOccupied) {
    Serial.printf("%s is occupied\n", slotName);
    slotOccupied = true;
    if (!entrySlotPublish.publish((char*)formattedTime.c_str())) {
      Serial.println("Failed to publish entry time for slot.");
    }
  } else if (!slotState && slotOccupied) {
    Serial.printf("%s is vacant\n", slotName);
    slotOccupied = false;
    if (!exitSlotPublish.publish((char*)formattedTime.c_str())) {
      Serial.println("Failed to publish exit time for slot.");
    }
  }
}
void openCloseGate(bool open) {
  int targetAngle = open ? OPEN_ANGLE : CLOSE_ANGLE;  // Chọn góc mục tiêu (OPEN_ANGLE hoặc CLOSE_ANGLE)
  int currentAngle = myservo.read();  // Lấy góc hiện tại của servo

  // Nếu góc hiện tại khác góc mục tiêu, thực hiện chuyển động dần dần
  if (currentAngle != targetAngle) {
    int step = (targetAngle > currentAngle) ? 1 : -1;  // Đặt bước nhảy (tăng hoặc giảm góc)

    // Di chuyển servo đến góc mục tiêu dần dần
    for (int angle = currentAngle; angle != targetAngle; angle += step) {
      myservo.write(angle);  // Cập nhật góc servo
      delay(15);  // Độ trễ giữa các bước, có thể điều chỉnh để giảm tốc độ servo
    }
  }
  Serial.println(open ? "Gate opened" : "Gate closed");
}


void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      Serial.println("Failed to connect to MQTT, check config.");
      while (1);
    }
  }
}

void handleGateCommand(const char* command, const char* gateName) {
  Serial.printf("Command from %s: %s\n", gateName, command);

  // Check if the command is "ON" or "OFF"
  if (strcmp(command, "ON") == 0) {
    openCloseGate(true);  // Open gate
  } else if (strcmp(command, "OFF") == 0) {
    openCloseGate(false); // Close gate
  }
}
