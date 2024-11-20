#include <SPI.h>
#include <MFRC522.h>

// RFID Pins
#define SS_PIN 53
#define RST_PIN 5

// AM312-PIR Motion Sensor Pins
#define MOTION_SENSOR_PIN1 42
#define MOTION_SENSOR_PIN2 43
#define MOTION_SENSOR_PIN3 44
#define MOTION_SENSOR_PIN4 45

// Trigger and Echo Pins for the HC-SR04 Sonar Sensor
#define TRIGGER_PIN1 32
#define ECHO_PIN1 33
#define TRIGGER_PIN2 34
#define ECHO_PIN2 35
#define TRIGGER_PIN3 36
#define ECHO_PIN3 37
#define TRIGGER_PIN4 38
#define ECHO_PIN4 39

// Pins for LEDs and buzzer
#define GREEN_LED 22
#define YELLOW_LED 23
#define RED_LED 24
#define BUZZER 25

byte readCard[4];
String MasterTag = "A135EC1B"; // Replace with your master tag
String tagID = "";

// Create instance for RFID
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Define variables
bool motionState;
long duration1, duration2, duration3, duration4;
int distance1, distance2, distance3, distance4;
int minDistance;
bool systemFrozen = false;  // Flag to indicate if the system is frozen
bool systemArmed = true;    // Flag to indicate if the system is active

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  SPI.begin();
  mfrc522.PCD_Init();
  delay(4);
  mfrc522.PCD_DumpVersionToSerial();

  pinMode(MOTION_SENSOR_PIN1, INPUT);
  pinMode(MOTION_SENSOR_PIN2, INPUT);
  pinMode(MOTION_SENSOR_PIN3, INPUT);
  pinMode(MOTION_SENSOR_PIN4, INPUT);

  pinMode(TRIGGER_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIGGER_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(TRIGGER_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);
  pinMode(TRIGGER_PIN4, OUTPUT);
  pinMode(ECHO_PIN4, INPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(GREEN_LED, HIGH);  // System starts armed
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER, LOW);

  connectToNetwork();
}

void loop() {
  if (getID()) {
    handleRFID();
  }

  if (!systemArmed) {
    // If the system is disarmed, turn off all indicators and skip detection
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, LOW);
    digitalWrite(BUZZER, LOW);
    return;
  }

  if (systemFrozen) {
    if(getID()) {
      handleRFID();
      //return;
    }  
      // Keep system frozen
  }

  if (motionDetected()) {
    readDistances();
    minDistance = findMinDistance();

    if (minDistance >= 15 && minDistance > 0) {
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER, LOW);
    }
    else if (minDistance < 15 && minDistance >= 10) {
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER, LOW);
      Serial.println("WARNING: Proximity Threshold Breached!");
    }
    else if (minDistance < 10 && minDistance > 0) {
      freezeSystem();
    }
  }
}

void freezeSystem() {
  systemFrozen = true;
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BUZZER, HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  Serial.println("SYSTEM FROZEN: Intruder Alert!");
  sendIFTTTNotification("SYSTEM FROZEN: Intruder Alert!");
}

void handleRFID() {
  if (tagID == MasterTag) {
    if (systemArmed && !systemFrozen) {
      // Disarm the system
      systemArmed = false;
      //systemFrozen = false;  // Reset frozen state if applicable
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER, LOW);
      Serial.println("System DISARMED by AUTHORIZED RFID Key.");
      sendIFTTTNotification("System DISARMED by AUTHORIZED RFID Key.");
    }
    else if (systemArmed && systemFrozen) {
      // Disarm the system
      //systemArmed = false;
      //systemFrozen = false;  // Reset frozen state if applicable
      resetSystem();
      Serial.println("System has been RESET by AUTHORIZED RFID Key.");
      sendIFTTTNotification("System has been RESET by AUTHORIZED RFID Key.");
    }
    else if (!systemArmed) {
      // Arm the system
      systemArmed = true;
      systemFrozen = false;
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(RED_LED, LOW);
      digitalWrite(BUZZER, LOW);
      Serial.println("System ARMED by AUTHORIZED RFID Key.");
      sendIFTTTNotification("System ARMED by AUTHORIZED RFID Key.");
    }
  }
  else {
    Serial.println("UNAUTHORIZED RFID Key Scanned: ACCESS DENIED!");
    sendIFTTTNotification("UNAUTHORIZED RFID Key Scanned: ACCESS DENIED!");
  }
}

void resetSystem() {
  systemFrozen = false;
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER, LOW);
}

bool motionDetected() {
  int motionState1 = digitalRead(MOTION_SENSOR_PIN1);
  int motionState2 = digitalRead(MOTION_SENSOR_PIN2);
  int motionState3 = digitalRead(MOTION_SENSOR_PIN3);
  int motionState4 = digitalRead(MOTION_SENSOR_PIN4);

  if (motionState1 == HIGH) {
    Serial.println("MOTION DETECTED at Sensor 1");
    delay(250);
    return true;
  } else if (motionState2 == HIGH) {
    Serial.println("MOTION DETECTED at Sensor 2");
    delay(250);
    return true;
  } else if (motionState3 == HIGH) {
    Serial.println("MOTION DETECTED at Sensor 3");
    delay(250);
    return true;
  } else if (motionState4 == HIGH) {
    Serial.println("MOTION DETECTED at Sensor 4");
    delay(250);
    return true;
  } else {
    return false;
  }
}

void readDistances() {
  distance1 = measureDistance(TRIGGER_PIN1, ECHO_PIN1);
  distance2 = measureDistance(TRIGGER_PIN2, ECHO_PIN2);
  distance3 = measureDistance(TRIGGER_PIN3, ECHO_PIN3);
  distance4 = measureDistance(TRIGGER_PIN4, ECHO_PIN4);
}

int measureDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration / 58;
}

int findMinDistance() {
  int distances[] = {distance1, distance2, distance3, distance4};
  int minDist = distances[0];
  for (int i = 1; i < 4; i++) {
    minDist = min(minDist, distances[i]);
  }
  return minDist;
}

boolean getID() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return false;
  }

  if (!mfrc522.PICC_ReadCardSerial()) {
    return false;
  }

  tagID = "";
  for (uint8_t i = 0; i < 4; i++) {
    tagID.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  tagID.toUpperCase();
  mfrc522.PICC_HaltA();
  return true;
}

void connectToNetwork() {
  // Set connection type to GPRS
  Serial1.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  delay(2000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  // Set APN
  Serial1.println("AT+SAPBR=3,1,\"APN\",\"internet\"");
  delay(2000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  // Retry mechanism
  int retryCount = 0;
  while (retryCount < 3) {
    Serial1.println("AT+SAPBR=1,1");
    delay(3000);
    if (Serial1.find("OK")) {
      break;
    } else {
      retryCount++;
      Serial.println("Retrying GPRS context setup...");
    }
  }

  // Verify GPRS context
  Serial1.println("AT+SAPBR=2,1");
  delay(1000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("\nGPRS Connection Setup Complete\n");
}

void sendIFTTTNotification(String message) {
  String url = "http://18.141.143.81/ifttt_trigger?event=RFID_access&value1=" + message;

  Serial.println("Initializing HTTP...");
  Serial1.println("AT+HTTPINIT");
  delay(3000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("Setting HTTP SSL...");
  Serial1.println("AT+HTTPSSL=0");
  delay(3000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("Setting HTTP parameters...");
  Serial1.println("AT+HTTPPARA=\"CID\",1");
  delay(3000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("Setting URL...");
  Serial1.println("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  delay(6000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("Starting HTTP GET request...");
  Serial1.println("AT+HTTPACTION=0");  // 0 = GET
  delay(6000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("Reading HTTP response...");
  Serial1.println("AT+HTTPREAD");
  delay(2000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("Terminating HTTP...");
  Serial1.println("AT+HTTPTERM");
  delay(3000);
  while (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  Serial.println("HTTP request complete.");
}
