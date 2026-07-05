#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <Firebase_ESP_Client.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// -------- WIFI --------
#define WIFI_SSID "4G-UFI"
#define WIFI_PASSWORD "12345678"
#define ledbrd 2

#define RXD1 32
#define TXD1 33
// -------- SENSORS & ACTUATORS --------
#define VIBRATION_PIN 25  // SW-420 Data Pin
#define RELAY_PIN 26      // Relay Module Pin

// -------- FIREBASE --------
#define API_KEY "AIzaSyBhbEVMDlnByC9kwB95RcO7kuswvNlZVlw"
#define DATABASE_URL "https://tugas-akhir-74754-default-rtdb.firebaseio.com/"
#define FIREBASE_PROJECT_ID "tugas-akhir-74754"

// -------- GPS --------
#define RXD2 16
#define TXD2 17

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// -------- MPU6050 --------
Adafruit_MPU6050 mpu;

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Timing & Tracking variables
unsigned long lastSend = 0;
const unsigned long interval = 5000;  // Upload live data to RTDB every 5s

unsigned long lastPathSend = 0;
const unsigned long pathInterval = 10000; // RTDB Map Line: Save a point every 10s

unsigned long lastFirestoreSend = 0;
const unsigned long firestoreInterval = 60000; // Firestore History: Save a point every 60s

// --- NEW: Daily Reset Variable ---
int currentDay = -1; 

// Vibration Logic Variables
unsigned long lastVibrationTime = 0;
bool isRelayOn = false;
const int RELAY_HOLD_TIME = 2000;  // Keep relay ON for 2s after vibration stops

// LiDAR Relay Variables
unsigned long lidarRelayStartTime = 0;
bool lidarRelayActive = false;
bool lidarThresholdExceeded = false;
const int LIDAR_THRESHOLD = 400; // 4 meters in cm
const int LIDAR_RELAY_PULSE = 1000; // 1 second pulse

// GPS Status Indicator Variables
unsigned long lastGpsBlink = 0;
const int GPS_BLINK_INTERVAL = 500;  // Blink every 500ms when searching
bool gpsLedState = false;

int dist;     
int strength; 
float temprature;
unsigned char check;
int i;

unsigned char uart[9];       
const int HEADER = 0x59;     
int rec_debug_state = 0x01;  


void setup() {
  Serial.begin(115200);

  // Initialize Pins
  pinMode(VIBRATION_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ledbrd, OUTPUT);

  digitalWrite(RELAY_PIN, LOW); 
  digitalWrite(ledbrd, LOW);    

  // GPS UART
  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //UART LIDAR
  Serial2.begin(115200, SERIAL_8N1, RXD1, TXD1);

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  } else {
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(ledbrd, HIGH);
    delay(500);
    digitalWrite(ledbrd, LOW);
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // Firebase config
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = nullptr;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase auth OK");
  } else {
    Serial.printf("Firebase error: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  Get_Lidar_data();

  // Read GPS (Don't block)
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // GPS Status Indicator
  if (!gps.location.isValid()) {
    if (millis() - lastGpsBlink > GPS_BLINK_INTERVAL) {
      lastGpsBlink = millis();
      gpsLedState = !gpsLedState;
      digitalWrite(ledbrd, gpsLedState);
    }
  } else {
    digitalWrite(ledbrd, HIGH);
  }

  // --- LiDAR Trigger ---
  if (dist > 0 && dist <= LIDAR_THRESHOLD) {
    if (!lidarThresholdExceeded) {
      lidarRelayStartTime = millis();
      lidarRelayActive = true;
      lidarThresholdExceeded = true; 
    }
  } else if (dist > LIDAR_THRESHOLD + 10) { 
    lidarThresholdExceeded = false;
  }

  if (lidarRelayActive && (millis() - lidarRelayStartTime > LIDAR_RELAY_PULSE)) {
    lidarRelayActive = false;
  }

  // --- Vibration Trigger ---
  int vibReading = digitalRead(VIBRATION_PIN);
  bool isVibrationTriggered = false;

  if (vibReading == HIGH) {
    lastVibrationTime = millis();
    isRelayOn = true;
    isVibrationTriggered = true;
  } else {
    if (isRelayOn && (millis() - lastVibrationTime > RELAY_HOLD_TIME)) {
      isRelayOn = false;
    }
    isVibrationTriggered = isRelayOn;
  }

  // --- Final Relay Control ---
  if (isVibrationTriggered || lidarRelayActive) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }

  // ==========================================
  // FIREBASE UPLOAD LOGIC
  // ==========================================
  if (millis() - lastSend > interval) {
    lastSend = millis();

    String vibStatus = isRelayOn ? "VIBRATION DETECTED!" : "Stable";

    // -- Upload MPU6050 & Lidar --
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    FirebaseJson json;
    json.set("ax", a.acceleration.x);
    json.set("ay", a.acceleration.y);
    json.set("az", a.acceleration.z);

    Firebase.RTDB.setString(&fbdo, "/sensor/vibration", vibStatus);
    Firebase.RTDB.updateNode(&fbdo, "/sensor/mpu", &json);
    Firebase.RTDB.setInt(&fbdo, "/lidar/distance", dist);
    Firebase.RTDB.setInt(&fbdo, "/lidar/strength", strength);

    // -- Process GPS Data --
    if (gps.location.isValid() && gps.date.isValid()) {
      double latitude = gps.location.lat();
      double longitude = gps.location.lng();
      float speed = gps.speed.kmph();

      // --- 1. DAILY RESET LOGIC ---
      // Note: gps.date.day() operates in UTC time.
      if (currentDay == -1) {
        currentDay = gps.date.day(); // Set the day when it first gets a signal
      } else if (currentDay != gps.date.day()) {
        // The day has rolled over!
        Serial.println("--- NEW DAY DETECTED! Clearing RTDB Path... ---");
        if (Firebase.RTDB.deleteNode(&fbdo, "/gps/path")) {
          Serial.println("RTDB Path successfully cleared.");
        }
        currentDay = gps.date.day(); // Update to the new day
      }

      // Blink to show upload
      digitalWrite(ledbrd, LOW);
      delay(50);
      digitalWrite(ledbrd, HIGH);

      // --- 2. Live RTDB Overwrite (The moving dot) ---
      Firebase.RTDB.setFloat(&fbdo, "/gps/speed", speed);
      Firebase.RTDB.setDouble(&fbdo, "/gps/latitude", latitude);
      Firebase.RTDB.setDouble(&fbdo, "/gps/longitude", longitude);

      

      // --- 4. FIRESTORE PERMANENT HISTORY (Every 60s if moving) ---
      // Creates a structure like: tracks/2026-05-14/points/[AUTO_ID]
      if (speed > 3.0 && (millis() - lastFirestoreSend > firestoreInterval)) {
        lastFirestoreSend = millis();
        
        char collectionPath[64];
        sprintf(collectionPath, "tracks/%04d-%02d-%02d/points", gps.date.year(), gps.date.month(), gps.date.day());

        FirebaseJson content;
        content.set("fields/latitude/doubleValue", latitude);
        content.set("fields/longitude/doubleValue", longitude);
        content.set("fields/speed/doubleValue", speed);

        char timeBuf[32];
        sprintf(timeBuf, "%04d-%02d-%02dT%02d:%02d:%02dZ", 
                gps.date.year(), gps.date.month(), gps.date.day(),
                gps.time.hour(), gps.time.minute(), gps.time.second());
        content.set("fields/timestamp/timestampValue", timeBuf);

        Serial.printf("Saving point to Firestore: %s\n", collectionPath);
        
        // Push to Firestore Subcollection
        if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", collectionPath, content.raw())) {
          Serial.println("Firestore: Point saved.");
        } else {
          Serial.print("Firestore Error: ");
          Serial.println(fbdo.errorReason());
        }
      } 
    }
  }
}

void Get_Lidar_data() {
  while (Serial2.available()) {
    uint8_t c = Serial2.read();
    switch (rec_debug_state) {
      case 0x01:
        if (c == HEADER) { uart[0] = c; check = c; rec_debug_state = 0x02; }
        break;
      case 0x02:
        if (c == HEADER) { uart[1] = c; check += c; rec_debug_state = 0x03; } 
        else { rec_debug_state = 0x01; }
        break;
      default:
        uart[rec_debug_state - 1] = c;
        if (rec_debug_state < 0x09) { check += c; rec_debug_state++; } 
        else {
          if (c == check) {
            dist = uart[2] + uart[3] * 256;
            strength = uart[4] + uart[5] * 256;
            temprature = (uart[6] + uart[7] * 256) / 8 - 256;
          }
          rec_debug_state = 0x01;
        }
        break;
    }
  }
}