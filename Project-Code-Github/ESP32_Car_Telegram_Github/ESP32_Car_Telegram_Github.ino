#include <WiFi.h>
#include <WiFiUDP.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

const char* AP_SSID        = "";
const char* AP_PASSWORD    = "";

const char* HOME_SSID      = "";
const char* HOME_PASSWORD  = "";

const char* BOT_TOKEN      = "";

const char* CHAT_ID        = "";

const char* OWNER_NAME     = "";
const char* LOCATION       = "";

const int UDP_PORT = 4210;
WiFiUDP udp;

#define CMD_STOP    'S'
#define CMD_FORWARD 'F'
#define CMD_LEFT    'L'
#define CMD_RIGHT   'R'

#define ENA  26
#define IN1  27
#define IN2  14
#define IN3  12
#define IN4  13
#define ENB  25

#define PWM_FREQ     5000
#define PWM_RES      8

#define PWM_LEFT_FWD    130
#define PWM_RIGHT_FWD   130
#define PWM_LEFT_TURN   160
#define PWM_RIGHT_TURN  160

#define TRIG_PIN             18
#define ECHO_PIN             19
#define OBSTACLE_DISTANCE_CM 20
#define ULTRASONIC_CHECK_MS  50
#define ULTRASONIC_CONFIRM   3

Adafruit_MPU6050 mpu;
#define TILT_THRESHOLD_DEG  45.0f
#define TILT_CONFIRM_MS     300
#define ALERT_COOLDOWN_MS   30000
#define MPU_CHECK_MS        50

unsigned long lastMPUCheckMs  = 0;
unsigned long tiltStartMs     = 0;
unsigned long lastAlertMs     = 0;
bool inTiltWindow             = false;
bool mpuReady                 = false;

#define TURN_DURATION_MS 300
#define BAUD_RATE        115200

bool obstacleDetected     = false;
int  obstacleConfirmCount = 0;
unsigned long lastUltrasonicCheckMs = 0;
char currentCmd     = CMD_STOP;
bool focusActive    = false;
bool turning        = false;
int  pendingTurnDir = 0;
unsigned long turnStartTime = 0;

String urlEncode(String str) {
  String encoded = "";
  for (int i = 0; i < (int)str.length(); i++) {
    char c = str.charAt(i);
    if      (c == ' ')  encoded += "%20";
    else if (c == '\n') encoded += "%0A";
    else if (c == '!')  encoded += "%21";
    else if (c == '"')  encoded += "%22";
    else if (c == '#')  encoded += "%23";
    else if (c == '&')  encoded += "%26";
    else if (c == '+')  encoded += "%2B";
    else if (c == ':')  encoded += "%3A";
    else if (c == '=')  encoded += "%3D";
    else if (c == '?')  encoded += "%3F";
    else                encoded += c;
  }
  return encoded;
}

void sendTelegram(String message) {
  Serial.println("Switching to mobile hotspot for Telegram alert...");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.mode(WIFI_STA);
  delay(200);

  WiFi.begin(HOME_SSID, HOME_PASSWORD);
  int attempts = 0;
  Serial.print("Connecting to hotspot");
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500); Serial.print("."); attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nHotspot connected — IP: " + WiFi.localIP().toString());

    Serial.print("Token length: "); Serial.println(strlen(BOT_TOKEN));
    Serial.print("Chat ID: "); Serial.println(CHAT_ID);

    WiFiClientSecure client;
    client.setInsecure();

    String url = "https://api.telegram.org/bot"
                 + String(BOT_TOKEN)
                 + "/sendMessage?chat_id="
                 + String(CHAT_ID)
                 + "&text="
                 + urlEncode(message);

    Serial.print("URL length: "); Serial.println(url.length());

    HTTPClient http;
    http.begin(client, url);
    http.setTimeout(10000);
    int code = http.GET();

    if (code == 200) {
      Serial.println("Telegram alert sent successfully!");
    } else {
      Serial.print("Telegram failed. HTTP code: ");
      Serial.println(code);
      Serial.println(http.getString());
    }
    http.end();

  } else {
    Serial.println("Mobile hotspot unreachable — check hotspot is ON");
    Serial.print("HOME_SSID: "); Serial.println(HOME_SSID);
  }

  Serial.print("Reconnecting to BCI_CAR AP");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500);
  WiFi.mode(WIFI_STA);
  delay(200);
  WiFi.begin(AP_SSID, AP_PASSWORD);
  attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500); Serial.print("."); attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    udp.begin(UDP_PORT);
    Serial.println(" — BCI control restored");
  } else {
    Serial.println(" — reconnect failed, restart device");
  }
}

void pollMPU(unsigned long nowMs) {
  if (!mpuReady) return;
  if ((nowMs - lastMPUCheckMs) < MPU_CHECK_MS) return;
  lastMPUCheckMs = nowMs;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float roll  = atan2(ay, az) * 180.0f / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;

  bool tilted = (fabs(roll)  > TILT_THRESHOLD_DEG ||
                 fabs(pitch) > TILT_THRESHOLD_DEG);

  if (tilted) {
    if (!inTiltWindow) {
      inTiltWindow = true;
      tiltStartMs  = nowMs;
      Serial.print("Tilt detected — Roll: "); Serial.print(roll, 1);
      Serial.print("  Pitch: "); Serial.println(pitch, 1);
    } else if ((nowMs - tiltStartMs) >= TILT_CONFIRM_MS) {
      if ((nowMs - lastAlertMs) >= ALERT_COOLDOWN_MS) {
        lastAlertMs = nowMs;
        Serial.println("ACCIDENT CONFIRMED — sending Telegram alert!");

        stopMotors();
        focusActive    = false;
        turning        = false;
        pendingTurnDir = 0;
        currentCmd     = CMD_STOP;

        String msg = "ACCIDENT ALERT!\n\n";
        msg += "Owner: "    + String(OWNER_NAME) + "\n";
        msg += "Location: " + String(LOCATION)   + "\n\n";
        msg += "The BCI car has tipped over!\n";
        msg += "Roll:  "  + String(roll,  1) + " degrees\n";
        msg += "Pitch: "  + String(pitch, 1) + " degrees\n\n";
        msg += "Please check the vehicle immediately!";

        sendTelegram(msg);
      } else {
        Serial.println("Alert cooldown active — skipping");
      }
      inTiltWindow = false;
    }
  } else {
    inTiltWindow = false;
  }
}

void stopMotors() {
  ledcWrite(ENA, 0); ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void driveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ENA, PWM_LEFT_FWD);
  ledcWrite(ENB, PWM_RIGHT_FWD);
}

void doTurnLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, PWM_RIGHT_TURN);
}

void doTurnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
  ledcWrite(ENA, PWM_LEFT_TURN);
  ledcWrite(ENB, 0);
}

bool isSafeToMove() {
  if (turning) return true;
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration == 0) return true;
  float dist = (duration / 2.0f) * 0.0343f;
  return !(dist > 0 && dist <= OBSTACLE_DISTANCE_CM);
}

void pollUltrasonic(unsigned long nowMs) {
  if (turning) return;
  if ((nowMs - lastUltrasonicCheckMs) < ULTRASONIC_CHECK_MS) return;
  lastUltrasonicCheckMs = nowMs;

  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  bool reading = false;
  if (duration > 0) {
    float dist = (duration / 2.0f) * 0.0343f;
    reading = (dist > 0 && dist <= OBSTACLE_DISTANCE_CM);
  }

  if (reading) { if (obstacleConfirmCount < ULTRASONIC_CONFIRM) obstacleConfirmCount++; }
  else         { if (obstacleConfirmCount > 0) obstacleConfirmCount--; }

  bool wasDetected = obstacleDetected;
  obstacleDetected = (obstacleConfirmCount >= ULTRASONIC_CONFIRM);

  if (!wasDetected && obstacleDetected) {
    stopMotors(); focusActive = false; turning = false;
    pendingTurnDir = 0; currentCmd = CMD_STOP;
    Serial.println("OBSTACLE DETECTED — All motors stopped!");
  }
  if (wasDetected && !obstacleDetected) {
    Serial.println("Path clear — BCI control resumed");
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(100);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
  stopMotors();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin(21, 22);
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpuReady = true;
    Serial.println("MPU6050 ready");
  } else {
    Serial.println("MPU6050 not found — check SDA=21 SCL=22 VCC=3.3V");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASSWORD);
  Serial.print("Connecting to BCI_CAR");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300); Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected — IP: "); Serial.println(WiFi.localIP());
  udp.begin(UDP_PORT);

  Serial.println("================================");
  Serial.println("ESP32 BCI Car Ready");
  Serial.print("Forward  PWM — L:"); Serial.print(PWM_LEFT_FWD);
  Serial.print("  R:"); Serial.println(PWM_RIGHT_FWD);
  Serial.print("Turn     PWM — L:"); Serial.print(PWM_LEFT_TURN);
  Serial.print("  R:"); Serial.println(PWM_RIGHT_TURN);
  Serial.print("Tilt threshold : "); Serial.print(TILT_THRESHOLD_DEG); Serial.println(" deg");
  Serial.print("Obstacle stop  : "); Serial.print(OBSTACLE_DISTANCE_CM); Serial.println(" cm");
  Serial.println("================================");
}

void loop() {
  unsigned long nowMs = millis();

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char cmd = (char)udp.read();
    Serial.print("RX: "); Serial.println(cmd);
    if      (cmd == CMD_FORWARD) { currentCmd = CMD_FORWARD; focusActive = true; }
    else if (cmd == CMD_STOP)    { currentCmd = CMD_STOP; focusActive = false; stopMotors(); }
    else if (cmd == CMD_LEFT)    { pendingTurnDir = 1; }
    else if (cmd == CMD_RIGHT)   { pendingTurnDir = 2; }
  }

  pollMPU(nowMs);

  pollUltrasonic(nowMs);

  if (focusActive && currentCmd == CMD_FORWARD) {
    if (!obstacleDetected && !turning) driveForward();
    else if (obstacleDetected) {
      focusActive = false; currentCmd = CMD_STOP;
      stopMotors(); Serial.println("Focus blocked — obstacle!");
    }
  }

  if (focusActive && obstacleDetected) {
    focusActive = false; currentCmd = CMD_STOP;
    stopMotors(); Serial.println("Obstacle mid-drive — Emergency Stop!");
  }

  if (pendingTurnDir != 0 && !turning) {
    if (!obstacleDetected) {
      if (pendingTurnDir == 1) { Serial.println("LEFT");  doTurnLeft();  }
      else                     { Serial.println("RIGHT"); doTurnRight(); }
      turnStartTime = millis(); turning = true;
    } else {
      Serial.println("Turn blocked — obstacle!");
    }
    pendingTurnDir = 0;
  }

  if (turning && (millis() - turnStartTime >= TURN_DURATION_MS)) {
    stopMotors(); turning = false; Serial.println("Turn complete");
  }

  if (turning && obstacleDetected) {
    stopMotors(); turning = false;
    Serial.println("Obstacle mid-turn — Emergency Stop!");
  }
}
