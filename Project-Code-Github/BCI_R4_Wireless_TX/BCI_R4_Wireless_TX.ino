#include <Arduino.h>
#include "arm_math.h"
#include <math.h>
#include <SoftwareSerial.h>

SoftwareSerial espSerial(7, 8);
#define ESP_BAUD 9600

#define CMD_STOP    'S'
#define CMD_FORWARD 'F'
#define CMD_LEFT    'L'
#define CMD_RIGHT   'R'

#define SAMPLE_RATE       512
#define FFT_SIZE          512
#define BAUD_RATE         115200
#define INPUT_PIN         A0
#define LED_PIN           LED_BUILTIN

#define ENVELOPE_WINDOW_MS 100
#define ENVELOPE_WINDOW_SIZE ((ENVELOPE_WINDOW_MS * SAMPLE_RATE) / 1000)
#define SEGMENT_SEC 1
#define SAMPLES_PER_SEGMENT (SAMPLE_RATE * SEGMENT_SEC)

const float BETA_THRESHOLD        = 7.0;
const float BLINK_LOWER_THRESHOLD = 25.0;
const float BLINK_UPPER_THRESHOLD = 50.0;

const unsigned long FOCUS_DEBOUNCE_MS      = 100;
const unsigned long BLINK_DEBOUNCE_MS      = 200;
const unsigned long DOUBLE_BLINK_WINDOW_MS = 500;
const unsigned long TRIPLE_BLINK_WINDOW_MS = 700;

#define DELTA_LOW    0.5f
#define DELTA_HIGH   4.0f
#define THETA_LOW    4.0f
#define THETA_HIGH   8.0f
#define ALPHA_LOW    8.0f
#define ALPHA_HIGH   13.0f
#define BETA_LOW     13.0f
#define BETA_HIGH    30.0f
#define GAMMA_LOW    30.0f
#define GAMMA_HIGH   45.0f

#define SMOOTHING_FACTOR 0.63f
#define EPS              1e-7f

typedef struct {
  float delta, theta, alpha, beta, gamma, total;
} BandpowerResults;

#define BUFFER_SIZE 64
float eegCircBuffer[BUFFER_SIZE];
float eogCircBuffer[BUFFER_SIZE];
int writeIndex = 0;
int readIndex  = 0;
int samplesAvailable = 0;

float eegBuffer[SAMPLES_PER_SEGMENT] = {0};
float eogBuffer[SAMPLES_PER_SEGMENT] = {0};
uint16_t segmentIndex = 0;
unsigned long lastSegmentTimeMs = 0;
float eegAvg=0, eogAvg=0;
float eegMin=0, eegMax=0, eogMin=0, eogMax=0;

unsigned long lastFocusTime = 0;
bool focusKeyPressed = false;
float BetaPower = 0;

unsigned long lastBlinkTime   = 0;
unsigned long firstBlinkTime  = 0;
unsigned long secondBlinkTime = 0;
int blinkCount = 0;
float currentEOGEnvelope = 0;
int pendingTurnDirection = 0;

char lastSentCmd = CMD_STOP;

float eogEnvelopeBuffer[ENVELOPE_WINDOW_SIZE] = {0};
int   eogEnvelopeIndex = 0;
float eogEnvelopeSum   = 0;

float inputBuffer[FFT_SIZE];
float fftOutputBuffer[FFT_SIZE];
float powerSpectrum[FFT_SIZE / 2];
BandpowerResults smoothedPowers = {0};
arm_rfft_fast_instance_f32 S;

void sendCommand(char cmd) {
  if (cmd == CMD_FORWARD || cmd == CMD_STOP) {
    if (cmd == lastSentCmd) return;
    lastSentCmd = cmd;
  } else {
    lastSentCmd = CMD_STOP;
  }
  espSerial.write((uint8_t)cmd);
  Serial.print("TX: "); Serial.println(cmd);
}

float Notch(float input) {
  float output = input;
  { static float z1, z2;
    float x = output - -1.58696045*z1 - 0.96505858*z2;
    output = 0.96588529*x + -1.57986211*z1 + 0.96588529*z2;
    z2=z1; z1=x; }
  { static float z1, z2;
    float x = output - -1.62761184*z1 - 0.96671306*z2;
    output = 1.00000000*x + -1.63566226*z1 + 1.00000000*z2;
    z2=z1; z1=x; }
  return output;
}

float EEGFilter(float input) {
  float output = input;
  { static float z1, z2;
    float x = output - -1.24200128*z1 - 0.45885207*z2;
    output = 0.05421270*x + 0.10842539*z1 + 0.05421270*z2;
    z2=z1; z1=x; }
  return output;
}

float EOGFilter(float input) {
  float output = input;
  { static float z1, z2;
    float x = output - -1.91327599*z1 - 0.91688335*z2;
    output = 0.95753983*x + -1.91507967*z1 + 0.95753983*z2;
    z2=z1; z1=x; }
  return output;
}

float updateEOGEnvelope(float sample) {
  float absSample = fabs(sample);
  eogEnvelopeSum -= eogEnvelopeBuffer[eogEnvelopeIndex];
  eogEnvelopeSum += absSample;
  eogEnvelopeBuffer[eogEnvelopeIndex] = absSample;
  eogEnvelopeIndex = (eogEnvelopeIndex + 1) % ENVELOPE_WINDOW_SIZE;
  return eogEnvelopeSum / ENVELOPE_WINDOW_SIZE;
}

BandpowerResults calculateBandpower(float *ps, float binRes, int halfSize) {
  BandpowerResults r = {0};
  for (int i = 1; i < halfSize; i++) {
    float freq = i * binRes, p = ps[i];
    r.total += p;
    if      (freq >= DELTA_LOW && freq < DELTA_HIGH) r.delta += p;
    else if (freq >= THETA_LOW && freq < THETA_HIGH) r.theta += p;
    else if (freq >= ALPHA_LOW && freq < ALPHA_HIGH) r.alpha += p;
    else if (freq >= BETA_LOW  && freq < BETA_HIGH)  r.beta  += p;
    else if (freq >= GAMMA_LOW && freq < GAMMA_HIGH) r.gamma += p;
  }
  return r;
}

void smoothBandpower(const BandpowerResults *raw, BandpowerResults *s) {
  s->delta = SMOOTHING_FACTOR*raw->delta + (1-SMOOTHING_FACTOR)*s->delta;
  s->theta = SMOOTHING_FACTOR*raw->theta + (1-SMOOTHING_FACTOR)*s->theta;
  s->alpha = SMOOTHING_FACTOR*raw->alpha + (1-SMOOTHING_FACTOR)*s->alpha;
  s->beta  = SMOOTHING_FACTOR*raw->beta  + (1-SMOOTHING_FACTOR)*s->beta;
  s->gamma = SMOOTHING_FACTOR*raw->gamma + (1-SMOOTHING_FACTOR)*s->gamma;
  s->total = SMOOTHING_FACTOR*raw->total + (1-SMOOTHING_FACTOR)*s->total;
}

void initFFT() {
  arm_status status = arm_rfft_fast_init_f32(&S, FFT_SIZE);
  if (status != ARM_MATH_SUCCESS) {
    Serial.println("FFT init failed"); while (1) delay(10);
  }
}

void processFFT() {
  arm_rfft_fast_f32(&S, inputBuffer, fftOutputBuffer, 0);
  const int half = FFT_SIZE / 2;
  powerSpectrum[0] = fftOutputBuffer[0] * fftOutputBuffer[0];
  for (int i = 1; i < half; i++) {
    const float re = fftOutputBuffer[2*i];
    const float im = fftOutputBuffer[2*i+1];
    powerSpectrum[i] = re*re + im*im;
  }
  float binRes = float(SAMPLE_RATE) / FFT_SIZE;
  BandpowerResults raw = calculateBandpower(powerSpectrum, binRes, half);
  smoothBandpower(&raw, &smoothedPowers);
  BetaPower = (smoothedPowers.beta / (smoothedPowers.total + EPS)) * 100;
}

void setup() {
  Serial.begin(BAUD_RATE);
  delay(100);
  pinMode(INPUT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  espSerial.begin(ESP_BAUD);
  delay(200);

  digitalWrite(LED_PIN, HIGH); delay(300); digitalWrite(LED_PIN, LOW);
  initFFT();
  lastSegmentTimeMs = millis();
  Serial.println("BCI Wireless TX Started");
}

void loop() {
  static uint16_t idx = 0;
  static unsigned long lastMicros = 0;
  static long timer = 0;

  digitalWrite(LED_PIN, LOW);

  unsigned long currentMicros = micros();
  unsigned long interval = currentMicros - lastMicros;
  lastMicros = currentMicros;

  timer -= interval;
  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    int raw      = analogRead(INPUT_PIN);
    float filtered = Notch(raw);
    float eeg    = EEGFilter(filtered);
    float eog    = EOGFilter(eeg);
    eegCircBuffer[writeIndex] = eeg;
    eogCircBuffer[writeIndex] = eog;
    writeIndex = (writeIndex + 1) % BUFFER_SIZE;
    samplesAvailable++;
    if (samplesAvailable > BUFFER_SIZE) {
      samplesAvailable = BUFFER_SIZE;
      readIndex = (readIndex + 1) % BUFFER_SIZE;
    }
  }

  while (samplesAvailable > 0) {
    float eeg = eegCircBuffer[readIndex];
    float eog = eogCircBuffer[readIndex];
    readIndex = (readIndex + 1) % BUFFER_SIZE;
    samplesAvailable--;
    currentEOGEnvelope = updateEOGEnvelope(eog);
    inputBuffer[idx++] = eeg;
    if (segmentIndex < SAMPLES_PER_SEGMENT) {
      eegBuffer[segmentIndex] = BetaPower;
      eogBuffer[segmentIndex] = currentEOGEnvelope;
      segmentIndex++;
    }
    if (idx >= FFT_SIZE) { processFFT(); idx = 0; }
  }

  unsigned long nowMs = millis();

  if ((nowMs - lastSegmentTimeMs) >= (1000UL * SEGMENT_SEC)) {
    if (segmentIndex > 0) {
      eegMin = eegBuffer[0]; eegMax = eegBuffer[0];
      eogMin = eogBuffer[0]; eogMax = eogBuffer[0];
      float eegSum = 0, eogSum = 0;
      for (uint16_t i = 0; i < segmentIndex; i++) {
        float ev = eegBuffer[i], ov = eogBuffer[i];
        if (ev < eegMin) eegMin=ev; if (ev > eegMax) eegMax=ev; eegSum+=ev;
        if (ov < eogMin) eogMin=ov; if (ov > eogMax) eogMax=ov; eogSum+=ov;
      }
      eegAvg = eegSum / segmentIndex;
      eogAvg = eogSum / segmentIndex;
      #ifdef DEBUG
      Serial.print("Beta: "); Serial.print(BetaPower);
      Serial.print(" (Avg: "); Serial.print(eegAvg);
      Serial.print(", Min: "); Serial.print(eegMin);
      Serial.print(", Max: "); Serial.print(eegMax); Serial.print(")");
      Serial.print(" | EOG: (Avg: "); Serial.print(eogAvg);
      Serial.print(", Min: "); Serial.print(eogMin);
      Serial.print(", Max: "); Serial.print(eogMax); Serial.println(")");
      #endif
    }
    lastSegmentTimeMs = nowMs;
    segmentIndex = 0;
  }

  if (BetaPower > BETA_THRESHOLD) {
    if (!focusKeyPressed && (nowMs - lastFocusTime) >= FOCUS_DEBOUNCE_MS) {
      lastFocusTime = nowMs;
      focusKeyPressed = true;
      digitalWrite(LED_PIN, HIGH);
      sendCommand(CMD_FORWARD);
      Serial.println("Focus - Forward");
    }
  } else {
    if (focusKeyPressed) {
      focusKeyPressed = false;
      digitalWrite(LED_PIN, LOW);
      sendCommand(CMD_STOP);
      Serial.println("Focus released - Stopped");
    }
  }

  if (currentEOGEnvelope > BLINK_LOWER_THRESHOLD &&
      currentEOGEnvelope < BLINK_UPPER_THRESHOLD &&
      (nowMs - lastBlinkTime) >= BLINK_DEBOUNCE_MS) {

    lastBlinkTime = nowMs;

    if (blinkCount == 0) {
      firstBlinkTime = nowMs; blinkCount = 1;
    }
    else if (blinkCount == 1 && (nowMs - firstBlinkTime) <= DOUBLE_BLINK_WINDOW_MS) {
      secondBlinkTime = nowMs; blinkCount = 2;
    }
    else if (blinkCount == 2 && (nowMs - firstBlinkTime) <= TRIPLE_BLINK_WINDOW_MS) {
      blinkCount = 0;
      pendingTurnDirection = 2;
    }
    else {
      firstBlinkTime = nowMs; blinkCount = 1;
    }
  }

  if (blinkCount == 2 && (nowMs - firstBlinkTime) > TRIPLE_BLINK_WINDOW_MS) {
    blinkCount = 0;
    pendingTurnDirection = 1;
  }

  if (blinkCount == 1 && (nowMs - firstBlinkTime) > DOUBLE_BLINK_WINDOW_MS) {
    blinkCount = 0;
  }

  if (pendingTurnDirection != 0) {
    if (pendingTurnDirection == 1) {
      Serial.println("Double Blink - Left");
      sendCommand(CMD_LEFT);
    } else if (pendingTurnDirection == 2) {
      Serial.println("Triple Blink - Right");
      sendCommand(CMD_RIGHT);
    }
    pendingTurnDirection = 0;
  }
}
