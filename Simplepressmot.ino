```cpp
#include <Servo.h>
#include <ZooUART_MEGA.h>
#include <math.h>

/* ─── DEVICE / NETWORK CONFIG ───────────────────── */
const int deviceID  = 7;                   // This board’s unique ZooUART ID
const int numServos = 9;                   // Number of servos used
const int servoPins[numServos]  = {2, 3, 4, 5, 6, 7, 8, 9, 10}; // Digital pins for servos

const int NUM_SENSORS = 9;                 // Number of pressure sensors
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7, A8}; // Analog pins for sensors

/* ─── SENSOR CONFIG ─────────────────────────────── */
const int SENSOR_THRESHOLD    = 100;       // Minimum delta from baseline to register a press
const unsigned long WINDOW_MS = 5000;      // Time window (ms) to capture peak after press

/* ─── SENSOR STATE VARIABLES ────────────────────── */
int baseline[NUM_SENSORS];                 // Baseline (idle) values per sensor

bool          peakMode      = false;       // Whether we are currently in a peak-capturing window
int           activeSensor  = -1;          // Which sensor triggered the window
unsigned long windowStart   = 0;           // When the window started
int           peakReading   = 0;           // Highest value recorded during window

/* ─── SERVO CONFIG ─────────────────────────────── */
int initialAngle[numServos] = {            // Manually defined initial servo angles
  0, 0, 0, 0, 0, 0, 0, 0, 0
};

const int MAX_OFFSET = 90;                 // Maximum allowed offset from initial angle

/* ─── OBJECTS ───────────────────────────────────── */
Servo   servos[numServos];
ZooUART zoo(deviceID);

/* ─── SETUP ─────────────────────────────────────── */
void setup() {
  Serial.begin(9600);
  zoo.begin();

  // Attach all servos at their defined initial positions
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(initialAngle[i]);
    delay(20);
  }
  Serial.println(F("Servos attached at user‑defined initial angles."));

  // Register message handlers for network communication
  zoo.onGlobalMessage(GlobalMessageHandler);
  zoo.onDirectMessage(DirectMessageHandler);
    
  // Record sensor baselines (initial idle readings)
  for (int i = 0; i < NUM_SENSORS; i++) {
    baseline[i] = analogRead(sensorPins[i]);
    delay(5);
  }
  Serial.println(F("Pressure baselines recorded."));
}

/* ─── MAIN LOOP ─────────────────────────────────── */
void loop() {
  if (!peakMode) {
    zoo.watch();           // Keep listening for incoming messages
    checkSensors();        // Check if any sensor is triggered
  } else {
    capturePeak();         // If in peak mode, keep recording until window ends
  }
}

/* ─── MESSAGE HANDLERS ─────────────────────────── */
// Handle global messages from the network
void GlobalMessageHandler(int sender, int value) {
  Serial.print(F("GLOBAL from ")); Serial.print(sender);
  Serial.print(F(" | Intensity ")); Serial.println(value);
  triggerPattern(sender, value);
}

// Handle direct messages from the network
void DirectMessageHandler(int sender, int value) {
  Serial.print(F("DIRECT from ")); Serial.print(sender);
  Serial.print(F(" | Intensity ")); Serial.println(value);
  triggerPattern(sender, value);
}

/* ─── SERVO PATTERN DISPATCH ───────────────────── */
// Trigger motor movement patterns based on sender ID
void triggerPattern(int sender, int value) {
  int id           = sender % numServos;                   // Map sender to a servo index
  int angleOffset = map(value, 0, 255, 15, 60);            // Map intensity to movement offset

  switch (id) {
    case 0: pulseAll(angleOffset);        break;
    case 1: waveLeftToRight(angleOffset); break;
    case 2: waveRightToLeft(angleOffset); break;
    case 3: centerOutward(angleOffset);   break;
    case 4: alternateEvenOdd(angleOffset);break;
    case 5: sequentialUp(angleOffset);    break;
    case 6: sequentialDown(angleOffset);  break;
    case 7: bounceEnds(angleOffset);      break;
    case 8: sineOscillation(angleOffset); break;
  }
}

/* ─── SERVO MOVEMENT HELPERS ───────────────────── */
// Clamp servo movement within safe offset limits
int clampAngle(int base, int angle) {
  int lowerBound = max(0, base - MAX_OFFSET);
  int upperBound = min(180, base + MAX_OFFSET);
  if (angle < lowerBound) return lowerBound;
  if (angle > upperBound) return upperBound;
  return angle;
}

// Pattern: All servos pulse together
void pulseAll(int offset) {
  for (int r = 0; r < 2; r++) {
    for (int i = 0; i < numServos; i++)
      servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + offset));
    delay(100);
    resetServos();
    delay(100);
  }
}

// Pattern: Wave left to right
void waveLeftToRight(int offset) {
  for (int i = 0; i < numServos; i++) {
    servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + offset));
    delay(60);
    servos[i].write(initialAngle[i]);
  }
}

// Pattern: Wave right to left
void waveRightToLeft(int offset) {
  for (int i = numServos - 1; i >= 0; i--) {
    servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + offset));
    delay(60);
    servos[i].write(initialAngle[i]);
  }
}

// Pattern: Expand from center outward
void centerOutward(int offset) {
  int mid = numServos / 2;
  for (int r = 0; r <= mid; r++) {
    if (mid - r >= 0)
      servos[mid - r].write(clampAngle(initialAngle[mid - r], initialAngle[mid - r] + offset));
    if (mid + r < numServos)
      servos[mid + r].write(clampAngle(initialAngle[mid + r], initialAngle[mid + r] + offset));
    delay(60);
  }
  delay(100);
  resetServos();
}

// Pattern: Alternate even and odd servos
void alternateEvenOdd(int offset) {
  for (int r = 0; r < 2; r++) {
    for (int i = 0; i < numServos; i++) {
      int o = (i % 2 == 0) ? offset : -offset;
      servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + o));
    }
    delay(90);
    resetServos();
    delay(90);
  }
}

// Pattern: Sequential up
void sequentialUp(int offset) {
  for (int i = 0; i < numServos; i++) {
    servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + offset));
    delay(60);
    servos[i].write(initialAngle[i]);
  }
}

// Pattern: Sequential down
void sequentialDown(int offset) {
  for (int i = numServos - 1; i >= 0; i--) {
    servos[i].write(clampAngle(initialAngle[i], initialAngle[i] - offset));
    delay(60);
    servos[i].write(initialAngle[i]);
  }
}

// Pattern: Bounce between end servos
void bounceEnds(int offset) {
  for (int r = 0; r < 2; r++) {
    servos[0].write(clampAngle(initialAngle[0], initialAngle[0] + offset));
    servos[numServos - 1].write(clampAngle(initialAngle[numServos - 1], initialAngle[numServos - 1] - offset));
    delay(120);
    resetServos();
    delay(120);
  }
}

// Pattern: Oscillating sine wave motion
void sineOscillation(int offset) {
  for (int t = 0; t <= 180; t += 6) {
    float rad = t * PI / 180.0;
    for (int i = 0; i < numServos; i++) {
      float dynamicOffset = sin(rad + i * 0.35) * offset;
      int pos = clampAngle(initialAngle[i], initialAngle[i] + (int)dynamicOffset);
      servos[i].write(pos);
    }
    delay(20);
  }
  resetServos();
}

// Reset all servos back to initial angles
void resetServos() {
  for (int i = 0; i < numServos; i++)
    servos[i].write(initialAngle[i]);
}

/* ─── SENSOR CHECKING & MESSAGE SENDING ────────── */
// Look for a sensor press (trigger event)
void checkSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);
    int diff = raw - baseline[i];
    if (diff > SENSOR_THRESHOLD) {      
      peakMode     = true;                // Enter peak capture mode
      activeSensor = i;                   // Mark which sensor triggered
      windowStart  = millis();            // Start capture window
      peakReading  = diff;                // Initialize peak with first reading
      Serial.print(F("Sensor ")); Serial.print(i);
      Serial.println(F(" triggered → entering 5s window"));
      return;                             // Exit loop after first trigger
    }
  }
}

// During the 5s window, record peak value and send message
void capturePeak() {
  int raw  = analogRead(sensorPins[activeSensor]);
  int diff = raw - baseline[activeSensor];
  if (diff > peakReading) peakReading = diff;  // Track highest value

  if (millis() - windowStart >= WINDOW_MS) {   // When window ends
    int intensity = map(peakReading, 0, 1023, 0, 255);
    intensity = constrain(intensity, 0, 255);

    // Construct and send ZooUART message to corresponding device
    String msg = zoo.createMessage(
        /*ping*/   false,
        /*global*/ false,
        /*address*/ activeSensor,   
        /*value*/   intensity,
        /*sender*/  deviceID);

    zoo.sendMessage(msg);           
    Serial.print(F("Sent intensity ")); Serial.print(intensity);
    Serial.print(F(" to device ")); Serial.println(activeSensor);

    // Reset state after sending
    peakMode      = false;          
    activeSensor  = -1;
  }
}
```
