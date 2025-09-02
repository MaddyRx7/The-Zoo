#include <Servo.h>
#include <ZooUART_MEGA.h>
#include <math.h>


const int deviceID  = 7;
const int numServos = 9;
const int servoPins[numServos]  = {2, 3, 4, 5, 6, 7, 8, 9, 10};

const int NUM_SENSORS = 9;
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7, A8};

const int SENSOR_THRESHOLD    = 100;          
const unsigned long WINDOW_MS = 5000;  

int baseline[NUM_SENSORS];   // 


bool          peakMode      = false;
int           activeSensor  = -1;
unsigned long windowStart   = 0;
int           peakReading   = 0;



int initialAngle[numServos] = {
  0, // Servo 0  (pin 2)
  0, // Servo 1  (pin 3)
  0, // Servo 2  (pin 4)
  0,// Servo 3  (pin 5)
  0, // Servo 4  (pin 6)
  0, // Servo 5  (pin 7)
  0,// Servo 6  (pin 8)
  0, // Servo 7  (pin 9)
  0,  // Servo 8  (pin 10)
};

const int MAX_OFFSET = 90; 


Servo   servos[numServos];
ZooUART zoo(deviceID);


void setup() {
  Serial.begin(9600);
  zoo.begin();

  
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(initialAngle[i]);
    delay(20);
  }
  Serial.println(F("Servos attached at user‑defined initial angles."));

  zoo.onGlobalMessage(GlobalMessageHandler);
  zoo.onDirectMessage(DirectMessageHandler);
    
  for (int i = 0; i < NUM_SENSORS; i++) {
    baseline[i] = analogRead(sensorPins[i]);
    delay(5);
  }
  Serial.println(F("Pressure baselines recorded."));

}


void loop() {
  
  if (!peakMode) {
    zoo.watch();           // receive as usual
    checkSensors();        // look for a new trigger
  } else {
    capturePeak();         // we’re in the 10‑s window
  }
}



void GlobalMessageHandler(int sender, int value) {
  Serial.print(F("GLOBAL from ")); Serial.print(sender);
  Serial.print(F(" | Intensity ")); Serial.println(value);
  triggerPattern(sender, value);
}

void DirectMessageHandler(int sender, int value) {
  Serial.print(F("DIRECT from ")); Serial.print(sender);
  Serial.print(F(" | Intensity ")); Serial.println(value);
  triggerPattern(sender, value);
}


void triggerPattern(int sender, int value) {
  int id           = sender % numServos;
  int angleOffset = map(value, 0, 255, 15, 60);  

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



int clampAngle(int base, int angle) {
  
  int lowerBound = max(0, base - MAX_OFFSET);
  int upperBound = min(180, base + MAX_OFFSET);
  if (angle < lowerBound) return lowerBound;
  if (angle > upperBound) return upperBound;
  return angle;
}

void pulseAll(int offset) {
  for (int r = 0; r < 2; r++) {
    for (int i = 0; i < numServos; i++)
      servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + offset));
    delay(100);
    resetServos();
    delay(100);
  }
}

void waveLeftToRight(int offset) {
  for (int i = 0; i < numServos; i++) {
    servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + offset));
    delay(60);
    servos[i].write(initialAngle[i]);
  }
}

void waveRightToLeft(int offset) {
  for (int i = numServos - 1; i >= 0; i--) {
    servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + offset));
    delay(60);
    servos[i].write(initialAngle[i]);
  }
}

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

void sequentialUp(int offset) {
  for (int i = 0; i < numServos; i++) {
    servos[i].write(clampAngle(initialAngle[i], initialAngle[i] + offset));
    delay(60);
    servos[i].write(initialAngle[i]);
  }
}

void sequentialDown(int offset) {
  for (int i = numServos - 1; i >= 0; i--) {
    servos[i].write(clampAngle(initialAngle[i], initialAngle[i] - offset));
    delay(60);
    servos[i].write(initialAngle[i]);
  }
}

void bounceEnds(int offset) {
  for (int r = 0; r < 2; r++) {
    servos[0].write(clampAngle(initialAngle[0], initialAngle[0] + offset));
    servos[numServos - 1].write(clampAngle(initialAngle[numServos - 1], initialAngle[numServos - 1] - offset));
    delay(120);
    resetServos();
    delay(120);
  }
}

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



void resetServos() {
  for (int i = 0; i < numServos; i++)
    servos[i].write(initialAngle[i]);
}


void checkSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);
    int diff = raw - baseline[i];
    if (diff > SENSOR_THRESHOLD) {      
      peakMode     = true;
      activeSensor = i;
      windowStart  = millis();
      peakReading  = diff;
      Serial.print(F("Sensor ")); Serial.print(i);
      Serial.println(F(" triggered → entering 5s window"));
      return;                            
    }
  }
}


void capturePeak() {
  
  int raw  = analogRead(sensorPins[activeSensor]);
  int diff = raw - baseline[activeSensor];
  if (diff > peakReading) peakReading = diff;

  
  if (millis() - windowStart >= WINDOW_MS) {
    int intensity = map(peakReading, 0, 1023, 0, 255);
    intensity = constrain(intensity, 0, 255);

    
    String msg = zoo.createMessage(
        /*ping*/   false,
        /*global*/ false,
        /*address*/ activeSensor,   
        /*value*/   intensity,
        /*sender*/  deviceID);

    zoo.sendMessage(msg);           
    Serial.print(F("Sent intensity ")); Serial.print(intensity);
    Serial.print(F(" to device ")); Serial.println(activeSensor);

    peakMode      = false;          
    activeSensor  = -1;
  }
}
