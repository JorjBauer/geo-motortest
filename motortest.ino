#include <Arduino.h>

#define sensorA 15
#define sensorB 14
#define sensorC 16
#define sensorPower 13

#define bitShiftModifier 7

typedef struct _sensorState {
  bool a;
  bool b;
  bool c;
} sensorState;

typedef struct _sensorReading {
  bool isValid;
  bool movingForward;
  bool movingBackward;
  unsigned long periodA;
  unsigned long periodB;
  unsigned long periodC;
  float estimatedPeriodOverall;
} sensorReading;

// convenience macros for determining which sensors are "on" (low). Note repitition of logic for ease of understanding (e.g. AB is the same as BA).
#define isA(x) (x.a==LOW && x.b==HIGH && x.c==HIGH)
#define isAB(x) (x.a==LOW && x.b==LOW && x.c==HIGH)
#define isB(x) (x.a==HIGH && x.b==LOW && x.c==HIGH)
#define isBC(x) (x.a==HIGH && x.b==LOW && x.c==LOW)
#define isC(x) (x.a==HIGH && x.b==HIGH && x.c==LOW)
#define isCA(x) (x.a==LOW && x.b==HIGH && x.c==LOW)

#define isBA(x) (x.a==LOW && x.b==LOW && x.c==HIGH)
#define isCB(x) (x.a==HIGH && x.b==LOW && x.c==LOW)
#define isAC(x) (x.a==LOW && x.b==HIGH && x.c==LOW)

#define isABC(x) (x.a==LOW && x.b==LOW && x.c==LOW)
#define isNONE(x) (x.a==HIGH && x.b==HIGH && x.c==HIGH)

void setup() {
  pinMode(sensorA, INPUT);
  pinMode(sensorB, INPUT);
  pinMode(sensorC, INPUT);
  pinMode(sensorPower, INPUT);

  Serial.begin(115200);
}

void loop() {
  static sensorState currentState = { true, true, true }, previousState = { true, true, true };
  static sensorReading currentReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousReading = { false, false, false, 0, 0, 0, 0 };
  static unsigned long startA=0, startB=0, startC=0;
  static unsigned long periodA=0, periodB=0, periodC=0;

  // Read the current state of the three sensors
  unsigned long sampleTime = micros();
  readSensors(&currentState);

  // We want to know two things: direction and speed.

  // To get speed, we need to measure the off-pulse length.
  if (currentState.a == false) {
    if (startA == 0) {
      startA = sampleTime;
      // can't tell the speed yet; we just started
    } else {
      // We know the period now, b/c we know when it started.
      periodA = sampleTime - startA;
      // ... but we might be wrong b/c of some transient noise. So we don't reset startA. If the next read is also low, we'll extend periodA.
    }
  } else {
    // ... an all-high period tells us to reset startA.
    startA = 0;

    if (periodA) {
      // This is a true reading now.
      currentReading.periodA = periodA;
    }
  }

  if (currentState.b == false) {
    if (startB == 0) {
      startB = sampleTime;
    } else {
      periodB = sampleTime - startB;
    }
  } else {
    startB = 0;
    if (periodB) {
      currentReading.periodB = periodB;
    }
  }

  if (currentState.c == false) {
    if (startC == 0) {
      startC = sampleTime;
    } else {
      periodC = sampleTime - startC;
    }
  } else {
    startC = 0;
    if (periodC) {
      currentReading.periodC = periodC;
    }
  }
  
  // To find direction, we need to know the firing order of A, B, C.
  // Either we're going AB B BC C CA A or we're going AC C CB B BA A.
  if (isAB(previousState)) {
    // forward would be AB -> B; backward would be BA -> A.
    // FIXME: Do we need double-jump testing (AB -> BC and BA -> AC)?
    if (isB(currentState)) {
      currentReading.movingForward = true;
      currentReading.movingBackward = false;
    }
    else if (isA(currentState)) {
      currentReading.movingBackward = true;
      currentReading.movingForward = false;
    }

  } else if (isB(previousState)) {
    // Forward would be B -> BC; backward would be B -> BA.
    if (isBC(currentState)) {
      currentReading.movingForward = true;
      currentReading.movingBackward = false;
    } else if (isBA(currentState)) {
      currentReading.movingBackward = true;
      currentReading.movingForward = false;
    }
    
  } else if (isBC(previousState)) {
    // Forward would be BC -> C; backward would be BC -> B.
    if (isC(currentState)) {
      currentReading.movingForward = true;
      currentReading.movingBackward = false;
    } else if (isB(currentState)) {
      currentReading.movingBackward = true;
      currentReading.movingForward = false;
    }
  } else if (isC(previousState)) {
    // Forward would be C -> CA; backward would be C -> CB.
    if (isCA(currentState)) {
      currentReading.movingForward = true;
      currentReading.movingBackward = false;
    } else if (isCB(currentState)) {
      currentReading.movingBackward = true;
      currentReading.movingForward = false;
    }
  } else if (isCA(previousState)) {
    // Forward would be CA -> A; backward would be CA -> C.
    if (isA(currentState)) {
      currentReading.movingForward = true;
      currentReading.movingBackward = false;
    } else if (isC(currentState)) {
      currentReading.movingBackward = true;
      currentReading.movingForward = false;
    }
  } else if (isA(previousState)) {
    // Forward would be A -> AB; backward would be A -> AC.
    if (isAB(currentState)) {
      currentReading.movingForward = true;
      currentReading.movingBackward = false;
    } else if (isAC(currentState)) {
      currentReading.movingBackward = true;
      currentReading.movingForward = false;
    }
  }

  // FIXME: how do we get "not moving"? Need some sort of "sat on AB too long" or whatever (for each state)

#if 0
  // debugging - show when the sensors change
  bool didPrint = false;
  if (currentState.a != previousState.a) {
    if (currentState.a) {
      Serial.print("A");
    } else {
      Serial.print("a");
    }
    didPrint = true;
  }
  if (currentState.b != previousState.b) {
    if (currentState.b) {
      Serial.print("B");
    } else {
      Serial.print("b");
    }
    didPrint = true;
  }
  if (currentState.c != previousState.c) {
    if (currentState.c) {
      Serial.print("C");
    } else {
      Serial.print("c");
    }
    didPrint = true;
  }
  if (didPrint) {
    Serial.println();
  }
#endif
  
  // If we have good data then mark this as valid and estimate the overall period.
  if ( (currentReading.movingBackward || currentReading.movingForward) &&
       (currentReading.periodA || currentReading.periodB || currentReading.periodC) ) {
        if (currentReading.periodA && abs(currentReading.periodA - currentReading.periodB) < 20) {
          currentReading.isValid = true;
          currentReading.estimatedPeriodOverall = currentReading.periodA >> bitShiftModifier;
        } else if (currentReading.periodB && abs(currentReading.periodB - currentReading.periodC) < 20) {
          currentReading.isValid = true;
          currentReading.estimatedPeriodOverall = currentReading.periodB >> bitShiftModifier;
        } else if (abs(currentReading.periodC - currentReading.periodA) < 20) {
          currentReading.isValid = true;
          currentReading.estimatedPeriodOverall = currentReading.periodC >> bitShiftModifier;
        } else {
          currentReading.isValid = false;
        }
  } else {
    currentReading.isValid = false;
  }

#if 1
// If we have a valid reading, then report it periodically
  if (currentReading.isValid &&
      ( (currentReading.movingForward != previousReading.movingForward) || 
        (currentReading.movingBackward != previousReading.movingBackward) ||
        (currentReading.estimatedPeriodOverall != previousReading.estimatedPeriodOverall) ) ) {

    if (currentReading.isValid) {
      if (currentReading.movingForward) {
        Serial.print("F ");
      } else {
        Serial.print("B ");
      }
      Serial.println(currentReading.estimatedPeriodOverall);
    }
  }
#endif

  // Set the previous reading now.
  memcpy(&previousState, &currentState, sizeof(currentState));
  memcpy(&previousReading, &currentReading, sizeof(currentReading));
}

// Read for ~20mS and see if there's anything on each of a, b, c. LOW is active.
void readSensors(sensorState *s) 
{
  bool a = true, b=true, c=true;
  
//  unsigned long startTime = millis();

  // coded for rollover...
//  while (startTime + 20 > millis()) {
    if (digitalRead(sensorA) == LOW) {
      a = false;
    }
    if (digitalRead(sensorB) == LOW) {
      b = false;
    }
    if (digitalRead(sensorC) == LOW) {
      c = false;
    }
//  }

  s->a = a;
  s->b = b;
  s->c = c;
}

