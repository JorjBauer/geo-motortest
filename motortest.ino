#include <Arduino.h>

#define LsensorA 15
#define LsensorB 14
#define LsensorC 16
#define LsensorPower 13

#define RsensorA 11
#define RsensorB 10
#define RsensorC 9
#define RsensorPower 12


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
  pinMode(LsensorA, INPUT);
  pinMode(LsensorB, INPUT);
  pinMode(LsensorC, INPUT);
  pinMode(LsensorPower, INPUT);

  pinMode(RsensorA, INPUT);
  pinMode(RsensorB, INPUT);
  pinMode(RsensorC, INPUT);
  pinMode(RsensorPower, INPUT);

  Serial.begin(115200);
}

void loop() {
  static sensorState currentLeftState = { true, true, true }, previousLeftState = { true, true, true };
  static sensorReading currentLeftReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousLeftReading = { false, false, false, 0, 0, 0, 0 };
  static unsigned long LstartA=0, LstartB=0, LstartC=0;
  static unsigned long LperiodA=0, LperiodB=0, LperiodC=0;

  static sensorState currentRightState = { true, true, true }, previousRightState = { true, true, true };
  static sensorReading currentRightReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousRightReading = { false, false, false, 0, 0, 0, 0 };
  static unsigned long RstartA = 0, RstartB = 0, RstartC = 0;
  static unsigned long RperiodA=0, RperiodB = 0, RperiodC = 0;

  // Read the current state of the three sensors
  unsigned long sampleTime = micros();
  readSensors(&currentLeftState, 0);
  readSensors(&currentRightState, 1);

  // We want to know two things: direction and speed.

  // To get speed, we need to measure the off-pulse length.
  if (currentLeftState.a == false) {
    if (LstartA == 0) {
      LstartA = sampleTime;
      // can't tell the speed yet; we just started
    } else {
      // We know the period now, b/c we know when it started.
      LperiodA = sampleTime - LstartA;
      // ... but we might be wrong b/c of some transient noise. So we don't reset LstartA. If the next read is also low, we'll extend LperiodA.
    }
  } else {
    // ... an all-high period tells us to reset LstartA.
    LstartA = 0;

    if (LperiodA) {
      // This is a true reading now.
      currentLeftReading.periodA = LperiodA;
    }
  }

  if (currentLeftState.b == false) {
    if (LstartB == 0) {
      LstartB = sampleTime;
    } else {
      LperiodB = sampleTime - LstartB;
    }
  } else {
    LstartB = 0;
    if (LperiodB) {
      currentLeftReading.periodB = LperiodB;
    }
  }

  if (currentLeftState.c == false) {
    if (LstartC == 0) {
      LstartC = sampleTime;
    } else {
      LperiodC = sampleTime - LstartC;
    }
  } else {
    LstartC = 0;
    if (LperiodC) {
      currentLeftReading.periodC = LperiodC;
    }
  }
  
  // To find direction, we need to know the firing order of A, B, C.
  // Either we're going AB B BC C CA A or we're going AC C CB B BA A.
  if (isAB(previousLeftState)) {
    // forward would be AB -> B; backward would be BA -> A.
    // FIXME: Do we need double-jump testing (AB -> BC and BA -> AC)?
    if (isB(currentLeftState)) {
      currentLeftReading.movingForward = true;
      currentLeftReading.movingBackward = false;
    }
    else if (isA(currentLeftState)) {
      currentLeftReading.movingBackward = true;
      currentLeftReading.movingForward = false;
    }

  } else if (isB(previousLeftState)) {
    // Forward would be B -> BC; backward would be B -> BA.
    if (isBC(currentLeftState)) {
      currentLeftReading.movingForward = true;
      currentLeftReading.movingBackward = false;
    } else if (isBA(currentLeftState)) {
      currentLeftReading.movingBackward = true;
      currentLeftReading.movingForward = false;
    }
    
  } else if (isBC(previousLeftState)) {
    // Forward would be BC -> C; backward would be BC -> B.
    if (isC(currentLeftState)) {
      currentLeftReading.movingForward = true;
      currentLeftReading.movingBackward = false;
    } else if (isB(currentLeftState)) {
      currentLeftReading.movingBackward = true;
      currentLeftReading.movingForward = false;
    }
  } else if (isC(previousLeftState)) {
    // Forward would be C -> CA; backward would be C -> CB.
    if (isCA(currentLeftState)) {
      currentLeftReading.movingForward = true;
      currentLeftReading.movingBackward = false;
    } else if (isCB(currentLeftState)) {
      currentLeftReading.movingBackward = true;
      currentLeftReading.movingForward = false;
    }
  } else if (isCA(previousLeftState)) {
    // Forward would be CA -> A; backward would be CA -> C.
    if (isA(currentLeftState)) {
      currentLeftReading.movingForward = true;
      currentLeftReading.movingBackward = false;
    } else if (isC(currentLeftState)) {
      currentLeftReading.movingBackward = true;
      currentLeftReading.movingForward = false;
    }
  } else if (isA(previousLeftState)) {
    // Forward would be A -> AB; backward would be A -> AC.
    if (isAB(currentLeftState)) {
      currentLeftReading.movingForward = true;
      currentLeftReading.movingBackward = false;
    } else if (isAC(currentLeftState)) {
      currentLeftReading.movingBackward = true;
      currentLeftReading.movingForward = false;
    }
  }

  // FIXME: how do we get "not moving"? Need some sort of "sat on AB too long" or whatever (for each state)

  // If we have good data then mark this as valid and estimate the overall period.
  if ( (currentLeftReading.movingBackward || currentLeftReading.movingForward) &&
       (currentLeftReading.periodA || currentLeftReading.periodB || currentLeftReading.periodC) ) {
        if (currentLeftReading.periodA && abs(currentLeftReading.periodA - currentLeftReading.periodB) < 20) {
          currentLeftReading.isValid = true;
          currentLeftReading.estimatedPeriodOverall = currentLeftReading.periodA >> bitShiftModifier;
        } else if (currentLeftReading.periodB && abs(currentLeftReading.periodB - currentLeftReading.periodC) < 20) {
          currentLeftReading.isValid = true;
          currentLeftReading.estimatedPeriodOverall = currentLeftReading.periodB >> bitShiftModifier;
        } else if (abs(currentLeftReading.periodC - currentLeftReading.periodA) < 20) {
          currentLeftReading.isValid = true;
          currentLeftReading.estimatedPeriodOverall = currentLeftReading.periodC >> bitShiftModifier;
        } else {
          currentLeftReading.isValid = false;
        }
  } else {
    currentLeftReading.isValid = false;
  }

  // FIXME: start of copypasta
// To get speed, we need to measure the off-pulse length.
if (currentRightState.a == false) {
    if (RstartA == 0) {
        RstartA = sampleTime;
        // can't tell the speed yet; we just started
    } else {
        // We know the period now, b/c we know when it started.
        RperiodA = sampleTime - RstartA;
        // ... but we might be wrong b/c of some transient noise. So we don't reset RstartA. If the next read is also low, we'll extend RperiodA.
    }
} else {
    // ... an all-high period tells us to reset RstartA.
    RstartA = 0;
    
    if (RperiodA) {
        // This is a true reading now.
        currentRightReading.periodA = RperiodA;
    }
}

if (currentRightState.b == false) {
    if (RstartB == 0) {
        RstartB = sampleTime;
    } else {
        RperiodB = sampleTime - RstartB;
    }
} else {
    RstartB = 0;
    if (RperiodB) {
        currentRightReading.periodB = RperiodB;
    }
}

if (currentRightState.c == false) {
    if (RstartC == 0) {
        RstartC = sampleTime;
    } else {
        RperiodC = sampleTime - RstartC;
    }
} else {
    RstartC = 0;
    if (RperiodC) {
        currentRightReading.periodC = RperiodC;
    }
}

// To find direction, we need to know the firing order of A, B, C.
// Either we're going AB B BC C CA A or we're going AC C CB B BA A.
if (isAB(previousRightState)) {
    // forward would be AB -> B; backward would be BA -> A.
    // FIXME: Do we need double-jump testing (AB -> BC and BA -> AC)?
    if (isB(currentRightState)) {
        currentRightReading.movingForward = true;
        currentRightReading.movingBackward = false;
    }
    else if (isA(currentRightState)) {
        currentRightReading.movingBackward = true;
        currentRightReading.movingForward = false;
    }
    
} else if (isB(previousRightState)) {
    // Forward would be B -> BC; backward would be B -> BA.
    if (isBC(currentRightState)) {
        currentRightReading.movingForward = true;
        currentRightReading.movingBackward = false;
    } else if (isBA(currentRightState)) {
        currentRightReading.movingBackward = true;
        currentRightReading.movingForward = false;
    }
    
} else if (isBC(previousRightState)) {
    // Forward would be BC -> C; backward would be BC -> B.
    if (isC(currentRightState)) {
        currentRightReading.movingForward = true;
        currentRightReading.movingBackward = false;
    } else if (isB(currentRightState)) {
        currentRightReading.movingBackward = true;
        currentRightReading.movingForward = false;
    }
} else if (isC(previousRightState)) {
    // Forward would be C -> CA; backward would be C -> CB.
    if (isCA(currentRightState)) {
        currentRightReading.movingForward = true;
        currentRightReading.movingBackward = false;
    } else if (isCB(currentRightState)) {
        currentRightReading.movingBackward = true;
        currentRightReading.movingForward = false;
    }
} else if (isCA(previousRightState)) {
    // Forward would be CA -> A; backward would be CA -> C.
    if (isA(currentRightState)) {
        currentRightReading.movingForward = true;
        currentRightReading.movingBackward = false;
    } else if (isC(currentRightState)) {
        currentRightReading.movingBackward = true;
        currentRightReading.movingForward = false;
    }
} else if (isA(previousRightState)) {
    // Forward would be A -> AB; backward would be A -> AC.
    if (isAB(currentRightState)) {
        currentRightReading.movingForward = true;
        currentRightReading.movingBackward = false;
    } else if (isAC(currentRightState)) {
        currentRightReading.movingBackward = true;
        currentRightReading.movingForward = false;
    }
}

// FIXME: how do we get "not moving"? Need some sort of "sat on AB too long" or whatever (for each state)

// If we have good data then mark this as valid and estimate the overall period.
if ( (currentRightReading.movingBackward || currentRightReading.movingForward) &&
    (currentRightReading.periodA || currentRightReading.periodB || currentRightReading.periodC) ) {
    if (currentRightReading.periodA && abs(currentRightReading.periodA - currentRightReading.periodB) < 20) {
        currentRightReading.isValid = true;
        currentRightReading.estimatedPeriodOverall = currentRightReading.periodA >> bitShiftModifier;
    } else if (currentRightReading.periodB && abs(currentRightReading.periodB - currentRightReading.periodC) < 20) {
        currentRightReading.isValid = true;
        currentRightReading.estimatedPeriodOverall = currentRightReading.periodB >> bitShiftModifier;
    } else if (abs(currentRightReading.periodC - currentRightReading.periodA) < 20) {
        currentRightReading.isValid = true;
        currentRightReading.estimatedPeriodOverall = currentRightReading.periodC >> bitShiftModifier;
    } else {
        currentRightReading.isValid = false;
    }
} else {
    currentRightReading.isValid = false;
}

  // FIXME: end of copypasta
  

#if 1
// If we have a valid reading, then report it periodically
  if (currentLeftReading.isValid &&
      ( (currentLeftReading.movingForward != previousLeftReading.movingForward) || 
        (currentLeftReading.movingBackward != previousLeftReading.movingBackward) ||
        (currentLeftReading.estimatedPeriodOverall != previousLeftReading.estimatedPeriodOverall) ) ) {

    if (currentLeftReading.isValid) {
      if (currentLeftReading.movingForward) {
        Serial.print("LF ");
      } else {
        Serial.print("LB ");
      }
      Serial.println(currentLeftReading.estimatedPeriodOverall);
    }
  }

  if (currentRightReading.isValid &&
      ( (currentRightReading.movingForward != previousRightReading.movingForward) || 
        (currentRightReading.movingBackward != previousRightReading.movingBackward) ||
        (currentRightReading.estimatedPeriodOverall != previousRightReading.estimatedPeriodOverall) ) ) {

    if (currentRightReading.isValid) {
      if (currentRightReading.movingForward) {
        Serial.print("RF ");
      } else {
        Serial.print("RB ");
      }
      Serial.println(currentRightReading.estimatedPeriodOverall);
    }
  }
#endif

  // Set the previous reading now.
  memcpy(&previousLeftState, &currentLeftState, sizeof(currentLeftState));
  memcpy(&previousLeftReading, &currentLeftReading, sizeof(currentLeftReading));

  memcpy(&previousRightState, &currentRightState, sizeof(currentRightState));
  memcpy(&previousRightReading, &currentRightReading, sizeof(currentRightReading));
}

// Read for ~20mS and see if there's anything on each of a, b, c. LOW is active.
void readSensors(sensorState *s, bool isLeft) 
{
  bool a = true, b=true, c=true;
  
//  unsigned long startTime = millis();

  // coded for rollover...
//  while (startTime + 20 > millis()) {
    if (digitalRead(isLeft ? LsensorA : RsensorA) == LOW) {
      a = false;
    }
    if (digitalRead(isLeft ? LsensorB : RsensorB) == LOW) {
      b = false;
    }
    if (digitalRead(isLeft ? LsensorC : RsensorC) == LOW) {
      c = false;
    }
//  }

  s->a = a;
  s->b = b;
  s->c = c;
}

