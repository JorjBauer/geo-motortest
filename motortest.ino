#include <Arduino.h>
#include <SPI.h>
#include <MCP4261.h>

#define MCP4261_SLAVE_SELECT_PIN 10 //arduino   <->   Chip Select               -> CS  (Pin 01 on MCP4261 DIP)

// Its recommended to measure the rated end-end resistance (terminal A to terminal B)
// Because this can vary by a large margin, up to -+ 20%. And temperature variations.
float rAB_ohms = 5090.00; // 5k Ohm

// Instantiate Mcp4261 object, with default rW (=117.5 ohm, its typical resistance)
MCP4261 Mcp4261 = MCP4261( MCP4261_SLAVE_SELECT_PIN, rAB_ohms );

#define LsensorA 16
#define LsensorB 15
#define LsensorC 17
#define LsensorPower 14

#define RsensorA 8
#define RsensorB 7
#define RsensorC 6
#define RsensorPower 9

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

typedef struct _sensorHistory {
  unsigned long startA;
  unsigned long periodA;
  unsigned long startB;
  unsigned long periodB;
  unsigned long startC;
  unsigned long periodC;
} sensorHistory;

float currentLeftOhms = 0;

// convenience macros for determining which sensors are "on" (low). Note repitition of logic for ease of understanding (e.g. AB is the same as BA).
#define isA(x) ((x).a==LOW && (x).b==HIGH && (x).c==HIGH)
#define isAB(x) ((x).a==LOW && (x).b==LOW && (x).c==HIGH)
#define isB(x) ((x).a==HIGH && (x).b==LOW && (x).c==HIGH)
#define isBC(x) ((x).a==HIGH && (x).b==LOW && (x).c==LOW)
#define isC(x) ((x).a==HIGH && (x).b==HIGH && (x).c==LOW)
#define isCA(x) ((x).a==LOW && (x).b==HIGH && (x).c==LOW)

#define isBA(x) ((x).a==LOW && (x).b==LOW && (x).c==HIGH)
#define isCB(x) ((x).a==HIGH && (x).b==LOW && (x).c==LOW)
#define isAC(x) ((x).a==LOW && (x).b==HIGH && (x).c==LOW)

#define isABC(x) ((x).a==LOW && (x).b==LOW && (x).c==LOW)
#define isNONE(x) ((x).a==HIGH && (x).b==HIGH && (x).c==HIGH)

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

  SPI.begin();
  
  Mcp4261.wiper0_pos(0); // rAW = rW_ohms
  Mcp4261.wiper1_pos(0); // rAW = rW_ohms
}

void loop() {
  static sensorState currentLeftState = { true, true, true }, previousLeftState = { true, true, true };
  static sensorReading currentLeftReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousLeftReading = { false, false, false, 0, 0, 0, 0 };
  static sensorHistory leftSensor = { 0, 0, 0, 0, 0, 0 };

  static sensorState currentRightState = { true, true, true }, previousRightState = { true, true, true };
  static sensorReading currentRightReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousRightReading = { false, false, false, 0, 0, 0, 0 };
  static sensorHistory rightSensor = { 0, 0, 0, 0, 0, 0 };

  // Read the current state of the three sensors
  unsigned long sampleTime = micros();
  readSensors(&currentLeftState, 0);
  readSensors(&currentRightState, 1);

  // We want to know two things: direction and speed.
  
  measureSensor(sampleTime, &currentLeftState, &previousLeftState, &leftSensor, &currentLeftReading);
  measureSensor(sampleTime, &currentRightState, &previousRightState, &rightSensor, &currentRightReading);
  
#if 1
// If we have a valid reading, then report it periodically
  if (currentLeftReading.isValid &&
      ( (currentLeftReading.movingForward != previousLeftReading.movingForward) || 
        (currentLeftReading.movingBackward != previousLeftReading.movingBackward) ||
        (currentLeftReading.estimatedPeriodOverall != previousLeftReading.estimatedPeriodOverall) ) ) {

    if (currentLeftReading.isValid) {
      if (currentLeftReading.movingForward) {
        Serial.print("LF ");
      } else if (currentLeftReading.movingBackward) {
        Serial.print("LB ");
      } else {
        Serial.print("L- ");
      }
      Serial.println(currentLeftReading.estimatedPeriodOverall);
    }
  }

  if (currentRightReading.isValid) {
    if ( (currentRightReading.movingForward != previousRightReading.movingForward) || 
        (currentRightReading.movingBackward != previousRightReading.movingBackward) ||
        (currentRightReading.estimatedPeriodOverall != previousRightReading.estimatedPeriodOverall) ) {
          
          if (currentRightReading.isValid) {
            if (currentRightReading.movingForward) {
              Serial.print("RF ");
              } else if (currentRightReading.movingBackward) {
                Serial.print("RB ");
              } else {
                Serial.print("R- ");
              }
               Serial.println(currentRightReading.estimatedPeriodOverall);
          }
    }
  }
#endif

  // Set the previous reading now.
  memcpy(&previousLeftState, &currentLeftState, sizeof(currentLeftState));
  memcpy(&previousLeftReading, &currentLeftReading, sizeof(currentLeftReading));

  memcpy(&previousRightState, &currentRightState, sizeof(currentRightState));
  memcpy(&previousRightReading, &currentRightReading, sizeof(currentRightReading));


  // Okay! Test 1:
  // If the left reading doesn't match the right reading, then try to emulate it using Mcp4261.wiper0_pos().
  if (currentRightReading.isValid &&
        ((!currentLeftReading.isValid) ||
        currentLeftReading.movingForward != currentRightReading.movingForward ||
        currentLeftReading.movingBackward != currentRightReading.movingBackward ||
        abs(currentLeftReading.estimatedPeriodOverall - currentRightReading.estimatedPeriodOverall) > 5)) {

      // update the left potentiometer.
      if ((currentRightReading.movingForward == false && currentRightReading.movingBackward == false)) {
        // stop the motor
        currentLeftOhms = 0;
//        Serial.println("stop");
      } else if (currentRightReading.estimatedPeriodOverall < currentLeftReading.estimatedPeriodOverall) {
        // go faster
        if (currentLeftOhms < rAB_ohms-1) {
//          Serial.println("increase");
          currentLeftOhms++;
        } else {
          currentLeftOhms = rAB_ohms;
        }
      } else if (currentRightReading.estimatedPeriodOverall > currentLeftReading.estimatedPeriodOverall) {
        // go slower
        if (currentLeftOhms >= 1) {
//          Serial.println("decrease");
          currentLeftOhms--;
        } else {
          currentLeftOhms = 0;
        }
      }
      // make the adjustment
//      Mcp4261.wiper0_pos(currentLeftOhms);
  }

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

void measureSensor(unsigned long sampleTime, sensorState *currentState, sensorState *previousState, sensorHistory *history, sensorReading *currentReading)
{
  // To get speed, we need to measure the off-pulse length.
  if (currentState->a == false) {
      if (history->startA == 0) {
          history->startA = sampleTime;
          // can't tell the speed yet; we just started
      } else {
          // We know the period now, b/c we know when it started.
          history->periodA = sampleTime - history->startA;
          // ... but we might be wrong b/c of some transient noise. So we don't reset rightSensor.startA. If the next read is also low, we'll extend rightSensor.periodA.
      }
  } else {
      // ... an all-high period tells us to reset rightSensor.startA.
      history->startA = 0;
      
      if (history->periodA) {
          // This is a true reading now.
          currentReading->periodA = history->periodA;
      }
  }
  
  if (currentState->b == false) {
      if (history->startB == 0) {
          history->startB = sampleTime;
      } else {
          history->periodB = sampleTime - history->startB;
      }
  } else {
      history->startB = 0;
      if (history->periodB) {
          currentReading->periodB = history->periodB;
      }
  }
  
  if (currentState->c == false) {
      if (history->startC == 0) {
          history->startC = sampleTime;
      } else {
          history->periodC = sampleTime - history->startC;
      }
  } else {
      history->startC = 0;
      if (history->periodC) {
          currentReading->periodC = history->periodC;
      }
  }
  
  // To find direction, we need to know the firing order of A, B, C.
  // Either we're going AB B BC C CA A or we're going AC C CB B BA A.
  if (isAB(*previousState)) {
      // forward would be AB -> B; backward would be BA -> A.
      // FIXME: Do we need double-jump testing (AB -> BC and BA -> AC)?
      if (isB(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      }
      else if (isA(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
      
  } else if (isB(*previousState)) {
      // Forward would be B -> BC; backward would be B -> BA.
      if (isBC(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isBA(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
      
  } else if (isBC(*previousState)) {
      // Forward would be BC -> C; backward would be BC -> B.
      if (isC(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isB(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
  } else if (isC(*previousState)) {
      // Forward would be C -> CA; backward would be C -> CB.
      if (isCA(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isCB(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
  } else if (isCA(*previousState)) {
      // Forward would be CA -> A; backward would be CA -> C.
      if (isA(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isC(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
  } else if (isA(*previousState)) {
      // Forward would be A -> AB; backward would be A -> AC.
      if (isAB(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isAC(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
  }
  
  // FIXME: how do we get "not moving"? Need some sort of "sat on AB too long" or whatever (for each state)
  
  // If we have good data then mark this as valid and estimate the overall period.
  if ( (currentReading->movingBackward || currentReading->movingForward) &&
      (currentReading->periodA || currentReading->periodB || currentReading->periodC) ) {
      if (currentReading->periodA && abs(currentReading->periodA - currentReading->periodB) < 20) {
          currentReading->isValid = true;
          currentReading->estimatedPeriodOverall = currentReading->periodA >> bitShiftModifier;
      } else if (currentReading->periodB && abs(currentReading->periodB - currentReading->periodC) < 20) {
          currentReading->isValid = true;
          currentReading->estimatedPeriodOverall = currentReading->periodB >> bitShiftModifier;
      } else if (abs(currentReading->periodC - currentReading->periodA) < 20) {
          currentReading->isValid = true;
          currentReading->estimatedPeriodOverall = currentReading->periodC >> bitShiftModifier;
      } else {
        // If the previous reading was valid, then assume we're stopped.
        currentReading->movingBackward = currentReading->movingForward = 0;
      }
  }
}

