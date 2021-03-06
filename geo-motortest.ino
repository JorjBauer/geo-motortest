#include <Arduino.h>
#include <SPI.h>
#include <MCP4261.h>
#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

/* PID tuning.
 *  
 *  The proportional coefficient Kp reduces the rise time. It can reduce, but not eliminate, steady-state error.
 *  The integral coefficient Ki reduces or eliminates the steady-state error for a constant/step input, but slows the response.
 *  The derivative coefficient Kd increases the stability, reduces overshoot, and improves transient response.
 *  
 *  With this setup, we're going to set the right motor spinning to a somewhat arbitrary value (probably somewhere about 
 *  half its range); then we're going to engage the Teensy to bring the left motor to the same speed. So one iteration of 
 *  the test looks like this:
 *  
 *     1. Upload a zero-coefficient version of code to the Teensy;
 *     2. Set the right wheel potentiometer to zero (no right motor movement);
 *     3. Turn on power;
 *     4. Adjust the right wheel to desired speed so that it rings slightly;
 *     5. Tweak the PID values (see below);
 *     6. upload the new values to the Teensy and repeat #5 as necessary.
 *     
 *  Step #3's "rings slightly" is to say that it overshoots the target by about half, then undershoots by less than half, 
 *  and eventually reaches its target (overshooting and undershooting by less each iteration).
 *  
 *  Now comes the tuning in steps #5 and #6.
 *  
 *  A. Start by tuning Kd. Find the largest Kd that suppresses the oscillation that already exists. If your value doesn't 
 *  cause any oscillation, then turn it up; if it does cause oscillation, then turn it down. Eventually you'll find the point 
 *  where a slightly-larger value will cause the ringing; but the value you're at doesn't ring at all. Now you know your Kd 
 *  limit. Set Kd to 1/3 that value; at the end, remember that you can make the response faster by decreasing this value.
 *  
 *  B. Tune Kp. Starting at 0, turn up Kp until you get an initial overshoot of about 50% of your target. Hopefully now you've 
 *  got a system that overshoots by half, then undershoots by something like a quarter, overshoots by an eighth, and so on - 
 *  coming to something near the desired target fairly quickly. (It may not get the actual right value yet.)
 *  
 *  C. Tune Ki. This will fix the actual target at the end. Set Ki to the maximum output value (100 in this case) and then 
 *  decrease it until you find a point where the left motor exactly matches the target speed of the right motor. When you 
 *  find that point, keep going - there will probably be a large window of possible values, so tune until you find that it no 
 *  longer matches the target value. Pick something in the middle as a starting position for final tuning.
 *  
 *  Now you should be in the ballpark. You can make Kd smaller to get the system to equilibrium faster; Ki should give you some 
 *  fine-tuning on the way; and if it just doesn't seem good enough, you can pick a new Kp to control the initial overshoot, 
 *  which can make the system reach equilibrium faster.
 *  
 */

//Define the aggressive and conservative Tuning Parameters
//double aggKp=4, aggKi=0.2, aggKd=1;
//double consKp=1, consKi=0.05, consKd=0.25;
double aggKp = 0, consKp = 0.25;
double aggKi = 0, consKi = 0;
double aggKd = 1, consKd = 1;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE); // REVERSE or DIRECT

// When we're not moving, how many microseconds-per-pulse do we tell the PID we've got?
#define NOTMOVING 300

// If targeting time estimate [x], set to what estimated percent?
// Note: these are *forward* values.
uint8_t bigChangeEstimates[256] = { 
  0, // 0
  0, // 1
  0, // 2
  0, // 3
  0, // 4
  0, // 5
  0, // 6
  0, // 7
  0, // 8
  0, // 9
  0, // 10
  0, // 11
  0, // 12
  0, // 13
  0, // 14
  0, // 15
  0, // 16
  0, // 17
  0, // 18
  0, // 19
  0, // 20
  0, // 21
  76, // 22 = 76% (unstable readings of forward/backward)
  74, // 23 = 74% (unstable readings of forward/backward)
  73, // 24 = 73%
  70, // 25 = 70%
  69, // 26 = 69%
  67, // 27 = 67%
  66, // 28
  65, // 29
  64, // 30
  63, // 31
  62, // 32
  61, // 33
  60, // 34
  59, // 35
  58, // 36
  58, // 37
  58, // 38
  58, // 39
  57, // 40
  56, // 41
  54, // 42
  54, // 43
  53, // 44
  52, // 45
  52, // 46
  51, // 47
  51, // 48
  51, // 49
  50, // 50
  49, // 51
  49, // 52
  49, // 53
  49, // 54
  48, // 55
  48, // 56
  48, // 57
  47, // 58
  47, // 59
  47, // 60
  47, // 61
  46, // 62
  46, // 63
  46, // 64
  46, // 65
  46, // 66
  45, // 67
  45, // 68
  45, // 69
  44, // 70
  44, // 71
  44, // 72
  44, // 73
  44, // 74
  44, // 75
  44, // 76
  43, // 77
  43, // 78
  43, // 79
  43, // 80
  43, // 81
  43, // 82
  43, // 83
  43, // 84
  43, // 85
  43, // 86
  43, // 87
  43, // 88
  41, // 89
  41, // 90
  41, // 91
  41, // 92
  41, // 93
  41, // 94
  41, // 95
  41, // 96
  41, // 97
  41, // 98
  41, // 99
  41, // 100
  41, // 101
  40, // 102
  40, // 103
  40, // 104
  40, // 105
  40, // 106
  40, // 107
  40, // 108
  40, // 109
  40, // 110
  40, // 111
  40, // 112
  39, // 113
  39, // 114
  39, // 115
  39, // 116
  39, // 117
  39, // 118
  39, // 119
  39, // 120
  39, // 121
  39, // 122
  39, // 123
  39, // 124
  39, // 125
  38, // 126
  38, // 127
  38, // 128
  38, // 129
  38, // 130
  38, // 131
  38, // 132
  38, // 133
  38, // 134
  38, // 135
  38, // 136
  38, // 137
  38, // 138
  38, // 139
  38, // 140
  38, // 141
  38, // 142
  38, // 143
  38, // 144
  38, // 145
  38, // 146
  38, // 147
  38, // 148
  38, // 149
  38, // 150
  38, // 151
  38, // 152
  38, // 153
  38, // 154
  38, // 155
  38, // 156
  38, // 157
  38, // 158
  38, // 159
  38, // 160
  38, // 161
  38, // 162
  38, // 163
  38, // 164
  38, // 165
  38, // 166
  38, // 167
  38, // 168
  38, // 169
  38, // 170
  38, // 171
  38, // 172
  38, // 173
  38, // 174
  38, // 175
  38, // 176
  38, // 177
  38, // 178
  38, // 179
  38, // 180
  38, // 181
  38, // 182
  38, // 183
  38, // 184
  38, // 185
  38, // 186
  38, // 187
  38, // 188
  38, // 189
  38, // 190
  38, // 191
  38, // 192
  38, // 193
  38, // 194
  38, // 195
  38, // 196
  38, // 197
  38, // 198
  38, // 199
  38, // 200
  38, // 201
  38, // 202
  38, // 203
  38, // 204
  38, // 205
  38, // 206
  38, // 207
  35, // 208
  35, // 209
  35, // 210
  35, // 211
  35, // 212
  35, // 213
  35, // 214
  35, // 215
  35, // 216
  35, // 217
  35, // 218
  35, // 219
  35, // 220
  35, // 221
  35, // 222
  35, // 223
  35, // 224
  35, // 225
  35, // 226
  35, // 227
  35, // 228
  35, // 229
  35, // 230
  35, // 231
  35, // 232
  35, // 233
  35, // 234
  35, // 235
  35, // 236
  35, // 237
  35, // 238
  35, // 239
  35, // 240
  35, // 241
  35, // 242
  35, // 243
  35, // 244
  35, // 245
  35, // 246
  35, // 247
  35, // 248
  35, // 249
  35, // 250
  35, // 251
  35, // 252
  35, // 253
  35, // 254
  35, // 255
};


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

  unsigned long timeOfLastReading;
} sensorHistory;

float currentLeftPercent = 0;

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
  
  Mcp4261.scale = 100.0;
  Mcp4261.wiper0_pos(0); // rAW = rW_ohms
  Mcp4261.wiper1_pos(0); // rAW = rW_ohms
  delay(1000);

  // PID initial setup
  Setpoint = NOTMOVING;
  Input = NOTMOVING;
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  static sensorState currentLeftState = { true, true, true }, previousLeftState = { true, true, true };
  static sensorReading currentLeftReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousLeftReading = { false, false, false, 0, 0, 0, 0 };
  static sensorHistory leftSensor = { 0, 0, 0, 0, 0, 0, 0 };

  static sensorState currentRightState = { true, true, true }, previousRightState = { true, true, true };
  static sensorReading currentRightReading = { false, false, false, 0, 0, 0, 0 };
  static sensorReading previousRightReading = { false, false, false, 0, 0, 0, 0 };
  static sensorHistory rightSensor = { 0, 0, 0, 0, 0, 0, 0 };

  // Read the current state of the three sensors
  unsigned long sampleTime = micros();
  readSensors(&currentLeftState, 0);
  readSensors(&currentRightState, 1);

  // We want to know two things: direction and speed.
  
  measureSensor(sampleTime, &currentLeftState, &previousLeftState, &leftSensor, &currentLeftReading, &previousLeftReading, false);
  measureSensor(sampleTime, &currentRightState, &previousRightState, &rightSensor, &currentRightReading, &previousRightReading, true);
  
#if 0
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
#endif

#if 0
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
  static unsigned long nextAdjustTime = 0;
  if (nextAdjustTime <= millis()) { // don't adjust too often.
    if (currentRightReading.isValid) {
      int cur = currentLeftReading.estimatedPeriodOverall;
      if (!currentLeftReading.isValid || cur==0 || cur > NOTMOVING) {
        cur = NOTMOVING;
      }
      int desired = currentRightReading.estimatedPeriodOverall;
      if (!desired || desired > NOTMOVING) {
        desired = NOTMOVING;
      }

      // Run a PID loop to determine output setting for Left motor
      Setpoint = desired;
      Input = cur;
#if 0
      double gap = abs(Setpoint-Input); //distance away from setpoint
      if(gap<10)
      {  //we're close to setpoint, use conservative tuning parameters
        myPID.SetTunings(consKp, consKi, consKd);
      }
      else
      {
         //we're far from setpoint, use aggressive tuning parameters
         myPID.SetTunings(aggKp, aggKi, aggKd);
      }
#endif

      myPID.Compute();

      if (currentLeftPercent != min(Output, 100)) {
        currentLeftPercent = min(Output, 100);

        Serial.println("!");
        Serial.println(currentLeftPercent);
        Mcp4261.wiper0(currentLeftPercent);
      }
    }
    nextAdjustTime = millis()+1;
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

void measureSensor(unsigned long sampleTime, sensorState *currentState, sensorState *previousState, sensorHistory *history, sensorReading *currentReading, sensorReading *previousReading, bool debugFlag)
{
  
  // To get speed, we need to measure the off-pulse length. If we have *none*, then we'll copy anything that exists in the old one.

  // Reset the current reading
  currentReading->periodA = currentReading->periodB = currentReading->periodC = 0;
  
  if (currentState->a == false) {
      if (history->startA == 0) {
          history->startA = sampleTime;
          // can't tell the speed yet; we just started
      } else {
          // We know the period now, b/c we know when it started.
          history->periodA = sampleTime - history->startA;
          history->timeOfLastReading = sampleTime;
          // ... but we might be wrong b/c of some transient noise. So we don't reset rightSensor.startA. If the next read is also low, we'll extend rightSensor.periodA.
      }
  } else {
      // ... an all-high period tells us to reset rightSensor.startA.
      history->startA = 0;
      
      if (history->periodA) {
          // This is a true reading now.
          currentReading->periodA = history->periodA;
          history->periodA = 0;
      }
  }
  
  if (currentState->b == false) {
      if (history->startB == 0) {
          history->startB = sampleTime;
      } else {
          history->periodB = sampleTime - history->startB;
          history->timeOfLastReading = sampleTime;
      }
  } else {
      history->startB = 0;
      if (history->periodB) {
          currentReading->periodB = history->periodB;
      }
      history->periodB = 0;
  }
  
  if (currentState->c == false) {
      if (history->startC == 0) {
          history->startC = sampleTime;
      } else {
          history->periodC = sampleTime - history->startC;
          history->timeOfLastReading = sampleTime;
      }
  } else {
      history->startC = 0;
      if (history->periodC) {
          currentReading->periodC = history->periodC;
      }
      history->periodC = 0;
  }

  if (currentReading->periodA) {
    currentReading->isValid = true;
    currentReading->estimatedPeriodOverall = currentReading->periodA >> bitShiftModifier;
  } else if (currentReading->periodB) {
    currentReading->isValid = true;
    currentReading->estimatedPeriodOverall = currentReading->periodB >> bitShiftModifier;
  } else if (currentReading->periodC) {
    currentReading->isValid = true;
    currentReading->estimatedPeriodOverall = currentReading->periodC >> bitShiftModifier;
  } else if ((sampleTime - history->timeOfLastReading) >= 66304) {
    currentReading->isValid = true;
    currentReading->movingForward = currentReading->movingBackward = false;
    currentReading->estimatedPeriodOverall = 0;
  }

  if (!currentReading->periodA && !currentReading->periodB && !currentReading->periodC) {
    currentReading->periodA = previousReading->periodA;
    currentReading->periodB = previousReading->periodB;
    currentReading->periodC = previousReading->periodC;
  }

  
  
  // To find direction, we need to know the firing order of A, B, C.
  // Either we're going AB B BC C CA A or we're going AC C CB B BA A.
  // Start by assuming we're in the exact state as the previous time, and then check to see if anything has changed.
  // We look two steps in the future in case we missed one.
  currentReading->movingForward = previousReading->movingForward;
  currentReading->movingBackward = currentReading->movingBackward;
  if (isAB(*previousState)) {
      // forward would be AB -> B; backward would be BA -> A.
      // FIXME: Do we need double-jump testing (AB -> BC and BA -> AC)?
      if (isB(*currentState) || isBC(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      }
      else if (isA(*currentState) || isAC(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
      
  } else if (isB(*previousState)) {
      // Forward would be B -> BC; backward would be B -> BA.
      if (isBC(*currentState) || isC(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isBA(*currentState) || isA(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
      
  } else if (isBC(*previousState) || isC(*previousState)) {
      // Forward would be BC -> C; backward would be BC -> B.
      if (isC(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isB(*currentState) || isBA(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
  } else if (isC(*previousState)) {
      // Forward would be C -> CA; backward would be C -> CB.
      if (isCA(*currentState) || isA(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isCB(*currentState) || isB(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
  } else if (isCA(*previousState)) {
      // Forward would be CA -> A; backward would be CA -> C.
      if (isA(*currentState) || isAB(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isC(*currentState) || isCB(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
  } else if (isA(*previousState)) {
      // Forward would be A -> AB; backward would be A -> AC.
      if (isAB(*currentState) || isB(*currentState)) {
          currentReading->movingForward = true;
          currentReading->movingBackward = false;
      } else if (isAC(*currentState) || isC(*currentState)) {
          currentReading->movingBackward = true;
          currentReading->movingForward = false;
      }
  }

  if (currentReading->estimatedPeriodOverall >= 2 * 259) {
    // We're stopped.
    currentReading->movingForward = currentReading->movingBackward = 0;
    currentReading->estimatedPeriodOverall = 0;
  }

}

