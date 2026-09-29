/*
Basic Code Template for the LF-2 robot using 16 channel Analog Sensor
Modified: turnSpeed for sharp turns + reliable turning when line is lost
*/

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
//------------------------------------------------------------

#define s0 14  // A0 defined as digital pin 14
#define s1 15  // A1 defined as digital pin 15
#define s2 16  // A2 defined as digital pin 16
#define s3 17  // A3 defined as digital pin 17

//--------Enter Line Details here---------
bool isBlackLine = 1;  //keep 1 in case of black line. In case of white line change this to 0
unsigned int numSensors = 16;
//-----------------------------------------

int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 200;
int currentSpeed = 150;
int turnSpeed = 90;     // NEW: slower forward speed at sharp turns
int lastTurnDir = 0;    // NEW: 1 = line last seen on sensor 0 side, -1 = sensor 15 side
int sensorWeight[16] = { 7, 6, 5, 4, 3, 2, 1, 0, 0, -1, -2, -3, -4, -5, -6, -7 };

int activeSensors;
float Kp = 0.1;
float Kd = 1;
float Ki = 0;
int onLine = 1;
int minValues[16], maxValues[16], threshold[16], sensorValue[16], sensorArray[16];

double startMillis, prevMillis, elapsedTime;

void setup() {

  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(11, INPUT_PULLUP);  //Pushbutton
  pinMode(12, INPUT_PULLUP);  //Pushbutton
  pinMode(13, OUTPUT);        //LED

  pinMode(5, OUTPUT);     //standby for older carrier boards
  digitalWrite(5, HIGH);  //enables the motor driver

  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
}

void loop() {
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1) {
    startMillis = millis();
    readLine();

    // NEW: remember which side the line was last seen on
    if (sensorArray[0] || sensorArray[1]) {
      lastTurnDir = 1;
    } else if (sensorArray[14] || sensorArray[15]) {
      lastTurnDir = -1;
    }

    // NEW: outer sensors see the line = sharp turn, so slow down
    if (sensorArray[0] || sensorArray[1] || sensorArray[14] || sensorArray[15]) {
      currentSpeed = turnSpeed;
    } else if (currentSpeed < lfSpeed) {
      currentSpeed++;  // ramp back up after the turn
    }

    if (onLine == 1) {  // PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
    } else {
      // NEW: line lost, always turn toward the side it was last seen
      digitalWrite(13, LOW);
      currentSpeed = turnSpeed;

      if (lastTurnDir == 1) {
        motor1run(-100);
        motor2run(255);
      } else if (lastTurnDir == -1) {
        motor1run(255);
        motor2run(-100);
      } else {
        // fallback if edge sensors never saw the line: use error sign
        if (error >= 0) {
          motor1run(-100);
          motor2run(255);
        } else {
          motor1run(255);
          motor2run(-100);
        }
      }
    }
    elapsedTime = millis() - startMillis;
  }
}

void linefollow() {
  error = 0;
  activeSensors = 0;

  for (int i = 0; i < 16; i++) {
    if (sensorArray[i]) {
      error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
    }
    activeSensors += sensorArray[i];
  }
  error = error / activeSensors;

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255) lsp = 255;
  if (lsp < -100) lsp = -100;
  if (rsp > 255) rsp = 255;
  if (rsp < -100) rsp = -100;

  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  for (int i = 0; i < 16; i++) {
    minValues[i] = sensorRead(i);
    maxValues[i] = sensorRead(i);
  }

  for (int i = 0; i < 3000; i++) {
    motor1run(70);
    motor2run(-70);

    for (int i = 0; i < 16; i++) {
      if (sensorRead(i) < minValues[i]) {
        minValues[i] = sensorRead(i);
      }
      if (sensorRead(i) > maxValues[i]) {
        maxValues[i] = sensorRead(i);
      }
    }
  }

  for (int i = 0; i < 16; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motor1run(0);
  motor2run(0);
}

void readLine() {
  onLine = 0;
  for (int i = 0; i < 16; i++) {
    if (isBlackLine) {
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(sensorRead(i), minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    sensorArray[i] = sensorValue[i] > 500;

    if (sensorArray[i]) onLine = 1;
  }
  //printAdjValues();
}

void printAdjValues() {
  for (int i = 0; i < 16; i++) {
    Serial.print(sensorValue[i]);
    Serial.print("  ");
  }
  Serial.println();
}

//--------Function to run Motor 1-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}

int sensorRead(int sensor) {
  digitalWrite(s0, sensor & 0x01);
  digitalWrite(s1, sensor & 0x02);
  digitalWrite(s2, sensor & 0x04);
  digitalWrite(s3, sensor & 0x08);
  return analogRead(4);
}
