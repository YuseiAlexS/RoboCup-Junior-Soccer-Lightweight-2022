/* BNO055 */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
double heading, headingRPM;

/* Line Sensors */
const int L_MUX[5] = {40, 39, 38, 37, 36};
#define IOPIN 41
bool lineState = 0;
double dirInside = -1, firstDirInside;
#define BORDER_WHITE_LINE 180
#define BORDER_GREEN 180
#define MULTI_AVG_LINE 0.2
#define QTY_LINE 32
const int P_LINE[] = {32, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6,
                      5, 4, 3, 2, 1, 17, 18, 19, 20, 21, 22, 23,
                      24, 25, 26, 27, 28, 29, 30, 31
                     };//上から見て前から時計回り
int valueLine[QTY_LINE];
boolean isLineBlack[QTY_LINE];
boolean isLineWhite[QTY_LINE];
bool isOutsideLine = false, isHalfOut = false;
double prvDirInside = -1, returnDir = -1; //戻る方向は存在しない

/* IR Sensors */
const int   ballArray[16]     = {23, 11, 10, 26, 22, 15, 14, 35, 34, 29, 28, 1, 0, 12, 24, 25};

double vectorX, vectorY, maxPulseWidth, secondPulseWidth, thirdPulseWidth,
       ballAngle, ballDistance, motorRunBall, pulseWidth[16];
const double unitVectorX[16] = {0.000, 0.383, 0.707, 0.924, 1.000, 0.924, 0.707, 0.383, 0.000,
                                -0.383, -0.707, -0.924, -1.000, -0.924, -0.707, -0.383
                               };
const double unitVectorY[16] = {1.000, 0.924, 0.707, 0.383, 0.000, -0.383, -0.707, -0.924, -1.000,
                                -0.924, -0.707, -0.383, 0.000, 0.383, 0.707, 0.924
                               };

/* Motors */
#include <PID_v1.h>
#define motorPWM1 2
#define motorPWM2 3
#define motorPWM3 4
#define motorPWM4 5
#define motorDIR1 6
#define motorDIR2 7
#define motorDIR3 8
#define motorDIR4 9
#define encoder1 27
#define encoder2 30
#define encoder3 31
#define encoder4 32
double motorOutput1, setpoint1, motorOutput2, setpoint2, motorOutput3, setpoint3,
       motorOutput4, setpoint4, RPM1, RPM2, RPM3, RPM4,
       motorRun1, motorRun2, motorRun3, motorRun4, headingSetpoint = 0;
bool tickNow1, tickOld1, tickNow2, tickOld2, tickNow3, tickOld3, tickNow4, tickOld4;
unsigned long timeOld, timeOld1, timeOld2, timeOld3, timeOld4, timeOldMillis;

/* PID */
double KP1 = 0.03, KI1 = 0.275;
PID Motor1(&RPM1, &motorOutput1, &setpoint1, KP1, KI1, 0, DIRECT);
double KP2 = 0.03, KI2 = 0.3;
PID Motor2(&RPM2, &motorOutput2, &setpoint2, KP2, KI2, 0, DIRECT);
double KP3 = 0.025, KI3 = 0.4;
PID Motor3(&RPM3, &motorOutput3, &setpoint3, KP3, KI3, 0, DIRECT);
double KP4 = 0.01, KI4 = 0.35;
PID Motor4(&RPM4, &motorOutput4, &setpoint4, KP4, KI4, 0, DIRECT);
double headingKP = 17.5, headingKI = 1, headingKD = 1;
PID headingPID(&heading, &headingRPM, &headingSetpoint, headingKP, headingKI, headingKD, DIRECT);






void setup() {
  /* setup for bno055 */
  bno.begin(bno.OPERATION_MODE_IMUPLUS);
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS))
  {
    while (1);
  }
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;
  EEPROM.get(eeAddress, bnoID);
  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id) {
  } else {
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    bno.setSensorOffsets(calibrationData);
    foundCalib = true;
  }
  bno.setExtCrystalUse(true);
  bno.setMode(bno.OPERATION_MODE_IMUPLUS);
  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib) {
    while (!bno.isFullyCalibrated()) {
      bno.getEvent(&event);
    }
  } else {
    while (!bno.isFullyCalibrated()) {
      bno.getEvent(&event);
    }
  }
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;
  EEPROM.put(eeAddress, bnoID);
  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);

  /* setup for line sensors */
  for (int x = 0; x < 5; x++) {
    pinMode(L_MUX[x], OUTPUT);
    digitalWrite(L_MUX[x], LOW);
  }
  pinMode(IOPIN, INPUT);

  /* setup for IR sensors */
  for (int x = 0; x < 16; x++) {
    pinMode(ballArray[x], INPUT);
  }

  /* setup for motors */
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  pinMode(encoder3, INPUT);
  pinMode(encoder4, INPUT);
  pinMode(motorPWM1, OUTPUT);
  pinMode(motorPWM2, OUTPUT);
  pinMode(motorPWM3, OUTPUT);
  pinMode(motorPWM4, OUTPUT);
  pinMode(motorDIR1, OUTPUT);
  pinMode(motorDIR2, OUTPUT);
  pinMode(motorDIR3, OUTPUT);
  pinMode(motorDIR4, OUTPUT);
  analogWriteFrequency(motorDIR1, 100000);
  analogWriteFrequency(motorDIR2, 100000);
  analogWriteFrequency(motorDIR3, 100000);
  analogWriteFrequency(motorDIR4, 100000);

  Motor1.SetOutputLimits(0, 100);
  Motor1.SetMode(AUTOMATIC);
  Motor1.SetTunings(KP1, KI1, 0);
  Motor1.SetControllerDirection(DIRECT);

  Motor2.SetOutputLimits(0, 100);
  Motor2.SetMode(AUTOMATIC);
  Motor2.SetTunings(KP2, KI2, 0);
  Motor2.SetControllerDirection(DIRECT);

  Motor3.SetOutputLimits(0, 100);
  Motor3.SetMode(AUTOMATIC);
  Motor3.SetTunings(KP3, KI3, 0);
  Motor3.SetControllerDirection(DIRECT);

  Motor4.SetOutputLimits(0, 100);
  Motor4.SetMode(AUTOMATIC);
  Motor4.SetTunings(KP4, KI4, 0);
  Motor4.SetControllerDirection(DIRECT);

  headingPID.SetOutputLimits(-1000, 1000);
  headingPID.SetMode(AUTOMATIC);
  headingPID.SetTunings(headingKP, headingKI, headingKD);
  headingPID.SetControllerDirection(DIRECT);
}







void loop() {
  /* 姿勢制御 */
  sensors_event_t event;
  bno.getEvent(&event);
  heading = event.orientation.x;
  if (heading > 180) {
    heading -= 360;
  }
  headingPID.Compute();

  /* ボール追従制御 */
  BallChase();

  /* ライン回避制御 */
  calLine();
  if (ballDistance < 100) {
    if (lineState == 1) {
      motorRun(dirInside, 400, 0);
    } else if (returnDir != -1) {
      motorRun(returnDir, 400, 0);
    } else {
      if (abs(heading) > 10) {
        motorRun(0, 0, headingRPM);
      } else {
        motorRun(0, 0, 0);
      }
    }
  } else {
    if (lineState == 1) {
      motorRun(dirInside, 400, 0);
    } else if ((dirInside >= 22.5 && dirInside <= 67.5 && ballAngle >= 0 && ballAngle <= 90) ||
               (dirInside >= 112.5 && dirInside <= 157.5 && ballAngle >= 90 && ballAngle <= 180) ||
               (dirInside >= 202.5 && dirInside <= 247.5 && ballAngle >= 180 && ballAngle <= 270) ||
               (dirInside >= 292.5 && dirInside <= 337.5 && ballAngle >= 270 && ballAngle <= 360)) {
      motorRun(dirInside, 400, 0);
    } else if (returnDir != -1) {
      motorRun(returnDir, 300, 0);
    }
    else if (ballAngle >= 0 && ballAngle <= 80 && dirInside >= 225 && dirInside <= 315) {
      motorRun(0, 400, headingRPM);
    } else if (ballAngle >= 80 && ballAngle <= 100 && dirInside >= 225 && dirInside <= 315) {
      if (abs(heading) > 10) {
        motorRun(0, 0, headingRPM);
      } else {
        motorRun(0, 0, 0);
      }
    } else if (ballAngle >= 100 && ballAngle <= 180 && dirInside >= 225 && dirInside <= 315) {
      motorRun(180, 400, headingRPM);
    } else if (ballAngle >= 180 && ballAngle <= 260 && dirInside >= 45 && dirInside <= 135) {
      motorRun(180, 400, headingRPM);
    } else if (ballAngle >= 260 && ballAngle <= 280 && dirInside >= 45 && dirInside <= 135) {
      if (abs(heading) > 10) {
        motorRun(0, 0, headingRPM);
      } else {
        motorRun(0, 0, 0);
      }
    } else if (ballAngle >= 280 && ballAngle <= 360 && dirInside >= 45 && dirInside <= 135) {
      motorRun(0, 600, headingRPM);
    } else if (ballAngle >= 10 && ballAngle <= 90 && dirInside >= 135 && dirInside <= 225) {
      motorRun(120, 400, headingRPM);
    } else if (ballAngle >= 0 && ballAngle <= 10 && dirInside >= 135 && dirInside <= 225) {
      if (abs(heading) > 10) {
        motorRun(0, 0, headingRPM);
      } else {
        motorRun(0, 0, 0);
      }
    } else if (ballAngle >= 270 && ballAngle <= 350 && dirInside >= 135 && dirInside <= 225) {
      motorRun(250, 400, headingRPM);
    } else if (ballAngle >= 350 && ballAngle <= 360 && dirInside >= 135 && dirInside <= 225) {
      if (abs(heading) > 10) {
        motorRun(0, 0, headingRPM);
      } else {
        motorRun(0, 0, 0);
      }
    } else {
      motorRun(motorRunBall, 1000, headingRPM);
    }
  }
}

int prvPrvDirInside = -1;

void calLine() {
  prvPrvDirInside = prvDirInside;
  prvDirInside = dirInside;
  int qtyILB = 0; //暗いと判断した個数
  int qtyILW = 0; //明るいと判断した個数
  for (int numLine = 0; numLine < QTY_LINE; numLine ++) { //一つずつのラインセンサーの値を読み取る
    pinSelect(P_LINE[numLine]);
    valueLine[numLine] = analogRead(IOPIN);
    isLineBlack[numLine] = valueLine[numLine] <= BORDER_GREEN; //valueLine[numLine] == BORDER_GREEN が真ならisLineBlack[numLine] =　1、儀ならisLineBlack[numLine] =　1
    isLineWhite[numLine] = valueLine[numLine] >= BORDER_WHITE_LINE;
    if (isLineBlack[numLine]) {
      qtyILB ++;
    }
    if (isLineWhite[numLine]) {
      qtyILW ++;
    } //全てのセンサーは黒か白を読み取らなくてもいい
  }

#define MULTI_AVG_DI 0.5

  if (qtyILB == QTY_LINE) { //フィールド内
    isOutsideLine = false;
    isHalfOut = false;
    dirInside = -1;
    lineState = 0;
  } else if (qtyILW <= 1) { //もうすこしでライン外 or ライン内
    isOutsideLine = isHalfOut; //isOutsideLine というライン外かどうかの変数には isHalfOut という半分以上外に出ているかどうかの変数をそのまま入れます
    dirInside = isOutsideLine ? prvDirInside : -1; //dirInside という戻るべき方向を示す変数には、ライン外(isOutsideLine == 1)
    //ならprvDirInside を入れて直前のままにし、ライン内(isOutsideLine == 0)なら-1 を入れて戻るべき方向は存在しないとします。
  } else { //ライン上
    isOutsideLine = false;
    int posILW[qtyILW];
    int numILW = 0;
    for (int numLine = 0; numLine < QTY_LINE; numLine ++) { //白の番号を調べる
      if (isLineWhite[numLine]) {
        posILW[numILW] = numLine; //posILW[] に何番目のラインセンサがライン上にあるかを入れる
        numILW ++;
      }
    }
    int intvLine[qtyILW];
    for (int numLine = 0; numLine < qtyILW - 1; numLine ++) { //白の間隔を調べる
      intvLine[numLine] = posILW[numLine + 1] - posILW[numLine];
      //intvLine[] に連続する2つのposILW[] の差、つまりライン上のラインセンサの間隔の大きさを入れる
    }
    intvLine[qtyILW - 1] = posILW[0] - posILW[qtyILW - 1] + QTY_LINE;

    int maxIntvL = 0;
    int posMaxIntvL = 0;
    for (numILW = 0; numILW < qtyILW; numILW ++) { //ラインの方向を調べる
      if (maxIntvL < intvLine[numILW]) { //次に、intvLine[] の値を順々に比べていくことで、どの間隔が一番広いかがわかり、
        maxIntvL = intvLine[numILW]; //maxIntvL にその最大の間隔の大きさを入れる。
        posMaxIntvL = numILW; //posMaxIntvL に間隔の場所を入れる
      }
    }
    double numDirInside = posILW[posMaxIntvL] + maxIntvL * 0.5;
    //this is the number of a line sensor, which its direction is the direction the robot should return to
    if (numDirInside / QTY_LINE * 360 > 360) { //ロボットが戻る方向が計算でき、その結果をdirInside に入れています。
      dirInside = numDirInside / QTY_LINE * 360 - 360;
    } else {
      dirInside = numDirInside / QTY_LINE * 360;
    }

    //前回と比較
    if (prvDirInside >= 0) {
      //半分以上外か
      isHalfOut = false;
      if (abs(dirInside - prvDirInside) >= 90 && abs(dirInside - prvDirInside) <= 270) {
        dirInside = prvDirInside;
        isHalfOut = true;
      }

      //平均値計算
      if (abs(dirInside - prvDirInside) <= 180) {
        dirInside = prvDirInside * MULTI_AVG_DI + dirInside * (1 - MULTI_AVG_DI);
      } else {
        dirInside = prvDirInside * MULTI_AVG_DI + dirInside * (1 - MULTI_AVG_DI)
                    + 360 * (dirInside >= prvDirInside ? MULTI_AVG_DI : 1 - MULTI_AVG_DI);
      }
      if (dirInside > 360) {
        dirInside = dirInside  - 360;
      }
    }

    if (prvPrvDirInside == -1 && prvDirInside == -1 && (valueLine[0] >= BORDER_WHITE_LINE || valueLine[1] >= BORDER_WHITE_LINE || valueLine[2] >= BORDER_WHITE_LINE ||
        valueLine[31] >= BORDER_WHITE_LINE ||
        valueLine[30] >= BORDER_WHITE_LINE ||
        valueLine[16] >= BORDER_WHITE_LINE || valueLine[15] >= BORDER_WHITE_LINE || valueLine[14] >= BORDER_WHITE_LINE ||
        valueLine[17] >= BORDER_WHITE_LINE ||
        valueLine[18] >= BORDER_WHITE_LINE || valueLine[19] >= BORDER_WHITE_LINE)) {
      lineState = 1;
    } else if (lineState == 0 && (isHalfOut || isOutsideLine)) {
      returnDir = dirInside;
    } else {
      returnDir = -1;
    }
  }
}


void pinSelect(int pinNum) {
  pinNum -= 1;
  for (int z = 0; z < 5; z++) {
    byte state = bitRead(pinNum, z);
    digitalWriteFast(L_MUX[z], state);
  }
}

void motorRun(double runAngle, double runRPM, double rotationRPM) { //Angle and Rotation = Clockwise
  motorRun1 = runRPM / 2 * cos((runAngle - 45) * PI / 180) + rotationRPM / 4;
  motorRun2 = -runRPM / 2 * sin((runAngle - 45) * PI / 180) + rotationRPM / 4;
  motorRun3 = -runRPM / 2 * cos((runAngle - 45) * PI / 180) + rotationRPM / 4;
  motorRun4 = runRPM / 2 * sin((runAngle - 45) * PI / 180) + rotationRPM / 4;
  motorPID(motorRun1, motorRun2, motorRun3, motorRun4);
}

void readEncoders () {
  /* front left motor */
  tickNow1 = digitalRead(encoder1);
  if (tickNow1 == HIGH && tickOld1 == LOW) {
    RPM1 = (60000000.00000 / (micros() - timeOld1)) / 16.00000;
    timeOld1 = micros();
  } else {
    if (micros() - timeOld1 > 100000) {
      RPM1 = 0;
    }
  }
  tickOld1 = tickNow1;

  /* back left motor */
  tickNow2 = digitalRead(encoder2);
  if (tickNow2 == HIGH && tickOld2 == LOW) {
    RPM2 = (60000000.00000 / (micros() - timeOld2)) / 16.00000;
    timeOld2 = micros();
  } else {
    if (micros() - timeOld2 > 100000) {
      RPM2 = 0;
    }
  }
  tickOld2 = tickNow2;

  /* back right motor */
  tickNow3 = digitalRead(encoder3);
  if (tickNow3 == HIGH && tickOld3 == LOW) {
    RPM3 = (60000000.00000 / (micros() - timeOld3)) / 16.00000;
    timeOld3 = micros();
  } else {
    if (micros() - timeOld3 > 100000) {
      RPM3 = 0;
    }
  }
  tickOld3 = tickNow3;

  /* front right motor */
  tickNow4 = digitalRead(encoder4);
  if (tickNow4 == HIGH && tickOld4 == LOW) {
    RPM4 = (60000000.00000 / (micros() - timeOld4)) / 16.00000;
    timeOld4 = micros();
  } else {
    if (micros() - timeOld4 > 100000) {
      RPM4 = 0;
    }
  }
  tickOld4 = tickNow4;
}

void BallChase() {
  vectorX = 0;
  vectorY = 0;
  maxPulseWidth = 0;
  secondPulseWidth = 0;
  thirdPulseWidth = 0;
  ballAngle = 0;
  motorRunBall = 0;

  for (int i = 0; i < 16; i++) {
    pulseWidth[i] = 0;
  }

  const unsigned long startTime_us = micros();
  do {
    for (int i = 0; i < 16; i++) {
      if (digitalReadFast(ballArray[i]) == false) {
        pulseWidth[i] += 1.0;
      }
    }
  } while ((micros() - startTime_us) < 833);

  for (int i = 0; i < 16; i++) {
    if (maxPulseWidth < pulseWidth[i]) {
      thirdPulseWidth = secondPulseWidth;
      secondPulseWidth = maxPulseWidth;
      maxPulseWidth = pulseWidth[i];
    } else if (secondPulseWidth < pulseWidth[i]) {
      thirdPulseWidth = secondPulseWidth;
      secondPulseWidth = pulseWidth[i];
    } else if (thirdPulseWidth < pulseWidth[i]) {
      thirdPulseWidth = pulseWidth[i];
    }
  }

  for (int i = 0; i < 16; i ++) {
    if (maxPulseWidth == pulseWidth[i]) {
      vectorX += (maxPulseWidth * unitVectorX[i]);
      vectorY += (maxPulseWidth * unitVectorY[i]);
    } else if (secondPulseWidth == pulseWidth[i]) {
      vectorX += (secondPulseWidth * unitVectorX[i]);
      vectorY += (secondPulseWidth * unitVectorY[i]);
    } else if (thirdPulseWidth == pulseWidth[i]) {
      vectorX += (thirdPulseWidth * unitVectorX[i]);
      vectorY += (thirdPulseWidth * unitVectorY[i]);
    }
  }
  if (vectorX > 0 && vectorY > 0) {
    ballAngle = atan(abs(vectorX / vectorY)) * 180 / PI;
  } else if (vectorX > 0 && vectorY < 0) {
    ballAngle = atan(abs(vectorY / vectorX)) * 180 / PI + 90;
  } else if (vectorX < 0 && vectorY < 0) {
    ballAngle = atan(abs(vectorX / vectorY)) * 180 / PI + 180;
  } else if (vectorX < 0 && vectorY > 0) {
    ballAngle = atan(abs(vectorY / vectorX)) * 180 / PI + 270;
  }

  ballDistance = sqrt(pow(vectorX, 2) + pow(vectorY, 2));

  if (ballDistance >= 2500) {
    if (ballAngle <= 90 && ballAngle >= 0) {
      //motorRunBall = -0.0167 * pow(ballAngle, 2) + 3.25 * ballAngle;
      motorRunBall = -0.0139 * pow(ballAngle, 2) + 2.875 * ballAngle;
    } else if (ballAngle <= 180 && ballAngle > 90) {
      if (prvDirInside == -1) {
      motorRunBall = 0.0028 * pow(ballAngle, 2) + 0.375 * ballAngle + 90;
      } else {
        motorRunBall = 90;
      }
    } else if (ballAngle <= 270 && ballAngle > 180) {
      if (prvDirInside == -1) {
      motorRunBall = -0.0028 * pow(ballAngle, 2) + 2.375 * ballAngle - 225;
      } else {
        motorRunBall = 270;
      }
    } else if (ballAngle <= 360 && ballAngle > 270) {
      motorRunBall = 0.0139 * pow(ballAngle, 2) - 7.125 * ballAngle + 1125;
    }
  } else if (ballDistance < 2500) {
    motorRunBall = ballAngle;
  }
}

void motorPID(double motor1RPM, double motor2RPM, double motor3RPM, double motor4RPM) {
  readEncoders();
  if (motor1RPM > 0) {
    setpoint1 = motor1RPM;
    Motor1.Compute();
    digitalWrite(motorPWM1, HIGH);
    analogWrite(motorDIR1, map(motorOutput1, 0, 100, 128, 170));
  } else if (motor1RPM < 0) {
    setpoint1 = -motor1RPM;
    Motor1.Compute();
    digitalWrite(motorPWM1, HIGH);
    analogWrite(motorDIR1, map(motorOutput1, 0, 100, 128, 70));
  } else {
    setpoint1 = 0;
    Motor1.Compute();
    digitalWrite(motorPWM1, LOW);
  }

  if (motor2RPM > 0) {
    setpoint2 = motor2RPM;
    Motor2.Compute();
    digitalWrite(motorPWM2, HIGH);
    analogWrite(motorDIR2, map(motorOutput2, 0, 100, 128, 170));
  } else if (motor2RPM < 0) {
    setpoint2 = -motor2RPM;
    Motor2.Compute();
    digitalWrite(motorPWM2, HIGH);
    analogWrite(motorDIR2, map(motorOutput2, 0, 100, 128, 70));
  } else {
    setpoint2 = 0;
    Motor2.Compute();
    digitalWrite(motorPWM2, LOW);
  }

  if (motor3RPM > 0) {
    setpoint3 = motor3RPM;
    Motor3.Compute();
    digitalWrite(motorPWM3, HIGH);
    analogWrite(motorDIR3, map(motorOutput3, 0, 100, 128, 170));
  } else if (motor3RPM < 0) {
    setpoint3 = -motor3RPM;
    Motor3.Compute();
    digitalWrite(motorPWM3, HIGH);
    analogWrite(motorDIR3, map(motorOutput3, 0, 100, 128, 70));
  } else {
    setpoint3 = 0;
    Motor3.Compute();
    digitalWrite(motorPWM3, LOW);
  }

  if (motor4RPM > 0) {
    setpoint4 = motor4RPM;
    Motor4.Compute();
    digitalWrite(motorPWM4, HIGH);
    analogWrite(motorDIR4, map(motorOutput4, 0, 100, 128, 170));
  } else if (motor4RPM < 0) {
    setpoint4 = -motor4RPM;
    Motor4.Compute();
    digitalWrite(motorPWM4, HIGH);
    analogWrite(motorDIR4, map(motorOutput4, 0, 100, 128, 70));
  } else {
    setpoint4 = 0;
    Motor4.Compute();
    digitalWrite(motorPWM4, LOW);
  }
}
