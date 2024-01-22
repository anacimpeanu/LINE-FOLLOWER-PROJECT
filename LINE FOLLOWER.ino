#include <QTRSensors.h>

const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;
int m1Speed = 0;
int m2Speed = 0;

float kp = 7;
float ki = 0;
float kd = 4;
int p = 1;
int i = 0;
int d = 1;

int bigAngleTh = 20;
int adjustValue = 30;

int error = 0;
int lastError = 0;
const int maxSpeed = 255;
const int minSpeed = -255;
const int baseSpeed = 255;

QTRSensors qtr;
const int sensorCount = 6;
int sensorValues[sensorCount];

unsigned long timeToChangeDir = 500;
unsigned long lastTime = 0;
bool engineDir = true;
const int m1CalibrationSpeed = 200.5;

void autocalibrate() {
    qtr.read(sensorValues);

    const int minBlackThreshold = 600;
    bool isBlackDetected = false;

    for (int i = 0; i < sensorCount; i++) {
        if (sensorValues[i] > minBlackThreshold) {
            isBlackDetected = true;
            break;
        }
    }

    if (!isBlackDetected && (millis() - lastTime > timeToChangeDir)) {
        engineDir = !engineDir;
        lastTime = millis();
    }

    if (engineDir) {
        setMotorSpeed(m1CalibrationSpeed, 0);
    } else {
        setMotorSpeed(-m1CalibrationSpeed, 0);
    }
}


void setup() {
    // pinMode setup
    pinMode(m11Pin, OUTPUT);
    pinMode(m12Pin, OUTPUT);
    pinMode(m21Pin, OUTPUT);
    pinMode(m22Pin, OUTPUT);
    pinMode(m1Enable, OUTPUT);
    pinMode(m2Enable, OUTPUT);
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5 }, sensorCount);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    for (uint16_t i = 0; i < 200; i++) {
        qtr.calibrate();
        autocalibrate();
    }
    digitalWrite(LED_BUILTIN, LOW);
    
}
void loop() {
  qtr.read(sensorValues);

    m1Speed = baseSpeed;
    m2Speed = baseSpeed;

    pidControl(kp, ki, kd);

    m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
    m2Speed = constrain(m2Speed, minSpeed, maxSpeed);

    setMotorSpeed(m1Speed, m2Speed);
}

// calculate PID value based on error, kp, kd, ki, p, i and d.
void pidControl(float kp, float ki, float kd) {
    int error = map(qtr.readLineBlack(sensorValues), 0, 5000, -50, 50);

    p = error;
    i = i + error;
    d = error - lastError;
    int motorSpeed = kp * p + ki * i + kd * d;  

    if (error < 0) {
    m1Speed += motorSpeed;

    if (error <= (-1 * bigAngleTh)) {
        m2Speed -= adjustValue;
        m1Speed += adjustValue;
    }
} else if (error > 0) {
    m2Speed -= motorSpeed;

    if (error >= bigAngleTh) {
        m1Speed -= adjustValue;
        m2Speed += adjustValue;
    }
}

    lastError = error;
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {
    if (motor1Speed == 0) {
        digitalWrite(m11Pin, LOW);
        digitalWrite(m12Pin, LOW);
        analogWrite(m1Enable, motor1Speed);
    } else {
        if (motor1Speed > 0) {
            digitalWrite(m11Pin, HIGH);
            digitalWrite(m12Pin, LOW);
            analogWrite(m1Enable, motor1Speed);
        }
        if (motor1Speed < 0) {
            digitalWrite(m11Pin, LOW);
            digitalWrite(m12Pin, HIGH);
            analogWrite(m1Enable, -motor1Speed);
        }
    }
    if (motor2Speed == 0) {
        digitalWrite(m21Pin, LOW);
        digitalWrite(m22Pin, LOW);
        analogWrite(m2Enable, motor2Speed);
    } else {
        if (motor2Speed > 0) {
            digitalWrite(m21Pin, HIGH);
            digitalWrite(m22Pin, LOW);
            analogWrite(m2Enable, motor2Speed);
        }
        if (motor2Speed < 0) {
            digitalWrite(m21Pin, LOW);
            digitalWrite(m22Pin, HIGH);
            analogWrite(m2Enable, -motor2Speed);
        }
    }
}