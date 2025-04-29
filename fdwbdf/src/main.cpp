#include <Arduino.h>
#include <Ps3Controller.h>
#include <Wire.h>
#include <MPU6050.h>

// --- Déclarations moteur
#define M1_IN1 2
#define M1_IN2 4
#define M2_IN1 5
#define M2_IN2 18
#define M3_IN1 19
#define M3_IN2 21
#define M4_IN1 22
#define M4_IN2 23

// --- Calibration moteurs
float motor1_gain = 1.00;
float motor2_gain = 0.95;
float motor3_gain = 1.05;
float motor4_gain = 0.98;
float motor_with_max_gain = motor3_gain;  // pour remap

// --- PID rotation
struct PID {
  float Kp, Ki, Kd;
  float integral;
  float previous_error;
};

PID pid_rotation = {1.5, 0.0, 0.05, 0, 0};  // À ajuster après tests

float computePID(PID &pid, float setpoint, float actual) {
  float error = setpoint - actual;
  pid.integral += error;
  float derivative = error - pid.previous_error;
  pid.previous_error = error;
  float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
  return output;
}

// --- IMU
MPU6050 imu;

// --- Remap PWM
int remap(float value, float val_min, float val_max) {
  if (value <= val_min) return 0;
  if (value >= val_max) return 255;
  return (value - val_min) * 255 / (val_max - val_min);
}

// --- PS3 Events
void notify() {
  if (Ps3.data.button.cross)    Serial.println("Pressing the cross button");
  if (Ps3.data.button.square)   Serial.println("Pressing the square button");
  if (Ps3.data.button.triangle) Serial.println("Pressing the triangle button");
  if (Ps3.data.button.circle)   Serial.println("Pressing the circle button");
}

void onConnect() {
  Serial.println("Connected!.");
}

// --- Setup
void setup() {
  Serial.begin(115200);
  Wire.begin();

  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println("Erreur IMU !");
    while (1);
  }

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("00:00:00:00:00:00"); // Remplace par ton adresse Bluetooth
  
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);

  Serial.println("Ready.");
}

// --- Loop
void loop() {
  int8_t ly = -Ps3.data.analog.stick.ly; // Joystick gauche (avance/recul)
  int8_t rx = Ps3.data.analog.stick.rx;  // Joystick droit (rotation)

  float forward = ly / 127.0; // [-1 .. 1]
  float turn = rx / 127.0;    // [-1 .. 1]

  // Lire rotation actuelle avec Gyroscope
  int16_t gyroZ;
  imu.getRotation(NULL, NULL, &gyroZ); 
  float rotation_actual = gyroZ / 131.0;  // en degrés/seconde (standard MPU6050)

  // Correction PID sur la rotation
  float rotation_setpoint = turn * 90;  // Viser max 90°/s rotation
  float correction = computePID(pid_rotation, rotation_setpoint, rotation_actual);

  // Calcul des vitesses moteurs gauche/droite corrigées
  float left_speed = forward * 255 + correction;
  float right_speed = forward * 255 - correction;

  // Limiter pour éviter dépassement PWM
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

  // Application sur moteurs
  if (abs(ly) > 10 || abs(rx) > 10) { // Deadzone
    // Avant ou arrière
    int pwm1, pwm2, pwm3, pwm4;

    if (left_speed >= 0) {
      pwm1 = remap(left_speed * motor1_gain, 0, 254 * motor_with_max_gain);
      pwm3 = remap(left_speed * motor3_gain, 0, 254 * motor_with_max_gain);
      analogWrite(M1_IN1, pwm1);
      analogWrite(M1_IN2, 0);
      analogWrite(M3_IN1, pwm3);
      analogWrite(M3_IN2, 0);
    } else {
      pwm1 = remap(-left_speed * motor1_gain, 0, 254 * motor_with_max_gain);
      pwm3 = remap(-left_speed * motor3_gain, 0, 254 * motor_with_max_gain);
      analogWrite(M1_IN1, 0);
      analogWrite(M1_IN2, pwm1);
      analogWrite(M3_IN1, 0);
      analogWrite(M3_IN2, pwm3);
    }

    if (right_speed >= 0) {
      pwm2 = remap(right_speed * motor2_gain, 0, 254 * motor_with_max_gain);
      pwm4 = remap(right_speed * motor4_gain, 0, 254 * motor_with_max_gain);
      analogWrite(M2_IN1, pwm2);
      analogWrite(M2_IN2, 0);
      analogWrite(M4_IN1, pwm4);
      analogWrite(M4_IN2, 0);
    } else {
      pwm2 = remap(-right_speed * motor2_gain, 0, 254 * motor_with_max_gain);
      pwm4 = remap(-right_speed * motor4_gain, 0, 254 * motor_with_max_gain);
      analogWrite(M2_IN1, 0);
      analogWrite(M2_IN2, pwm2);
      analogWrite(M4_IN1, 0);
      analogWrite(M4_IN2, pwm4);
    }

  } else {
    // Stop
    analogWrite(M1_IN1, 0); analogWrite(M1_IN2, 0);
    analogWrite(M2_IN1, 0); analogWrite(M2_IN2, 0);
    analogWrite(M3_IN1, 0); analogWrite(M3_IN2, 0);
    analogWrite(M4_IN1, 0); analogWrite(M4_IN2, 0);
  }

  delay(10); // Pour éviter de saturer le bus I2C et Serial
}
