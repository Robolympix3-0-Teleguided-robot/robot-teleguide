#include <Arduino.h>
#include <PS3Controller.h>
#include <Wire.h>
#include <MPU6050.h>

// --- Déclarations moteur
#define MFR_1 2
#define MFR_2 4
#define MFL_1 5
#define MFL_2 18
#define MBR_1 19
#define MBR_2 21
#define MBL_1 22
#define MBL_2 23






void setMotor(int speed, int pin1, int pin2) {
  if (speed > 10) {
    analogWrite(pin1, speed);
    analogWrite(pin2, 0);
  } else if (speed < -10) {
    analogWrite(pin1, 0);
    analogWrite(pin2, -speed);
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

void move_back_or_forward(int speed){
  setMotor(speed,MFL_1,MFL_2);
  setMotor(speed,MFR_1,MFR_2);
  setMotor(speed,MBL_1,MBL_2);
  setMotor(speed,MBR_1,MBR_2);

}


// --- Ps3 Events
void notify() {
  if (Ps3.data.button.cross)    Serial.println("Pressing the cross button");
  if (Ps3.data.button.square)   Serial.println("Pressing the square button");
  if (Ps3.data.button.triangle) Serial.println("Pressing the triangle button");
  if (Ps3.data.button.circle)   Serial.println("Pressing the circle button");
  Serial.println("left x = " + String(Ps3.data.analog.stick.lx) + " y = " + String(Ps3.data.analog.stick.ly));
  Serial.println("right x = " + String(Ps3.data.analog.stick.rx) + " y = " + String(Ps3.data.analog.stick.ry));
}

void onConnect() {
  Serial.println("Connected!.");
}

// --- Setup
void setup() {
  Serial.begin(115200);
  

  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("00:00:00:00:00:00"); // Remplace par ton adresse Bluetooth
  
  pinMode(MFR_1, OUTPUT);
  pinMode(MFR_2, OUTPUT);
  pinMode(MFL_1, OUTPUT);
  pinMode(MFL_2, OUTPUT);
  pinMode(MBR_1, OUTPUT);
  pinMode(MBR_2, OUTPUT);
  pinMode(MBL_1, OUTPUT);
  pinMode(MBL_2, OUTPUT);

  Serial.println("Ready.");
}

// --- Loop
void loop() {
  if (Ps3.isConnected()) {
    int ly = Ps3.data.analog.stick.ly; // Stick gauche Y
    int rx = Ps3.data.analog.stick.rx; // Stick droit X
    move_back_or_forward(ly);
  }
  delay(10); // Pour éviter de saturer le bus I2C et Serial
}
