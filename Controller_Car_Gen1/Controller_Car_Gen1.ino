#include "BluetoothSerial.h"

// Verificar si Bluetooth está habilitado
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// Pines para el motor derecho
const int rightMotorPin1 = 18;
const int rightMotorPin2 = 19;
const int enableRightMotor = 5;

// Pines para el motor izquierdo
const int leftMotorPin1 = 16;
const int leftMotorPin2 = 17;
const int enableLeftMotor = 4;

// Configuración de PWM
const int PWMFreq = 5000;
const int PWMResolution = 8;

// Variables de velocidad
int baseSpeed = 200;  // Velocidad base (0-255)
int turnSpeed = 150;  // Velocidad para giros

// Variables de control
String command = "";
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 500; // Timeout en ms para parar si no hay comandos

void setUpPinModes() {
  // Pines como salida
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Configurar PWM directamente en los pines
  ledcAttach(enableRightMotor, PWMFreq, PWMResolution);
  ledcAttach(enableLeftMotor, PWMFreq, PWMResolution);

  // Iniciar motores apagados
  rotateMotor(0, 0);
}

// Función para controlar velocidad (-255 a 255)
void rotateMotor(int leftSpeed, int rightSpeed) {
  // Motor izquierdo
  if (leftSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  // Motor derecho
  if (rightSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  // PWM velocidad absoluta
  ledcWrite(enableLeftMotor, abs(leftSpeed));
  ledcWrite(enableRightMotor, abs(rightSpeed));
}

// Funciones de movimiento
void moveForward() {
  rotateMotor(baseSpeed, baseSpeed);
  Serial.println("Adelante");
  SerialBT.println("Adelante");
}

void moveBackward() {
  rotateMotor(-baseSpeed, -baseSpeed);
  Serial.println("Atrás");
  SerialBT.println("Atrás");
}

void turnLeft() {
  rotateMotor(-turnSpeed, turnSpeed);
  Serial.println("Izquierda");
  SerialBT.println("Izquierda");
}

void turnRight() {
  rotateMotor(turnSpeed, -turnSpeed);
  Serial.println("Derecha");
  SerialBT.println("Derecha");
}

void stopMotors() {
  rotateMotor(0, 0);
  Serial.println("Parado");
}

void processCommand(String cmd) {
  cmd.toUpperCase();
  cmd.trim();
  
  if (cmd == "F" || cmd == "FORWARD" || cmd == "ADELANTE") {
    moveForward();
    lastCommandTime = millis();
  }
  else if (cmd == "B" || cmd == "BACKWARD" || cmd == "ATRAS") {
    moveBackward();
    lastCommandTime = millis();
  }
  else if (cmd == "L" || cmd == "LEFT" || cmd == "IZQUIERDA") {
    turnLeft();
    lastCommandTime = millis();
  }
  else if (cmd == "R" || cmd == "RIGHT" || cmd == "DERECHA") {
    turnRight();
    lastCommandTime = millis();
  }
  else if (cmd == "S" || cmd == "STOP" || cmd == "PARAR") {
    stopMotors();
  }
  else if (cmd == "+" || cmd == "PLUS" || cmd == "MAS") {
    baseSpeed = min(255, baseSpeed + 20);
    turnSpeed = min(255, turnSpeed + 15);
    Serial.println("Velocidad aumentada: " + String(baseSpeed));
    SerialBT.println("Velocidad: " + String(baseSpeed));
  }
  else if (cmd == "-" || cmd == "MINUS" || cmd == "MENOS") {
    baseSpeed = max(100, baseSpeed - 20);
    turnSpeed = max(80, turnSpeed - 15);
    Serial.println("Velocidad disminuida: " + String(baseSpeed));
    SerialBT.println("Velocidad: " + String(baseSpeed));
  }
  else if (cmd == "STATUS" || cmd == "ESTADO") {
    SerialBT.println("Velocidad actual: " + String(baseSpeed));
    SerialBT.println("Comandos: F/B/L/R/S/+/-");
  }
  else if (cmd.length() > 0) {
    SerialBT.println("Comando no reconocido: " + cmd);
    SerialBT.println("Usa: F(adelante), B(atrás), L(izquierda), R(derecha), S(parar), +(más rápido), -(más lento)");
  }
}

void setup() {
  Serial.begin(115200);
  setUpPinModes();
  
  // Inicializar Bluetooth Serial
  SerialBT.begin("ESP32_Car_Controller_Gen1"); // Nombre del dispositivo Bluetooth
  Serial.println("ESP32 Car Controller iniciado");
  Serial.println("Dispositivo Bluetooth: ESP32_Car_Controller");
  Serial.println("Comandos: F(adelante), B(atrás), L(izquierda), R(derecha), S(parar)");
  
  SerialBT.println("=== ESP32 Car Controller ===");
  SerialBT.println("Comandos disponibles:");
  SerialBT.println("F o FORWARD - Adelante");
  SerialBT.println("B o BACKWARD - Atrás");
  SerialBT.println("L o LEFT - Izquierda");
  SerialBT.println("R o RIGHT - Derecha");
  SerialBT.println("S o STOP - Parar");
  SerialBT.println("+ - Aumentar velocidad");
  SerialBT.println("- - Disminuir velocidad");
  SerialBT.println("STATUS - Ver estado");
}

void loop() {
  // Leer comandos desde Bluetooth
  if (SerialBT.available()) {
    command = SerialBT.readString();
    processCommand(command);
  }
  
  // Leer comandos desde Serial (para debug)
  if (Serial.available()) {
    command = Serial.readString();
    processCommand(command);
  }
  
  // Parar automáticamente si no hay comandos recientes (seguridad)
  if (millis() - lastCommandTime > commandTimeout && lastCommandTime > 0) {
    stopMotors();
    lastCommandTime = 0; // Reset para evitar parar continuamente
  }
  
  delay(50);
}