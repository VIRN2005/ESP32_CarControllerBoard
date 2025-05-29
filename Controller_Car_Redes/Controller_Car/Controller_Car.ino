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

// Pin LED indicador (opcional)
const int ledPin = 2;

// Configuración de PWM
const int PWMFreq = 5000;
const int PWMResolution = 8;

// Variables de velocidad
int currentSpeed = 200;  // Velocidad actual (100-255)
int turnSpeed = 150;     // Velocidad para giros
const int minSpeed = 100;
const int maxSpeed = 255;

// Variables de control
String command = "";
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000; // Timeout en ms para parar si no hay comandos
bool isMoving = false;
unsigned long lastHeartbeat = 0;

void setUpPinModes() {
  // Pines como salida
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Configurar PWM directamente en los pines
  ledcAttach(enableRightMotor, PWMFreq, PWMResolution);
  ledcAttach(enableLeftMotor, PWMFreq, PWMResolution);

  // Iniciar motores apagados
  rotateMotor(0, 0);
  
  // LED indicador
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
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
  
  // LED indicador de movimiento
  if (leftSpeed != 0 || rightSpeed != 0) {
    digitalWrite(ledPin, HIGH);
    isMoving = true;
  } else {
    digitalWrite(ledPin, LOW);
    isMoving = false;
  }
}

// Funciones de movimiento mejoradas
void moveForward() {
  rotateMotor(currentSpeed, currentSpeed);
  Serial.println("Adelante - Velocidad: " + String(currentSpeed));
  SerialBT.println("FORWARD_OK:" + String(currentSpeed));
  lastCommandTime = millis();
}

void moveBackward() {  
  rotateMotor(-currentSpeed, -currentSpeed);
  Serial.println("Atrás - Velocidad: " + String(currentSpeed));
  SerialBT.println("BACKWARD_OK:" + String(currentSpeed));
  lastCommandTime = millis();
}

void turnLeft() {
  rotateMotor(-turnSpeed, turnSpeed);
  Serial.println("Izquierda - Velocidad: " + String(turnSpeed));
  SerialBT.println("LEFT_OK:" + String(turnSpeed));
  lastCommandTime = millis();
}

void turnRight() {
  rotateMotor(turnSpeed, -turnSpeed);
  Serial.println("Derecha - Velocidad: " + String(turnSpeed));
  SerialBT.println("RIGHT_OK:" + String(turnSpeed));
  lastCommandTime = millis();
}

void stopMotors() {
  rotateMotor(0, 0);
  Serial.println("Parado");
  SerialBT.println("STOP_OK");
  isMoving = false;
}

// Función para procesar comandos de velocidad
void setSpeed(int newSpeed) {
  // Limitar velocidad entre min y max
  newSpeed = constrain(newSpeed, minSpeed, maxSpeed);
  currentSpeed = newSpeed;
  turnSpeed = max(80, newSpeed - 50); // Velocidad de giro proporcional
  
  Serial.println("Velocidad establecida: " + String(currentSpeed));
  SerialBT.println("SPEED_OK:" + String(currentSpeed));
}

// Función principal para procesar comandos
void processCommand(String cmd) {
  cmd.trim();
  
  // Comando de velocidad SPEED:XXX
  if (cmd.startsWith("SPEED:")) {
    int colonIndex = cmd.indexOf(':');
    if (colonIndex != -1) {
      String speedStr = cmd.substring(colonIndex + 1);
      int newSpeed = speedStr.toInt();
      if (newSpeed >= minSpeed && newSpeed <= maxSpeed) {
        setSpeed(newSpeed);
      } else {
        SerialBT.println("ERROR:Invalid speed range " + String(minSpeed) + "-" + String(maxSpeed));
      }
    }
    return;
  }
  
  // Convertir a mayúsculas para comandos de dirección
  cmd.toUpperCase();
  
  // Comandos de dirección
  if (cmd == "F" || cmd == "FORWARD" || cmd == "ADELANTE") {
    moveForward();
  }
  else if (cmd == "B" || cmd == "BACKWARD" || cmd == "ATRAS") {
    moveBackward();
  }
  else if (cmd == "L" || cmd == "LEFT" || cmd == "IZQUIERDA") {
    turnLeft();
  }
  else if (cmd == "R" || cmd == "RIGHT" || cmd == "DERECHA") {
    turnRight();
  }
  else if (cmd == "S" || cmd == "STOP" || cmd == "PARAR") {
    stopMotors();
  }
  // Comandos de velocidad alternativos
  else if (cmd == "+" || cmd == "PLUS" || cmd == "MAS") {
    setSpeed(currentSpeed + 20);
  }
  else if (cmd == "-" || cmd == "MINUS" || cmd == "MENOS") {
    setSpeed(currentSpeed - 20);
  }
  // Comandos de información
  else if (cmd == "STATUS" || cmd == "ESTADO") {
    SerialBT.println("=== ESP32 CAR STATUS ===");
    SerialBT.println("Current Speed: " + String(currentSpeed));
    SerialBT.println("Turn Speed: " + String(turnSpeed));
    SerialBT.println("Moving: " + String(isMoving ? "YES" : "NO"));
    SerialBT.println("Connection: OK");
    SerialBT.println("Commands: F/B/L/R/S or SPEED:XXX");
  }
  else if (cmd == "PING" || cmd == "HEARTBEAT") {
    SerialBT.println("PONG");
    lastHeartbeat = millis();
  }
  else if (cmd == "INFO") {
    SerialBT.println("ESP32_Car_Controller_v2.0");
    SerialBT.println("Compatible with React Native BLE");
  }
  // Comando no reconocido
  else if (cmd.length() > 0) {
    SerialBT.println("ERROR:Unknown command '" + cmd + "'");
    SerialBT.println("Available: F,B,L,R,S,SPEED:XXX,STATUS");
  }
}

// Función de seguridad mejorada
void safetyCheck() {
  // Parar automáticamente si no hay comandos recientes
  if (isMoving && (millis() - lastCommandTime > commandTimeout)) {
    stopMotors();
    Serial.println("Safety stop - No commands received");
    SerialBT.println("SAFETY_STOP");
  }
  
  // Parpadear LED si está conectado pero sin comandos
  if (millis() - lastHeartbeat > 5000 && !isMoving) {
    digitalWrite(ledPin, !digitalRead(ledPin));
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  setUpPinModes();
  
  // Inicializar Bluetooth Serial
  SerialBT.begin("ESP32_Car_Controller_Redes");
  
  Serial.println("=== ESP32 Car Controller v2.0 ===");
  Serial.println("Compatible with React Native BLE App");
  Serial.println("Device Name: ESP32_Car_Controller_Redes");
  Serial.println("Ready for connections...");
  
  // Mensaje de bienvenida por Bluetooth
  SerialBT.println("=== ESP32 CAR CONNECTED ===");
  SerialBT.println("Controller v2.0 Ready");
  SerialBT.println("Commands supported:");
  SerialBT.println("- F/B/L/R/S (Direction)");
  SerialBT.println("- SPEED:XXX (Set speed 100-255)");
  SerialBT.println("- STATUS (Get info)");
  SerialBT.println("- PING (Connection test)");
  SerialBT.println("Ready to receive commands!");
  
  lastHeartbeat = millis();
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
  
  // Verificaciones de seguridad
  safetyCheck();
  
  // Delay pequeño para no saturar el procesador
  delay(50);
}