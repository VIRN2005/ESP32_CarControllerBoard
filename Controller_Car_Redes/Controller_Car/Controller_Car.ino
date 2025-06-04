#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs
#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

// ========== PINES DEL L298N ==========
const int rightMotorPin1 = 18;  // IN1 
const int rightMotorPin2 = 19;  // IN2
const int enableRightMotor = 5; // ENA (PWM)

const int leftMotorPin1 = 16;   // IN3
const int leftMotorPin2 = 17;   // IN4  
const int enableLeftMotor = 4;  // ENB (PWM)

const int ledPin = 2;

// Variables
int motorSpeed = 200;  // Velocidad fija para pruebas
bool deviceConnected = false;
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

// ========== FUNCIONES DE MOTOR BÁSICAS ==========
void setupMotors() {
  Serial.println("🔧 Configurando pines de motores...");
  
  // Configurar todos los pines como OUTPUT
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(enableRightMotor, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  // Inicializar todo en LOW/0
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, 0);
  analogWrite(enableLeftMotor, 0);
  digitalWrite(ledPin, LOW);
  
  Serial.println("✅ Pines configurados");
}

void testMotorsIndividually() {
  Serial.println("🧪 === PRUEBA INDIVIDUAL DE MOTORES ===");
  
  // Test motor derecho adelante
  Serial.println("1. Motor DERECHO ADELANTE por 3 segundos...");
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  analogWrite(enableRightMotor, motorSpeed);
  delay(3000);
  analogWrite(enableRightMotor, 0);
  digitalWrite(rightMotorPin1, LOW);
  delay(1000);
  
  // Test motor izquierdo adelante  
  Serial.println("2. Motor IZQUIERDO ADELANTE por 3 segundos...");
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableLeftMotor, motorSpeed);
  delay(3000);
  analogWrite(enableLeftMotor, 0);
  digitalWrite(leftMotorPin1, LOW);
  delay(1000);
  
  // Test ambos motores
  Serial.println("3. AMBOS MOTORES ADELANTE por 3 segundos...");
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, motorSpeed);
  analogWrite(enableLeftMotor, motorSpeed);
  delay(3000);
  stopAllMotors();
  
  Serial.println("✅ Prueba individual completada");
}

void stopAllMotors() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, 0);
  analogWrite(enableLeftMotor, 0);
  Serial.println("🛑 Todos los motores detenidos");
}

void moveForward() {
  Serial.println("🚗 ADELANTE - Velocidad: " + String(motorSpeed));
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, motorSpeed);
  analogWrite(enableLeftMotor, motorSpeed);
}

void moveBackward() {
  Serial.println("🔄 ATRÁS - Velocidad: " + String(motorSpeed));
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  analogWrite(enableRightMotor, motorSpeed);
  analogWrite(enableLeftMotor, motorSpeed);
}

void turnLeft() {
  Serial.println("⬅️ IZQUIERDA");
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  analogWrite(enableRightMotor, motorSpeed);
  analogWrite(enableLeftMotor, motorSpeed);
}

void turnRight() {
  Serial.println("➡️ DERECHA");
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, motorSpeed);
  analogWrite(enableLeftMotor, motorSpeed);
}

// ========== BLE CALLBACKS ==========
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(ledPin, HIGH);
      Serial.println("🟢 Cliente BLE conectado");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(ledPin, LOW);
      stopAllMotors();
      Serial.println("🔴 Cliente BLE desconectado");
      BLEDevice::startAdvertising();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String receivedValue = pCharacteristic->getValue().c_str();
      
      Serial.println("📨 Comando recibido: '" + receivedValue + "'");
      
      // Procesar comando simple
      if (receivedValue == "F" || receivedValue == "f") {
        moveForward();
      }
      else if (receivedValue == "B" || receivedValue == "b") {
        moveBackward();
      }
      else if (receivedValue == "L" || receivedValue == "l") {
        turnLeft();
      }
      else if (receivedValue == "R" || receivedValue == "r") {
        turnRight();
      }
      else if (receivedValue == "S" || receivedValue == "s") {
        stopAllMotors();
      }
      else {
        Serial.println("❌ Comando desconocido: " + receivedValue);
      }
    }
};

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("🚗 ESP32 Car - Versión de Diagnóstico Simple");
  Serial.println("===============================================");
  
  // Configurar motores
  setupMotors();
  
  // IMPORTANTE: Hacer prueba automática al inicio
  Serial.println("⚠️ INICIANDO PRUEBA AUTOMÁTICA EN 5 SEGUNDOS...");
  Serial.println("⚠️ ASEGÚRATE DE QUE LAS RUEDAS ESTÉN LIBRES!");
  delay(5000);
  
  testMotorsIndividually();
  
  // Configurar BLE
  Serial.println("🔧 Configurando BLE...");
  BLEDevice::init("ESP32_Car_Simple");
  
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  
  Serial.println("✅ Sistema listo!");
  Serial.println("📱 Conecta desde tu app o usa Serial Monitor");
  Serial.println("💡 Comandos: F=adelante, B=atrás, L=izq, R=der, S=parar");
  Serial.println("===============================================");
}

// ========== LOOP ==========
void loop() {
  // Comandos desde Serial para debug
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "F" || command == "f") {
      moveForward();
    }
    else if (command == "B" || command == "b") {
      moveBackward();
    }
    else if (command == "L" || command == "l") {
      turnLeft();
    }
    else if (command == "R" || command == "r") {
      turnRight();
    }
    else if (command == "S" || command == "s") {
      stopAllMotors();
    }
    else if (command == "TEST") {
      testMotorsIndividually();
    }
    else if (command.startsWith("SPEED:")) {
      int newSpeed = command.substring(6).toInt();
      if (newSpeed >= 50 && newSpeed <= 255) {
        motorSpeed = newSpeed;
        Serial.println("⚡ Nueva velocidad: " + String(motorSpeed));
      }
    }
    else {
      Serial.println("❌ Comando desconocido: " + command);
    }
  }
  
  // LED de estado
  if (!deviceConnected) {
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      digitalWrite(ledPin, !digitalRead(ledPin));
      lastBlink = millis();
    }
  }
  
  delay(50);
}