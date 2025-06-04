#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h> // Keep this for other utilities if needed, but not for decodeBase64
#include <BLE2902.h>
#include <vector>
#include <string> // Required for std::string

// UUIDs que coinciden con tu app React Native
#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

// ========== CONFIGURACIÓN DE PINES ==========
// Para L298N Motor Driver:
const int rightMotorPin1 = 18;  // IN1
const int rightMotorPin2 = 19;  // IN2
const int enableRightMotor = 5; // ENA (PWM)

const int leftMotorPin1 = 16;   // IN3
const int leftMotorPin2 = 17;   // IN4
const int enableLeftMotor = 4;  // ENB (PWM)

// Pin LED indicador
const int ledPin = 2;

// ========== VARIABLES DE VELOCIDAD ==========
int currentSpeed = 180;        
int turnSpeed = 120;           
const int minSpeed = 80;       
const int maxSpeed = 255;
const int testSpeed = 150;     

// Variables de control
String command = "";
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 3000; // 3 segundos
bool isMoving = false;
bool deviceConnected = false;
bool motorsEnabled = true;     
bool bleInitialized = false;

// Variables BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;

// Contadores para debug
int commandCount = 0;
int responseCount = 0;

// ========== DECLARACIONES DE FUNCIONES ==========
void stopMotors();
void processCommand(String cmd);
void rotateMotor(int leftSpeed, int rightSpeed);
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void setSpeed(int newSpeed);
void sendResponse(String response);
void safetyCheck();
void setUpPinModes();
void testMotors();
void debugMotorStatus();
void debugBLEStatus();

// Custom Base64 Decode Function
// Base64 encoding table (standard)
static const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Helper function to get the value of a base64 char
inline int base64_val(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1; // Invalid character
}

std::string base64_decode(const std::string &encoded_string) {
    size_t len = encoded_string.length();
    if (len == 0) return "";

    // Calculate padding
    int padding = 0;
    if (len >= 1 && encoded_string[len - 1] == '=') padding++;
    if (len >= 2 && encoded_string[len - 2] == '=') padding++;

    // For short strings, pad them to make them valid Base64
    std::string padded_string = encoded_string;
    while (padded_string.length() % 4 != 0) {
        padded_string += '=';
    }
    
    len = padded_string.length();
    
    // Calculate output length
    size_t decoded_len = (len * 3) / 4 - padding;
    std::string decoded_string;
    decoded_string.reserve(decoded_len);

    size_t i = 0;
    while (i < len) {
        int b1 = base64_val(padded_string[i++]);
        if (b1 == -1) continue; // Skip invalid chars

        if (i >= len) break;
        int b2 = base64_val(padded_string[i++]);
        if (b2 == -1) continue; // Skip invalid chars

        decoded_string += (char)((b1 << 2) | ((b2 & 0x30) >> 4)); // First byte

        if (i < len && padded_string[i] != '=') {
            int b3 = base64_val(padded_string[i++]);
            if (b3 == -1) continue; // Skip invalid chars
            decoded_string += (char)(((b2 & 0x0F) << 4) | ((b3 & 0x3C) >> 2)); // Second byte
            
            if (i < len && padded_string[i] != '=') {
                int b4 = base64_val(padded_string[i++]);
                if (b4 == -1) continue; // Skip invalid chars
                decoded_string += (char)(((b3 & 0x03) << 6) | b4); // Third byte
            } else {
                // Handle padding == 1
                if (i < len) i++; // Skip '='
                break;
            }
        } else {
            // Handle padding == 2
            if (i < len) i++; // Skip '='
            if (i < len && padded_string[i] == '=') {
                i++; // Skip second '='
            }
            break;
        }
    }
    return decoded_string;
}

// ========== CALLBACKS BLE CON MÁS DEBUG ==========
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("🟢 ===== CLIENTE BLE CONECTADO =====");
      Serial.println("🔗 Conexión BLE establecida exitosamente");
      digitalWrite(ledPin, HIGH);
      sendResponse("CONNECTED_OK");
      
      // Debug adicional
      Serial.println("📊 Estado después de conexión:");
      Serial.println("   - Device Connected: " + String(deviceConnected));
      Serial.println("   - BLE Initialized: " + String(bleInitialized));
      Serial.println("   - Server OK: " + String(pServer != NULL));
      Serial.println("   - Characteristic OK: " + String(pCharacteristic != NULL));
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("🔴 ===== CLIENTE BLE DESCONECTADO =====");
      stopMotors();
      digitalWrite(ledPin, LOW);
      
      // Reiniciar advertising
      Serial.println("🔄 Reiniciando advertising BLE...");
      BLEDevice::startAdvertising();
      Serial.println("📡 Esperando nueva conexión BLE...");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      commandCount++;
      
      // Get the received value, which is Base64 encoded from React Native
      String rxValueEncoded = pCharacteristic->getValue().c_str();
      
      Serial.println("📨 ===== COMANDO BLE RECIBIDO (RAW) =====");
      Serial.println("📋 Raw (Base64) Comando #" + String(commandCount) + ": '" + rxValueEncoded + "'");
      Serial.println("📏 Longitud Raw: " + String(rxValueEncoded.length()) + " caracteres");
      
      // Debug: Print each character and its ASCII value
      Serial.print("🔍 Caracteres hex: ");
      for (int i = 0; i < rxValueEncoded.length(); i++) {
        Serial.print("0x");
        Serial.print((int)rxValueEncoded[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      
      if (rxValueEncoded.length() == 0) {
        Serial.println("❌ Comando vacío recibido");
        sendResponse("ERROR:Empty_command");
        return;
      }
      
      // Decode the Base64 string back to original command using our improved function
      std::string decodedStdString = base64_decode(rxValueEncoded.c_str());
      String rxValue = String(decodedStdString.c_str()); // Convert std::string to Arduino String
      
      Serial.println("✅ Comando Decodificado: '" + rxValue + "'");
      Serial.println("📏 Longitud Decodificada: " + String(rxValue.length()) + " caracteres");

      // Debug: Print decoded characters
      if (rxValue.length() > 0) {
        Serial.print("🔍 Decoded hex: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print("0x");
          Serial.print((int)rxValue[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        
        Serial.println("✅ Procesando comando decodificado...");
        processCommand(rxValue);
        lastCommandTime = millis();
        
        // Confirmación inmediata
        sendResponse("CMD_RECEIVED:" + rxValue);
      } else {
        Serial.println("❌ Comando vacío después de decodificación");
        Serial.println("🔧 Intentando procesar comando original sin decodificar...");
        
        // Fallback: try processing the original encoded string
        if (rxValueEncoded.length() == 1) {
          Serial.println("📝 Procesando como comando de un caracter: '" + rxValueEncoded + "'");
          processCommand(rxValueEncoded);
          lastCommandTime = millis();
          sendResponse("CMD_RECEIVED_FALLBACK:" + rxValueEncoded);
        } else {
          sendResponse("ERROR:Decode_failed_and_not_single_char");
        }
      }
      
      Serial.println("==============================");
    }
};

// ========== CONFIGURACIÓN DE PINES SIMPLIFICADA ==========
void setUpPinModes() {
  Serial.println("🔧 ===== CONFIGURANDO PINES =====");
  
  // Pines como salida
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Usar analogWrite para PWM (más compatible)
  pinMode(enableRightMotor, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);

  // Inicializar todo apagado
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, 0);
  analogWrite(enableLeftMotor, 0);
  
  Serial.println("✅ Pines configurados correctamente");
  Serial.println("📋 Configuración de pines:");
  Serial.println("   - Motor Derecho: Pin1=" + String(rightMotorPin1) + ", Pin2=" + String(rightMotorPin2) + ", PWM=" + String(enableRightMotor));
  Serial.println("   - Motor Izquierdo: Pin1=" + String(leftMotorPin1) + ", Pin2=" + String(leftMotorPin2) + ", PWM=" + String(enableLeftMotor));
  Serial.println("   - LED: Pin=" + String(ledPin));
  
  // Test LED - 5 parpadeos para confirmar
  Serial.println("🔄 Probando LED...");
  for(int i = 0; i < 5; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
  Serial.println("✅ Test LED completado");
}

// ========== CONTROL DE MOTORES SIMPLIFICADO ==========
void rotateMotor(int leftSpeed, int rightSpeed) {
  if (!motorsEnabled) {
    Serial.println("⚠️ MOTORES DESHABILITADOS - Ignorando comando");
    return;
  }

  Serial.println("🎮 ===== CONTROL DE MOTORES =====");
  Serial.println("🎯 Comando: Izq=" + String(leftSpeed) + ", Der=" + String(rightSpeed));
  
  // ===== MOTOR IZQUIERDO =====
  Serial.println("⬅️ Configurando motor izquierdo...");
  if (leftSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    analogWrite(enableLeftMotor, abs(leftSpeed));
    Serial.println("   ✅ Motor Izq: ADELANTE, PWM=" + String(abs(leftSpeed)));
  } else if (leftSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
    analogWrite(enableLeftMotor, abs(leftSpeed));
    Serial.println("   ✅ Motor Izq: ATRÁS, PWM=" + String(abs(leftSpeed)));
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
    analogWrite(enableLeftMotor, 0);
    Serial.println("   ✅ Motor Izq: PARADO");
  }

  // ===== MOTOR DERECHO =====
  Serial.println("➡️ Configurando motor derecho...");
  if (rightSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    analogWrite(enableRightMotor, abs(rightSpeed));
    Serial.println("   ✅ Motor Der: ADELANTE, PWM=" + String(abs(rightSpeed)));
  } else if (rightSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    analogWrite(enableRightMotor, abs(rightSpeed));
    Serial.println("   ✅ Motor Der: ATRÁS, PWM=" + String(abs(rightSpeed)));
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
    analogWrite(enableRightMotor, 0);
    Serial.println("   ✅ Motor Der: PARADO");
  }

  // Actualizar estado
  isMoving = (leftSpeed != 0 || rightSpeed != 0);
  Serial.println("🏃 Estado movimiento: " + String(isMoving ? "MOVIENDO" : "PARADO"));
  
  // Verificar estado de pines
  debugMotorStatus();
  Serial.println("==============================");
}

void moveForward() {
  Serial.println("🚗 COMANDO: ADELANTE - Velocidad: " + String(currentSpeed));
  rotateMotor(currentSpeed, currentSpeed);
  sendResponse("FORWARD_OK:" + String(currentSpeed));
}

void moveBackward() {  
  Serial.println("🔄 COMANDO: ATRÁS - Velocidad: " + String(currentSpeed));
  rotateMotor(-currentSpeed, -currentSpeed);
  sendResponse("BACKWARD_OK:" + String(currentSpeed));
}

void turnLeft() {
  Serial.println("⬅️ COMANDO: IZQUIERDA - Velocidad: " + String(turnSpeed));
  rotateMotor(-turnSpeed, turnSpeed);
  sendResponse("LEFT_OK:" + String(turnSpeed));
}

void turnRight() {
  Serial.println("➡️ COMANDO: DERECHA - Velocidad: " + String(turnSpeed));
  rotateMotor(turnSpeed, -turnSpeed);
  sendResponse("RIGHT_OK:" + String(turnSpeed));
}

void stopMotors() {
  Serial.println("🛑 COMANDO: PARAR MOTORES");
  rotateMotor(0, 0);
  sendResponse("STOP_OK");
  isMoving = false;
}

// ========== FUNCIONES DE PRUEBA ==========
void testMotors() {
  Serial.println("🧪 ===== TEST AUTOMÁTICO DE MOTORES =====");
  
  Serial.println("🔧 Test 1: Motor Izquierdo Adelante (2 seg)");
  rotateMotor(testSpeed, 0);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("🔧 Test 2: Motor Derecho Adelante (2 seg)");
  rotateMotor(0, testSpeed);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("🔧 Test 3: Ambos motores Adelante (2 seg)");
  rotateMotor(testSpeed, testSpeed);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("🔧 Test 4: Ambos motores Atrás (2 seg)");
  rotateMotor(-testSpeed, -testSpeed);
  delay(2000);
  stopMotors();
  
  Serial.println("✅ Test de motores completado");
  Serial.println("==============================");
}

void debugMotorStatus() {
  Serial.println("📊 ESTADO ACTUAL DE MOTORES:");
  Serial.println("   🔌 Pines digitales:");
  Serial.println("     Left: Pin1=" + String(digitalRead(leftMotorPin1)) + ", Pin2=" + String(digitalRead(leftMotorPin2)));
  Serial.println("     Right: Pin1=" + String(digitalRead(rightMotorPin1)) + ", Pin2=" + String(digitalRead(rightMotorPin2)));
  Serial.println("   ⚡ Estado general:");
  Serial.println("     Moving: " + String(isMoving));
  Serial.println("     Motors Enabled: " + String(motorsEnabled));
  Serial.println("     Current Speed: " + String(currentSpeed));
}

void debugBLEStatus() {
  Serial.println("📡 ===== ESTADO BLE =====");
  Serial.println("   🔗 Conectado: " + String(deviceConnected ? "SI" : "NO"));
  Serial.println("   🏗️ Inicializado: " + String(bleInitialized ? "SI" : "NO"));
  Serial.println("   🖥️ Server: " + String(pServer != NULL ? "OK" : "NULL"));
  Serial.println("   📡 Characteristic: " + String(pCharacteristic != NULL ? "OK" : "NULL"));
  Serial.println("   📨 Comandos recibidos: " + String(commandCount));
  Serial.println("   📤 Respuestas enviadas: " + String(responseCount));
  Serial.println("==============================");
}

// ========== FUNCIONES AUXILIARES ==========
void setSpeed(int newSpeed) {
  newSpeed = constrain(newSpeed, minSpeed, maxSpeed);
  currentSpeed = newSpeed;
  turnSpeed = max(80, newSpeed - 30); // Ensure turnSpeed is at least 80
  
  Serial.println("⚡ NUEVA VELOCIDAD: " + String(currentSpeed) + " (Turn: " + String(turnSpeed) + ")");
  sendResponse("SPEED_OK:" + String(currentSpeed));
}

void sendResponse(String response) {
  if (deviceConnected && pCharacteristic) {
    responseCount++;
    pCharacteristic->setValue(response.c_str());
    pCharacteristic->notify();
    Serial.println("📤 Respuesta #" + String(responseCount) + " enviada: '" + response + "'");
  } else {
    Serial.println("❌ No se puede enviar respuesta - BLE no disponible");
    Serial.println("   - Connected: " + String(deviceConnected));
    Serial.println("   - Characteristic: " + String(pCharacteristic != NULL));
  }
}

// ========== PROCESAMIENTO DE COMANDOS CON DEBUG ==========
void processCommand(String cmd) {
  cmd.trim(); // Eliminar espacios en blanco al inicio/fin
  
  Serial.println("🔄 ===== PROCESANDO COMANDO =====");
  Serial.println("📋 Comando original (después de trim): '" + cmd + "'");
  
  // Comando de velocidad SPEED:XXX
  if (cmd.startsWith("SPEED:")) {
    Serial.println("🎯 Detectado comando de velocidad");
    int colonIndex = cmd.indexOf(':');
    if (colonIndex != -1) {
      String speedStr = cmd.substring(colonIndex + 1);
      int newSpeed = speedStr.toInt();
      Serial.println("🔢 Nueva velocidad solicitada: " + String(newSpeed));
      if (newSpeed >= minSpeed && newSpeed <= maxSpeed) {
        setSpeed(newSpeed);
      } else {
        Serial.println("❌ Velocidad fuera de rango: " + String(newSpeed));
        sendResponse("ERROR:Invalid speed range " + String(minSpeed) + "-" + String(maxSpeed));
      }
    } else {
      Serial.println("❌ Formato de comando SPEED: incorrecto");
      sendResponse("ERROR:Malformed_SPEED_cmd");
    }
    return; // Importante para no procesar como otro comando
  }
  
  // Convertir a mayúsculas para comandos de una sola letra/palabra
  String upperCmd = cmd;
  upperCmd.toUpperCase();
  Serial.println("🔤 Comando en mayúsculas: '" + upperCmd + "'");
  
  // ===== COMANDOS DE MOVIMIENTO =====
  if (upperCmd == "F" || upperCmd == "FORWARD") {
    Serial.println("✅ Comando reconocido: ADELANTE");
    moveForward();
  }
  else if (upperCmd == "B" || upperCmd == "BACKWARD") {
    Serial.println("✅ Comando reconocido: ATRÁS");
    moveBackward();
  }
  else if (upperCmd == "L" || upperCmd == "LEFT") {
    Serial.println("✅ Comando reconocido: IZQUIERDA");
    turnLeft();
  }
  else if (upperCmd == "R" || upperCmd == "RIGHT") {
    Serial.println("✅ Comando reconocido: DERECHA");
    turnRight();
  }
  else if (upperCmd == "S" || upperCmd == "STOP") {
    Serial.println("✅ Comando reconocido: PARAR");
    stopMotors();
  }
  
  // ===== COMANDOS DE VELOCIDAD (incremento/decremento) =====
  else if (upperCmd == "+" || upperCmd == "PLUS") {
    Serial.println("✅ Comando reconocido: AUMENTAR VELOCIDAD");
    setSpeed(currentSpeed + 20);
  }
  else if (upperCmd == "-" || upperCmd == "MINUS") {
    Serial.println("✅ Comando reconocido: DISMINUIR VELOCIDAD");
    setSpeed(currentSpeed - 20);
  }
  
  // ===== COMANDOS DE DEBUG Y TEST =====
  else if (upperCmd == "TEST") {
    Serial.println("✅ Comando reconocido: TEST MOTORES");
    testMotors();
  }
  else if (upperCmd == "DEBUG") {
    Serial.println("✅ Comando reconocido: DEBUG");
    debugMotorStatus();
    debugBLEStatus();
  }
  else if (upperCmd == "ENABLE") {
    Serial.println("✅ Comando reconocido: HABILITAR MOTORES");
    motorsEnabled = true;
    sendResponse("MOTORS_ENABLED");
    Serial.println("✅ Motores habilitados");
  }
  else if (upperCmd == "DISABLE") {
    Serial.println("✅ Comando reconocido: DESHABILITAR MOTORES");
    stopMotors();
    motorsEnabled = false;
    sendResponse("MOTORS_DISABLED");
    Serial.println("❌ Motores deshabilitados");
  }
  
  // ===== COMANDOS DE STATUS =====
  else if (upperCmd == "STATUS") {
    Serial.println("✅ Comando reconocido: STATUS");
    debugBLEStatus();
    debugMotorStatus();
    sendResponse("STATUS_OK");
  }
  else if (upperCmd == "PING") {
    Serial.println("✅ Comando reconocido: PING");
    sendResponse("PONG");
  }
  else if (upperCmd == "INFO") {
    Serial.println("✅ Comando reconocido: INFO");
    sendResponse("ESP32_Car_DEBUG_v1.0");
  }
  else if (cmd.length() > 0) {
    Serial.println("❌ COMANDO DESCONOCIDO: '" + cmd + "'");
    sendResponse("ERROR:Unknown_cmd_'" + cmd + "'");
  }
  
  Serial.println("==============================");
}

void safetyCheck() {
  if (isMoving && (millis() - lastCommandTime > commandTimeout)) {
    Serial.println("⚠️ ===== SAFETY STOP =====");
    Serial.println("🕐 Sin comandos por " + String(commandTimeout) + "ms");
    stopMotors();
    sendResponse("SAFETY_STOP");
  }
}

// ========== SETUP PRINCIPAL ==========
void setup() {
  Serial.begin(115200);
  delay(3000); // Más tiempo para que se estabilice
  
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("🚗 ESP32 Car BLE Controller - DEBUG VERSION");
  Serial.println("🔧 Versión con diagnóstico completo");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  // Configurar hardware
  Serial.println("🔧 Paso 1: Configurando hardware...");
  setUpPinModes();
  
  // Test de motores
  Serial.println("🔧 Paso 2: Probando motores...");
  delay(1000);
  testMotors();
  
  // Inicializar BLE
  Serial.println("🔧 Paso 3: Inicializando BLE...");
  try {
    BLEDevice::init("Car_Redes_Debug");
    Serial.println("✅ BLE Device inicializado");
    
    pServer = BLEDevice::createServer();
    if (pServer) {
      Serial.println("✅ BLE Server creado");
      pServer->setCallbacks(new MyServerCallbacks());
      
      BLEService *pService = pServer->createService(SERVICE_UUID);
      if (pService) {
        Serial.println("✅ BLE Service creado");
        
        pCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID,
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_WRITE |
                            BLECharacteristic::PROPERTY_NOTIFY
                          );
        
        if (pCharacteristic) {
          Serial.println("✅ BLE Characteristic creado");
          pCharacteristic->setCallbacks(new MyCallbacks());
          // Adding a descriptor to the characteristic for notifications
          pCharacteristic->addDescriptor(new BLE2902()); 
          
          pService->start();
          Serial.println("✅ BLE Service iniciado");
          
          BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
          pAdvertising->addServiceUUID(SERVICE_UUID);
          pAdvertising->setScanResponse(true);
          pAdvertising->setMinPreferred(0x06); // Helps with faster connection on some devices
          pAdvertising->setMaxPreferred(0x12); // Interval
          
          BLEDevice::startAdvertising();
          Serial.println("✅ BLE Advertising iniciado");
          
          bleInitialized = true;
        } else {
          Serial.println("❌ Error creando BLE Characteristic");
        }
      } else {
        Serial.println("❌ Error creando BLE Service");
      }
    } else {
      Serial.println("❌ Error creando BLE Server");
    }
  } catch (const std::exception& e) {
    Serial.println("❌ Error inicializando BLE: " + String(e.what()));
  }
  
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  if (bleInitialized) {
    Serial.println("✅ SISTEMA COMPLETAMENTE LISTO");
  } else {
    Serial.println("❌ SISTEMA CON ERRORES EN BLE");
  }
  Serial.println("📡 Device Name: Car_Redes_Debug");
  Serial.println("🔑 Service UUID: " + String(SERVICE_UUID));
  Serial.println("🔑 Characteristic UUID: " + String(CHARACTERISTIC_UUID));
  Serial.println("📱 Esperando conexión desde React Native...");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("💡 Comandos disponibles por Serial:");
  Serial.println("   F/B/L/R/S - Movimiento básico");
  Serial.println("   SPEED:XXX - Ajustar velocidad (e.g., SPEED:180)");
  Serial.println("   + / - - Aumentar/Disminuir velocidad");
  Serial.println("   TEST - Probar motores");
  Serial.println("   DEBUG - Ver estado completo");
  Serial.println("   STATUS - Info del sistema");
  Serial.println("   ENABLE / DISABLE - Habilitar/Deshabilitar motores");
  Serial.println("   PING / INFO - Comandos de información");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

// ========== LOOP PRINCIPAL ==========
void loop() {
  static unsigned long lastStatusPrint = 0;
  unsigned long currentTime = millis();
  
  // Comandos desde Serial para debug
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println("🖥️ ===== COMANDO DESDE SERIAL =====");
    Serial.println("📋 Comando: '" + command + "'");
    processCommand(command);
  }
  
  // Verificaciones de seguridad
  safetyCheck();
  
  // LED de estado más informativo
  if (deviceConnected) {
    if (isMoving) {
      // LED sólido cuando se mueve
      digitalWrite(ledPin, HIGH);
    } else {
      // Parpadeo lento cuando conectado pero parado
      static unsigned long lastLedToggle = 0;
      if (currentTime - lastLedToggle > 1000) {
        digitalWrite(ledPin, !digitalRead(ledPin));
        lastLedToggle = currentTime;
      }
    }
  } else {
    // Parpadeo rápido cuando no conectado
    static unsigned long lastLedToggle = 0;
    if (currentTime - lastLedToggle > 250) {
      digitalWrite(ledPin, !digitalRead(ledPin));
      lastLedToggle = currentTime;
    }
  }
  
  // Status periódico cada 15 segundos
  if (currentTime - lastStatusPrint > 15000) {
    Serial.println("📊 ===== STATUS PERIÓDICO =====");
    Serial.println("🕐 Tiempo: " + String(currentTime/1000) + "s");
    debugBLEStatus();
    debugMotorStatus();
    lastStatusPrint = currentTime;
  }
  
  delay(50); // Pequeña pausa para estabilidad
}
