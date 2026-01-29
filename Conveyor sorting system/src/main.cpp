/************************************************************
 * ESP32-S3 + TMC2209 - Conveyor with Product Sorting System
 * MODULE 2: Băng chuyền phân loại (ESP-NOW Serial Receiver)
 * Module 1 MAC: 20:E7:C8:67:39:70 (ESP32)
 * Module 2 MAC: 10:20:BA:49:CD:D0 (ESP32-S3)
 * START: Chạy băng chuyền
 * STOP: Dừng băng chuyền
 * WEIGHT: Tăng/giảm khối lượng thủ công (test mode)
 * Sorting Logic (nhận từ Module 1 qua ESP-NOW Serial):
 *   Format: "Khoi_luong:XXX.XXXg"
 *   0-50g: Servo1 @ 70° (bin 1)
 *   50-200g: Servo1 @ 0°, Servo2 @ 115° (bin 2)
 *   200-1000g: Both @ home position (bin 3 - end of conveyor)
 * Pins:
 *   TMC2209: DIR=12, STEP=13, EN=14
 *   Buttons: START=4, STOP=5, WEIGHT=6
 *   LCD I2C: SDA=38, SCL=39
 *   SR04: TRIG=1, ECHO=2
 *   Servos: SERVO1=36, SERVO2=45
 ************************************************************/
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"

// ==== WEIGHT SETTINGS ====
#define MIN_WEIGHT 100   // grams - Khối lượng tối thiểu
#define MAX_WEIGHT 1000  // grams - Khối lượng tối đa
#define WEIGHT_STEP 100  // grams - Bước tăng/giảm mỗi lần nhấn

// TMC2209 Pins
#define PIN_DIR   12
#define PIN_STEP  13
#define PIN_EN    14

// Button Pins
#define BTN_START  4
#define BTN_STOP   5
#define BTN_ESTOP  6

// SR04 Ultrasonic Sensor Pins
#define US_TRIG  1
#define US_ECHO  2
#define US_TIMEOUT_US 10000

// LCD I2C Pins (ESP32-S3)
#define PIN_SDA 38
#define PIN_SCL 39

// Servo Pins
#define SERVO1_PIN 36
#define SERVO2_PIN 45

// Servo angles for sorting
#define SERVO1_HOME 175
#define SERVO1_SORT 45
#define SERVO2_HOME 180
#define SERVO2_SORT 115

// Motor Parameters
static const float SPEED_STEPS_S  = 3500.0f;   // steps/second
static const float ACCEL_STEPS_S2 = 30000.0f;  // steps/second^2

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

// LCD object
LiquidCrystal_I2C* lcd = nullptr;

// Servo objects
Servo servo1;
Servo servo2;

// State variables
bool isRunning = false;
bool directionForward = true;  // true = forward, false = backward

// Product counting variables
int productCount = 0;
bool objectDetected = false;
const float DETECTION_THRESHOLD = 70.0;  // mm - ngưỡng phát hiện sản phẩm
unsigned long lastCountTime = 0;
const unsigned long COUNT_COOLDOWN = 500;  // ms - thời gian chờ giữa 2 lần đếm
int currentWeight = 0;  // Khối lượng hiện tại (có thể điều chỉnh bằng nút hoặc nhận từ ESP-NOW Serial)
bool isIncreasing = true;  // true = đang tăng, false = đang giảm

// ESP-NOW Serial variables
#define ESPNOW_WIFI_CHANNEL 1
const MacAddress peer_mac({0x20, 0xE7, 0xC8, 0x67, 0x39, 0x70}); // MAC cua Module 1
ESP_NOW_Serial_Class NowSerial(peer_mac, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA);

bool newWeightReceived = false;
float receivedWeightKg = 0.0;
String receiveBuffer = "";  // Buffer để lưu dữ liệu nhận được

// Button state structure (similar to test code)
struct Btn {
  uint8_t pin;
  bool lastRaw;
  bool stable;
  uint32_t tDeb;
  uint32_t tDown;
};

enum Event { EV_NONE, EV_PRESS };

const uint16_t DEBOUNCE_MS = 25;

Btn btnStart = {BTN_START, HIGH, HIGH, 0, 0};
Btn btnStop  = {BTN_STOP, HIGH, HIGH, 0, 0};
Btn btnEstop = {BTN_ESTOP, HIGH, HIGH, 0, 0};

// Forward declarations
void updateLCD();
void displayStatus(const char* line1, const char* line2);
float readDistance_mm();
void checkProductDetection();
void handleStartButton();
void handleStopButton();
void handleWeightButton();
Event pollButton(Btn &b);
void sortProduct(int weight);
void resetServos();
void processReceivedData();

// Hàm xử lý dữ liệu nhận từ ESP-NOW Serial
void processReceivedData() {
  // Đọc dữ liệu từ NowSerial
  while (NowSerial.available()) {
    char c = NowSerial.read();
    
    if (c == '\n' || c == '\r') {
      // Đã nhận đủ một dòng, xử lý
      if (receiveBuffer.length() > 0 && receiveBuffer.startsWith("Khoi_luong:")) {
        // Parse chuỗi: "Khoi_luong:XXX.XXXg"
        // Bỏ phần "Khoi_luong:" (11 ký tự) và lấy phần số
        String weightStr = receiveBuffer.substring(11); // Bỏ "Khoi_luong:"
        weightStr.replace("g", ""); // Xóa ký tự 'g'
        weightStr.trim(); // Xóa khoảng trắng thừa
        
        Serial.printf("[DEBUG] Buffer: '%s'\n", receiveBuffer.c_str());
        Serial.printf("[DEBUG] weightStr after parse: '%s'\n", weightStr.c_str());
        
        float weight_g = weightStr.toFloat();
        
        // Cập nhật khối lượng - Dùng trực tiếp giá trị nhận được
        currentWeight = (int)weight_g;
        receivedWeightKg = weight_g / 1000.0;
        newWeightReceived = true;
          
        Serial.println("=================================================");
        Serial.println(">>> ESP-NOW Serial: Received weight data");
        Serial.printf("    Raw data: %s\n", receiveBuffer.c_str());
        Serial.printf("    Parsed weight string: %s\n", weightStr.c_str());
        Serial.printf("    Weight: %.3f kg (%d g)\n", receivedWeightKg, currentWeight);
        Serial.println("=================================================");
        
        // TỰ ĐỘNG BẬT BĂNG CHUYỀN nếu chưa chạy
        if (!isRunning && currentWeight > 0) {
          isRunning = true;
          if (directionForward) {
            stepper->runForward();
          } else {
            stepper->runBackward();
          }
          Serial.println(">>> AUTO-START: Conveyor started automatically!");
        }
        
        // Update LCD with received weight - Show "Ready to sort"
        if (lcd) {
          lcd->clear();
          lcd->setCursor(0, 0);
          lcd->print("Nhan du lieu:");
          lcd->setCursor(0, 1);
          lcd->print(currentWeight);
          lcd->print("g - San sang");
        }
        
        delay(2000);  // Show for 2 seconds
        updateLCD();  // Return to normal display
      }
      
      // Reset buffer
      receiveBuffer = "";
    } else {
      // Thêm ký tự vào buffer
      receiveBuffer += c;
      
      // Giới hạn buffer size
      if (receiveBuffer.length() > 100) {
        receiveBuffer = "";
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("=== Conveyor Control System - MODULE 2 ===");
  Serial.println("=== ESP-NOW Serial Receiver ===");
  
  // Initialize ESP-NOW Serial
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  
  while (!WiFi.STA.started()) {
    delay(100);
  }
  
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Channel: ");
  Serial.println(ESPNOW_WIFI_CHANNEL);
  Serial.print("Peer MAC (Module 1): ");
  Serial.println("20:E7:C8:67:39:70");
  
  // Khởi động ESP-NOW Serial
  NowSerial.begin(115200);
  Serial.println("[ESP-NOW Serial] Initialized successfully");
  Serial.println("[ESP-NOW Serial] Ready to receive weight data from Module 1");

  // Initialize button pins with internal pull-up
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(BTN_ESTOP, INPUT_PULLUP);
  
  // Initialize SR04 sensor pins
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);
  
  // Initialize I2C and LCD
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);  // 100 kHz
  Serial.printf("[I2C] SDA=%d  SCL=%d\n", PIN_SDA, PIN_SCL);
  
  // Scan for I2C LCD address
  Serial.println("[I2C] Scanning for LCD...");
  uint8_t lcdAddr = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      if (!lcdAddr) lcdAddr = addr;
    }
    delay(3);
  } 
  
  if (lcdAddr) {
    lcd = new LiquidCrystal_I2C(lcdAddr, 16, 2);
    lcd->init();
    lcd->begin(16, 2);
    lcd->backlight();
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("Conveyor System");
    lcd->setCursor(0, 1);
    lcd->print("Initializing...");
    Serial.println("[LCD] LCD initialized!");
    delay(2000);
  } else {
    Serial.println("[LCD] No LCD found - continuing without display");
  }

  // Initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  resetServos();
  Serial.println("[Servo] Servos initialized at home position");

  // Initialize stepper motor
  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (!stepper) {
    Serial.println("ERROR: Stepper init failed!");
    return;
  }

  // Configure stepper
  stepper->setDirectionPin(PIN_DIR, false);
  stepper->setEnablePin(PIN_EN, true);   // EN active LOW
  stepper->setAutoEnable(true);

  stepper->setSpeedInHz((uint32_t)SPEED_STEPS_S);
  stepper->setAcceleration((uint32_t)ACCEL_STEPS_S2);

  Serial.println("System ready!");
  Serial.println("START: GPIO4 | STOP: GPIO5 | WEIGHT: GPIO6");
  Serial.println("SR04 Sensor: TRIG=GPIO1 | ECHO=GPIO2");
  Serial.println("Product counter initialized.");
  Serial.printf("Weight range: %d-%dg, Step: %dg\n", MIN_WEIGHT, MAX_WEIGHT, WEIGHT_STEP);
  
  // Display initial LCD screen
  updateLCD();
}

// Update LCD display with current count and weight
void updateLCD() {
  if (!lcd) return;  // Skip if LCD not available
  
  lcd->clear();
  
  // Line 1: Product count
  lcd->setCursor(0, 0);
  lcd->print("Count: ");
  lcd->print(productCount);
  
  // Line 2: Weight
  lcd->setCursor(0, 1);
  lcd->print("Weight: ");
  lcd->print(currentWeight);
  lcd->print("g");
}

// Display status message on LCD (temporary, 2 seconds)
void displayStatus(const char* line1, const char* line2) {
  if (!lcd) return;
  
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print(line1);
  if (line2) {
    lcd->setCursor(0, 1);
    lcd->print(line2);
  }
  delay(2000);
  
  // Return to normal display
  updateLCD();
}

// Read distance from SR04 sensor
float readDistance_mm() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);
  
  unsigned long dur = pulseIn(US_ECHO, HIGH, US_TIMEOUT_US);
  if (dur == 0) return -1.0;  // No echo
  return (dur * 0.343f) * 0.5f;  // Return distance in mm
}

// Reset servos to home position
void resetServos() {
  servo1.write(SERVO1_HOME);
  servo2.write(SERVO2_HOME);
}

// Sort product based on weight
void sortProduct(int weight) {
  Serial.println(">>> Sorting product...");
  
  if (weight > 0 && weight <= 50) {
    // Light product: Servo1 @ 70°
    Serial.println("    Category: Light (0-50g) -> Bin 1");
    
    // Display on LCD
    if (lcd) {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Sorting: BIN 1");
      lcd->setCursor(0, 1);
      lcd->print("Light: ");
      lcd->print(weight);
      lcd->print("g");
    }
    
    servo1.write(SERVO1_SORT);
    servo2.write(SERVO2_HOME);
    delay(4000);  // Wait longer for product to pass completely (4 seconds)
    resetServos();
    
  } else if (weight > 50 && weight <= 200) {
    // Medium product: Servo2 @ 115°
    Serial.println("    Category: Medium (50-200g) -> Bin 2");
    
    // Display on LCD
    if (lcd) {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Sorting: BIN 2");
      lcd->setCursor(0, 1);
      lcd->print("Medium: ");
      lcd->print(weight);
      lcd->print("g");
    }
    
    servo1.write(SERVO1_HOME);
    servo2.write(SERVO2_SORT);
    delay(4000);  // Wait longer for product to pass completely (4 seconds)
    resetServos();
    
  } else {
    // Heavy product: Both @ home position (pass through)
    Serial.println("    Category: Heavy (200-1000g) -> Bin 3 (End)");
    
    // Display on LCD
    if (lcd) {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Sorting: BIN 3");
      lcd->setCursor(0, 1);
      lcd->print("Heavy: ");
      lcd->print(weight);
      lcd->print("g");
    }
    
    resetServos();
    delay(1500);
  }
}

// Check for product and count
void checkProductDetection() {
  float distance = readDistance_mm();
  
  // Only check if conveyor is running
  if (!isRunning) return;
  
  if (distance > 0 && distance < DETECTION_THRESHOLD) {
    // Object detected within threshold
    if (!objectDetected && (millis() - lastCountTime > COUNT_COOLDOWN)) {
      objectDetected = true;
      productCount++;
      lastCountTime = millis();
      
      Serial.println("=================================================");
      Serial.print(">>> Product detected! Count: ");
      Serial.println(productCount);
      Serial.printf("    Distance: %.1f mm\n", distance);
      
      // Sử dụng khối lượng từ ESP-NOW nếu có, nếu không dùng currentWeight
      int weightToUse = currentWeight;
      if (newWeightReceived) {
        Serial.printf("    Weight (ESP-NOW): %d g (%.3f kg)\n", currentWeight, receivedWeightKg);
        newWeightReceived = false;  // Reset flag
      } else {
        Serial.printf("    Weight (Manual): %d g\n", currentWeight);
      }
      
      // Sort product based on weight
      sortProduct(weightToUse);
      Serial.println("=================================================");
      
      // Update LCD display
      updateLCD();
    }
  } else {
    // No object or out of range
    objectDetected = false;
  }
}

void handleStartButton() {
  if (!isRunning) {
    isRunning = true;
    if (directionForward) {
      stepper->runForward();
      Serial.println(">> Conveyor STARTED (Forward)");
    } else {
      stepper->runBackward();
      Serial.println(">> Conveyor STARTED (Backward)");
    }
    
    // Display on LCD
    displayStatus("START", "Khoi Dong");
  }
}

void handleStopButton() {
  if (isRunning) {
    isRunning = false;
    stepper->forceStopAndNewPosition(0);
    Serial.println(">> Conveyor STOPPED");
    
    // Display on LCD
    displayStatus("STOP", "Tam Dung");
  }
}

void handleWeightButton() {
  // Điều chỉnh khối lượng
  if (isIncreasing) {
    currentWeight += WEIGHT_STEP;
    if (currentWeight >= MAX_WEIGHT) {
      currentWeight = MAX_WEIGHT;
      isIncreasing = false;  // Đổi sang giảm
      Serial.println(">> Weight reached MAX, switching to DECREASE mode");
    }
  } else {
    currentWeight -= WEIGHT_STEP;
    if (currentWeight <= MIN_WEIGHT) {
      currentWeight = MIN_WEIGHT;
      isIncreasing = true;  // Đổi sang tăng
      Serial.println(">> Weight reached MIN, switching to INCREASE mode");
    }
  }
  
  Serial.print(">> Weight adjusted to: ");
  Serial.print(currentWeight);
  Serial.println("g");
  
  // Update LCD display
  updateLCD();
}

Event pollButton(Btn &b) {
  bool raw = digitalRead(b.pin);
  if (raw != b.lastRaw) { 
    b.tDeb = millis(); 
    b.lastRaw = raw; 
  }

  if (millis() - b.tDeb > DEBOUNCE_MS) {
    if (b.stable != raw) {
      // Trạng thái ổn định vừa đổi
      b.stable = raw;
      if (b.stable == LOW) {
        b.tDown = millis();  // Vừa nhấn xuống
      } else {
        // Vừa nhả ra -> phát hiện sự kiện
        return EV_PRESS;
      }
    }
  }
  return EV_NONE;
}

void loop() {
  // Process received data from ESP-NOW Serial
  processReceivedData();
  
  // Check all buttons with debounce (detect on button release)
  Event e1 = pollButton(btnStart);
  Event e2 = pollButton(btnStop);
  Event e3 = pollButton(btnEstop);

  if (e1 == EV_PRESS) {
    Serial.println(">> Button START pressed");
    handleStartButton();
  }
  
  if (e2 == EV_PRESS) {
    Serial.println(">> Button STOP pressed");
    handleStopButton();
  }
  
  if (e3 == EV_PRESS) {
    Serial.println(">> Button WEIGHT pressed");
    handleWeightButton();
  }
  
  // Check for product detection when conveyor is running
  checkProductDetection();
  
  delay(10);  // Small delay for stability
}
