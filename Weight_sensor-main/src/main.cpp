#include <Arduino.h>        
#include <Wire.h>           
#include <LiquidCrystal_I2C.h> 
#include "HX711.h"          
#include <WiFi.h>           
#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"
#include <ESP32Servo.h>     

// --- Cau hinh LCD ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 
#define I2C_SDA 21
#define I2C_SCL 22

// --- Cau hinh HX711 ---
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 5;
HX711 scale;

// --- Cau hinh Servo ---
#define SERVO_PIN 17
Servo myServo;

// --- Cau hinh ESP-NOW Serial ---
// Module 1 MAC: 20:E7:C8:67:39:70 (ESP32)
// Module 2 MAC: 10:20:BA:49:CD:D0 (ESP32-S3)
#define ESPNOW_WIFI_CHANNEL 1
const MacAddress peer_mac({0x10, 0x20, 0xBA, 0x49, 0xCD, 0xD0}); // MAC cua Module 2
ESP_NOW_Serial_Class NowSerial(peer_mac, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA);

// --- He so hieu chuan ---
float calibration_factor = 401.94;

// --- Bien cho May trang thai (State Machine) ---
enum ScaleState {
  CONNECTING,
  WAITING,
  MEASURING,
  DISPLAYING
};
ScaleState currentState = CONNECTING;
bool hasDisplayed = false;
bool isConnected = false;

unsigned long measurementStartTime = 0;     
float finalWeight = 0.0;          

// --- Cau hinh ---
const float TRIGGER_WEIGHT = 30.0; // Nguong de bat dau can (gram)
const float REMOVE_WEIGHT = 10.0;  // Nguong de reset (gram)
const float DEAD_ZONE = 2.0;       
const int MEASURE_TIME = 3000;     // Thoi gian do (3 giay)

// --- Cau hinh Servo ---
const int SERVO_STEP_DELAY = 25;    // Độ trễ giữa các bước cho phân loại (ms)
const int SERVO_STEP_SIZE = 2;      // Độ lớn mỗi bước quay cho phân loại (độ)
const int RETURN_STEP_DELAY = 25;   // Độ trễ giữa các bước khi đưa về giữa (ms)
const int RETURN_STEP_SIZE = 2;     // Độ lớn mỗi bước quay khi đưa về giữa (độ)

// Hàm gửi kết quả cân nặng qua ESP-NOW Serial
void sendWeightResult(float weight_kg) {
  // Chuyển sang gram
  float weight_g = weight_kg * 1000.0;
  
  // Tạo chuỗi format: "Khoi_luong:XXX.XXXg\n"
  char buffer[50];
  snprintf(buffer, sizeof(buffer), "Khoi_luong:%.3fg\n", weight_g);
  
  // Gửi qua ESP-NOW Serial
  if (NowSerial.availableForWrite()) {
    NowSerial.print(buffer);
    Serial.printf(">>> Gửi: %s", buffer);
    isConnected = true;
  } else {
    Serial.println(">>> ESP-NOW Serial không sẵn sàng!");
  }
}

// Hàm di chuyển servo từ từ
void moveServoSlowly(int startAngle, int endAngle, bool isReturning = false) {
  int stepDelay = isReturning ? RETURN_STEP_DELAY : SERVO_STEP_DELAY;
  int stepSize = isReturning ? RETURN_STEP_SIZE : SERVO_STEP_SIZE;
  
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle += stepSize) {
      myServo.write(angle);
      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle -= stepSize) {
      myServo.write(angle);
      delay(stepDelay);
    }
  }
  myServo.write(endAngle); // Đảm bảo đến đúng vị trí cuối
}

void setup() {
  Serial.begin(115200);
  Serial.print("Dia chi MAC cua ESP (sender): ");
  Serial.println(WiFi.macAddress());

  // Khởi động HX711
  Serial.println("Khoi dong HX711...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare(); 
  Serial.println("HX711 san sang.");

  // Khởi động LCD
  Serial.println("Khoi dong LCD I2C...");
  Wire.begin(I2C_SDA, I2C_SCL); 
  lcd.init();
  lcd.backlight();
  
  // Khởi động Servo
  ESP32PWM::allocateTimer(0);
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  myServo.write(90);  // Đưa servo về vị trí giữa
  
  // Khởi động ESP-NOW Serial
  Serial.println("Khoi dong ESP-NOW Serial...");
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  
  while (!WiFi.STA.started()) {
    delay(100);
  }
  
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Channel: ");
  Serial.println(ESPNOW_WIFI_CHANNEL);
  Serial.print("Peer MAC (Module 2): ");
  Serial.println("10:20:BA:49:CD:D0");
  
  // Khởi động ESP-NOW Serial
  Serial.println("ESP-NOW communication starting...");
  NowSerial.begin(115200);
  Serial.println("ESP-NOW Serial san sang.");
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ket noi voi bang");
  lcd.setCursor(0, 1);
  lcd.print("chuyen...");
  Serial.println("Dang o trang thai CONNECTING.");
}

void loop() {
  // --- LENH TRU BI KHAN CAP ---
  if (Serial.available()) {
    char temp = Serial.read();
    if ((temp == 't' || temp == 'T') && (currentState == WAITING || currentState == DISPLAYING)) {
      scale.tare(); 
      Serial.println("DA TRU BI!");
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("DA TRU BI!");
      delay(1000); 
      currentState = WAITING; 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("San sang can!");
    }
  }

  // --- CO MAY TRANG THAI CHINH ---
  switch (currentState) {
    case CONNECTING: {
      // Gửi gói dữ liệu để kiểm tra kết nối
      if (NowSerial.availableForWrite()) {
        NowSerial.print("Khoi_luong:0.000g\n");
        isConnected = true;
      }
      
      if (isConnected) {
        // Đã kết nối thành công
        Serial.println("Da ket noi voi ESP kia!");
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Da ket noi!");
        delay(2000);
        
        // Chuyển sang trạng thái WAITING
        currentState = WAITING;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("San sang can!");
      }
      break;
    }
    
    case WAITING: {
      float currentWeight = scale.get_units(5);
      
      // === PHAN SUA DOI DE LOAI BO NHAY SO VA SO AM ===
      float displayWeight_kg;

      // Neu khoi luong am hoac nam trong "vung chet", thi coi la 0
      if (currentWeight <= 0 || (currentWeight > -DEAD_ZONE && currentWeight < DEAD_ZONE)) {
        displayWeight_kg = 0.0;
      } else {
        displayWeight_kg = currentWeight / 1000.0;
      }
      
      // Hien thi "live" da duoc xu ly
      lcd.setCursor(0, 1);
      lcd.print(displayWeight_kg, 3);
      lcd.print(" kg "); // Co khoang trang de xoa so cu
      // === KET THUC SUA DOI ===

      // Kiem tra de bat dau can (van dung gia tri GOC)
      if (currentWeight > TRIGGER_WEIGHT) {
        Serial.println("Phat hien vat nang > 30g. Bat dau do...");
        currentState = MEASURING;
        measurementStartTime = millis(); 
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Dang do...");
      }
      break;
    }
    
    case MEASURING: {
      unsigned long elapsed = millis() - measurementStartTime;
      
      // Đọc cân nặng hiện tại
      float currentWeight = scale.get_units(3);  // Lấy 3 mẫu để đọc nhanh
      float currentWeight_kg = currentWeight / 1000.0;

      // Hiển thị "Dang do" và cân nặng
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Dang do...");
      
      // Hiển thị cân nặng ở hàng 2
      lcd.setCursor(0, 1);
      if (currentWeight <= 0 || (currentWeight > -DEAD_ZONE && currentWeight < DEAD_ZONE)) {
        lcd.print("0.000 kg");
      } else {
        lcd.print(currentWeight_kg, 3);
        lcd.print(" kg");
      }

      // Kiểm tra hết giờ
      if (elapsed >= MEASURE_TIME) {
        Serial.println("Het gio. Lay ket qua.");
        finalWeight = scale.get_units(10);  // Lấy kết quả cuối với 10 mẫu
        hasDisplayed = false;
        currentState = DISPLAYING;
      }
      break;
    }
    
    case DISPLAYING: {
      // --- KHOI LOGIC CHI CHAY MOT LAN ---
      if (!hasDisplayed) {
        // Chuyen sang kg và hiển thị kết quả
        float finalWeight_kg = finalWeight / 1000.0;
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Khoi luong:");
        lcd.setCursor(0, 1);
        lcd.print(finalWeight_kg, 3);
        lcd.print(" kg");
        
        delay(2000); // Hiển thị kết quả 2 giây
        
        // Gửi dữ liệu qua ESP-NOW ngay sau khi có kết quả
        sendWeightResult(finalWeight_kg);
        
        // Thông báo đẩy xuống băng chuyền
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Day xuong");
        lcd.setCursor(0, 1);
        lcd.print("bang chuyen...");
        
        delay(2000); // Chờ 2 giây
        
        // Điều khiển servo - quay một hướng
        int currentAngle = myServo.read();
        Serial.println("Quay servo de day hang xuong...");
        moveServoSlowly(currentAngle, 180);
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Cho lay hang...");
        
        hasDisplayed = true;
      }

      // Kiểm tra cân đã về 0 chưa
      float currentWeight = scale.get_units(5);
      if (currentWeight < REMOVE_WEIGHT) {
        // Chờ và kiểm tra lại để đảm bảo cân đã ổn định
        delay(500);
        currentWeight = scale.get_units(5);
        
        if (currentWeight < REMOVE_WEIGHT) {
          Serial.println("Can da ve 0, dua servo ve giua...");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Dua servo ve...");
          
          // Đưa servo về giữa với tốc độ nhanh hơn
          int currentAngle = myServo.read();
          moveServoSlowly(currentAngle, 90, true);  // true = sử dụng tốc độ trả về
          
          currentState = WAITING;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("San sang can!");
        }
      }
      break;
    }
  }

  delay(200);
}