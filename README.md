# 🚘Integrative Safety Management System for Vehicles

## Introduction
The Integrative Safety Management System for Vehicles is designed to enhance vehicle safety by integrating multiple features into a single system.
This system aims to address critical safety issues such as driving under the influence, accident management, and pedestrian safety.
The system includes:
1. Alcohol Detection and Engine Locking System
2. Accident Assistance System (using GSM and GPS)
3. Pedestrian Safety System (slowing down the vehicle based on ultrasonic sensor data)

## Problem Statement
Vehicle accidents are a major concern globally, and various factors contribute to them, including impaired driving, accidents due to lack of immediate assistance, and insufficient pedestrian safety measures. This project aims to address these issues by implementing an integrated safety system that combines alcohol detection, accident assistance, and pedestrian safety features.

## Solution
The proposed system integrates three key safety components:
1. **Alcohol Detection and Engine Locking**: Measures the driver's blood alcohol concentration (BAC) and prevents the engine from starting if the BAC exceeds a predefined limit.
2. **Accident Assistance**: Utilizes GSM and GPS technologies to automatically send alerts and location information to emergency contacts or services in the event of an accident.
3. **Pedestrian Safety**: Uses ultrasonic sensors to detect pedestrians and automatically slows down the vehicle if a pedestrian is detected within a certain range.

## Components Used
- **Microcontroller**: ESP32 or Arduino
- **Alcohol Sensor**: MQ-3 or similar
- **GSM Module**: SIM800 or SIM900
- **GPS Module**: NEO-6M or similar
- **Ultrasonic Sensor**: HC-SR04
- **Relay Module**: For engine locking
- **Power Supply**: 12V DC (for vehicle power)
- **Miscellaneous**: Resistors, capacitors, connectors, wires

## Circuit
The circuit diagram involves connecting the sensors and modules to the microcontroller. Key connections include:
- **Alcohol Sensor**: Analog output to an ADC pin on the microcontroller
- **GSM Module**: Serial communication (TX/RX) with the microcontroller
- **GPS Module**: Serial communication (TX/RX) with the microcontroller
- **Ultrasonic Sensor**: Trigger and Echo pins connected to digital I/O pins on the microcontroller
- **Relay Module**: Controlled by a digital output pin from the microcontroller

![Circuit Diagram](path/to/your/circuit-diagram.png)

## Code
The code is divided into different modules for each component. Here's a high-level overview:

### Integrative solution code 
```cpp
#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define sensorDigital A0
#define Motor 9
#define buzzer 8
#define LED 6
#define bt_C 2

bool MQ3;

int trig = 7;
int echo = 4;
SoftwareSerial sim900aSerial(11,10);

TinyGPSPlus gps;
char phone_no[] = "+917204764847";

MPU6050 mpu;

int timeInmicron;
int distanceIncm;
bool isUltrasonicStop = false;
bool isDigitalStop = false;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int t = 20;

struct MyData {
  byte X;
  byte Y;
  byte Z;
};

MyData data;

unsigned long lastMessageTime = 0;
unsigned long messageInterval = 40000; // Interval between sending messages (milliseconds)

void setup() {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(Motor, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(sensorDigital, INPUT);
  pinMode(LED, OUTPUT);

  //lcd initialization
  Wire.begin();
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  lcd.setCursor(0, 0);
  lcd.clearWriteError();

  Serial.begin(9600);
  Serial.println("SAFETY MANAGEMENT SYSTEM");
  lcd.println("SAFETY MANAGMENT");
  lcd.setCursor(5,1);
  lcd.println("SYSTEM      ");
  delay(2000);
  lcd.clear();
  
  mpu.initialize();
  
  sim900aSerial.begin(9600);
  // Initialize SIM900A
  Serial.println("Initializing GSM module...");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("Initializing GSM");
  //initModule("AT", "OK", 1000); // Handshake test
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("Initialized                   ");
  delay(2000);

  // Wait for GPS initialization
  Serial.println("Waiting for GPS initialization...");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.println("Initializing GPS");
  delay(2000);
  waitForGPS();
}

void loop() {
  lcd.println("POWERED ON     ");
  readUltrasonic();
  readDigitalSensor();
  readMPU();

  printValues();

  controlSystem();

  if (isCollision()) {
    handleCollision();
  }

  processIncomingMessages();
}

void initModule(String cmd, char *res, int t) {
  while (1) {
    Serial.println(cmd);
    sim900aSerial.println(cmd);
    delay(100);
    while (sim900aSerial.available() > 0) {
      if (sim900aSerial.find(res)) {
        Serial.println(res);
        delay(t);
        return;
      } else {
        Serial.println("Error");
      }
    }
    delay(t);
  }
}

void callUp(char *number) {
 
  Serial.println("Calling UP");
  sim900aSerial.print("ATD"); sim900aSerial.print(number); sim900aSerial.println(";"); // Call the specific number
  delay(5000); // Wait for 20 seconds...
 sim900aSerial.println("ATH"); // Hang up
  delay(100);

}
void waitForGPS() {
  for (int i = 0; i < 10 && !gps.location.isValid(); i++) {
    delay(1000);
  }

  if (gps.location.isValid()) {
    Serial.println("GPS initialized and location valid");
  } else {
    Serial.println("GPS initialization failed or location not yet acquired");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.println("GPS,failed :(              ");
  }
}

void readUltrasonic() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  timeInmicron = pulseIn(echo, HIGH);
  distanceIncm = timeInmicron / 29 / 2;
}

void readDigitalSensor() {
  MQ3 = digitalRead(sensorDigital);
  delay(50);
}

void readMPU() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  data.X = map(ax, -17000, 17000, 0, 255); // X axis data
  data.Y = map(ay, -17000, 17000, 0, 255);
  data.Z = map(az, -17000, 17000, 0, 255); // Y axis data
  delay(500);
}

void printValues() {
  Serial.print("Object|");
  Serial.print("Alchohol||");
  Serial.print("Axis X|");
  Serial.print("Axis Y|");
  Serial.println("Axis Z|");
  Serial.print(distanceIncm);
  delay(1000);
  Serial.print("      ");
  Serial.print(MQ3);
  delay(100);
  Serial.print("      ");
  Serial.print(data.X);
  Serial.print("    ");
  Serial.print(data.Y);
  Serial.print("    ");
  Serial.println(data.Z);
  if (isUltrasonicStop) {
    
        Serial.println("Engine stopped due to ultrasonic sensor.");
       
        lcd.setCursor(0,0);
         lcd.clear();
        lcd.println("Object detected!");
        lcd.setCursor(0,1);
        lcd.println("Distance:");
        lcd.setCursor(9,1);
        lcd.println(distanceIncm);
        lcd.setCursor(10,1);
        lcd.println("          ");
    } else if (isDigitalStop) {
        Serial.println("Engine stopped due to digital sensor.");
        lcd.clear();
        lcd.println("Alcohol detected");
        lcd.setCursor(0,1);
       // lcd.println("Content:");
        lcd.setCursor(8,1);
        lcd.println(MQ3);
        lcd.setCursor(11,1);
        //lcd.println("          ");

    }
    else {
      lcd.clear();
    }
}

void controlSystem() {
  if (distanceIncm < 10) {
    digitalWrite(Motor, LOW);
    digitalWrite(buzzer, HIGH);
    isUltrasonicStop = true;
    isDigitalStop = false;
  } else if (MQ3==0) {
    digitalWrite(Motor, LOW);
    digitalWrite(LED, HIGH);
    Serial.println("sending SMS!");
    lcd.println("   SMS SENT       ");
    //sendSMSWithLocation();
    sendSMS("EMERGENCY ALERT::Driver is drunk!!");
    isUltrasonicStop = false;
    isDigitalStop = true;
  } else {
    lcd.clear();
    digitalWrite(Motor, HIGH);
    digitalWrite(buzzer, LOW);
    digitalWrite(LED, LOW);
    isUltrasonicStop = false;
    isDigitalStop = false;
  }
}

bool isCollision() {
  return (data.X < t || data.Y < t || data.Z < t);
}

void handleCollision() {
  Serial.println("collided");
  lcd.clear();
  lcd.println("COLLIDED !!           ");
  digitalWrite(buzzer, HIGH);
  unsigned long currentTime = millis();
  if (currentTime - lastMessageTime >= messageInterval) {
     delay(3000);
   if (digitalRead(bt_C) == 0){
    Serial.println("Calling UP");
    lcd.clear();
    delay(2000);
    lcd.println("CONTACTING...... ");
    lcd.setCursor(0,1);
    lcd.println("  +917204764847     ");
    callUp(phone_no);
    delay(500);
    Serial.println("sending SMS!");
    lcd.println("   SMS SENT       ");
    //sendSMSWithLocation();
    sendSMS("EMERGENCY ALERT::Collision detected, contact driver !!");
    lastMessageTime = currentTime;
    Serial.println("SMS sent to mobile number");
   }
     Serial.println("Button pressed, call hang up !");
  } else {
    digitalWrite(buzzer, LOW);
  }
}

void processIncomingMessages() {
  while (sim900aSerial.available()) {
    char c = sim900aSerial.read();
    Serial.write(c); // Print the received character to Serial monitor
  }
}

void sendSMSWithLocation() {
  if (gps.location.isValid()) {
    String message = "Latitude: " + String(gps.location.lat(), 6) + ", Longitude: " + String(gps.location.lng(), 6);
    sendSMS(message);
  } else {
    Serial.println("GPS location not available");
    lcd.clear();
    lcd.println("GPS unavailable !");
  }
}

void sendSMS(String message) {
 
  sim900aSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
  sim900aSerial.println("AT+CMGS=\"+917204764847\"");
  delay(1000);
  sim900aSerial.println(message);
  delay(1000);
  sim900aSerial.write(26); // Send Ctrl+Z
  delay(10000); // Wait for response
  Serial.println("SMS sent to mobile number");

}
```



## Conclusion
The Integrative Safety Management System effectively enhances vehicle safety by addressing key issues such as impaired driving, accident management, and pedestrian safety. The combination of alcohol detection, GSM-based accident assistance, and ultrasonic-based pedestrian safety provides a comprehensive solution to improve overall road safety. Future improvements could include integrating additional sensors, refining algorithms for better accuracy, and enhancing user interfaces.

---

Feel free to adjust the details to better fit your specific implementation or add additional sections as needed.
🪪License
This project is licensed under the MIT License - see the LICENSE file for details.

🤝Acknowledgments
Adafruit for the MPU6050 and OLED libraries.
Community contributions for similar projects and ideas.
Team members for the dedication and constant involvement.# Integrative-Safety-Management-System-for-Vehicles
