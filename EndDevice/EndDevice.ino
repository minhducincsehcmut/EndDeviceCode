#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdint.h>
//#include "./data_package.h"
#include <SoftwareSerial.h>

#define HEARTBEAT 1
#define DATA 2
#define RESPONSE 3 

// Define
#define TRIG_PIN 3
#define ECHO_PIN 2
#define MAX_DISTANCE 200 // Maximum distance to measure (adjust as needed)
//Config
uint8_t ID_1 = 1;
int status_1 = 0;
int floor_1 = 1;
int trash_1 = 1;
int signal_1 = 0;
uint8_t send_data_heartbeat[8];
uint8_t send_data[8];
int send_data_flag = 0;
int send_data_check = 0;
int statusFlag =false;
// Ultrasonic
int distance = 0;
unsigned long duration = 0;
int percentage = 0;

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns, and 2 rows
//Data Receive
const byte numBytes = 8;
byte buffer_reg[numBytes];
bool newData = false;
//countOpen
int countOpen = 0;
int flag = 0;


unsigned long lastUltrasonicTime = 0;
unsigned long ultrasonicInterval = 5000; // Interval for ultrasonic measurement in milliseconds

unsigned long lastHeartbeatTime = 0;
unsigned long heartbeatInterval = 30000; // Interval for heartbeat in milliseconds
//unsigned long heartbeatInterval = 5000; // Interval for heartbeat in milliseconds

unsigned long lastDataPackageTime = 0;
//unsigned long dataPackageInterval = 180000; // Interval for data package in milliseconds
unsigned long dataPackageInterval = 52000;//Interval for data package in milliseconds

bool countOpenFlag = false; // Flag to control countOpen increment


/******************************************************************************Function for Data Transmit***************************************************************************************************/
int checkSum(uint8_t data[]) {
  uint16_t check_sum = (data[5] << 8) + data[6];
  uint16_t sum = data[1];
  for (int i = 2; i < 5; i++)
          sum += data[i];
  if (sum == check_sum)
    return 1;
  return 0;
}

void heartbeatPackage(uint8_t ID, int status, int floor, int trash, int signal, uint8_t data[]) {
  data[0] = 0x0A;
  data[7] = 0x0B;
  data[1] = ID;
  data[2] = 0xA0;
  if (status)
    data[2] += 0x01;
  data[3] = (floor << 4) + trash;
  data[4] = ((signal / 10) << 4) + (signal % 10);
  uint16_t check_sum = 0;
  for (int i = 1; i < 5; i++)
    check_sum += data[i];
  data[5] = check_sum >> 8;
  data[6] = check_sum;
}

void dataPackage(uint8_t ID, int capacity, uint8_t data[]) {
  data[0] = 0x0A;
  data[7] = 0x0B;
  data[1] = ID;
  data[2] = 0xB0;
  data[3] = ((capacity / 10) << 4) + (capacity % 10);
  data[4] = 0x00;
  uint16_t check_sum = 0;
  for (int i = 1; i < 5; i++)
    check_sum += data[i];
  data[5] = check_sum >> 8;
  data[6] = check_sum;
}

int checkPackage(uint8_t data[]) {
  if (data[0] != 0x0A || data[7] != 0x0B) {
   printf("Data is invalid!");
    return 0;
  }
  if (!checkSum(data)) {
   printf("Checksum fail! Data is incorrect!");
    return 0;
  }
  return 1;
}

int getType(uint8_t data[]) {
  uint8_t command = data[2] >> 4;
  if (command == 0x0A)
    return HEARTBEAT;
  if (command == 0x0B)
    return DATA;
  if (command == 0x0C)
    return RESPONSE;
 printf("Data command is invalid!");
  return 0;
}

int getResponse(uint8_t data[]) {
  if (checkPackage(data) == 1) {
    if (getType(data) == RESPONSE) {
        uint8_t type = data[4] & 0x0f;
        uint16_t mess = ((data[2] & 0x0f) << 12) + (data[3] << 4) + ((data[4] & 0xf0) >> 4);
        uint8_t first = mess >> 8;
        uint8_t second = mess;
        if (first == 0x4F && second == 0x4B) {
          if (type == HEARTBEAT)
            return HEARTBEAT;
          if (type == DATA)
            return DATA;
        }
          return 0;
    }
  }
  return 0;
}
/******************************************************************************Function for Data Transmit***************************************************************************************************/
/*********************************************************************************Operate Functions*********************************************************************************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd.clear();
  lcd.init();      // initialize the lcd
  lcd.backlight(); // open the backlight
  lcd.print("Smart Trash Bin");
  lcd.clear();
  lcd.print("ChickDevs");
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
    // Trigger ultrasonic sensor every 5 seconds
  if (currentTime - lastUltrasonicTime >= ultrasonicInterval) {
    // Your ultrasonic measurement code here
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(200);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(TRIG_PIN, LOW);

    // Wait for the pulse to return
    delayMicroseconds(200); // Adjust this delay as needed

    // Receive ultrasonic sensor response
    duration = pulseIn(ECHO_PIN, HIGH);
    // Recalculate distance
    distance = duration * 0.034 / 2;
    
    // Update last ultrasonic measurement time
    lastUltrasonicTime = currentTime;
  }

  // Send heartbeat every 60 seconds
  if (currentTime - lastHeartbeatTime >= heartbeatInterval) {
      heartbeatPackage(ID_1, status_1, floor_1, trash_1, signal_1, send_data_heartbeat);
      Serial.write(send_data_heartbeat, 8);
      lastHeartbeatTime = currentTime;
  }

  // Send data every 180 seconds
  if (currentTime - lastDataPackageTime >= dataPackageInterval) {
      dataPackage(ID_1, percentage, send_data);
      Serial.write(send_data, 8);
      // Update last data package time
      lastDataPackageTime = currentTime;
  } 
  // Update LCD and countOpen
  lcd.setCursor(0, 1);
  lcd.print("                "); // Clear the line
  lcd.setCursor(0, 1);
  if (distance >= 13 && distance <= 15) {
    lcd.print("Empty");
    percentage = 0;
    countOpenFlag = false; // Reset flag
    status_1=0;
    statusFlag =false;
  } else if (distance > 10 && distance <= 13) {
    lcd.print("20%");
    percentage = 20;
    countOpenFlag = false; // Reset flag
    statusFlag =false;
  } else if (distance > 7 && distance <= 10) {
    lcd.print("40%");
    percentage = 40;
    countOpenFlag = false; // Reset flag
    statusFlag =false;
  } else if (distance >= 5 && distance <= 7) {
    lcd.print("60%");
    percentage = 60;
    countOpenFlag = false; // Reset flag
    statusFlag =false;
  } else if (distance >= 2 && distance <= 4) {
    lcd.print("80%");
    percentage = 80;
    countOpenFlag = false; // Reset flag
    statusFlag =false;
  } else if (distance < 2) {
    lcd.print("Full");
    percentage = 100;
    countOpenFlag = false; // Reset flag
    status_1=1;
    statusFlag =false;
  } else if (distance > 14.5 && !countOpenFlag) {
    countOpen++;
    lcd.print("Open");
    flag = 1;
    countOpenFlag = true; // Set flag to avoid multiple increments
  }
  delay(100);
}
