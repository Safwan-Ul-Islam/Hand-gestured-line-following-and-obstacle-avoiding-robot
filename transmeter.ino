#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <MPU6050.h>

// nRF24L01 setup
RF24 radio(8, 10); // CE and CSN pins for nRF24L01
const byte address[6] = "00001"; // Communication address

// MPU6050 setup
MPU6050 mpu;

// Data structure for transmission
struct Data {
  int xAxis;   // Scaled x-axis acceleration
  int yAxis;   // Scaled y-axis acceleration
  int mode;    // 0: Gesture Control, 1: Obstacle Avoidance, 2: Line Following
};

Data sendData;

// Button pin and mode control
const int buttonPin = 2;
int currentMode = 0; // Initial mode: Gesture Control

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1); // Stop execution if MPU6050 is not connected
  }
  Serial.println("MPU6050 connected successfully.");

  // Initialize nRF24L01
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();

  // Configure button as input with pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Check button press for mode switching
  static bool lastButtonState = HIGH;
  bool currentButtonState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && currentButtonState == LOW) {
    currentMode = (currentMode + 1) % 3; // Cycle through modes: 0, 1, 2
    switch (currentMode) {
      case 0:
        Serial.println("Gesture Control Mode");
        break;
      case 1:
        Serial.println("Obstacle Avoidance Mode");
        break;
      case 2:
        Serial.println("Line Following Mode");
        break;
    }
    delay(200); // Debounce delay
  }
  lastButtonState = currentButtonState;

  // Read acceleration data from MPU6050
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Scale acceleration values by dividing by 200
  sendData.xAxis = ax / 200;
  sendData.yAxis = ay / 200;
  sendData.mode = currentMode; // Send the current mode

  // Send data over nRF24L01
  radio.stopListening();
  radio.write(&sendData, sizeof(Data));
  radio.startListening();

  // Print data and transmission status to Serial Monitor
  Serial.print("X: ");
  Serial.print(sendData.xAxis);
  Serial.print(", Y: ");
  Serial.print(sendData.yAxis);
  Serial.print(", Mode: ");
  switch (sendData.mode) {
    case 0:
      Serial.println("Gesture Control");
      break;
    case 1:
      Serial.println("Obstacle Avoidance");
      break;
    case 2:
      Serial.println("Line Following");
      break;
  }

  delay(100); // Adjust delay as needed
}
