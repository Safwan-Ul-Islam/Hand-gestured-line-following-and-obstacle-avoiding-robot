
# Gesture-Controlled & Autonomous Robot Car

**Hand Gesture Control • Obstacle Avoidance • Line Following**

This project presents the development of an **advanced robot car** that combines **hand gesture control** with **autonomous navigation capabilities**, creating a versatile and interactive robotic platform. Using an **MPU6050 sensor** for real-time gesture recognition and **Arduino microcontrollers** for processing, users can seamlessly guide the robot manually or switch to fully autonomous modes depending on environmental conditions.

---

## 📌 Project Overview

* **Three Operation Modes:**

  1. **Gesture Control:** Guide the car using intuitive hand movements.
  2. **Obstacle Avoidance:** Automatically navigate around obstacles using ultrasonic sensors.
  3. **Line Following:** Follow predefined tracks autonomously using IR sensors.

* **Objective:** Merge **natural human-machine interaction** with **autonomous navigation** to showcase practical robotic applications in manufacturing, logistics, and personal assistance.

---

## 🛠 Hardware Components

* **Arduino Uno / Nano** – Microcontroller for sensor data processing and motor control
* **MPU6050** – Motion sensor (accelerometer + gyroscope) for gesture recognition
* **Ultrasonic Sensor (HC-SR04)** – For obstacle detection up to 100 cm
* **IR Sensors** – For line tracking and path following
* **Motor Driver (L298N or L293D)** – Controls motor speed and direction
* **DC Motors with Chassis** – Robot car base
* **Power Supply (Li-ion or battery pack)** – Stable power delivery
* **Bluetooth Module (optional)** – For wireless switching between modes

---

## ⚙️ Working Principle

1. **Gesture Control Mode**

   * MPU6050 sensor worn on the hand detects pitch/roll movements.
   * Arduino processes sensor data and sends corresponding motor commands to the robot.

2. **Obstacle Avoidance Mode**

   * Ultrasonic sensor detects obstacles within 100 cm.
   * The robot stops or reroutes automatically to avoid collisions.

3. **Line Following Mode**

   * IR sensors detect line contrast on the floor.
   * Arduino adjusts motor speeds to keep the robot on track.

**Mode Switching:**

* Modes can be changed via a button or Bluetooth commands without rewiring.

---

## 📊 Results and Performance

* **Gesture Recognition Accuracy:** \~90% under controlled conditions
* **Obstacle Detection:** Reliable up to 100 cm, ensuring smooth navigation
* **Line Following:** Consistently tracked predefined paths without major deviation

---

## 🌍 Applications

* **Manufacturing & Logistics:** Human-assisted and autonomous delivery robots
* **Personal Assistance:** Intuitive control for service robots
* **Education:** Demonstrates human-machine interaction and sensor fusion
* **Research & Prototyping:** Base platform for further autonomous robotics projects

---

## ✅ Advantages

* **Intuitive Control:** Natural gesture interface improves user experience
* **Versatile Operation:** Manual and autonomous modes in one system
* **Cost-Effective:** Built from widely available, affordable components
* **Scalable:** Can integrate more sensors or advanced algorithms (e.g., AI-based path planning)

---

## 🚀 Future Enhancements

* **Wireless Mode Switching via App**
* **Machine Learning for Gesture Recognition** to improve accuracy
* **Camera-based Vision Navigation** for advanced path detection
* **Closed-loop Motor Control** using encoders for precise movement

