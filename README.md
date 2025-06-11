# 🤖 ESP32 Autonomous Rover with Manual & Auto Navigation

This project implements a smart six-wheeled rover entirely controlled by an **ESP32**, capable of both **manual control via Wi-Fi** and **autonomous navigation** using an **ultrasonic sensor** mounted on a servo. Additional support includes **PID balance control** and a **basic maze-solving algorithm**.

---

## 🚗 Core Features

- ✅ Fully controlled by a ESP32 
- 📡 Wi-Fi control using HTTP/WebSocket
- 🛞 Dual H-Bridge motor control (L298N or Motor Shield)
- 🕹️ Manual driving commands (forward, backward, left, right, stop)
- 🧠 Autonomous driving using ultrasonic scanning
- 🧭 Simple decision-making algorithm for navigation
- ⚙️ Optional PID stabilization using MPU6050
- 🧩 Optional maze-solving logic based on wall-following

---

## 🧰 Hardware Components

| Component       | Description                                   |
|----------------|-----------------------------------------------|
| ESP32           | Main and only microcontroller                 |
| 4 DC Motors     | Two pairs of wheels controlled independently  |
| Motor Driver    | L298N or Adafruit Motor Shield                |
| HC-SR04         | Ultrasonic sensor for distance measurement    |
| Servo Motor     | Rotates ultrasonic sensor (left/front/right) |
| MPU6050         | Optional: Gyro & accelerometer for PID        |
| DHT11 / BMP180  | Optional: Environmental sensors               |

---

## 📶 Communication and Control

- The ESP32 creates a Wi-Fi Access Point:
  - **SSID**: `AndroidAP5916`
  - **Password**: `caaf59io67`
- HTTP commands allow real-time manual control:
/forward
/backward
/left
/right
/stop
/auto # Enables autopilot mode
/normal # Disables autopilot mode

- WebSocket support for sending sensor data to external applications

---

## 🕹️ Manual Driving Mode

In this mode, the rover responds directly to HTTP commands. Commands are interpreted by the ESP32 and translated into motor driver instructions.

---

## 🤖 Autonomous Mode: Maze-Solving Algorithm

When /auto is activated, the ESP32 performs environmental scanning and makes decisions using the ultrasonic sensor mounted on a servo.

### 🔍 Scanning Logic
The servo rotates to:
- 0° → Right
- 90° → Front
- 180° → Left

Distance is measured in all three directions sequentially.

### 🧠 Decision Rules
- If front is clear → Go forward
- If right is clear → Turn right
- If left is clear → Turn left
- If all directions blocked → Backtrack and rotate
- If wall too close → Reverse briefly

---

## ⚖️ Optional: PID Balance Control
If the rover uses a balancing platform (e.g., two wheels), a PID controller with the MPU6050 is implemented to maintain vertical stability, adjust speed depending on .

PID Parameters:
- Kp = 1.0
- Ki = 250.0
- Kd = 2.0

The ESP32 reads pitch angle via DMP, computes PID output, and adjusts motor speeds to maintain upright position.

---

## 📸 Media

![RoversS](https://github.com/user-attachments/assets/3bb989aa-4a4a-437a-bc16-51e47ee48e34)


https://github.com/user-attachments/assets/1ab791e0-5eca-4a90-96a9-5543729df7c2

https://github.com/user-attachments/assets/33423d3e-a974-4484-a564-31d0df325292




## 👥 Credits

Josh Sebastián López Murcia  
Julián Humberto Lozada Silva

