🔥 Autonomous Fire-Fighting and Rescue Robot

📋 Description
This is a fully autonomous fire-fighting and rescue robot designed to operate in complex environments. It intelligently navigates, detects fires and victims, extinguishes flames with a high-pressure turbine, and streams its camera feed to a live web interface. The system also supports manual control and GPS-based location tracking via the website.

🧠 Core Capabilities
- Autonomous navigation using PID control.
- Flame detection and extinguishing.
- Victim detection with siren activation.
- Live camera streaming via a custom website.
- Manual override and GPS tracking via web.
- Reliable closed-loop motor control using encoder feedback.

🌐 Web Interface Features
- Live Camera Feed from the robot’s onboard 8MP camera.
- Manual/Autonomous Mode Toggle for remote control.
- GPS Tracking to view the robot’s location on a map.

🛠️ Hardware Components
Component                      | Function
------------------------------|----------------------------------------------
ESP32                         | Controls motors, sensors, handles PID loop
Raspberry Pi 3B+              | Hosts website, camera stream, MQTT client
5 Flame Sensors               | Detect fire from multiple directions
3 Ultrasonic SRF05 Sensors    | Measure distances to avoid obstacles
MCP3008 ADC                   | Expands analog input pins via SPI
8MP Camera                    | Provides live video stream
High-Pressure Air Turbine     | Extinguishes detected fire
GPS Module                    | Tracks and sends robot location
Buzzer                        | Plays ambulance siren when victim detected
2 JGA25-370 Motors            | Drive system, controlled via PID
2 All-Terrain Wheels          | For robust ground mobility
6 Li-ion Batteries + Regulator| Power source

🔄 Control & Communication
- PID Algorithm: Used for flame tracking and navigation.
- Closed-loop feedback: Motor encoders provide real-time speed correction for precise movement.
- Protocols Used:
  - SPI: For ADC communication (MCP3008 with ESP32).
  - MQTT: Lightweight messaging between ESP32 and Raspberry Pi (e.g., mode switch, sensor updates).

🧰 Software Stack
- Arduino (ESP32): Firmware for sensors, motors, and decision logic.
- Python (Raspberry Pi): MQTT broker/client, camera stream, and Flask-based web server.
- HTML/CSS/JS: Responsive web interface for control and monitoring.
- OpenCV (optional): Image processing and potential AI-based detection.

🚀 Getting Started

Prerequisites
- Arduino IDE with ESP32 board support.
- Python 3.x installed on the Raspberry Pi.
- Required Python libraries: paho-mqtt, flask, opencv-python, etc.

Setup Instructions

ESP32
1. Open the esp32_code.ino file in Arduino IDE.
2. Connect your ESP32 board.
3. Upload the code.

Raspberry Pi
1. Clone the repository:
   git clone https://github.com/AutonomousRobotZ/Autonomous.git
   cd Autonomous

2. Install dependencies:
   pip3 install -r requirements.txt

3. Run the web interface and MQTT logic:
   python3 app.py

4. Access the web interface by entering the Pi’s IP address in a browser.

💡 Features Summary
- Fire detection and high-pressure air extinguishing
- Victim detection with buzzer/siren alert
- GPS-based tracking
- 8MP live camera streaming
- Manual vs. autonomous control from the web
- PID-based motor and flame tracking
- Obstacle avoidance via ultrasonic sensors
- Closed-loop speed control using encoders
- MQTT messaging between ESP32 and Raspberry Pi

🙏 Acknowledgements
- Thanks to open-source communities and contributors.
- Special thanks to our instructors for their guidance.

> Built for smart rescue missions with a blend of robotics, IoT, and real-time web control.
