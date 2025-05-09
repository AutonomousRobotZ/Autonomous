#include <ESP32Encoder.h>
#include <Adafruit_MCP3008.h>
#include <TinyGPS.h>
#include <HardwareSerial.h>
#define CS_PIN 5  // Chip select (can be any digital pin)
#include <WiFi.h>
#include <PubSubClient.h>
#include <NewPing.h>
#define MAX_DISTANCE 600
#define trigF 14
#define echoF 15
#define trigL 13
#define echoL 12
#define trigR 10
#define echoR 11


NewPing sonarF(trigF, echoF, MAX_DISTANCE);
NewPing sonarL(trigL, echoL, MAX_DISTANCE);
NewPing sonarR(trigL, echoL, MAX_DISTANCE);
#define ULTRASONIC_SAMPLE_SIZE 5 // Number of samples to average

int distanceLSamples[ULTRASONIC_SAMPLE_SIZE] = {0}; // jnab imin sensor sample buffer
int indexL = 0; // Index for jnab ultrasonic circular buffer
int kodem , imin, issar;

// PID variables for ultrasonic control
float kp_s = 9.5, kd_s = 72, ki_s = 0;
float error_s = 0.0, P_s = 0.0, D_s = 0.0, I_s = 0.0;
float PIDvalue_s = 0.0, lasterror_s = 0.0;

const char* ssid = "Ooredoo-4F1105";  // Votre réseau WiFi
const char* password = "to44&$MP";  // Mot de passe du réseau WiFi
const char* mqtt_server = "192.168.1.9";  // IP du Raspberry Pi
const char* topic_flame = "robotPhoenix/sensors/flame";
const char* topic_gps = "robotPhoenix/sensors/gps";

WiFiClient espClient;
PubSubClient client(espClient);

TinyGPS gps;
HardwareSerial GPS(2); // UART2

Adafruit_MCP3008 adc;

ESP32Encoder rightEncoder;
ESP32Encoder leftEncoder;



const int NUM_SENSORS = 5;
  int sensorValues[NUM_SENSORS];
int sensorPins[NUM_SENSORS] = {0, 1, 2, 3, 4}; // MCP3008 channels

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorRaw[NUM_SENSORS];
int sensorCalibrated[NUM_SENSORS];

// PID constants (tune these)
float kp_flame = 0.05;
float ki_flame = 0.0005;
float kd_flame = 0.25;

// PID terms
int position = 0;
int error = 0;
int lastError = 0;
int P = 0, I = 0, D = 0;
bool autoMode = false;


volatile unsigned long rightTicks = 0;  // Tick count for the right wheel
volatile unsigned long leftTicks = 0;   // Tick count for the left wheel
unsigned long lastUpdateTime = 0;       // Last update time in microseconds
float rightRPM = 0;                     // Right motor RPM
float leftRPM = 0;   
int n;                   // Left motor RPM

const int ticksPerRevolution = 408;     // Encoder ticks per revolution
const unsigned long interval = 3000;    // Interval in microseconds (1 ms)
long prevRightTicks = 0, prevLeftTicks = 0; 
unsigned long lastRightUpdate = 0, lastLeftUpdate = 0;
  long rightTicksNow;
  long leftTicksNow ;
  long rightTicksDiff;
  long leftTicksDiff;

   long lt_ticks_left = 0;
   long lt_ticks_right = 0;

   int  leftBaseRPM ;
int  rightBaseRPM ;

float robotCircumference=PI*8.5;
float cmPerTick=robotCircumference/408;
float width=22.8;

float rpmErrorR = 0, rpmErrorL = 0;     // RPM errors
float rpmP_R = 0, rpmI_R = 0, rpmD_R = 0; // PID components for right motor
float rpmP_L = 0, rpmI_L = 0, rpmD_L = 0; // PID components for left motor
float lastRpmErrorR = 0, lastRpmErrorL = 0; // Previous RPM errors
float targetRPMRight = 0.0;           // Target RPM for right motor
float targetRPMLeft = 0.0;    
float base_rpm=0;        // Target RPM for left motor
//float rpm_kp = 0.0585, rpm_ki = 0.000, rpm_kd = 0.37; //250
float rpm_kp = 0.0585, rpm_ki = 0.000, rpm_kd = 0.37; // PID coefficients
float integralLimit = 0.5;             // Limit for integral component
int pwmRight = 0, pwmLeft = 0;          // PWM values for motors

int Ilim;
float pidAdjustmentRight,pidAdjustmentLeft;
float smoothingFactor;
int left_speed,right_speed;
float k,d;
//motor pins 

const int leftF = 13;
const int leftB = 33;
const int rightB = 17;
const int rightF = 16;
const int BUZZER_PIN=4;
bool done=false;
int s0,s1,s2,s3,s4;

void playSiren(int duration_ms = 5000) {
  unsigned long startTime = millis();
  while (millis() - startTime < duration_ms) {
    // Sweep frequency up
    for (int freq = 800; freq <= 2000; freq += 20) {
      int period = 1000000 / freq;   // period in microseconds
      int pulse = period / 2;        // 50% duty cycle
      unsigned long t = millis();
      while (millis() - t < 20) {    // Play each tone for 20 ms
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(pulse);
      }
    }

    // Sweep frequency down
    for (int freq = 2000; freq >= 800; freq -= 20) {
      int period = 1000000 / freq;
      int pulse = period / 2;
      unsigned long t = millis();
      while (millis() - t < 20) {
        digitalWrite(BUZZER_PIN, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(BUZZER_PIN, LOW);
        delayMicroseconds(pulse);
      }
    }
  }
}


void calibrateSensors(int samples = 1500) {
  // Initialize min/max arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  unsigned long startTime = millis();

  // Phase 1: 4 seconds calibration without flame
  while (millis() - startTime < 4000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int value = adc.readADC(sensorPins[i]);
      if (value < sensorMin[i]) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
    delay(10); // Sampling delay
  }

  // Buzzer beep to notify flame calibration
  digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);

  // Phase 2: Continue calibration for accurate flame readings
  for (int j = 0; j < samples; j++) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int value = adc.readADC(sensorPins[i]);
      if (value < sensorMin[i]) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
    delay(10);
  }
    digitalWrite(BUZZER_PIN, HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN, LOW);

}

void setup() {
  adc.begin(CS_PIN);  // Using hardware SPI
    rightEncoder.attachFullQuad(32, 27);  // Right motor (A, B)
  leftEncoder.attachFullQuad(26, 25);   // Left motor (A, B)

  pinMode(leftF, OUTPUT);
  pinMode(leftB, OUTPUT);
  pinMode(rightF, OUTPUT);
  pinMode(rightB, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(115200);
  GPS.begin(9600, SERIAL_8N1, 35, 14);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);


     // calibrateSensors();
      // Base RPMs for both motors
 leftBaseRPM = 100;
 rightBaseRPM = 100;
 kp_flame = 2.2, kd_flame = 1.1;
 n=0;

  lastLeftUpdate = micros();
  lastRightUpdate= micros();
  integralLimit=10;

}

void loop() {
}



void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.print("Message reçu [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == "robot/control") { // Vérifie le bon topic
    if (message == "auto") {
      Serial.println("Passage en mode AUTO !");
      autoMode = true;
    } 
    else if (message == "manual") {
      Serial.println("Passage en mode MANUEL !");
      autoMode = false;
    } 
    else if (message == "droite") {
      Serial.println("Commande : tourner à DROITE");
      // Ici ta fonction pour tourner à droite :
      right(2);
    } 
    else if (message == "gauche") {
      Serial.println("Commande : tourner à GAUCHE");
      left(2);
    }
    else if (message == "haut") {
      Serial.println("Commande : AVANCER");
      forward(2);
    }
    else if (message == "bas") {
      Serial.println("Commande : RECULER");
      backward(2);
    }
    else {
      Serial.println("Commande non reconnue !");
    }
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connexion MQTT...");
    if (client.connect("PhoenixClient")) {
      Serial.println("connecté !");
      
      // --> S'abonner au topic pour recevoir les ordres de mode
      client.subscribe("robot/control");
      
    } else {
      Serial.print("Échec, rc=");
      Serial.println(client.state()); 
      Serial.println(" nouvelle tentative dans 5 secondes");
      delay(5000);
    }
  }
}

void publishSensorData() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // Lire capteurs de flamme
  readCalibratedSensors(sensorValues);
  String flameData = "[";
  for (int i = 0; i < NUM_SENSORS; i++) {
    flameData += String(sensorValues[i]);
    if (i < NUM_SENSORS - 1) flameData += ",";
  }
  flameData += "]";

  // Lire GPS
  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);

  String gpsData = "{";
  gpsData += "\"lat\":" + String(flat, 6) + ",";
  gpsData += "\"lon\":" + String(flon, 6);
  gpsData += "}";

  // Publier
  client.publish(topic_flame, flameData.c_str());
  client.publish(topic_gps, gpsData.c_str());

  Serial.println("MQTT Publish : ");
  Serial.println("Flamme : " + flameData);
  Serial.println("GPS : " + gpsData);
}


void PID_Flame() {
  readCalibratedSensors(sensorValues);
  // Calculate position (weighted average)
  long weightedSum = 0;
  int sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i]; // Use directly: 0 = strong flame, 1000 = no flame

    weightedSum += (long)value * i * 1000; // i*1000 to spread out positions
    sum += value;
  }

  position = weightedSum / sum;

  error = position - ((NUM_SENSORS - 1) * 1000 / 2); // target center
  Serial.println(error);

  P = error;
  D = error - lastError;
  I += error;

  float u = kp_flame * P + ki_flame * I + kd_flame * D;
  lastError = error;

  targetRPMLeft  = leftBaseRPM + u;
  targetRPMRight = rightBaseRPM - u;

  targetRPMLeft  = constrain(targetRPMLeft, 0, 250);
  targetRPMRight = constrain(targetRPMRight, 0, 250);
}



void readCalibratedSensors(int *sensorValues) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = adc.readADC(sensorPins[i]);

    // Avoid division by zero
    int range = sensorMax[i] - sensorMin[i];
    if (range == 0) range = 1;

    // Normalize to 0 - 1000
    int calibrated = ((raw - sensorMin[i]) * 1000) / range;
    
    // Clamp the value
    if (calibrated < 0)
    { calibrated = 0;
    }
    if (calibrated > 1000){ 
      calibrated = 1000;
    }

    sensorValues[i] = calibrated;
}
}




void StrongStop(float duration) {
  float now=millis();
  while(millis()-now<duration) {
    RPMmotors(0,0);
  }
}

void fullFlamePid(){
  PID_Flame();
  PIDMotors();
}

void PIDMotors() {
  calculateRPM();
  PIDControlRightMotor();
  PIDControlLeftMotor();
}

void RPMmotors(float x , float y) {
  targetRPMRight=y;
  targetRPMLeft=x;
  calculateRPM();
  PIDControlRightMotor();
  PIDControlLeftMotor();
}


void turnByAngle(float angle, float speed) {
    float  arcLength = width * abs(angle)* (PI/180) ;
    int targetTicks =  arcLength / cmPerTick;
    int32_t initPosI1 = leftEncoder.getCount();
    int32_t initPosI2 = rightEncoder.getCount();
    if(angle>0){
        while (!done) {
            if (leftEncoder.getCount() - initPosI1 >= (targetTicks)) {
                done = !done;
            }
            else{
                RPMmotors(speed,0);
            }


        }
        done=false;
    }
    else if(angle<0){
        while (!done) {
            if (rightEncoder.getCount() - initPosI2 >= (targetTicks)) {
                done = !done;
            }
            else{
                RPMmotors(0,speed);
            }
        }
        done=false;


    }

}


void turnByAngle22(float angle, float speed) {
    float  arcLength = width * abs(angle)* (PI/180) ;
    int targetTicks =  arcLength / cmPerTick;
    int32_t initPosI1 = leftEncoder.getCount();
    int32_t initPosI2 = rightEncoder.getCount();
    if(angle>0){
        while (!done) {
            if (leftEncoder.getCount() - initPosI1 >= (targetTicks)) {
                done = !done;
            }
            else{
                RPMmotors(speed,-speed);
            }


        }
        done=false;
    }
    else if(angle<0){
        while (!done) {
            if (rightEncoder.getCount() - initPosI2 >= (targetTicks)) {
                done = !done;
            }
            else{
                RPMmotors(-speed,speed);
            }
        }
        done=false;


    }

}

void reachDistanceWithoutLine(float cm, float v) {
    int t = cm * (1/cmPerTick);  // Pre-calculation for distance
    long long   initPosI1 = leftEncoder.getCount(); // Initial encoder positions
    long long  initPosI2 = rightEncoder.getCount();
    while (!done) {
        long long  currentPosI1 = leftEncoder.getCount();
        long long  currentPosI2 = rightEncoder.getCount();
        long long  currentPositionAvg = (currentPosI1 - initPosI1 + currentPosI2 - initPosI2) / 2;

        if (currentPositionAvg  > t) {
            done=!done;
        }
        else{
            RPMmotors(v,v);
        }
    }
    done=false;
}


void calculateRPM() {
  unsigned long currentTime = micros();
  unsigned long elapsedTime = currentTime - lastUpdateTime;
  // Update every 1ms
  if (elapsedTime >= interval) { // Store previous tick count
    // Read current encoder values
    noInterrupts();  // Disable interrupts for atomic read
     rightTicksNow = rightEncoder.getCount();
     leftTicksNow = leftEncoder.getCount();
    interrupts();
    // Calculate the tick difference since the last update
     rightTicksDiff = rightTicksNow - prevRightTicks;
     leftTicksDiff = leftTicksNow - prevLeftTicks;
    // Update previous tick count
    prevRightTicks = rightTicksNow;
    prevLeftTicks = leftTicksNow;
      rightRPM = (rightTicksDiff * 60.0 * 1000000.0) / (ticksPerRevolution * elapsedTime);
      leftRPM = (leftTicksDiff * 60.0 * 1000000.0) / (ticksPerRevolution * elapsedTime);
    // Update last update time
    lastUpdateTime = currentTime;

  }
}




void PIDControlRightMotor() {
  rpmErrorR = targetRPMRight - rightRPM;
  rpmP_R = rpmErrorR;
  rpmI_R += rpmErrorR;
  rpmI_R = constrain(rpmI_R, -integralLimit, integralLimit);
  rpmD_R = (rpmErrorR - lastRpmErrorR);
  if (targetRPMRight<50) {
    rpm_kp = 0.25, rpm_ki = 0.004, rpm_kd = 0; 
  }
 
  else if (targetRPMRight>=50 && targetRPMRight<150) {
    rpm_kp = 0.25, rpm_ki = 0.004, rpm_kd = 0; 
  }
  else if (targetRPMRight>=150 && targetRPMRight<250) {
   rpm_kp = 0.33, rpm_ki = 0.0083, rpm_kd = 0.015; 
  }
  else if (targetRPMRight>=250 && targetRPMRight<350) {
    rpm_kp = 0.34, rpm_ki = 0.012, rpm_kd = 0.01;   //0.3 smoothing
  }
  else if (targetRPMRight>=350 && targetRPMRight<450) {
    rpm_kp = 0.37, rpm_ki = 0.016, rpm_kd = 0.01;  //0.31 smoothing
  }
   else if (targetRPMRight>=450 && targetRPMRight<550) {
    rpm_kp = 0.4, rpm_ki = 0.021, rpm_kd = 0.015;    //0.31 smoothing
  }
  
  else  {
    rpm_kp = 0.42, rpm_ki = 0.022, rpm_kd = 0.017;   //0.4
  }
   pidAdjustmentRight = (rpm_kp * rpmP_R) + (rpm_ki * rpmI_R) + (rpm_kd * rpmD_R);
  lastRpmErrorR = rpmErrorR;

  if (pidAdjustmentRight > 0) {
    analogWrite(rightF, constrain(pidAdjustmentRight, 0, 255));
    analogWrite(rightB, 0);
  } else {
    analogWrite(rightB, constrain(abs(pidAdjustmentRight), 0, 255));
    analogWrite(rightF, 0);
  }
}

void PIDControlLeftMotor() {
  rpmErrorL = targetRPMLeft - leftRPM;
  rpmP_L = rpmErrorL;
  rpmI_L += rpmErrorL;
  rpmI_L = constrain(rpmI_L, -integralLimit, integralLimit);
  rpmD_L = (rpmErrorL - lastRpmErrorL);
   if (targetRPMLeft<50) {
    rpm_kp = 0.25, rpm_ki = 0.004, rpm_kd = 0; 
  }
 
  else if (targetRPMLeft>=50 && targetRPMLeft<150) {
    rpm_kp = 0.33, rpm_ki = 0.0083, rpm_kd = 0.015; 
  }
  else if (targetRPMLeft>=150 && targetRPMLeft<250) {
     rpm_kp = 0.33, rpm_ki = 0.0083, rpm_kd = 0.01; 
  }
  else if (targetRPMLeft>=250 && targetRPMLeft<350) {
    rpm_kp = 0.34, rpm_ki = 0.012, rpm_kd = 0.01; 
  }
  else if (targetRPMLeft>=350 && targetRPMLeft<450) {
   rpm_kp = 0.37, rpm_ki = 0.016, rpm_kd = 0.01;  //0.31 smoothing
  }
   else if (targetRPMLeft>=450 && targetRPMLeft<550) {
    rpm_kp = 0.4, rpm_ki = 0.021, rpm_kd = 0.015;   //0.31 smoothing
  }
  
  else  {
    rpm_kp = 0.42, rpm_ki = 0.022, rpm_kd = 0.017;  //0.4
  }
   pidAdjustmentLeft = (rpm_kp * rpmP_L) + (rpm_ki * rpmI_L) + (rpm_kd * rpmD_L);
  lastRpmErrorL = rpmErrorL;


  if (pidAdjustmentLeft > 0) {
    analogWrite(leftF, constrain(pidAdjustmentLeft, 0, 255));
    analogWrite(leftB, 0);
  } else {
    analogWrite(leftB, constrain(abs(pidAdjustmentLeft), 0, 255));
    analogWrite(leftF, 0);
  }
}

void backward(int x) {
  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightB, 0);
  analogWrite(leftB, 0);
  delay(x);
}

void forward(int x) {
  analogWrite(rightF, 255);
  analogWrite(leftF, 255);
  analogWrite(rightB, 0);
  analogWrite(leftB, 0);
  delay(x);
}


void forwardPID() {
  analogWrite(rightF, right_speed);
  analogWrite(leftF, left_speed);
  analogWrite(rightB, 0);
  analogWrite(leftB, 0);
  
}

// Turn right for a specific time
void right(int x) {
  analogWrite(rightF, 0);
  analogWrite(leftF, 140);
  analogWrite(rightB, 150);
  analogWrite(leftB, 0);
  delay(x);
}

// Turn left for a specific time
void left(int x) {
  analogWrite(rightF, 140);
  analogWrite(leftF, 0);
  analogWrite(rightB, 0);
  analogWrite(leftB, 140);
  delay(x);
}

void stop(int x) {
  analogWrite(rightF, 0);
  analogWrite(leftF, 0);
  analogWrite(rightB, 0);
  analogWrite(leftB, 0);
  delay(x);
}

void readsensor(){
  s0=adc.readADC(0); 
  s1=adc.readADC(1); 
  s2=adc.readADC(2); 
  s3=adc.readADC(3); 
  s4=adc.readADC(4); 
    Serial.print("S0: "); Serial.println(s0);
  Serial.print("S1: "); Serial.println(s1);
  Serial.print("S2: "); Serial.println(s2);
  Serial.print("S3: "); Serial.println(s3);
  Serial.print("S4: "); Serial.println(s4);
}



void getGPSLink(unsigned long timeout_ms) {
  unsigned long start = millis();
  float lat = 0.0, lon = 0.0;

  while (millis() - start < timeout_ms) {
    while (GPS.available()) {
      char c = GPS.read();
      if (gps.encode(c)) {
        gps.f_get_position(&lat, &lon);
        if (lat != 0.0 && lon != 0.0) {
          // Create and print the Google Maps link
          String link = "https://www.google.com/maps?q=";
          link += String(lat, 6) + "," + String(lon, 6);
          Serial.println(link);
          return;
        }
      }
    }
  }

  Serial.println("⚠️ No GPS fix within timeout.");

  
  Serial.println("------------------");
}

void handleSerialCommand() {
  String command;
  Serial.println(" receiving data ");
  if (Serial.available()) {
     command = Serial.readStringUntil('\n');
    command.trim(); // Remove any extra whitespace or newline characters
    Serial.println(" received ");
  }
    if (command == "LED_ON") {
       digitalWrite(4, HIGH);
      delay(1000);  // Keep LED on for 1 second
      digitalWrite(4, LOW);
            getGPSLink(60000);
    
  }
}


int getDistanceL() {
    int rawDistance = sonarL.ping_cm();
    if (rawDistance < 2 ) {
        rawDistance = 25; // Handle invalid readings
    }

    // Add the new reading to the circular buffer
    distanceLSamples[indexL] = rawDistance;
    indexL = (indexL + 1) % ULTRASONIC_SAMPLE_SIZE; // Circular buffer index update

    // Calculate the moving average
    int sum = 0;
    for (int i = 0; i < ULTRASONIC_SAMPLE_SIZE; i++) {
        sum += distanceLSamples[i];
    }
    int averagedDistance = sum / ULTRASONIC_SAMPLE_SIZE;

    return averagedDistance;
}

int getDistanceF() {
    int distanceF = sonarF.ping_cm();
    if (distanceF < 2 || distanceF > 70) {
        return MAX_DISTANCE;
    } else {
        return distanceF;
    }
}

void PIDleft() {
    error_s = getDistanceL() - 25;
    P_s = error_s;
    D_s = error_s - lasterror_s;
    I_s = error_s + I_s;
     float u = (kp_s * P_s) + (kd_s * D_s) + (ki_s * I_s);
  lastError = error;
  targetRPMLeft = leftBaseRPM - u;
  targetRPMRight = rightBaseRPM + u;
  targetRPMLeft = constrain(targetRPMLeft, 0, 500);
  targetRPMRight = constrain(targetRPMRight, 0, 500);
}