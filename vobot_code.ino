#include <Servo.h>

// *Motor Driver Pins*
#define ENA 9    // Speed control for Motor 1 & 2
#define IN1 8    // Motor 1 Direction
#define IN2 7    // Motor 1 Direction
#define IN3 6    // Motor 3 Direction
#define IN4 5    // Motor 3 Direction
#define ENB 10   // Speed control for Motor 3 & 4

// *Ultrasonic Sensor Pins*
#define TRIG_PIN A2  // Trigger Pin
#define ECHO_PIN A3  // Echo Pin

// *Servo Motor Pin*
#define SERVO_PIN 3

int Speeed = 255;  // Motor Speed (0-255)
String command = "";
String lastCommand = "stop";  // Initialize with "stop" to keep motors off
Servo servoMotor;  // Servo object
bool obstacleDetected = false;

void setup() {
  Serial.begin(9600);  // Initialize Serial for Bluetooth Communication
  Serial.println("Voice Controlled Robot Ready...");

  // *Set Motor Pins as Outputs*
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // *Set Ultrasonic Sensor Pins*
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // *Initialize Servo Motor*
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(90);  // Keep servo in the center

  // *Ensure Motors are Stopped Initially*
  Stop();
}

void loop() {
  // *Check for obstacles using servo scanning*
  obstacleDetected = checkObstacles();

  // *Check for new Bluetooth commands*
  if (Serial.available()) {   
    command = Serial.readString();  
    command.trim();  
    Serial.println("Received Command: " + command);

    if (!obstacleDetected) {
      lastCommand = command;  
    }
  }

  // *Execute Last Command Continuously Unless an Obstacle is Found*
  if (!obstacleDetected) {
    if (lastCommand == "forward") {
      forward();
    } 
    else if (lastCommand == "backward") {
      back();
    } 
    else if (lastCommand == "left") {
      left();
    } 
    else if (lastCommand == "right") {
      right();
    } 
    else if (lastCommand == "stop") {
      Stop();
    }
  } else {
    Stop();
    Serial.println("🚨 Obstacle detected! Stopping robot.");
  }
}

// *Motor Control Functions*
void forward() {
  analogWrite(ENA, Speeed);
  analogWrite(ENB, Speeed);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void back() {
  analogWrite(ENA, Speeed);
  analogWrite(ENB, Speeed);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void left() {
  analogWrite(ENA, Speeed);
  analogWrite(ENB, Speeed);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  analogWrite(ENA, Speeed);
  analogWrite(ENB, Speeed);
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// *Function to Scan for Obstacles Using Servo*
bool checkObstacles() {
  int leftDistance, centerDistance, rightDistance;

  // *Check Center*
  servoMotor.write(90);
  delay(500);
  centerDistance = getDistance();
  
  // *Check Left*
  servoMotor.write(0);
  delay(500);
  leftDistance = getDistance();
  
  // *Check Right*
  servoMotor.write(180);
  delay(500);
  rightDistance = getDistance();
  
  // *Move Servo Back to Center*
  servoMotor.write(90);

  Serial.print("Left: "); Serial.print(leftDistance);
  Serial.print(" | Center: "); Serial.print(centerDistance);
  Serial.print(" | Right: "); Serial.println(rightDistance);

  // *If any direction has an obstacle closer than 15 cm, stop*
  if (centerDistance < 15 || leftDistance < 15 || rightDistance < 15) {
    return true;
  }
  return false;
}

// *Function to Measure Distance Using Ultrasonic Sensor*
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;  // Convert to cm

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}
