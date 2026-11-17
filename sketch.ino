#include <Servo.h>

// --- Data Structure Returned by the Heavy Machinery (Plant) ---
struct PlantState {
  float angle;  // Degrees (-135 to +135)
  float speed;  // Degrees per second
};

// --- Controller Parameters (Arduino 1: Pilot) ---
float Kp = 2.0;
float Ki = 0.05;
float Kd = 1.2;
float integralError = 0.0;
float lastError = 0.0;

// --- Plant Parameters (Arduino 2: Heavy Machinery) ---
float maxMotorTorque = 150.0; // Max torque at 100% control signal
float inertia = 5.0;          // Mass resistance to turning
float friction = 0.8;         // Rotational drag

// --- Simulation State ---
PlantState machine = {0.0, 0.0};
unsigned long lastTime = 0;
Servo myServo;

void setup() {
  Serial.begin(115200);
  myServo.attach(9);
  lastTime = millis();
}

// =========================================================================
// FUNCTION 1: CONTROLLER (Will live on Arduino 1)
// Combines Target + Current Angle -> Outputs Control Signal (-100% to +100%)
// =========================================================================
float computeController(float targetAngle, float measuredAngle, float dt) {
  // Combine command and feedback into Error
  float error = targetAngle - measuredAngle;

  // PID math
  integralError = constrain(integralError + error * dt, -100.0, 100.0);
  float derivative = (error - lastError) / dt;
  lastError = error;

  float rawOutput = (Kp * error) + (Ki * integralError) + (Kd * derivative);

  // Normalize into a clean -100% to +100% actuator control signal
  return constrain(rawOutput, -100.0, 100.0);
}

// =========================================================================
// FUNCTION 2: HEAVY MACHINERY PLANT (Will live on Arduino 2)
// Receives ONLY Control Signal (-100% to +100%) -> Outputs Angle & Speed
// =========================================================================
PlantState simulatePlant(float controlSignal, float dt) {
  // Convert -100%..+100% command into physical motor torque
  float appliedTorque = (controlSignal / 100.0) * maxMotorTorque;
  float dragTorque = friction * machine.speed;

  // F = m*a (Rotational equivalent)
  float acceleration = (appliedTorque - dragTorque) / inertia;
  machine.speed += acceleration * dt;
  machine.angle += machine.speed * dt;

  // Mechanical hard stops
  if (machine.angle > 135.0) {
    machine.angle = 135.0;
    machine.speed = 0.0;
  } else if (machine.angle < -135.0) {
    machine.angle = -135.0;
    machine.speed = 0.0;
  }

  return machine;
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;

  if (dt >= 0.02) { // 50 Hz loop
    lastTime = now;

    // 1. Read Pilot Input
    float targetAngle = (analogRead(A0) / 1023.0) * 270.0 - 135.0;

    // 2. Controller combines Target + Feedback -> produces Control Signal
    float controlSignal = computeController(targetAngle, machine.angle, dt);

    // 3. Plant receives Control Signal -> updates Angle & Speed
    machine = simulatePlant(controlSignal, dt);

    // 4. Send simulated angle to Servo (maps -135..135 -> 0..180)
    myServo.write(map((int)machine.angle, -135, 135, 0, 180));

    // 5. Plot all signals together
    Serial.print("Target:");
    Serial.print(targetAngle);
    Serial.print(",Control:");
    Serial.print(controlSignal);
    Serial.print(",Angle:");
    Serial.print(machine.angle);
    Serial.print(",Speed:");
    Serial.println(machine.speed);
  }
}
