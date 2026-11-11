#include <Arduino.h>
#include <ESP32Servo.h>

// ---------------- Pins ----------------
#define BT_RX 16        // ESP32 RX2  <- HC-05 TXD
#define BT_TX 17        // ESP32 TX2  -> HC-05 RXD
#define SERVO_PIN 18

// L298N pins (original naming: channel A = ENA/IN1/IN2, channel B = ENB/IN3/IN4)
#define ENA 25
#define IN1 27
#define IN2 26
#define ENB 14
#define IN3 33
#define IN4 32

// Physical wiring: left motors are on OUT3/OUT4 (channel B),
// right motors are on OUT1/OUT2 (channel A). Swapped in software here.
#define LEFT_EN  ENB
#define LEFT_IN_A  IN3
#define LEFT_IN_B  IN4
#define RIGHT_EN ENA
#define RIGHT_IN_A IN1
#define RIGHT_IN_B IN2

// ---------------- Tunables (verify by testing) ----------------
// Motor polarity is UNVERIFIED. If a side spins backward on an F command,
// set that side's flag to true. Do not rewire.
const bool LEFT_INVERT  = false;
const bool RIGHT_INVERT = false;

const int PWM_FREQ = 1000;        // Hz; same as your earlier sketch
const int PWM_RESOLUTION = 8;     // duty 0..255
const int MIN_DUTY = 0;           // raise (e.g. 80-120) if motors hum without turning at low speeds
const int INNER_WHEEL_PERCENT = 30; // slow side speed during a turn, % of base speed
const int DIFF_MIN_INTENSITY = 1; // steering intensity at/above which differential applies

// 0 = disabled. If your app resends commands continuously, set e.g. 1000 (ms)
// so the robot stops when Bluetooth drops. Leave 0 if the app only sends on change.
const unsigned long COMMAND_TIMEOUT_MS = 0;

HardwareSerial BT(2);
Servo steeringServo;

char command[32];
int commandPos = 0;
unsigned long lastCommandMs = 0;

// ---------------- Motors ----------------
void driveSide(int enPin, int inA, int inB, int duty, bool forward, bool invert) {
  if (duty <= 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    ledcWrite(enPin, 0);
    return;
  }
  bool dir = forward ^ invert;
  digitalWrite(inA, dir ? HIGH : LOW);
  digitalWrite(inB, dir ? LOW : HIGH);
  ledcWrite(enPin, duty);
}

int speedToDuty(int speed) {            // speed 0..99 -> duty
  if (speed <= 0) return 0;
  return map(speed, 1, 99, MIN_DUTY, 255);
}

void stopMotors() {
  driveSide(LEFT_EN, LEFT_IN_A, LEFT_IN_B, 0, true, false);
  driveSide(RIGHT_EN, RIGHT_IN_A, RIGHT_IN_B, 0, true, false);
}

// ---------------- Command handling ----------------
void processCommand(char *cmd) {
  // Validate the whole command first; ignore it entirely if malformed.
  if (strlen(cmd) != 6) {
    Serial.println("Invalid command length");
    return;
  }
  char moveDir = cmd[0];
  char steerDir = cmd[3];

  if ((moveDir != 'F' && moveDir != 'B') ||
      !isDigit(cmd[1]) || !isDigit(cmd[2]) ||
      (steerDir != 'L' && steerDir != 'R') ||
      !isDigit(cmd[4]) || !isDigit(cmd[5])) {
    Serial.println("Invalid command format");
    return;
  }

  int speed = (cmd[1] - '0') * 10 + (cmd[2] - '0');
  int intensity = (cmd[4] - '0') * 10 + (cmd[5] - '0');

  if (intensity > 60) {
    Serial.println("Invalid steering intensity");
    return;
  }

  // ---- Servo: exactly the verified mapping, independent of speed ----
  int angle = 90;
  if (steerDir == 'R') {
    angle = map(intensity, 0, 60, 90, 40);
  } else {
    angle = map(intensity, 0, 60, 90, 140);
  }
  steeringServo.write(angle);

  // ---- Motors ----
  bool forward = (moveDir == 'F');
  int baseDuty = speedToDuty(speed);
  int leftDuty = baseDuty;
  int rightDuty = baseDuty;

  if (intensity >= DIFF_MIN_INTENSITY) {
    int slowDuty = baseDuty * INNER_WHEEL_PERCENT / 100;
    if (steerDir == 'L') leftDuty = slowDuty;
    else                 rightDuty = slowDuty;
  }

  driveSide(LEFT_EN, LEFT_IN_A, LEFT_IN_B, leftDuty, forward, LEFT_INVERT);
  driveSide(RIGHT_EN, RIGHT_IN_A, RIGHT_IN_B, rightDuty, forward, RIGHT_INVERT);

  lastCommandMs = millis();
  Serial.printf("Received: %s | Angle: %d | L duty: %d | R duty: %d | %s\n",
                cmd, angle, leftDuty, rightDuty, forward ? "FWD" : "BACK");
}

void setup() {
  Serial.begin(115200);
  BT.begin(9600, SERIAL_8N1, BT_RX, BT_TX);

  // Servo first, exactly as in the verified servo-only sketch.
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(90);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
  stopMotors();

  lastCommandMs = millis();
  Serial.println("Robot ready");
}

void loop() {
  while (BT.available()) {
    char c = BT.read();

    if (c == '\n' || c == '\r') {
      if (commandPos > 0) {
        command[commandPos] = '\0';
        processCommand(command);
        commandPos = 0;
      }
    } else if (c >= 32 && c <= 126) {
      if (commandPos < (int)sizeof(command) - 1) {
        command[commandPos++] = c;
      } else {
        commandPos = 0;
      }
    }
  }

  if (COMMAND_TIMEOUT_MS > 0 && millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }
}
