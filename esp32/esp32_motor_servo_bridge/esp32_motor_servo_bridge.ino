// ===== ESP32 (Arduino-ESP32 core 3.x) =====
// ROS2 motor_serial_bridge_node compatible UART protocol
// RX: "CMD spd=0.50 steer=0.10 estop=0" OR "CMD pwm=1600 steer_us=1700 estop=0"
// TX: "FB vel_mps=0.48 steer=0.10 enc=12345 fault=0"
//
// Hardware:
// - 1 DC motor (IN1=PWM, IN2=DIR, ENA=enable)
// - 1 steering servo on GPIO33

#include <HardwareSerial.h>

// ---------------- Pins ----------------
const int ENA = 14;
const int IN1 = 27;      // motor PWM
const int IN2 = 26;      // motor DIR
const int SERVO_PIN = 33;  // steering servo signal (yellow)
const int ENC_A_PIN = 34;  // wheel encoder pulse input
const int ENC_B_PIN = 35;  // quadrature B pin
const bool ENC_INVERT_SIGN = false;
const bool ENC_COUNT_RISING_EDGE = true;

// UART2 (ESP32)
const int ESP_RX = 16;   // <- Jetson TX
const int ESP_TX = 17;   // -> Jetson RX
HardwareSerial JetsonSerial(2);

// ---------------- Motor PWM ----------------
const int PWM_FREQ = 20000;
const int PWM_RES = 8;     // 0~255
const int PWM_MAX = 255;

// ---------------- Servo PWM ----------------
// 50 Hz standard servo pulse
const int SERVO_FREQ = 50;
const int SERVO_RES = 16;  // wider duty precision
const int SERVO_MIN_US = 900;
const int SERVO_MAX_US = 2100;
const int SERVO_CENTER_US = 1500;
const int SERVO_LEFT_US = 1700;
const int SERVO_RIGHT_US = 1300;
const int SERVO_DEADBAND_US = 10;
const bool CENTER_SERVO_ON_ESTOP = true;
const bool CENTER_SERVO_ON_TIMEOUT = true;
const bool INVERT_STEER_SIGN = true;

// ROS-side max steer (must match yaml max_steer_rad if using steer=rad)
float MAX_STEER_RAD = 0.15f;

// ---------------- Tuning ----------------
// Must match ROS-side max_speed_mps in yaml
float MAX_SPEED_MPS = 1.0f;
int MIN_EFFECTIVE_PWM_FWD = 80;
int MIN_EFFECTIVE_PWM_REV = 45;
int MAX_USE_PWM_FWD = 220;
int MAX_USE_PWM_REV = 70;
int PWM_RAMP_STEP = 5;  // per control update
int START_BOOST_PWM_FWD = 120;
int START_BOOST_PWM_REV = 55;
const unsigned long START_BOOST_MS = 140;

bool DIR_IN2_HIGH_IS_REVERSE = true;

// ---------------- Speed PID ----------------
// Feedforward(speed->PWM) + PID correction.
const bool ENABLE_SPEED_PID = true;
const unsigned long SPEED_PID_PERIOD_MS = 20;  // 50 Hz control loop
float SPEED_KP = 60.0f;
float SPEED_KI = 20.0f;
float SPEED_KD = 2.0f;
float SPEED_PID_I_CLAMP = 80.0f;
float SPEED_PID_D_LPF_ALPHA = 0.7f;            // 0.0: no filtering, 1.0: heavy filtering
float SPEED_PID_MAX_CORR_FWD = 70.0f;          // max + correction pwm
float SPEED_PID_MAX_CORR_REV = 50.0f;          // max - correction pwm

// ---------------- State ----------------
String rxLine;

float cmdSpeedMps = 0.0f;
float cmdSteerRad = 0.0f;
int cmdSteerUs = SERVO_CENTER_US;
bool estopLatched = false;
bool motorRunning = false;
volatile int currentPwm = 0;

// Encoder calibration: replace with measured value.
float ENC_COUNTS_PER_METER = 541.1f;
volatile int32_t encCount = 0;

unsigned long lastCmdMs = 0;
unsigned long lastFbMs = 0;
unsigned long startBoostUntilMs = 0;
int startBoostSign = 0;  // +1 forward, -1 reverse
float measuredVelMps = 0.0f;
bool hasMeasuredVel = false;
bool speedControlActive = false;
unsigned long lastSpeedPidMs = 0;
float speedPidIntegral = 0.0f;
float speedPidPrevError = 0.0f;
float speedPidDerivFilt = 0.0f;

// ESP32-side safety timeout (independent of Jetson timeout)
const unsigned long CMD_TIMEOUT_MS = 300;
const unsigned long FB_PERIOD_MS = 50; // 20Hz feedback

void IRAM_ATTR onEncoderEdge() {
  int dir = 0;
  const int pwm_snapshot = currentPwm;
  if (pwm_snapshot > 0) {
    dir = 1;
  } else if (pwm_snapshot < 0) {
    dir = -1;
  }

  const int a = digitalRead(ENC_A_PIN);
  const int b = digitalRead(ENC_B_PIN);
  dir = (a == b) ? 1 : -1;
  if (pwm_snapshot < 0) {
    dir = -dir;
  }
  if (ENC_INVERT_SIGN) {
    dir = -dir;
  }
  if (dir != 0) {
    encCount += dir;
  }
}

// ---------------- Helpers ----------------
int usToDuty(int us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  // duty = (us / period_us) * (2^res - 1), period_us = 20000 at 50Hz
  const uint32_t dutyMax = (1UL << SERVO_RES) - 1UL;
  return (int)(((uint32_t)us * dutyMax) / 20000UL);
}

void writeServoUs(int us) {
  int targetUs = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  if (abs(targetUs - cmdSteerUs) <= SERVO_DEADBAND_US) {
    return;
  }
  cmdSteerUs = targetUs;
  ledcWrite(SERVO_PIN, usToDuty(cmdSteerUs));
}

int steerRadToUs(float steerRad) {
  float s = constrain(steerRad, -MAX_STEER_RAD, MAX_STEER_RAD);
  if (MAX_STEER_RAD <= 1e-6f) return SERVO_CENTER_US;

  // Match ROS default polarity in yaml: left(+) => larger us
  if (s >= 0.0f) {
    float t = s / MAX_STEER_RAD;
    return (int)(SERVO_CENTER_US + t * (SERVO_LEFT_US - SERVO_CENTER_US));
  }
  float t = (-s) / MAX_STEER_RAD;
  return (int)(SERVO_CENTER_US + t * (SERVO_RIGHT_US - SERVO_CENTER_US));
}

void setMotorOutput(int signedPwm) {
  if (signedPwm > 0) {
    signedPwm = constrain(signedPwm, 0, MAX_USE_PWM_FWD);
  } else if (signedPwm < 0) {
    signedPwm = constrain(signedPwm, -MAX_USE_PWM_REV, 0);
  }

  if (signedPwm > 0 && signedPwm < MIN_EFFECTIVE_PWM_FWD) {
    signedPwm = MIN_EFFECTIVE_PWM_FWD;
  } else if (signedPwm < 0 && (-signedPwm) < MIN_EFFECTIVE_PWM_REV) {
    signedPwm = -MIN_EFFECTIVE_PWM_REV;
  }

  // Short start boost to break static friction from standstill.
  if (currentPwm == 0 && signedPwm != 0) {
    startBoostSign = (signedPwm > 0) ? 1 : -1;
    startBoostUntilMs = millis() + START_BOOST_MS;
  }

  int targetPwm = signedPwm;
  int delta = targetPwm - currentPwm;
  if (delta > PWM_RAMP_STEP) {
    targetPwm = currentPwm + PWM_RAMP_STEP;
  } else if (delta < -PWM_RAMP_STEP) {
    targetPwm = currentPwm - PWM_RAMP_STEP;
  }

  if (millis() < startBoostUntilMs) {
    if (startBoostSign > 0) {
      if (targetPwm < START_BOOST_PWM_FWD) {
        targetPwm = START_BOOST_PWM_FWD;
      }
    } else if (startBoostSign < 0) {
      if (targetPwm > -START_BOOST_PWM_REV) {
        targetPwm = -START_BOOST_PWM_REV;
      }
    }
  }

  if (targetPwm == 0) {
    ledcWrite(IN1, 0);
    digitalWrite(IN2, LOW);
    currentPwm = 0;
    motorRunning = false;
    startBoostUntilMs = 0;
    startBoostSign = 0;
    return;
  }

  bool reverse = targetPwm < 0;
  int pwm = abs(targetPwm);

  if (DIR_IN2_HIGH_IS_REVERSE) {
    digitalWrite(IN2, reverse ? HIGH : LOW);
  } else {
    digitalWrite(IN2, reverse ? LOW : HIGH);
  }

  ledcWrite(IN1, pwm);
  currentPwm = reverse ? -pwm : pwm;
  motorRunning = true;
}

void stopMotor() {
  setMotorOutput(0);
}

void resetSpeedPid() {
  speedPidIntegral = 0.0f;
  speedPidPrevError = 0.0f;
  speedPidDerivFilt = 0.0f;
  lastSpeedPidMs = 0;
}

float computeSpeedPidCorrection(float targetMps, float currentMps, float dtSec) {
  if (dtSec <= 1e-4f) {
    return 0.0f;
  }

  const float error = targetMps - currentMps;
  const float p = SPEED_KP * error;

  speedPidIntegral += error * dtSec;
  speedPidIntegral = constrain(speedPidIntegral, -SPEED_PID_I_CLAMP, SPEED_PID_I_CLAMP);
  const float i = SPEED_KI * speedPidIntegral;

  const float derivRaw = (error - speedPidPrevError) / dtSec;
  speedPidDerivFilt =
      SPEED_PID_D_LPF_ALPHA * speedPidDerivFilt + (1.0f - SPEED_PID_D_LPF_ALPHA) * derivRaw;
  const float d = SPEED_KD * speedPidDerivFilt;

  speedPidPrevError = error;
  float corr = p + i + d;
  corr = constrain(corr, -SPEED_PID_MAX_CORR_REV, SPEED_PID_MAX_CORR_FWD);
  return corr;
}

int speedMpsToPwm(float speedMps) {
  float s = constrain(speedMps, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  if (fabs(s) < 1e-4f) return 0;

  if (s > 0.0f) {
    float norm = s / MAX_SPEED_MPS;
    return (int)(norm * MAX_USE_PWM_FWD);
  }

  float norm = (-s) / MAX_SPEED_MPS;
  return -(int)(norm * MAX_USE_PWM_REV);
}

bool parseIntField(const String& line, const char* key, int& out) {
  String k = String(key) + "=";
  int idx = line.indexOf(k);
  if (idx < 0) return false;
  idx += k.length();

  int end = line.indexOf(' ', idx);
  String v = (end < 0) ? line.substring(idx) : line.substring(idx, end);
  v.trim();

  char* endptr = nullptr;
  long n = strtol(v.c_str(), &endptr, 10);
  if (endptr == v.c_str()) return false;
  out = (int)n;
  return true;
}

bool parseFloatField(const String& line, const char* key, float& out) {
  String k = String(key) + "=";
  int idx = line.indexOf(k);
  if (idx < 0) return false;
  idx += k.length();

  int end = line.indexOf(' ', idx);
  String v = (end < 0) ? line.substring(idx) : line.substring(idx, end);
  v.trim();

  char* endptr = nullptr;
  float n = strtof(v.c_str(), &endptr);
  if (endptr == v.c_str()) return false;
  out = n;
  return true;
}

// command_mode = mps_rad
void handleCmdMpsRad(const String& line) {
  float spd = cmdSpeedMps;
  float steer = cmdSteerRad;
  int estop = estopLatched ? 1 : 0;

  (void)parseFloatField(line, "spd", spd);
  (void)parseFloatField(line, "steer", steer);
  (void)parseIntField(line, "estop", estop);

  cmdSpeedMps = spd;
  cmdSteerRad = INVERT_STEER_SIGN ? -steer : steer;
  estopLatched = (estop != 0);
  lastCmdMs = millis();

  // Steering should still be updated even with speed=0
  writeServoUs(steerRadToUs(cmdSteerRad));

  if (estopLatched) {
    stopMotor();
    resetSpeedPid();
    speedControlActive = false;
    if (CENTER_SERVO_ON_ESTOP) {
      writeServoUs(SERVO_CENTER_US);
    }
    return;
  }

  speedControlActive = true;
}

// command_mode = pwm_servo_us
void handleCmdPwmServo(const String& line) {
  int pwm = 1500;
  int steerUs = cmdSteerUs;
  int estop = estopLatched ? 1 : 0;

  bool hasPwm = parseIntField(line, "pwm", pwm);
  bool hasSteerUs = parseIntField(line, "steer_us", steerUs);
  (void)parseIntField(line, "estop", estop);

  estopLatched = (estop != 0);
  lastCmdMs = millis();

  if (hasSteerUs) {
    writeServoUs(steerUs);
    // keep a rough rad echo for FB readability
    cmdSteerRad = ((float)(cmdSteerUs - SERVO_CENTER_US) / 400.0f) * MAX_STEER_RAD;
  }

  if (estopLatched) {
    stopMotor();
    resetSpeedPid();
    speedControlActive = false;
    if (CENTER_SERVO_ON_ESTOP) {
      writeServoUs(SERVO_CENTER_US);
    }
    return;
  }

  speedControlActive = false;
  resetSpeedPid();

  if (hasPwm) {
    // ROS pwm is commonly 1000~2000 with 1500 neutral
    int signedPwm = 0;
    if (pwm > 1500) {
      signedPwm = map(pwm, 1501, 2000, 0, MAX_USE_PWM_FWD);
    } else if (pwm < 1500) {
      signedPwm = -map(pwm, 1499, 1000, 0, MAX_USE_PWM_REV);
    }
    setMotorOutput(signedPwm);
  }
}

void runSpeedController() {
  if (!speedControlActive || estopLatched) return;

  const unsigned long nowMs = millis();
  if (lastSpeedPidMs == 0) {
    lastSpeedPidMs = nowMs;
    return;
  }

  const unsigned long elapsedMs = nowMs - lastSpeedPidMs;
  if (elapsedMs < SPEED_PID_PERIOD_MS) return;
  lastSpeedPidMs = nowMs;

  if (fabs(cmdSpeedMps) < 1e-4f) {
    resetSpeedPid();
    setMotorOutput(0);
    return;
  }

  const float measured = hasMeasuredVel ? measuredVelMps : 0.0f;
  const float ffPwm = (float)speedMpsToPwm(cmdSpeedMps);
  float correction = 0.0f;
  if (ENABLE_SPEED_PID) {
    correction = computeSpeedPidCorrection(cmdSpeedMps, measured, elapsedMs * 0.001f);
  }

  int targetPwm = (int)(ffPwm + correction);
  setMotorOutput(targetPwm);

  static unsigned long lastLogMs = 0;
  if ((nowMs - lastLogMs) > 200) {
    lastLogMs = nowMs;
    Serial.print("PID tgt=");
    Serial.print(cmdSpeedMps, 3);
    Serial.print(" vel=");
    Serial.print(measured, 3);
    Serial.print(" ff=");
    Serial.print(ffPwm, 1);
    Serial.print(" corr=");
    Serial.print(correction, 1);
    Serial.print(" pwm=");
    Serial.println(targetPwm);
  }
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  Serial.print("RX line: ");
  Serial.println(line);

  if (!line.startsWith("CMD")) {
    return;
  }

  if (line.indexOf("spd=") >= 0) {
    handleCmdMpsRad(line);
  } else if (line.indexOf("pwm=") >= 0) {
    handleCmdPwmServo(line);
  }
}

void pollJetsonSerial() {
  while (JetsonSerial.available()) {
    char c = (char)JetsonSerial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      handleLine(rxLine);
      rxLine = "";
    } else {
      rxLine += c;
      if (rxLine.length() > 180) {
        rxLine = "";
      }
    }
  }
}

void safetyTimeoutCheck() {
  unsigned long nowMs = millis();
  if ((nowMs - lastCmdMs) > CMD_TIMEOUT_MS) {
    cmdSpeedMps = 0.0f;
    stopMotor();
    speedControlActive = false;
    resetSpeedPid();
    if (CENTER_SERVO_ON_TIMEOUT) {
      writeServoUs(SERVO_CENTER_US);
      cmdSteerRad = 0.0f;
    }
  }
}

void sendFeedback() {
  unsigned long nowMs = millis();
  if ((nowMs - lastFbMs) < FB_PERIOD_MS) return;
  const unsigned long prevFbMs = lastFbMs;
  lastFbMs = nowMs;

  static bool hasPrevEnc = false;
  static int32_t prevEnc = 0;

  int32_t encSnapshot = 0;
  noInterrupts();
  encSnapshot = encCount;
  interrupts();

  float estVel = 0.0f;
  const float dt = (prevFbMs == 0) ? 0.0f : ((float)(nowMs - prevFbMs) * 0.001f);
  if (hasPrevEnc && dt > 1e-4f && ENC_COUNTS_PER_METER > 1e-4f) {
    const int32_t dCount = encSnapshot - prevEnc;
    estVel = ((float)dCount / ENC_COUNTS_PER_METER) / dt;
  }
  prevEnc = encSnapshot;
  hasPrevEnc = true;
  measuredVelMps = estVel;
  hasMeasuredVel = hasPrevEnc;

  JetsonSerial.print("FB vel_mps=");
  JetsonSerial.print(estVel, 3);
  JetsonSerial.print(" steer=");
  JetsonSerial.print(cmdSteerRad, 3);
  JetsonSerial.print(" enc=");
  JetsonSerial.print(encSnapshot);
  JetsonSerial.print(" fault=0");
  JetsonSerial.print(" batt=0.0");
  JetsonSerial.println();

  Serial.print("TX FB vel_mps="); 
  Serial.print(estVel, 3);
  Serial.print(" enc=");
  Serial.println(encSnapshot);
}

void setup() {
  Serial.begin(115200);
  JetsonSerial.begin(115200, SERIAL_8N1, ESP_RX, ESP_TX);

  pinMode(ENA, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(ENA, HIGH);
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), onEncoderEdge,
                  ENC_COUNT_RISING_EDGE ? RISING : FALLING);

  // Motor PWM on IN1
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcWrite(IN1, 0);

  // Servo PWM on GPIO33
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  writeServoUs(SERVO_CENTER_US);

  stopMotor();
  lastCmdMs = millis();
  lastFbMs = 0;

  Serial.println("ESP32 ready (ROS2 bridge protocol + servo GPIO33).");
  Serial.println("Expect: CMD spd=... steer=... estop=...");
  Serial.println("or:     CMD pwm=... steer_us=... estop=...");
  Serial.print("Encoder counts/m: ");
  Serial.println(ENC_COUNTS_PER_METER, 3);
  JetsonSerial.println("FB fault=0");
}

void loop() {
  pollJetsonSerial();
  runSpeedController();
  safetyTimeoutCheck();
  sendFeedback();

  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}
