// ===== ESP32 (Arduino-ESP32 core 3.x) =====
// ROS2 motor_serial_bridge_node compatible UART protocol
// RX: "CMD spd=0.50 steer=0.10 estop=0" OR "CMD pwm=1600 steer_us=1700 estop=0"
// TX: "FB vel_mps=0.48 steer=0.10 enc=12345 fault=0"
//
// Hardware:
// - 1 DC motor (IN1=PWM, IN2=DIR, ENA=enable)  [L298N]
// - 1 steering servo on GPIO33
//
// v2 changes (software-only, no hardware change):
//   [1] PWM 8-bit → 10-bit (256 → 1024 steps)
//   [2] Encoder ISR: digitalRead → direct register read
//   [3] Speed measurement synced with PID period (20ms)
//   [4] Time-based PWM ramping (loop-speed independent)
//   [5] Active brake + dwell before direction reversal
//   [6] Critical section: noInterrupts → portENTER_CRITICAL
//   [7] Shadow register for ledcWrite (skip redundant writes)

#include <HardwareSerial.h>

// ---------------- Pins ----------------
const int ENA = 14;
const int IN1 = 27;      // motor PWM
const int IN2 = 26;      // motor DIR
const int SERVO_PIN = 33;  // steering servo signal (yellow)
const int ENC_A_PIN = 18;  // wheel encoder pulse input
const int ENC_B_PIN = 19;  // quadrature B pin
const bool ENC_INVERT_SIGN = false;

// UART2 (ESP32)
const int ESP_RX = 16;   // <- Jetson TX
const int ESP_TX = 17;   // -> Jetson RX
HardwareSerial JetsonSerial(2);

// ---------------- Motor PWM ----------------
// [1] 10-bit resolution: 0~1023 (was 8-bit 0~255)
const int PWM_FREQ = 4500;
const int PWM_RES = 10;     // 0~1023
const int PWM_MAX = 1023;

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
// [1] All PWM constants scaled ×4 for 10-bit
// Must match ROS-side max_speed_mps in yaml
float MAX_SPEED_MPS = 1.0f;
int MIN_EFFECTIVE_PWM_FWD = 320;   // was 80
int MIN_EFFECTIVE_PWM_REV = 25;    // lower reverse floor to avoid fixed fast reverse
int MAX_USE_PWM_FWD = 600;         // was 200
int MAX_USE_PWM_REV = 110;         // tighter reverse cap
int START_BOOST_PWM_FWD = 480;     // was 120
int START_BOOST_PWM_REV = 25;      // minimal reverse boost
const unsigned long START_BOOST_MS = 140;
const unsigned long START_BOOST_MS_REV = 0;

// [4] Time-based ramp: PWM units per second (replaces fixed PWM_RAMP_STEP)
float PWM_RAMP_RATE = 3000.0f;     // ~0.33s from 0 to full (was ~5/loop ≈ irregular)

// [5] Brake dwell: hold brake this long before allowing direction reversal
const unsigned long BRAKE_DWELL_MS = 80;

bool DIR_IN2_HIGH_IS_REVERSE = true;

// ---------------- Speed PID ----------------
// [1] Gains & clamps scaled ×4 for 10-bit PWM output
const bool ENABLE_SPEED_PID = true;
const bool USE_PID_IN_REVERSE = false;         // manual reverse: open-loop for stability
const unsigned long SPEED_PID_PERIOD_MS = 20;  // 50 Hz control loop
float SPEED_KP = 240.0f;                       // was 60
float SPEED_KI = 40.0f;                        // was 20
float SPEED_KD = 8.0f;                         // was 2
float SPEED_PID_I_CLAMP = 320.0f;              // was 80
float SPEED_PID_D_LPF_ALPHA = 0.7f;
float SPEED_PID_MAX_CORR_FWD = 180.0f;         // was 70
float SPEED_PID_MAX_CORR_REV = 80.0f;          // limit reverse correction burst

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
volatile uint8_t encPrevState = 0;

// [6] Critical section mutex (replaces noInterrupts/interrupts)
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

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

// [3] Independent encoder snapshot for PID velocity
int32_t pidPrevEnc = 0;
bool pidHasPrevEnc = false;

// [4] Time-based ramp state
unsigned long lastRampMs = 0;

// [5] Brake dwell state
bool brakeDwellActive = false;
unsigned long brakeDwellUntilMs = 0;

// [7] Shadow registers for ledcWrite (skip redundant writes)
uint32_t shadowIN1 = 0;
uint32_t shadowServo = 0;

// ESP32-side safety timeout (independent of Jetson timeout)
const unsigned long CMD_TIMEOUT_MS = 300;
const unsigned long FB_PERIOD_MS = 50; // 20Hz feedback

// ================================================================
//  [2] Encoder ISR
// ================================================================
void IRAM_ATTR onEncoderEdge() {
  static const int8_t QUAD_TABLE[16] = {
      0, -1, +1, 0,
      +1, 0, 0, -1,
      -1, 0, 0, +1,
      0, +1, -1, 0};

  const uint8_t a = (uint8_t)digitalRead(ENC_A_PIN);
  const uint8_t b = (uint8_t)digitalRead(ENC_B_PIN);

  const uint8_t curr = (uint8_t)((a << 1) | b);
  const uint8_t idx = (uint8_t)((encPrevState << 2) | curr);
  int8_t step = QUAD_TABLE[idx];
  encPrevState = curr;

  if (ENC_INVERT_SIGN) {
    step = -step;
  }
  encCount += step;
}

// ================================================================
//  [7] Shadow-guarded ledcWrite helpers
// ================================================================
static inline void safeWriteIN1(uint32_t duty) {
  if (duty != shadowIN1) {
    ledcWrite(IN1, duty);
    shadowIN1 = duty;
  }
}

static inline void safeWriteServo(uint32_t duty) {
  if (duty != shadowServo) {
    ledcWrite(SERVO_PIN, duty);
    shadowServo = duty;
  }
}

// ================================================================
//  Helpers
// ================================================================
int usToDuty(int us) {
  us = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  const uint32_t dutyMax = (1UL << SERVO_RES) - 1UL;
  return (int)(((uint32_t)us * dutyMax) / 20000UL);
}

void writeServoUs(int us) {
  int targetUs = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  if (abs(targetUs - cmdSteerUs) <= SERVO_DEADBAND_US) {
    return;
  }
  cmdSteerUs = targetUs;
  safeWriteServo(usToDuty(cmdSteerUs));   // [7] shadow guard
}

int steerRadToUs(float steerRad) {
  float s = constrain(steerRad, -MAX_STEER_RAD, MAX_STEER_RAD);
  if (MAX_STEER_RAD <= 1e-6f) return SERVO_CENTER_US;

  if (s >= 0.0f) {
    float t = s / MAX_STEER_RAD;
    return (int)(SERVO_CENTER_US + t * (SERVO_LEFT_US - SERVO_CENTER_US));
  }
  float t = (-s) / MAX_STEER_RAD;
  return (int)(SERVO_CENTER_US + t * (SERVO_RIGHT_US - SERVO_CENTER_US));
}

// ================================================================
//  [5] Motor output with brake dwell & [4] time-based ramp
// ================================================================
void setMotorOutput(int signedPwm) {
  const unsigned long nowMs = millis();

  // ---- [5] Brake dwell: if active, hold brake until timer expires ----
  if (brakeDwellActive) {
    if (nowMs < brakeDwellUntilMs) {
      // Still dwelling — keep brake (IN1 LOW + IN2 LOW + ENA HIGH = L298N short brake)
      safeWriteIN1(0);
      digitalWrite(IN2, LOW);
      currentPwm = 0;
      motorRunning = false;
      return;
    }
    // Dwell complete
    brakeDwellActive = false;
  }

  // Clamp to allowed range
  if (signedPwm > 0) {
    signedPwm = constrain(signedPwm, 0, MAX_USE_PWM_FWD);
  } else if (signedPwm < 0) {
    signedPwm = constrain(signedPwm, -MAX_USE_PWM_REV, 0);
  }

  // Apply minimum effective PWM
  if (signedPwm > 0 && signedPwm < MIN_EFFECTIVE_PWM_FWD) {
    signedPwm = MIN_EFFECTIVE_PWM_FWD;
  } else if (signedPwm < 0 && (-signedPwm) < MIN_EFFECTIVE_PWM_REV) {
    signedPwm = -MIN_EFFECTIVE_PWM_REV;
  }

  // ---- [5] Detect direction reversal → enter brake dwell ----
  if (currentPwm != 0 && signedPwm != 0) {
    bool wasForward = (currentPwm > 0);
    bool wantForward = (signedPwm > 0);
    if (wasForward != wantForward) {
      // Direction change at non-zero speed: brake first
      safeWriteIN1(0);
      digitalWrite(IN2, LOW);
      currentPwm = 0;
      motorRunning = false;
      startBoostUntilMs = 0;
      startBoostSign = 0;
      brakeDwellActive = true;
      brakeDwellUntilMs = nowMs + BRAKE_DWELL_MS;
      lastRampMs = nowMs;
      return;
    }
  }

  // Start boost for breaking static friction from standstill
  if (currentPwm == 0 && signedPwm != 0) {
    startBoostSign = (signedPwm > 0) ? 1 : -1;
    startBoostUntilMs = nowMs + ((signedPwm > 0) ? START_BOOST_MS : START_BOOST_MS_REV);
  }

  // ---- [4] Time-based ramping ----
  int targetPwm = signedPwm;
  if (lastRampMs > 0) {
    const unsigned long dtMs = nowMs - lastRampMs;
    if (dtMs > 0) {
      const float maxDelta = PWM_RAMP_RATE * (float)dtMs * 0.001f;
      const int iMaxDelta = (int)maxDelta;
      if (iMaxDelta > 0) {
        int delta = targetPwm - currentPwm;
        if (delta > iMaxDelta) {
          targetPwm = currentPwm + iMaxDelta;
        } else if (delta < -iMaxDelta) {
          targetPwm = currentPwm - iMaxDelta;
        }
      }
    }
  }
  lastRampMs = nowMs;

  // Apply start boost (override ramp if still in boost window)
  if (nowMs < startBoostUntilMs) {
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

  // ---- Stop ----
  if (targetPwm == 0) {
    safeWriteIN1(0);          // [7] shadow guard
    digitalWrite(IN2, LOW);   // IN1=LOW, IN2=LOW, ENA=HIGH → L298N short brake
    currentPwm = 0;
    motorRunning = false;
    startBoostUntilMs = 0;
    startBoostSign = 0;
    return;
  }

  // ---- Drive ----
  bool reverse = targetPwm < 0;
  int pwm = abs(targetPwm);

  if (DIR_IN2_HIGH_IS_REVERSE) {
    digitalWrite(IN2, reverse ? HIGH : LOW);
  } else {
    digitalWrite(IN2, reverse ? LOW : HIGH);
  }

  safeWriteIN1(pwm);           // [7] shadow guard
  currentPwm = reverse ? -pwm : pwm;
  motorRunning = true;
}

void stopMotor() {
  setMotorOutput(0);
}

// ================================================================
//  Speed PID
// ================================================================
void resetSpeedPid() {
  speedPidIntegral = 0.0f;
  speedPidPrevError = 0.0f;
  speedPidDerivFilt = 0.0f;
  lastSpeedPidMs = 0;
  pidHasPrevEnc = false;
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

// ================================================================
//  Parsing
// ================================================================
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

// ================================================================
//  Command handlers
// ================================================================
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
    // [1] ROS pwm 1000~2000 → scaled to 10-bit range
    int signedPwm = 0;
    if (pwm > 1500) {
      signedPwm = map(pwm, 1501, 2000, 0, MAX_USE_PWM_FWD);
    } else if (pwm < 1500) {
      signedPwm = -map(pwm, 1499, 1000, 0, MAX_USE_PWM_REV);
    }
    setMotorOutput(signedPwm);
  }
}

// ================================================================
//  [3] Speed controller — measures velocity in sync with PID
// ================================================================
void runSpeedController() {
  if (!speedControlActive || estopLatched) return;

  const unsigned long nowMs = millis();
  if (lastSpeedPidMs == 0) {
    lastSpeedPidMs = nowMs;
    // [3] Initialize PID encoder snapshot
    portENTER_CRITICAL(&encMux);
    pidPrevEnc = encCount;
    portEXIT_CRITICAL(&encMux);
    pidHasPrevEnc = true;
    return;
  }

  const unsigned long elapsedMs = nowMs - lastSpeedPidMs;
  if (elapsedMs < SPEED_PID_PERIOD_MS) return;
  lastSpeedPidMs = nowMs;

  // [3] Measure velocity at PID rate (not at FB rate)
  int32_t encSnap;
  portENTER_CRITICAL(&encMux);       // [6] proper critical section
  encSnap = encCount;
  portEXIT_CRITICAL(&encMux);

  float pidMeasuredVel = 0.0f;
  if (pidHasPrevEnc && ENC_COUNTS_PER_METER > 1e-4f) {
    const float dtSec = (float)elapsedMs * 0.001f;
    const int32_t dCount = encSnap - pidPrevEnc;
    pidMeasuredVel = ((float)dCount / ENC_COUNTS_PER_METER) / dtSec;
  }
  pidPrevEnc = encSnap;
  pidHasPrevEnc = true;

  // Also update global for feedback (best estimate)
  measuredVelMps = pidMeasuredVel;
  hasMeasuredVel = true;

  if (fabs(cmdSpeedMps) < 1e-4f) {
    resetSpeedPid();
    setMotorOutput(0);
    return;
  }

  const float ffPwm = (float)speedMpsToPwm(cmdSpeedMps);
  float correction = 0.0f;
  const bool isReverseCmd = (cmdSpeedMps < 0.0f);
  const bool allowPid = ENABLE_SPEED_PID && (!isReverseCmd || USE_PID_IN_REVERSE);
  if (allowPid) {
    correction = computeSpeedPidCorrection(cmdSpeedMps, pidMeasuredVel,
                                           (float)elapsedMs * 0.001f);
  } else {
    // Prevent residual integral push when reverse PID is disabled.
    speedPidIntegral = 0.0f;
    speedPidPrevError = 0.0f;
    speedPidDerivFilt = 0.0f;
  }

  int targetPwm = (int)(ffPwm + correction);
  setMotorOutput(targetPwm);

  static unsigned long lastLogMs = 0;
  if ((nowMs - lastLogMs) > 200) {
    lastLogMs = nowMs;
    Serial.print("PID tgt=");
    Serial.print(cmdSpeedMps, 3);
    Serial.print(" vel=");
    Serial.print(pidMeasuredVel, 3);
    Serial.print(" ff=");
    Serial.print(ffPwm, 1);
    Serial.print(" corr=");
    Serial.print(correction, 1);
    Serial.print(" pwm=");
    Serial.println(targetPwm);
  }
}

// ================================================================
//  Line handler & serial polling
// ================================================================
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

// ================================================================
//  Safety timeout
// ================================================================
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

// ================================================================
//  Feedback (20 Hz to Jetson)
// ================================================================
void sendFeedback() {
  unsigned long nowMs = millis();
  if ((nowMs - lastFbMs) < FB_PERIOD_MS) return;
  const unsigned long prevFbMs = lastFbMs;
  lastFbMs = nowMs;

  static bool hasPrevEnc = false;
  static int32_t prevEnc = 0;

  // [6] Use portENTER_CRITICAL instead of noInterrupts
  int32_t encSnapshot = 0;
  portENTER_CRITICAL(&encMux);
  encSnapshot = encCount;
  portEXIT_CRITICAL(&encMux);

  float estVel = 0.0f;
  const float dt = (prevFbMs == 0) ? 0.0f : ((float)(nowMs - prevFbMs) * 0.001f);
  int32_t dCount = 0;
  if (hasPrevEnc && dt > 1e-4f && ENC_COUNTS_PER_METER > 1e-4f) {
    dCount = encSnapshot - prevEnc;
    estVel = ((float)dCount / ENC_COUNTS_PER_METER) / dt;
  }
  prevEnc = encSnapshot;
  hasPrevEnc = true;

  // [3] If PID is not running, update global from feedback measurement
  if (!speedControlActive) {
    measuredVelMps = estVel;
    hasMeasuredVel = hasPrevEnc;
  }

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
  Serial.print(encSnapshot);
  Serial.print(" dCount=");
  Serial.println(dCount);
}

// ================================================================
//  Setup & Loop
// ================================================================
void setup() {
  Serial.begin(115200);
  JetsonSerial.begin(115200, SERIAL_8N1, ESP_RX, ESP_TX);

  pinMode(ENA, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(ENA, HIGH);
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);

  // [2] Init encoder state
  {
    const uint8_t a = (uint8_t)digitalRead(ENC_A_PIN);
    const uint8_t b = (uint8_t)digitalRead(ENC_B_PIN);
    encPrevState = (uint8_t)((a << 1) | b);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), onEncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), onEncoderEdge, CHANGE);

  // Motor PWM on IN1 — [1] 10-bit resolution
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcWrite(IN1, 0);
  shadowIN1 = 0;

  // Servo PWM on GPIO33
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  writeServoUs(SERVO_CENTER_US);

  stopMotor();
  lastCmdMs = millis();
  lastFbMs = 0;
  lastRampMs = millis();

  Serial.println("ESP32 ready (ROS2 bridge v2: 10-bit PWM, fast ISR, time-ramp, brake-dwell).");
  Serial.println("Expect: CMD spd=... steer=... estop=...");
  Serial.println("or:     CMD pwm=... steer_us=... estop=...");
  Serial.print("Encoder counts/m: ");
  Serial.println(ENC_COUNTS_PER_METER, 3);
  Serial.printf("CFG max=%.3f rev_max=%d rev_min=%d rev_boost=%d pid_rev=%d\n",
                MAX_SPEED_MPS, MAX_USE_PWM_REV, MIN_EFFECTIVE_PWM_REV,
                START_BOOST_PWM_REV, USE_PID_IN_REVERSE ? 1 : 0);
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
