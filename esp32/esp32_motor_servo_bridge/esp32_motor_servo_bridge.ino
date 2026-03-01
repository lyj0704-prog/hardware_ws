// ===== ESP32 (Arduino-ESP32 core 3.x) =====
// ROS2 motor_serial_bridge_node compatible UART protocol
// RX: "CMD spd=0.50 steer=0.10 estop=0" OR "CMD pwm=1600 steer_us=1700 estop=0"
// TX: "FB vel_mps=0.48 steer=0.10 fault=0 batt=0.0 rpm=1234.5 duty=142 age_ms=10 estop=0"
//
// Hardware:
// - 1 DC motor (IN1=PWM, IN2=DIR, ENA=enable)
// - 1 steering servo on GPIO33
// - 1 quadrature encoder on GPIO18/19

#include <HardwareSerial.h>
#include <math.h>

// ---------------- Pins ----------------
const int ENA = 14;
const int IN1 = 27;       // motor PWM
const int IN2 = 26;       // motor DIR
const int SERVO_PIN = 33; // steering servo signal (yellow)

// Encoder A/B (confirmed wiring)
const int ENC_A = 18;
const int ENC_B = 19;

// UART2 (ESP32)
const int ESP_RX = 16; // <- Jetson TX
const int ESP_TX = 17; // -> Jetson RX
HardwareSerial JetsonSerial(2);

// ---------------- Motor PWM ----------------
const int PWM_FREQ = 20000;
const int PWM_RES = 8; // 0~255
const int PWM_MAX = 255;

// ---------------- Servo PWM ----------------
// 50 Hz standard servo pulse
const int SERVO_FREQ = 50;
const int SERVO_RES = 16; // wider duty precision
const int SERVO_MIN_US = 900;
const int SERVO_MAX_US = 2100;
const int SERVO_CENTER_US = 1500;   // electrical center
const int SERVO_STRAIGHT_US = 1750; // chassis straight trim (~+45deg install offset)
const int SERVO_LEFT_US = 1950;
const int SERVO_RIGHT_US = 1550;
const int SERVO_DEADBAND_US = 10;
const bool CENTER_SERVO_ON_ESTOP = true;
const bool CENTER_SERVO_ON_TIMEOUT = true;
const bool INVERT_STEER_SIGN = true;
const bool DEBUG_STEER = true;

// ROS-side max steer (must match yaml max_steer_rad if using steer=rad)
float MAX_STEER_RAD = 0.24f;

// ---------------- Motor tuning ----------------
int MIN_EFFECTIVE_PWM_FWD = 80;
int MIN_EFFECTIVE_PWM_REV = 45;
int MAX_USE_PWM_FWD = 220;
int MAX_USE_PWM_REV = 70;
int PWM_RAMP_STEP = 5; // per control update, used as slew limiter // TODO: PI/slew 튜닝
int START_BOOST_PWM_FWD = 120;
int START_BOOST_PWM_REV = 55;
const unsigned long START_BOOST_MS = 140;
bool DIR_IN2_HIGH_IS_REVERSE = true;

// ---------------- Encoder + control params ----------------
const float CONTROL_DT_S = 0.01f;                // 100 Hz
const unsigned long CONTROL_PERIOD_US = 10000UL; // 100 Hz
const unsigned long CMD_TIMEOUT_MS = 300;        // ESP32-side safety timeout
const unsigned long FB_PERIOD_MS = 50;           // 20 Hz feedback

const float WHEEL_RADIUS_M = 0.034f; // TODO: wheel radius 실측
const float GEAR_RATIO = 1.0f;       // TODO: gearbox ratio(G) 확정
const int ENCODER_CPR = 11;          // TODO: encoder CPR 데이터시트/실측 확정
const int QUAD_FACTOR = 4;           // TODO: 1x/2x/4x 실설정 확인
const float TICKS_PER_MOTOR_REV = (float)(ENCODER_CPR * QUAD_FACTOR);
const int ENCODER_SIGN = 1; // TODO: encoder sign(정/역) 검증

const float EMA_ALPHA = 0.3f;         // TODO: 저속 노이즈 기준 재튜닝
const float MAX_MOTOR_RPM = 4000.0f;  // TODO: 7.4V 실차 max RPM 측정 후 갱신
const float PI_KP = 0.3f;             // TODO: 반드시 실차 튜닝
const float PI_KI = 0.8f;             // TODO: 반드시 실차 튜닝
const float PI_INTEGRAL_MIN = -400.0f; // anti-windup
const float PI_INTEGRAL_MAX = 400.0f;  // anti-windup
const float STOP_SPEED_EPS_MPS = 0.02f;
const int MAX_DUTY_DELTA_PER_CYCLE = 50; // TODO: 급가속/슬립 기준 조정

const float PI_F = 3.14159265358979323846f;

// Quadrature transition table: [prev(AB), curr(AB)] -> tick delta
// index = (prev << 2) | curr
const int8_t QUAD_TABLE[16] = {
  0, -1, +1, 0,
  +1, 0, 0, -1,
  -1, 0, 0, +1,
  0, +1, -1, 0
};

enum DriveCommandMode {
  DRIVE_MODE_MPS_RAD = 0,
  DRIVE_MODE_PWM_DIRECT = 1
};

// ---------------- State ----------------
String rxLine;

volatile int32_t encoderTicks = 0;
volatile uint8_t encoderPrevState = 0;

float cmdSpeedMps = 0.0f;
float targetSpeedMps = 0.0f;
float cmdSteerRad = 0.0f;
int cmdSteerUs = SERVO_STRAIGHT_US;
bool estopLatched = false;
bool motorRunning = false;
int currentPwm = 0;
DriveCommandMode driveMode = DRIVE_MODE_MPS_RAD;

float motorRpmRef = 0.0f;
float motorRpmRaw = 0.0f;
float motorRpmMeas = 0.0f;
float piIntegral = 0.0f;
int lastPiDutyCmd = 0;
bool speedEstimatorInitialized = false;

int32_t prevEncoderTicks = 0;
unsigned long lastCmdMs = 0;
unsigned long lastFbMs = 0;
unsigned long lastControlUs = 0;
unsigned long startBoostUntilMs = 0;
int startBoostSign = 0; // +1 forward, -1 reverse

// ---------------- Encoder ISR ----------------
void IRAM_ATTR onEncoderEdge() {
  const uint8_t a = (uint8_t)digitalRead(ENC_A);
  const uint8_t b = (uint8_t)digitalRead(ENC_B);
  const uint8_t curr = (uint8_t)((a << 1) | b);
  const uint8_t idx = (uint8_t)((encoderPrevState << 2) | curr);
  encoderTicks += QUAD_TABLE[idx];
  encoderPrevState = curr;
}

int32_t readEncoderTicksAtomic() {
  noInterrupts();
  const int32_t ticks = encoderTicks;
  interrupts();
  return ticks;
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
    if (DEBUG_STEER) {
      Serial.print("SERVO hold(us): ");
      Serial.println(cmdSteerUs);
    }
    return;
  }
  cmdSteerUs = targetUs;
  ledcWrite(SERVO_PIN, usToDuty(cmdSteerUs));
  if (DEBUG_STEER) {
    Serial.print("SERVO write(us): ");
    Serial.println(cmdSteerUs);
  }
}

int steerRadToUs(float steerRad) {
  float s = constrain(steerRad, -MAX_STEER_RAD, MAX_STEER_RAD);
  if (MAX_STEER_RAD <= 1e-6f) return SERVO_STRAIGHT_US;

  // Match ROS default polarity in yaml: left(+) => larger us
  if (s >= 0.0f) {
    float t = s / MAX_STEER_RAD;
    return (int)(SERVO_STRAIGHT_US + t * (SERVO_LEFT_US - SERVO_STRAIGHT_US));
  }
  float t = (-s) / MAX_STEER_RAD;
  return (int)(SERVO_STRAIGHT_US + t * (SERVO_RIGHT_US - SERVO_STRAIGHT_US));
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

void resetSpeedControllerState() {
  targetSpeedMps = 0.0f;
  motorRpmRef = 0.0f;
  piIntegral = 0.0f;
  lastPiDutyCmd = 0;
}

void safeStopWithOptionalCenter(bool centerServo) {
  resetSpeedControllerState();
  stopMotor();
  if (centerServo) {
    writeServoUs(SERVO_STRAIGHT_US);
    cmdSteerRad = 0.0f;
  }
}

int pwm1500ToSignedPwm(int pwm) {
  if (pwm > 1500) {
    return map(pwm, 1501, 2000, 0, MAX_USE_PWM_FWD);
  }
  if (pwm < 1500) {
    return -map(pwm, 1499, 1000, 0, MAX_USE_PWM_REV);
  }
  return 0;
}

// ──── 속도(m/s) → 모터 RPM 변환 ────
// wheel_rpm_ref = v_ref / (2 * PI * R) * 60.0
// motor_rpm_ref = wheel_rpm_ref * G
//
// ──── 엔코더 tick → 모터 RPM 측정 ────
// ticks_per_motor_rev = ENCODER_CPR * QUAD_FACTOR
// motor_rpm_meas = (delta_ticks / ticks_per_motor_rev) / dt * 60.0
//
// ──── 파라미터 의미 ────
// R: 바퀴 반지름 [m] (타이어 외경/2)
// G: 감속비 (모터축 회전수 / 바퀴축 회전수). G>1이면 모터가 더 빨리 회전.
// CPR: 엔코더 Counts Per Revolution (모터축 기준, 단일 채널)
// QUAD_FACTOR: quadrature 디코딩 배수 (1x/2x/4x)
// dt: 제어 주기 [s]
float speedMpsToMotorRpmRef(float vRefMps) {
  const float wheelRpmRef = (vRefMps / (2.0f * PI_F * WHEEL_RADIUS_M)) * 60.0f;
  const float motorRpm = wheelRpmRef * GEAR_RATIO;
  return constrain(motorRpm, -MAX_MOTOR_RPM, MAX_MOTOR_RPM);
}

float motorRpmToVehicleMps(float motorRpm) {
  if (fabsf(GEAR_RATIO) < 1e-6f) {
    return 0.0f;
  }
  const float wheelRpm = motorRpm / GEAR_RATIO;
  return wheelRpm * (2.0f * PI_F * WHEEL_RADIUS_M) / 60.0f;
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

void updateSpeedEstimate(float dt) {
  const int32_t ticksNow = readEncoderTicksAtomic();
  const int32_t deltaTicks = (ticksNow - prevEncoderTicks) * ENCODER_SIGN;
  prevEncoderTicks = ticksNow;

  if (TICKS_PER_MOTOR_REV <= 0.0f || dt <= 1e-6f) {
    motorRpmRaw = 0.0f;
    return;
  }

  motorRpmRaw = ((float)deltaTicks / TICKS_PER_MOTOR_REV) / dt * 60.0f;
  if (!speedEstimatorInitialized) {
    motorRpmMeas = motorRpmRaw;
    speedEstimatorInitialized = true;
  } else {
    motorRpmMeas = EMA_ALPHA * motorRpmRaw + (1.0f - EMA_ALPHA) * motorRpmMeas;
  }
}

void runClosedLoopPI(float dt) {
  if (driveMode != DRIVE_MODE_MPS_RAD) {
    return;
  }
  if (estopLatched) {
    safeStopWithOptionalCenter(CENTER_SERVO_ON_ESTOP);
    return;
  }

  if (fabsf(targetSpeedMps) < STOP_SPEED_EPS_MPS) {
    safeStopWithOptionalCenter(false);
    return;
  }

  motorRpmRef = speedMpsToMotorRpmRef(targetSpeedMps);
  const float error = motorRpmRef - motorRpmMeas;

  float integralCandidate = piIntegral + error * dt;
  integralCandidate = constrain(integralCandidate, PI_INTEGRAL_MIN, PI_INTEGRAL_MAX);

  const float pTerm = PI_KP * error;
  const float uUnsat = pTerm + PI_KI * integralCandidate;
  float uSat = uUnsat;

  if (motorRpmRef >= 0.0f) {
    uSat = constrain(uUnsat, 0.0f, (float)MAX_USE_PWM_FWD);
  } else {
    uSat = constrain(uUnsat, -(float)MAX_USE_PWM_REV, 0.0f);
  }

  const bool satHigh = (uUnsat > uSat + 1e-3f);
  const bool satLow = (uUnsat < uSat - 1e-3f);
  const bool drivesHigh = (error > 0.0f);
  const bool drivesLow = (error < 0.0f);

  // anti-windup: only integrate when it does not push further into saturation
  if (!((satHigh && drivesHigh) || (satLow && drivesLow))) {
    piIntegral = integralCandidate;
  }

  float u = PI_KP * error + PI_KI * piIntegral;
  if (motorRpmRef >= 0.0f) {
    u = constrain(u, 0.0f, (float)MAX_USE_PWM_FWD);
  } else {
    u = constrain(u, -(float)MAX_USE_PWM_REV, 0.0f);
  }

  int signedDutyCmd = (int)roundf(u);

  // Duty slew-rate limiter (separate from setMotorOutput internal ramp)
  int dutyDelta = signedDutyCmd - lastPiDutyCmd;
  if (dutyDelta > MAX_DUTY_DELTA_PER_CYCLE) {
    signedDutyCmd = lastPiDutyCmd + MAX_DUTY_DELTA_PER_CYCLE;
  } else if (dutyDelta < -MAX_DUTY_DELTA_PER_CYCLE) {
    signedDutyCmd = lastPiDutyCmd - MAX_DUTY_DELTA_PER_CYCLE;
  }
  lastPiDutyCmd = signedDutyCmd;

  // Output stage handles DIR + PWM saturation + min effective + start boost.
  setMotorOutput(signedDutyCmd);
}

void controlUpdate() {
  const unsigned long nowUs = micros();
  if (lastControlUs == 0) {
    lastControlUs = nowUs;
    return;
  }

  const unsigned long elapsedUs = (unsigned long)(nowUs - lastControlUs);
  if (elapsedUs < CONTROL_PERIOD_US) {
    return;
  }
  lastControlUs = nowUs;

  float dt = (float)elapsedUs / 1000000.0f;
  if (dt <= 1e-6f || dt > 0.1f) {
    dt = CONTROL_DT_S;
  }

  updateSpeedEstimate(dt);
  runClosedLoopPI(dt);
}

// command_mode = mps_rad
void handleCmdMpsRad(const String& line) {
  float spd = cmdSpeedMps;
  float steer = cmdSteerRad;
  int estop = estopLatched ? 1 : 0;

  (void)parseFloatField(line, "spd", spd);
  (void)parseFloatField(line, "steer", steer);
  (void)parseIntField(line, "estop", estop);

  if (driveMode != DRIVE_MODE_MPS_RAD) {
    resetSpeedControllerState();
  }
  driveMode = DRIVE_MODE_MPS_RAD;

  cmdSpeedMps = spd;
  targetSpeedMps = spd;
  const float steerIn = steer;
  cmdSteerRad = INVERT_STEER_SIGN ? -steer : steer;
  estopLatched = (estop != 0);
  lastCmdMs = millis();

  // Steering should still be updated even with speed=0
  const int steerUs = steerRadToUs(cmdSteerRad);
  if (DEBUG_STEER) {
    Serial.print("STEER in(rad): ");
    Serial.print(steerIn, 3);
    Serial.print(" cmd(rad): ");
    Serial.print(cmdSteerRad, 3);
    Serial.print(" us: ");
    Serial.println(steerUs);
  }
  writeServoUs(steerUs);

  if (estopLatched) {
    safeStopWithOptionalCenter(CENTER_SERVO_ON_ESTOP);
  }
}

// command_mode = pwm_servo_us
void handleCmdPwmServo(const String& line) {
  int pwm = 1500;
  int steerUs = cmdSteerUs;
  int estop = estopLatched ? 1 : 0;

  const bool hasPwm = parseIntField(line, "pwm", pwm);
  const bool hasSteerUs = parseIntField(line, "steer_us", steerUs);
  (void)parseIntField(line, "estop", estop);

  driveMode = DRIVE_MODE_PWM_DIRECT;
  resetSpeedControllerState();

  estopLatched = (estop != 0);
  lastCmdMs = millis();

  if (hasSteerUs) {
    writeServoUs(steerUs);
    // keep a rough rad echo for FB readability
    cmdSteerRad = ((float)(cmdSteerUs - SERVO_STRAIGHT_US) / 400.0f) * MAX_STEER_RAD;
  }

  if (estopLatched) {
    safeStopWithOptionalCenter(CENTER_SERVO_ON_ESTOP);
    return;
  }

  if (hasPwm) {
    // ROS pwm is commonly 1000~2000 with 1500 neutral
    const int signedPwm = pwm1500ToSignedPwm(pwm);
    setMotorOutput(signedPwm);
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
  const unsigned long nowMs = millis();
  if ((nowMs - lastCmdMs) > CMD_TIMEOUT_MS) {
    cmdSpeedMps = 0.0f;
    safeStopWithOptionalCenter(CENTER_SERVO_ON_TIMEOUT);
  }
}

void sendFeedback() {
  const unsigned long nowMs = millis();
  if ((nowMs - lastFbMs) < FB_PERIOD_MS) return;
  lastFbMs = nowMs;

  const float velMps = motorRpmToVehicleMps(motorRpmMeas);
  const unsigned long cmdAgeMs = nowMs - lastCmdMs;

  JetsonSerial.print("FB vel_mps=");
  JetsonSerial.print(velMps, 3);
  JetsonSerial.print(" steer=");
  JetsonSerial.print(cmdSteerRad, 3);
  JetsonSerial.print(" fault=0");
  JetsonSerial.print(" batt=0.0");
  JetsonSerial.print(" rpm=");
  JetsonSerial.print(motorRpmMeas, 1);
  JetsonSerial.print(" duty=");
  JetsonSerial.print(currentPwm);
  JetsonSerial.print(" age_ms=");
  JetsonSerial.print(cmdAgeMs);
  JetsonSerial.print(" estop=");
  JetsonSerial.print(estopLatched ? 1 : 0);
  JetsonSerial.println();

  Serial.print("TX FB vel_mps=");
  Serial.print(velMps, 3);
  Serial.print(" rpm=");
  Serial.print(motorRpmMeas, 1);
  Serial.print(" duty=");
  Serial.println(currentPwm);
}

void setup() {
  Serial.begin(115200);
  JetsonSerial.begin(115200, SERIAL_8N1, ESP_RX, ESP_TX);

  pinMode(ENA, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(ENA, HIGH);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  encoderPrevState = (uint8_t)((digitalRead(ENC_A) << 1) | digitalRead(ENC_B));
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), onEncoderEdge, CHANGE);

  // Motor PWM on IN1
  ledcAttach(IN1, PWM_FREQ, PWM_RES);
  ledcWrite(IN1, 0);

  // Servo PWM on GPIO33
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
  writeServoUs(SERVO_STRAIGHT_US);

  stopMotor();
  resetSpeedControllerState();
  prevEncoderTicks = readEncoderTicksAtomic();
  lastCmdMs = millis();
  lastFbMs = 0;
  lastControlUs = micros();

  Serial.println("ESP32 ready (ROS2 bridge protocol + closed-loop speed + servo GPIO33).");
  Serial.println("Expect: CMD spd=... steer=... estop=...");
  Serial.println("or:     CMD pwm=... steer_us=... estop=...");
  JetsonSerial.println("FB fault=0");
}

void loop() {
  pollJetsonSerial();
  safetyTimeoutCheck();
  controlUpdate();
  sendFeedback();

  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleLine(line);
  }
}
