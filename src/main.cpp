// src/main.cpp
#include <Arduino.h>

// ===================== MODE SELECT =====================
// Set MODE to 1 for STEPPER, 2 for DUAL DC MOTOR
#define MODE_STEPPER 1
#define MODE_DC      2
#define MODE MODE_STEPPER
// =======================================================

// ===================== GPIO CONFIG (from platformio.ini) =====================
// Your platformio.ini provides pin mapping via build flags like:
//   -DD0_GPIO_CFG=3
//   -DD1_GPIO_CFG=46
//   -DD2_GPIO_CFG=9  ... -DD7_GPIO_CFG=14
//
// We consume those macros here. If a macro is missing, we fall back to a sane default.
// ============================================================================

// Status LEDs
#ifndef D0_GPIO_CFG
  #define D0_GPIO_CFG 3
#endif
#ifndef D1_GPIO_CFG
  #define D1_GPIO_CFG 46
#endif

static const int PIN_LED0 = (int)D0_GPIO_CFG;
static const int PIN_LED1 = (int)D1_GPIO_CFG;

// TB6612FNG pin map (defaults match your t-sim7080g-s3 config)
#ifndef D2_GPIO_CFG
  #define D2_GPIO_CFG 9
#endif
#ifndef D3_GPIO_CFG
  #define D3_GPIO_CFG 10
#endif
#ifndef D4_GPIO_CFG
  #define D4_GPIO_CFG 11
#endif
#ifndef D5_GPIO_CFG
  #define D5_GPIO_CFG 12
#endif
#ifndef D6_GPIO_CFG
  #define D6_GPIO_CFG 13
#endif
#ifndef D7_GPIO_CFG
  #define D7_GPIO_CFG 14
#endif

// ===== TB6612FNG pin map (ESP32 / ESP32-S3 GPIO numbers) =====
// STBY is wired to 3V3 (always enabled)
static const int PIN_PWMA = (int)D2_GPIO_CFG;   // default 9
static const int PIN_AIN2 = (int)D3_GPIO_CFG;   // default 10
static const int PIN_AIN1 = (int)D4_GPIO_CFG;   // default 11
static const int PIN_BIN2 = (int)D5_GPIO_CFG;   // default 12
static const int PIN_BIN1 = (int)D6_GPIO_CFG;   // default 13
static const int PIN_PWMB = (int)D7_GPIO_CFG;   // default 14

// ===================== STATUS LED HEARTBEAT =====================
static const uint32_t LED_BLINK_MS = 250; // matches your documentation
static uint32_t g_ledNextMs = 0;
static bool g_ledPhase = false;

static inline void ledsInit()
{
  pinMode(PIN_LED0, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);

  digitalWrite(PIN_LED0, LOW);
  digitalWrite(PIN_LED1, LOW);

  g_ledPhase = false;
  g_ledNextMs = millis() + LED_BLINK_MS;
}

static inline void ledsTask()
{
  uint32_t now = millis();
  if ((int32_t)(now - g_ledNextMs) < 0) return;

  g_ledNextMs += LED_BLINK_MS;
  g_ledPhase = !g_ledPhase;

  // Alternate blink: LED0 on while LED1 off, then swap
  digitalWrite(PIN_LED0, g_ledPhase ? HIGH : LOW);
  digitalWrite(PIN_LED1, g_ledPhase ? LOW  : HIGH);
}

// ===================== PWM (ESP32 LEDC) =====================
static const int PWM_FREQ_HZ = 20000;    // 20 kHz (quiet)
static const int PWM_BITS    = 8;        // 0..255
static const int PWM_CH_A    = 0;
static const int PWM_CH_B    = 1;

static void pwmInit()
{
  ledcSetup(PWM_CH_A, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(PWM_CH_B, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(PIN_PWMA, PWM_CH_A);
  ledcAttachPin(PIN_PWMB, PWM_CH_B);

  // start stopped
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
}

static inline void pwmWriteA(uint8_t duty) { ledcWrite(PWM_CH_A, duty); }
static inline void pwmWriteB(uint8_t duty) { ledcWrite(PWM_CH_B, duty); }

// ===================== Common H-bridge helpers =====================
static inline void coastA() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
}
static inline void coastB() {
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
}
static inline void brakeA() {
  // brake: both high (or both low depending on driver); both high is common for TB6612
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
}
static inline void brakeB() {
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
}

// =======================================================
// ===================== DC MOTOR MODE =====================
// =======================================================
#if MODE == MODE_DC

// speed: -255..+255 (sign = direction)
static void setMotorA(int speed)
{
  speed = max(-255, min(255, speed));
  uint8_t duty = (uint8_t)abs(speed);

  if (speed > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  } else {
    coastA();
  }
  pwmWriteA(duty);
}

static void setMotorB(int speed)
{
  speed = max(-255, min(255, speed));
  uint8_t duty = (uint8_t)abs(speed);

  if (speed > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  } else {
    coastB();
  }
  pwmWriteB(duty);
}

static void stopBoth()
{
  setMotorA(0);
  setMotorB(0);
}

static void printHelp()
{
  Serial.println();
  Serial.println("=== TB6612FNG Dual DC motor mode ===");
  Serial.println("Commands (send newline):");
  Serial.println("  A <speed>   Motor A speed -255..255   (e.g. A 120, A -200)");
  Serial.println("  B <speed>   Motor B speed -255..255");
  Serial.println("  S           Stop both");
  Serial.println("  H           Help");
  Serial.println();
}

static bool readLine(String &line)
{
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line = buf;
      buf = "";
      return true;
    }
    buf += c;
    if (buf.length() > 120) buf.remove(0, buf.length() - 120);
  }
  return false;
}

static void handleSerialDC()
{
  // Keep LEDs blinking even when idle in DC mode
  ledsTask();

  String line;
  if (!readLine(line)) return;

  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);

  if (cmd == 'H' || cmd == 'h') {
    printHelp();
    return;
  }
  if (cmd == 'S' || cmd == 's') {
    stopBoth();
    Serial.println("Stopped both motors.");
    return;
  }

  // parse "A 123" or "B -45"
  if (cmd == 'A' || cmd == 'a' || cmd == 'B' || cmd == 'b') {
    int sp = 0;
    // find first space
    int p = line.indexOf(' ');
    if (p < 0) {
      Serial.println("Format: A <speed> or B <speed> (e.g. A 120)");
      return;
    }
    sp = line.substring(p + 1).toInt();

    if (cmd == 'A' || cmd == 'a') {
      setMotorA(sp);
      Serial.print("Motor A = "); Serial.println(sp);
    } else {
      setMotorB(sp);
      Serial.print("Motor B = "); Serial.println(sp);
    }
    return;
  }

  Serial.println("Unknown command. Send 'H' for help.");
}

#endif // MODE_DC

// =======================================================
// ===================== STEPPER MODE =====================
// =======================================================
#if MODE == MODE_STEPPER

// ===== Wiring (confirmed working) =====
// Bridge A (A01/A02) = Orange + Pink
// Bridge B (B01/B02) = Yellow + Blue
// Red disconnected

// Fixed speed (steps/s)
static const float    FIXED_STEPS_PER_SEC = 400.0f;
static const uint32_t PERIOD_US = (uint32_t)(1000000.0f / FIXED_STEPS_PER_SEC); // 2500us

// run pattern
static const uint32_t RUN_DIR_MS  = 5000;
static const uint32_t COAST_MS    = 250;

// Polarity flips if needed (swap ONE coil in software)
static bool FLIP_COIL_A = false;
static bool FLIP_COIL_B = false;

static inline void driveA_pol(int pol) {
  if (FLIP_COIL_A) pol = -pol;
  if (pol > 0) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW); }
  else if (pol < 0) { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, HIGH); }
  else { coastA(); }
}
static inline void driveB_pol(int pol) {
  if (FLIP_COIL_B) pol = -pol;
  if (pol > 0) { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW); }
  else if (pol < 0) { digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, HIGH); }
  else { coastB(); }
}

static inline void stepperCoast() { coastA(); coastB(); }

// Full-step (2-phase ON)
static const int8_t FULLSTEP[4][2] = {
  { +1, +1 }, // A+, B+
  { -1, +1 }, // A-, B+
  { -1, -1 }, // A-, B-
  { +1, -1 }, // A+, B-
};

static inline void applyPhase(uint8_t idx) {
  idx &= 3;
  driveA_pol(FULLSTEP[idx][0]);
  driveB_pol(FULLSTEP[idx][1]);
}

static inline void waitUntil(uint32_t targetUs) {
  while ((int32_t)(micros() - targetUs) < 0) {
    // allow background tasks + keep LED blink alive even during tight stepping waits
    ledsTask();
    delay(0);
  }
}

static void handleSerialStepper()
{
  // also blink during serial handling
  ledsTask();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'a' || c == 'A') {
      FLIP_COIL_A = !FLIP_COIL_A;
      Serial.printf("FLIP_COIL_A=%d\n", FLIP_COIL_A);
    }
    if (c == 'b' || c == 'B') {
      FLIP_COIL_B = !FLIP_COIL_B;
      Serial.printf("FLIP_COIL_B=%d\n", FLIP_COIL_B);
    }
    if (c == 'r' || c == 'R') {
      FLIP_COIL_A = false;
      FLIP_COIL_B = false;
      Serial.println("Flips reset: FLIP_COIL_A=0 FLIP_COIL_B=0");
    }
  }
}

static void runFixedForMs(bool forward, uint32_t runMs) {
  uint8_t phase = forward ? 0 : 3;
  applyPhase(phase);

  uint32_t tEnd = millis() + runMs;
  uint32_t tNext = micros() + PERIOD_US;

  while ((int32_t)(millis() - tEnd) < 0) {
    waitUntil(tNext);
    tNext += PERIOD_US;

    phase = forward ? ((phase + 1) & 3) : ((phase + 3) & 3);
    applyPhase(phase);

    handleSerialStepper();
    ledsTask();
  }
}

#endif // MODE_STEPPER

// =======================================================
// ===================== setup/loop =======================
// =======================================================
void setup()
{
  Serial.begin(115200);
  delay(1500);

  // LEDs on D0/D1 (as defined by build flags D0_GPIO_CFG/D1_GPIO_CFG)
  ledsInit();

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  // PWM pins are driven by LEDC (still set as output is fine)
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);

  pwmInit();

#if MODE == MODE_STEPPER
  // Full power for stepper (100% duty)
  pwmWriteA(255);
  pwmWriteB(255);

  stepperCoast();

  Serial.println();
  Serial.println("=== TB6612FNG STEPPER mode ===");
  Serial.println("Wiring: A=Orange+Pink, B=Yellow+Blue, Red disconnected");
  Serial.print("Fixed speed: ");
  Serial.print(FIXED_STEPS_PER_SEC, 1);
  Serial.print(" steps/s (period_us=");
  Serial.print(PERIOD_US);
  Serial.println(")");
  Serial.println("Keys: a,b,r to flip coils.");
  Serial.println();

#elif MODE == MODE_DC
  stopBoth();
  printHelp();
#endif
}

void loop()
{
  // Always keep LEDs blinking
  ledsTask();

#if MODE == MODE_STEPPER
  Serial.println("Forward...");
  runFixedForMs(true, RUN_DIR_MS);
  stepperCoast();

  // blink-friendly delay
  uint32_t tEnd1 = millis() + COAST_MS;
  while ((int32_t)(millis() - tEnd1) < 0) { ledsTask(); delay(1); }

  Serial.println("Reverse...");
  runFixedForMs(false, RUN_DIR_MS);
  stepperCoast();

  // blink-friendly delay
  uint32_t tEnd2 = millis() + COAST_MS;
  while ((int32_t)(millis() - tEnd2) < 0) { ledsTask(); delay(1); }

#elif MODE == MODE_DC
  handleSerialDC();
  delay(5);
#endif
}