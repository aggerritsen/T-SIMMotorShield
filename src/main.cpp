#include <Arduino.h>
#include <Wire.h>

#define MODE_STEPPER 1
#define MODE_DC      2
#define MODE STEPPER

#ifndef D0_GPIO_CFG
  #define D0_GPIO_CFG 3
#endif
#ifndef D1_GPIO_CFG
  #define D1_GPIO_CFG 46
#endif

static const int PIN_LED0 = (int)D0_GPIO_CFG;
static const int PIN_LED1 = (int)D1_GPIO_CFG;

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

static const int PIN_PWMA = (int)D2_GPIO_CFG;
static const int PIN_AIN2 = (int)D3_GPIO_CFG;
static const int PIN_AIN1 = (int)D4_GPIO_CFG;
static const int PIN_BIN2 = (int)D5_GPIO_CFG;
static const int PIN_BIN1 = (int)D6_GPIO_CFG;
static const int PIN_PWMB = (int)D7_GPIO_CFG;

static constexpr uint32_t LED_BLINK_MS_DEFAULT = 1000; // 1 second per step

#ifdef GPIO_TEST_INTERVAL_MS_CFG
static constexpr uint32_t LED_BLINK_MS = (uint32_t)GPIO_TEST_INTERVAL_MS_CFG;
#else
static constexpr uint32_t LED_BLINK_MS = LED_BLINK_MS_DEFAULT;
#endif

static constexpr bool ENABLE_I2C_SCAN_DEFAULT = true;

#ifdef ENABLE_I2C_SCAN_CFG
static constexpr bool ENABLE_I2C_SCAN = (ENABLE_I2C_SCAN_CFG != 0);
#else
static constexpr bool ENABLE_I2C_SCAN = ENABLE_I2C_SCAN_DEFAULT;
#endif

#ifdef I2C1_SDA_GPIO_CFG
static constexpr int I2C1_SDA_GPIO = (int)I2C1_SDA_GPIO_CFG;
#else
static constexpr int I2C1_SDA_GPIO = -1;
#endif

#ifdef I2C1_SCL_GPIO_CFG
static constexpr int I2C1_SCL_GPIO = (int)I2C1_SCL_GPIO_CFG;
#else
static constexpr int I2C1_SCL_GPIO = -1;
#endif

#ifdef I2C1_FREQ_CFG
static constexpr uint32_t I2C1_FREQ = (uint32_t)I2C1_FREQ_CFG;
#else
static constexpr uint32_t I2C1_FREQ = 400000;
#endif

TwoWire I2Cbus1 = TwoWire(1);
static bool i2c1_initialized = false;

static bool     i2c_scan_active   = false;
static uint8_t  i2c_scan_addr     = 0x03;
static uint8_t  i2c_scan_found    = 0;
static uint32_t i2c_scan_start_ms = 0;
static uint8_t  i2c_scan_runs     = 0;
static const uint8_t I2C_SCAN_RUNS_MAX = 3;

static uint32_t g_ledNextMs = 0;
static uint8_t  g_ledIdx = 0;
static bool     g_ledCycleWrap = false;

static inline void gpio_write_safe(int pin, uint8_t level)
{
  if (pin < 0) return;
  digitalWrite(pin, level);
}

static void i2c1_init_once()
{
  if (!ENABLE_I2C_SCAN) return;
  if (i2c1_initialized) return;

  if (I2C1_SDA_GPIO < 0 || I2C1_SCL_GPIO < 0)
  {
    Serial.println("[I2C] I2C-1 pins not configured (I2C1_SDA_GPIO_CFG / I2C1_SCL_GPIO_CFG)");
    return;
  }

  Serial.println("=======================================");
  Serial.println(" I2C-1 INIT (external bus)");
  Serial.println("=======================================");
  Serial.printf(" SDA=%d SCL=%d freq=%lu Hz\n",
                I2C1_SDA_GPIO, I2C1_SCL_GPIO, (unsigned long)I2C1_FREQ);

  I2Cbus1.begin(I2C1_SDA_GPIO, I2C1_SCL_GPIO, I2C1_FREQ);
  i2c1_initialized = true;
}

static void i2c1_scan_start()
{
  if (!ENABLE_I2C_SCAN) return;
  if (!i2c1_initialized) return;
  if (i2c_scan_active) return;
  if (i2c_scan_runs >= I2C_SCAN_RUNS_MAX) return;

  i2c_scan_active   = true;
  i2c_scan_addr     = 0x03;
  i2c_scan_found    = 0;
  i2c_scan_start_ms = millis();

  Serial.println("=======================================");
  Serial.println(" I2C-1 SCAN START (after D0->D1 cycle)");
  Serial.println("=======================================");
}

static void i2c1_scan_tick()
{
  if (!ENABLE_I2C_SCAN) return;
  if (!i2c1_initialized) return;
  if (!i2c_scan_active) return;

  const uint8_t addr = i2c_scan_addr;

  I2Cbus1.beginTransmission(addr);
  if (I2Cbus1.endTransmission() == 0)
  {
    Serial.printf("  ✓ I2C device found at 0x%02X\n", addr);
    i2c_scan_found++;
  }

  if (i2c_scan_addr >= 0x77)
  {
    const uint32_t dur = millis() - i2c_scan_start_ms;

    if (i2c_scan_found == 0) Serial.println("  (no I2C devices found)");
    else Serial.printf("  Total devices: %u\n", i2c_scan_found);

    Serial.printf("  Scan duration: %lu ms\n", (unsigned long)dur);

    i2c_scan_runs++;
    i2c_scan_active = false;
    return;
  }

  i2c_scan_addr++;
  delay(0);
}

static void blink_init()
{
  if (PIN_LED0 >= 0) pinMode(PIN_LED0, OUTPUT);
  if (PIN_LED1 >= 0) pinMode(PIN_LED1, OUTPUT);

  gpio_write_safe(PIN_LED0, LOW);
  gpio_write_safe(PIN_LED1, LOW);

  g_ledIdx = 0;
  g_ledCycleWrap = false;
  g_ledNextMs = millis() + LED_BLINK_MS;
}

static void blink_tick()
{
  uint32_t now = millis();
  if ((int32_t)(now - g_ledNextMs) < 0) return;

  g_ledNextMs += LED_BLINK_MS;

  gpio_write_safe(PIN_LED0, (g_ledIdx == 0) ? HIGH : LOW);
  gpio_write_safe(PIN_LED1, (g_ledIdx == 1) ? HIGH : LOW);

  uint8_t prev = g_ledIdx;
  g_ledIdx = (uint8_t)((g_ledIdx + 1) & 0x01);
  g_ledCycleWrap = (prev == 1 && g_ledIdx == 0);

  if (g_ledCycleWrap) {
    i2c1_scan_start();
  }
}

static inline void bgTask()
{
  blink_tick();
  i2c1_scan_tick();
}

static const int PWM_FREQ_HZ = 20000;
static const int PWM_BITS    = 8;
static const int PWM_CH_A    = 0;
static const int PWM_CH_B    = 1;

static void pwmInit()
{
  ledcSetup(PWM_CH_A, PWM_FREQ_HZ, PWM_BITS);
  ledcSetup(PWM_CH_B, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(PIN_PWMA, PWM_CH_A);
  ledcAttachPin(PIN_PWMB, PWM_CH_B);

  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
}

static inline void pwmWriteA(uint8_t duty) { ledcWrite(PWM_CH_A, duty); }
static inline void pwmWriteB(uint8_t duty) { ledcWrite(PWM_CH_B, duty); }

static inline void coastA() { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, LOW); }
static inline void coastB() { digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, LOW); }
static inline void brakeA() { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); }
static inline void brakeB() { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, HIGH); }

#if MODE == MODE_DC

static void setMotorA(int speed)
{
  speed = max(-255, min(255, speed));
  uint8_t duty = (uint8_t)abs(speed);

  if (speed > 0) { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW); }
  else if (speed < 0) { digitalWrite(PIN_AIN1, LOW); digitalWrite(PIN_AIN2, HIGH); }
  else { coastA(); }
  pwmWriteA(duty);
}

static void setMotorB(int speed)
{
  speed = max(-255, min(255, speed));
  uint8_t duty = (uint8_t)abs(speed);

  if (speed > 0) { digitalWrite(PIN_BIN1, HIGH); digitalWrite(PIN_BIN2, LOW); }
  else if (speed < 0) { digitalWrite(PIN_BIN1, LOW); digitalWrite(PIN_BIN2, HIGH); }
  else { coastB(); }
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
    if (c == '\n') { line = buf; buf = ""; return true; }
    buf += c;
    if (buf.length() > 120) buf.remove(0, buf.length() - 120);
  }
  return false;
}

static void handleSerialDC()
{
  bgTask();

  String line;
  if (!readLine(line)) return;

  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);

  if (cmd == 'H' || cmd == 'h') { printHelp(); return; }
  if (cmd == 'S' || cmd == 's') { stopBoth(); Serial.println("Stopped both motors."); return; }

  if (cmd == 'A' || cmd == 'a' || cmd == 'B' || cmd == 'b') {
    int p = line.indexOf(' ');
    if (p < 0) { Serial.println("Format: A <speed> or B <speed> (e.g. A 120)"); return; }
    int sp = line.substring(p + 1).toInt();

    if (cmd == 'A' || cmd == 'a') { setMotorA(sp); Serial.print("Motor A = "); Serial.println(sp); }
    else { setMotorB(sp); Serial.print("Motor B = "); Serial.println(sp); }
    return;
  }

  Serial.println("Unknown command. Send 'H' for help.");
}

#endif

#if MODE == MODE_STEPPER

static const float    FIXED_STEPS_PER_SEC = 400.0f;
static const uint32_t PERIOD_US = (uint32_t)(1000000.0f / FIXED_STEPS_PER_SEC);

static const uint32_t RUN_DIR_MS  = 5000;
static const uint32_t COAST_MS    = 250;

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

static const int8_t FULLSTEP[4][2] = {
  { +1, +1 },
  { -1, +1 },
  { -1, -1 },
  { +1, -1 },
};

static inline void applyPhase(uint8_t idx) {
  idx &= 3;
  driveA_pol(FULLSTEP[idx][0]);
  driveB_pol(FULLSTEP[idx][1]);
}

static inline void waitUntil(uint32_t targetUs) {
  while ((int32_t)(micros() - targetUs) < 0) {
    bgTask();
    delay(0);
  }
}

static void handleSerialStepper()
{
  bgTask();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'a' || c == 'A') { FLIP_COIL_A = !FLIP_COIL_A; Serial.printf("FLIP_COIL_A=%d\n", FLIP_COIL_A); }
    if (c == 'b' || c == 'B') { FLIP_COIL_B = !FLIP_COIL_B; Serial.printf("FLIP_COIL_B=%d\n", FLIP_COIL_B); }
    if (c == 'r' || c == 'R') { FLIP_COIL_A = false; FLIP_COIL_B = false; Serial.println("Flips reset: FLIP_COIL_A=0 FLIP_COIL_B=0"); }
  }
}

static void runFixedForMs(bool forward, uint32_t runMs) {
  uint8_t phase = forward ? 0 : 3;
  applyPhase(phase);

  uint32_t tEnd  = millis() + runMs;
  uint32_t tNext = micros() + PERIOD_US;

  while ((int32_t)(millis() - tEnd) < 0) {
    waitUntil(tNext);
    tNext += PERIOD_US;

    phase = forward ? ((phase + 1) & 3) : ((phase + 3) & 3);
    applyPhase(phase);

    handleSerialStepper();
    bgTask();
  }
}

#endif

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=======================================");
  Serial.println(" BOOT");
  Serial.println("=======================================");
  Serial.printf("D0=%d D1=%d blink_ms=%lu\n", PIN_LED0, PIN_LED1, (unsigned long)LED_BLINK_MS);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);

  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);

  pwmInit();

  blink_init();
  i2c1_init_once();

#if MODE == MODE_STEPPER
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
  bgTask();

#if MODE == MODE_STEPPER
  Serial.println("Forward...");
  runFixedForMs(true, RUN_DIR_MS);
  stepperCoast();

  uint32_t tEnd1 = millis() + COAST_MS;
  while ((int32_t)(millis() - tEnd1) < 0) { bgTask(); delay(0); }

  Serial.println("Reverse...");
  runFixedForMs(false, RUN_DIR_MS);
  stepperCoast();

  uint32_t tEnd2 = millis() + COAST_MS;
  while ((int32_t)(millis() - tEnd2) < 0) { bgTask(); delay(0); }

#elif MODE == MODE_DC
  handleSerialDC();
  delay(5);
#endif
}
