#include <Arduino.h>
#include <Wire.h>

#define MODE_STEPPER 1
#define MODE_DC      2
#define MODE_UNSET   0

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

#ifdef ENABLE_VISION_UART_CFG
static constexpr bool ENABLE_VISION_UART = (ENABLE_VISION_UART_CFG != 0);
#else
static constexpr bool ENABLE_VISION_UART = false;
#endif

#ifdef USB_SERIAL_BAUD_CFG
static constexpr uint32_t USB_SERIAL_BAUD = (uint32_t)USB_SERIAL_BAUD_CFG;
#else
static constexpr uint32_t USB_SERIAL_BAUD = 115200;
#endif

#ifdef VISION_UART_BAUD_CFG
static constexpr uint32_t VISION_UART_BAUD = (uint32_t)VISION_UART_BAUD_CFG;
#else
static constexpr uint32_t VISION_UART_BAUD = 921600;
#endif

static constexpr bool ENABLE_VISION_UART_PERIODIC_TX = false;
static constexpr uint32_t VISION_UART_PERIODIC_TX_MS = 2000;
static const char *VISION_UART_PERIODIC_TX_LINE = "AT+ID?";

#ifdef VISION_UART_RX_GPIO_CFG
static constexpr int VISION_UART_RX_GPIO = (int)VISION_UART_RX_GPIO_CFG;
#else
static constexpr int VISION_UART_RX_GPIO = -1;
#endif

#ifdef VISION_UART_TX_GPIO_CFG
static constexpr int VISION_UART_TX_GPIO = (int)VISION_UART_TX_GPIO_CFG;
#else
static constexpr int VISION_UART_TX_GPIO = -1;
#endif

#ifdef GV2_POWER_GPIO_CFG
static constexpr int GV2_POWER_GPIO = (int)GV2_POWER_GPIO_CFG;
#else
static constexpr int GV2_POWER_GPIO = -1;
#endif


#ifdef ENABLE_UART_ONLY_CFG
static constexpr bool ENABLE_UART_ONLY = (ENABLE_UART_ONLY_CFG != 0);
#else
static constexpr bool ENABLE_UART_ONLY = false;
#endif
static const uint8_t kVisionJpegMagic[4] = {'V', 'S', 'T', 'J'};
static const uint8_t kVisionStateMagic[4] = {'V', 'S', 'T', 'S'};
struct VisionJpegRxState {
  uint8_t magic_window[4] = {0, 0, 0, 0};
  uint8_t magic_filled = 0;
  bool receiving_jpeg = false;
  uint32_t jpeg_remaining = 0;
  uint8_t frame_state = 0;
  uint8_t frame_class_idx = 0;
  uint8_t frame_conf_u8 = 0;
  uint32_t frame_len = 0;
  uint32_t image_counter = 0;
  uint32_t last_forwarded_ms = 0;
  bool forward_jpeg_usb = false;
};
struct VisionStateRxState {
  uint8_t magic_window[4] = {0, 0, 0, 0};
  uint8_t magic_filled = 0;
};
TwoWire I2Cbus1 = TwoWire(1);
static bool i2c1_initialized = false;
HardwareSerial VisionUART(1);
static bool vision_uart_initialized = false;
static uint32_t vision_uart_last_tx_ms = 0;
static VisionJpegRxState vision_jpeg_rx;
static VisionStateRxState vision_state_rx;
static uint32_t vision_uart_rx_total = 0;
static uint32_t vision_uart_last_rx_ms = 0;
static uint32_t vision_uart_status_last_ms = 0;
static uint8_t vision_uart_preview_count = 0;
static bool vision_uart_preview_open = false;

static bool     i2c_scan_active   = false;
static uint8_t  i2c_scan_addr     = 0x03;
static uint8_t  i2c_scan_found    = 0;
static uint32_t i2c_scan_start_ms = 0;
static uint8_t  i2c_scan_runs     = 0;
static const uint8_t I2C_SCAN_RUNS_MAX = 3;

static uint32_t g_ledNextMs = 0;
static uint8_t  g_ledIdx = 0;
static bool     g_ledCycleWrap = false;

static void printHelp();
static uint8_t g_mode = MODE_UNSET;

static inline void gpio_write_safe(int pin, uint8_t level)
{
  if (pin < 0) return;
  digitalWrite(pin, level);
}
static bool read_u32_le_from_vision_uart(uint32_t *out_value)
{
  if (!out_value) return false;

  uint8_t b[4];
  for (int i = 0; i < 4; ++i)
  {
    const int v = VisionUART.read();
    if (v < 0) return false;
    b[i] = (uint8_t)v;
  }

  *out_value = ((uint32_t)b[0]) |
               ((uint32_t)b[1] << 8) |
               ((uint32_t)b[2] << 16) |
               ((uint32_t)b[3] << 24);
  return true;
}

static void vision_shift_magic_window(uint8_t *window, uint8_t *filled, uint8_t byte_value)
{
  if (*filled < 4)
  {
    window[(*filled)++] = byte_value;
    return;
  }
  window[0] = window[1];
  window[1] = window[2];
  window[2] = window[3];
  window[3] = byte_value;
}

static bool vision_magic_matches(const uint8_t *window, uint8_t filled, const uint8_t *magic)
{
  if (filled < 4) return false;
  for (int i = 0; i < 4; ++i)
  {
    if (window[i] != magic[i]) return false;
  }
  return true;
}

static void vision_shift_magic_window(VisionJpegRxState *s, uint8_t byte_value)
{
  vision_shift_magic_window(s->magic_window, &s->magic_filled, byte_value);
}

static bool vision_magic_window_matches(const VisionJpegRxState &s)
{
  return vision_magic_matches(s.magic_window, s.magic_filled, kVisionJpegMagic);
}

static void vision_uart_preview_byte(uint8_t value)
{
  if (vision_uart_preview_count >= 32) return;

  if (!vision_uart_preview_open)
  {
    Serial.print("[VISION UART] raw preview:");
    vision_uart_preview_open = true;
  }

  Serial.print(' ');
  if (value < 0x10) Serial.print('0');
  Serial.print(value, HEX);
  vision_uart_preview_count++;

  if (vision_uart_preview_count >= 32)
  {
    Serial.println();
    vision_uart_preview_open = false;
  }
}

static void vision_uart_status_tick()
{
  if (!ENABLE_VISION_UART) return;
  if (!vision_uart_initialized) return;

  const uint32_t now = millis();
  if ((int32_t)(now - vision_uart_status_last_ms) < 3000) return;
  vision_uart_status_last_ms = now;

  if (vision_uart_rx_total == 0)
  {
    Serial.printf("[VISION UART] no bytes seen yet on RX=%d TX=%d baud=%lu\n",
                  VISION_UART_RX_GPIO,
                  VISION_UART_TX_GPIO,
                  (unsigned long)VISION_UART_BAUD);
    return;
  }

  Serial.printf("[VISION UART] rx_bytes=%lu jpeg_frames=%lu last_rx_ms_ago=%lu\n",
                (unsigned long)vision_uart_rx_total,
                (unsigned long)vision_jpeg_rx.image_counter,
                (unsigned long)(now - vision_uart_last_rx_ms));
}

static void vision_on_jpeg_frame_start(VisionJpegRxState *s)
{
  if (!s) return;

  const uint32_t now = millis();
  if (now - s->last_forwarded_ms < 500)
  {
    s->receiving_jpeg = true;
    s->jpeg_remaining = s->frame_len;
    s->forward_jpeg_usb = false;
    Serial.print("[jpeg] drop len=");
    Serial.print(s->frame_len);
    Serial.print(" class=");
    Serial.print(s->frame_class_idx);
    Serial.print(" conf_u8=");
    Serial.println(s->frame_conf_u8);
    return;
  }

  s->last_forwarded_ms = now;
  s->image_counter++;

  Serial.print("recv #");
  Serial.print(s->image_counter);
  Serial.print(" len=");
  Serial.print(s->frame_len);
  Serial.print(" state=");
  Serial.print(s->frame_state);
  Serial.print(" class=");
  Serial.print(s->frame_class_idx);
  Serial.print(" conf=");
  Serial.println((float)s->frame_conf_u8 / 255.0f, 3);

  s->receiving_jpeg = true;
  s->jpeg_remaining = s->frame_len;
  s->forward_jpeg_usb = false;
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
    if (i2c_scan_runs >= I2C_SCAN_RUNS_MAX && g_mode == MODE_DC) {
      printHelp();
    }

    i2c_scan_active = false;
    return;
  }

  i2c_scan_addr++;
  delay(0);
}

static void vision_uart_init_once()
{
  if (!ENABLE_VISION_UART) return;
  if (vision_uart_initialized) return;

  if (VISION_UART_RX_GPIO < 0 || VISION_UART_TX_GPIO < 0)
  {
    Serial.println("[VISION UART] pins not configured (VISION_UART_RX_GPIO_CFG / VISION_UART_TX_GPIO_CFG)");
    return;
  }

  Serial.println("=======================================");
  Serial.println(" VISION UART INIT");
  Serial.println("=======================================");
  Serial.printf(" RX=%d TX=%d baud=%lu\n",
                VISION_UART_RX_GPIO,
                VISION_UART_TX_GPIO,
                (unsigned long)VISION_UART_BAUD);
  Serial.println(" framed GV2 parser enabled");
  Serial.println(" raw preview + RX status enabled");

  VisionUART.begin(VISION_UART_BAUD, SERIAL_8N1, VISION_UART_RX_GPIO, VISION_UART_TX_GPIO);
  vision_uart_rx_total = 0;
  vision_uart_last_rx_ms = millis();
  vision_uart_status_last_ms = millis();
  vision_uart_preview_count = 0;
  vision_uart_preview_open = false;
  vision_uart_initialized = true;
}

static void gv2_power_enable_once()
{
  if (GV2_POWER_GPIO < 0) return;

  pinMode(GV2_POWER_GPIO, OUTPUT);
  digitalWrite(GV2_POWER_GPIO, HIGH);
  Serial.print("[GV2 POWER] enable GPIO");
  Serial.print(GV2_POWER_GPIO);
  Serial.println("=HIGH");
}

static void vision_uart_poll()
{
  if (!ENABLE_VISION_UART) return;
  if (!vision_uart_initialized) return;

  while (VisionUART.available() > 0)
  {
    if (vision_jpeg_rx.receiving_jpeg)
    {
      static uint8_t buf[256];
      const int avail_i = VisionUART.available();
      if (avail_i <= 0) break;

      uint32_t to_read = (uint32_t)avail_i;
      if (to_read > vision_jpeg_rx.jpeg_remaining) to_read = vision_jpeg_rx.jpeg_remaining;
      if (to_read > sizeof(buf)) to_read = sizeof(buf);

      const size_t n = VisionUART.readBytes(buf, (size_t)to_read);
      if (n == 0) break;

      if (vision_jpeg_rx.forward_jpeg_usb)
      {
        Serial.write(buf, n);
      }

      vision_uart_rx_total += (uint32_t)n;
      vision_uart_last_rx_ms = millis();
      for (size_t i = 0; i < n; ++i)
      {
        vision_uart_preview_byte(buf[i]);
      }

      vision_jpeg_rx.jpeg_remaining -= (uint32_t)n;
      if (vision_jpeg_rx.jpeg_remaining == 0)
      {
        vision_jpeg_rx.receiving_jpeg = false;
      }
      continue;
    }

    const int b = VisionUART.read();
    if (b < 0) break;
    const uint8_t value = (uint8_t)b;
    vision_uart_rx_total++;
    vision_uart_last_rx_ms = millis();
    vision_uart_preview_byte(value);

    vision_shift_magic_window(vision_state_rx.magic_window, &vision_state_rx.magic_filled, value);
    if (vision_magic_matches(vision_state_rx.magic_window, vision_state_rx.magic_filled, kVisionStateMagic))
    {
      if (VisionUART.available() >= 1)
      {
        (void)VisionUART.read();
      }
    }

    vision_shift_magic_window(&vision_jpeg_rx, value);
    if (!vision_magic_window_matches(vision_jpeg_rx))
    {
      continue;
    }

    if (VisionUART.available() < 7)
    {
      continue;
    }

    const int st = VisionUART.read();
    const int cls = VisionUART.read();
    const int conf = VisionUART.read();
    uint32_t len = 0;
    if (st < 0 || cls < 0 || conf < 0 || !read_u32_le_from_vision_uart(&len))
    {
      continue;
    }

    vision_jpeg_rx.frame_state = (uint8_t)st;
    vision_jpeg_rx.frame_class_idx = (uint8_t)cls;
    vision_jpeg_rx.frame_conf_u8 = (uint8_t)conf;
    vision_jpeg_rx.frame_len = len;

    vision_on_jpeg_frame_start(&vision_jpeg_rx);
  }
}

static void vision_uart_periodic_tx_tick()
{
  if (!ENABLE_VISION_UART) return;
  if (!vision_uart_initialized) return;
  if (!ENABLE_VISION_UART_PERIODIC_TX) return;

  uint32_t now = millis();
  if ((int32_t)(now - vision_uart_last_tx_ms) < (int32_t)VISION_UART_PERIODIC_TX_MS)
    return;

  vision_uart_last_tx_ms = now;
  VisionUART.print(VISION_UART_PERIODIC_TX_LINE);
  VisionUART.print("\r\n");
  Serial.print("[VISION TX] ");
  Serial.println(VISION_UART_PERIODIC_TX_LINE);
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
  vision_uart_periodic_tx_tick();
  vision_uart_poll();
  vision_uart_status_tick();
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

static void printModePrompt()
{
  Serial.println();
  Serial.println("Select mode: [D]C or [S]tepper");
  Serial.println("Waiting for valid input...");
}

static uint8_t readModeSelection()
{
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') continue;
    if (c == 'D' || c == 'd') return MODE_DC;
    if (c == 'S' || c == 's') return MODE_STEPPER;
  }
  return MODE_UNSET;
}

static void waitForModeSelection()
{
  const uint32_t promptIntervalMs = 10000;
  uint32_t nextPromptMs = millis() + promptIntervalMs;

  while (g_mode == MODE_UNSET) {
    uint8_t sel = readModeSelection();
    if (sel != MODE_UNSET) { g_mode = sel; break; }

    uint32_t now = millis();
    if ((int32_t)(now - nextPromptMs) >= 0) {
      nextPromptMs += promptIntervalMs;
      printModePrompt();
    }
    bgTask();
    delay(5);
  }
}

void setup()
{
  Serial.begin(USB_SERIAL_BAUD);
  delay(2500);

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
  gv2_power_enable_once();
  i2c1_init_once();
  vision_uart_init_once();

  if (ENABLE_UART_ONLY) {
    stopBoth();
    Serial.println();
    Serial.println("=== UART-only bring-up mode ===");
    Serial.println("Mode prompt and motor controls are disabled.");
    Serial.print("Vision UART RX=GPIO");
    Serial.print(VISION_UART_RX_GPIO);
    Serial.print(" TX=GPIO");
    Serial.println(VISION_UART_TX_GPIO);
  } else {
    printModePrompt();
    waitForModeSelection();
  }

  if (!ENABLE_UART_ONLY && g_mode == MODE_STEPPER) {
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
  } else if (!ENABLE_UART_ONLY && g_mode == MODE_DC) {
    stopBoth();
    printHelp();
  }
}

void loop()
{
  bgTask();

  if (ENABLE_UART_ONLY) {
    delay(5);
  } else if (g_mode == MODE_STEPPER) {
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
  } else if (!ENABLE_UART_ONLY && g_mode == MODE_DC) {
    handleSerialDC();
    delay(5);
  }
}
