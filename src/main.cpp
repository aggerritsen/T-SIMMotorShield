#include <Arduino.h>
#include <Wire.h>

/* =========================================================
   GPIO + I2C-1 TEST (UART REMOVED)
   - GPIO test limited to D0 and D1 only
   - Non-blocking alternating pattern: D0 HIGH then D1 HIGH, repeat
   - After each full D0->D1 cycle, start an I2C-1 scan
   - I2C scan is watchdog-safe: scans 1 address per loop tick
   - Uses build_flags:
       -DENABLE_GPIO_TEST_CFG
       -DGPIO_TEST_INTERVAL_MS_CFG
       -DD0_GPIO_CFG
       -DD1_GPIO_CFG
       -DENABLE_I2C_SCAN_CFG
       -DI2C1_SDA_GPIO_CFG
       -DI2C1_SCL_GPIO_CFG
       -DI2C1_FREQ_CFG
   ========================================================= */

/* ================================
   GPIO OUTPUT TEST (D0, D1 only)
   - Alternating "walking 1" for 2 pins
   ================================ */
static constexpr bool     ENABLE_GPIO_TEST_DEFAULT      = true;
static constexpr uint32_t GPIO_TEST_INTERVAL_MS_DEFAULT = 200;

#ifdef ENABLE_GPIO_TEST_CFG
static constexpr bool ENABLE_GPIO_TEST = (ENABLE_GPIO_TEST_CFG != 0);
#else
static constexpr bool ENABLE_GPIO_TEST = ENABLE_GPIO_TEST_DEFAULT;
#endif

#ifdef GPIO_TEST_INTERVAL_MS_CFG
static constexpr uint32_t GPIO_TEST_INTERVAL_MS = (uint32_t)GPIO_TEST_INTERVAL_MS_CFG;
#else
static constexpr uint32_t GPIO_TEST_INTERVAL_MS = GPIO_TEST_INTERVAL_MS_DEFAULT;
#endif

#ifdef D0_GPIO_CFG
static constexpr int D0_GPIO = (int)D0_GPIO_CFG;
#else
static constexpr int D0_GPIO = -1;
#endif

#ifdef D1_GPIO_CFG
static constexpr int D1_GPIO = (int)D1_GPIO_CFG;
#else
static constexpr int D1_GPIO = -1;
#endif

static const int GPIO_TEST_PINS[2] = { D0_GPIO, D1_GPIO };

static uint32_t gpio_next_step_ms = 0;
static uint8_t  gpio_step_index   = 0;   // 0 -> D0, 1 -> D1
static bool     gpio_completed_cycle = false;

static inline void gpio_write_safe(int pin, uint8_t level)
{
  if (pin < 0) return;
  digitalWrite(pin, level);
}

/* ================================
   I2C-1 SCAN (AFTER EACH GPIO CYCLE) - WATCHDOG SAFE
   - Must scan bus 1 (not bus 0)
   - Uses TwoWire(1)
   - Bus initialized once
   - Scan runs incrementally: 1 address per loop tick (no blocking)
   ================================ */
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

// incremental scan state
static bool     i2c_scan_active   = false;
static uint8_t  i2c_scan_addr     = 0x03;
static uint8_t  i2c_scan_found    = 0;
static uint32_t i2c_scan_start_ms = 0;

/* ================================
   LOGGING (timestamps)
   ================================ */
static inline void log_ts(const char* tag, const char* fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  Serial.printf("[%10lu] %s %s\n", (unsigned long)millis(), tag, buf);
}

/* ================================
   I2C helpers
   ================================ */
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

  // Initialize I2C bus 1 with explicit pins
  I2Cbus1.begin(I2C1_SDA_GPIO, I2C1_SCL_GPIO, I2C1_FREQ);

  i2c1_initialized = true;
}

static void i2c1_scan_start()
{
  if (!ENABLE_I2C_SCAN) return;
  if (!i2c1_initialized) return;

  // start a new scan
  i2c_scan_active   = true;
  i2c_scan_addr     = 0x03;
  i2c_scan_found    = 0;
  i2c_scan_start_ms = millis();

  Serial.println("=======================================");
  Serial.println(" I2C-1 SCAN START (after GPIO cycle)");
  Serial.println("=======================================");
}

static void i2c1_scan_tick()
{
  if (!ENABLE_I2C_SCAN) return;
  if (!i2c1_initialized) return;
  if (!i2c_scan_active) return;

  // Scan exactly ONE address per loop iteration (watchdog safe)
  const uint8_t addr = i2c_scan_addr;

  I2Cbus1.beginTransmission(addr);
  if (I2Cbus1.endTransmission() == 0)
  {
    Serial.printf("  ✓ I2C device found at 0x%02X\n", addr);
    i2c_scan_found++;
  }

  // advance
  if (i2c_scan_addr >= 0x77)
  {
    const uint32_t dur = millis() - i2c_scan_start_ms;

    if (i2c_scan_found == 0)
      Serial.println("  (no I2C devices found)");
    else
      Serial.printf("  Total devices: %u\n", i2c_scan_found);

    Serial.printf("  Scan duration: %lu ms\n", (unsigned long)dur);

    i2c_scan_active = false;
    return;
  }

  i2c_scan_addr++;

  // Give FreeRTOS/USB time; avoids WDT even on busy systems.
  delay(0);
}

/* ================================
   GPIO test init/tick (D0,D1 only)
   ================================ */
static void gpio_test_init()
{
  if (!ENABLE_GPIO_TEST) return;

  // Configure pins as outputs, set LOW
  for (int i = 0; i < 2; i++)
  {
    const int pin = GPIO_TEST_PINS[i];
    if (pin >= 0)
    {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    }
  }

  gpio_step_index = 0;
  gpio_completed_cycle = false;
  gpio_next_step_ms = millis() + GPIO_TEST_INTERVAL_MS;
}

static void gpio_test_tick()
{
  if (!ENABLE_GPIO_TEST) return;

  const uint32_t now = millis();
  if ((int32_t)(now - gpio_next_step_ms) < 0) return;

  // Alternating "walking 1": only the active index HIGH
  for (int i = 0; i < 2; i++)
  {
    gpio_write_safe(GPIO_TEST_PINS[i], (i == gpio_step_index) ? HIGH : LOW);
  }

  // Advance: 0->1->0...
  const uint8_t prev = gpio_step_index;
  gpio_step_index = (uint8_t)((gpio_step_index + 1) & 0x01);

  // A "full cycle" is D0 then D1, so detect wrap 1->0
  gpio_completed_cycle = (prev == 1 && gpio_step_index == 0);

  // After each full D0..D1 cycle, start I2C-1 scan
  if (gpio_completed_cycle)
  {
    i2c1_scan_start();
  }

  gpio_next_step_ms = now + GPIO_TEST_INTERVAL_MS;
}

/* ================================
   SETUP
   ================================ */
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("=======================================");
  Serial.println(" GPIO D0/D1 + I2C-1 SCAN TEST (UART REMOVED)");
  Serial.println("=======================================");
  log_ts("BOOT", "Build: %s %s", __DATE__, __TIME__);

  // GPIO test init
  if (ENABLE_GPIO_TEST)
  {
    log_ts("GPIO", "GPIO test enabled interval=%lu ms", (unsigned long)GPIO_TEST_INTERVAL_MS);
    log_ts("GPIO", "D0=%d D1=%d", D0_GPIO, D1_GPIO);
  }
  else
  {
    log_ts("GPIO", "GPIO test DISABLED");
  }
  gpio_test_init();

  // I2C-1 init once (scan runs incrementally after each GPIO cycle)
  if (ENABLE_I2C_SCAN)
  {
    log_ts("I2C", "I2C-1 scan enabled (after each GPIO cycle)");
    log_ts("I2C", "SDA=%d SCL=%d Freq=%lu", I2C1_SDA_GPIO, I2C1_SCL_GPIO, (unsigned long)I2C1_FREQ);
  }
  else
  {
    log_ts("I2C", "I2C-1 scan DISABLED");
  }
  i2c1_init_once();
}

/* ================================
   LOOP
   ================================ */
void loop()
{
  // GPIO test tick (non-blocking; triggers I2C scan start on full D0->D1 cycle)
  gpio_test_tick();

  // I2C scan tick (non-blocking; one address per loop)
  i2c1_scan_tick();
}