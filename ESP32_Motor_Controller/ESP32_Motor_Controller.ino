/*
 * ESP32 Motor Controller
 *
 * Single-loop architecture.
 * ISRs write volatile period data; loop() reads, filters, and dispatches.
 *
 * Protocol (8 bytes, unchanged from v1):
 *   Command:  [START=0x55][CMD][M1_SPD][M2_SPD][M3_SPD][M4_SPD][DIRS][CRC8]
 *   Response: [START=0x55][STATUS][M1_RPM][M2_RPM][M3_RPM][M4_RPM][VBATT][CRC8]
 *
 *   CRC-8/MAXIM covers bytes 0-6.
 *   DIRS = [0][0][0][0][M4_DIR][M3_DIR][M2_DIR][M1_DIR]
 *   VBATT = battery voltage scaled 0-255 (0-16V)
 *
 * Commands:
 *   0x01: Set all motors (speed + direction)
 *   0x02: Stop all motors
 *   0xFF: Ping / request telemetry
 *
 * Status:
 *   0x00: OK
 *   0x01: Invalid command
 */

#include <HardwareSerial.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// UART — using UART0 (USB via onboard USB-to-serial chip)
// Switch UART_USE_USB to 0 and set UART_TX_PIN/RX_PIN to use hardware pins instead.
#define UART_USE_USB  1          // 1 = Serial (USB/UART0), 0 = UART2 (pins 16/17)
#define UART_TX_PIN   17         // only used when UART_USE_USB = 0
#define UART_RX_PIN   16         // only used when UART_USE_USB = 0
#define UART_BAUDRATE 921600

// Battery ADC
#define VOLTAGE_PIN           13
#define VOLTAGE_DIVIDER_RATIO 0.17543859649123f  // 100k / (470k + 100k)
#define ADC_VREF              3.3f
#define ADC_CAL_FACTOR        (16.0f / 10.0f)    // empirical non-linearity correction

// Motor PWM
#define PWM_FREQ       20000
#define PWM_RESOLUTION 10      // 10-bit → 0-1023
#define PWM_MAX        1023

// Encoder
#define PPR                 270
#define FEEDBACK_TIMEOUT_US 6000                    // RPM → 0 if no pulse within this window
#define DEBOUNCE_MIN_US     50                      // ignore pulses shorter than this
#define DEBOUNCE_MAX_US     ((uint32_t)(FEEDBACK_TIMEOUT_US * 4))

// EMA filter (0.0–1.0: higher = faster response, more noise)
#define EMA_ALPHA 0.4f

// Comms watchdog — stop motors if no valid packet received within this window.
// Must be set to at least 3-5× your control loop's inter-packet interval.
// e.g. 100 Hz loop (10 ms/packet) → 50 ms minimum; 50 Hz (20 ms) → 100 ms.
// Too short and the watchdog fires between every packet, keeping motors stopped.
#define WATCHDOG_MS 200

// Protocol
#define PACKET_SIZE 8
#define START_BYTE  0x55

// Commands
#define CMD_SET_ALL  0x01
#define CMD_STOP_ALL 0x02
#define CMD_PING     0xFF

// Status bytes
#define STATUS_OK           0x00
#define STATUS_INVALID_CMD  0x01
#define STATUS_CHECKSUM_ERR 0x02

// ============================================================================
// MOTOR PIN MAP
// ============================================================================

struct MotorPins {
  int  dir_pin;
  int  pwm_pin;
  int  sig_pin;
  bool invert_dir;
};

const MotorPins MOTOR_CONFIG[4] = {
  {21, 22, 23, true },  // M1
  {18,  2, 19, false},  // M2
  {14, 12, 27, false},  // M3
  {25, 26, 33, true }   // M4
};

// ============================================================================
// MOTOR STATE
// ============================================================================

struct MotorState {
  volatile uint32_t last_rise_us;  // written by ISR
  volatile uint32_t period_us;     // written by ISR (latest raw period)
  float    ema_period_us;          // EMA-filtered period  (loop() only)
  uint8_t  rpm;                    // computed RPM         (loop() only)
};

MotorState motors[4];

// ============================================================================
// ENCODER ISRs
// No locks needed — loop() is the sole reader of ema_period_us / rpm,
// and 32-bit aligned volatile reads are atomic on Xtensa LX6.
// ============================================================================

void IRAM_ATTR motorISR0() {
  uint32_t now = micros();
  uint32_t p = now - motors[0].last_rise_us;
  if (p > DEBOUNCE_MIN_US && p < DEBOUNCE_MAX_US) motors[0].period_us = p;
  motors[0].last_rise_us = now;
}

void IRAM_ATTR motorISR1() {
  uint32_t now = micros();
  uint32_t p = now - motors[1].last_rise_us;
  if (p > DEBOUNCE_MIN_US && p < DEBOUNCE_MAX_US) motors[1].period_us = p;
  motors[1].last_rise_us = now;
}

void IRAM_ATTR motorISR2() {
  uint32_t now = micros();
  uint32_t p = now - motors[2].last_rise_us;
  if (p > DEBOUNCE_MIN_US && p < DEBOUNCE_MAX_US) motors[2].period_us = p;
  motors[2].last_rise_us = now;
}

void IRAM_ATTR motorISR3() {
  uint32_t now = micros();
  uint32_t p = now - motors[3].last_rise_us;
  if (p > DEBOUNCE_MIN_US && p < DEBOUNCE_MAX_US) motors[3].period_us = p;
  motors[3].last_rise_us = now;
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void setMotor(int id, int speed, bool dir) {
  speed = constrain(speed, 0, 255);
  digitalWrite(MOTOR_CONFIG[id].dir_pin, dir ^ MOTOR_CONFIG[id].invert_dir);
  int duty = ((255 - speed) * PWM_MAX) / 255;  // inverted: 0 speed → full PWM_MAX (stopped)
  ledcWrite(MOTOR_CONFIG[id].pwm_pin, duty);
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) setMotor(i, 0, 0);
}

// ============================================================================
// CRC-8/MAXIM (polynomial 0x31)
// ============================================================================

uint8_t crc8(const uint8_t* data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
}

// ============================================================================
// BATTERY ADC — 4-sample oversampling + voltage divider + calibration
// ============================================================================

uint8_t readBattery() {
  int sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(VOLTAGE_PIN);
  float v_meas = ((sum / 4) / 4095.0f) * ADC_VREF;
  float v_batt = (v_meas / VOLTAGE_DIVIDER_RATIO) * ADC_CAL_FACTOR;
  return (uint8_t)constrain((int)(v_batt * (255.0f / 16.0f)), 0, 255);
}

// ============================================================================
// RPM UPDATE — EMA filter applied every loop() iteration
// ============================================================================

void updateRPM() {
  uint32_t now = micros();
  for (int i = 0; i < 4; i++) {
    uint32_t last_rise = motors[i].last_rise_us;  // volatile 32-bit: atomic on LX6
    uint32_t period    = motors[i].period_us;

    if (period == 0 || (now - last_rise) > FEEDBACK_TIMEOUT_US) {
      motors[i].ema_period_us = 0.0f;
      motors[i].rpm = 0;
    } else {
      if (motors[i].ema_period_us == 0.0f) {
        motors[i].ema_period_us = (float)period;  // seed filter on first pulse
      } else {
        motors[i].ema_period_us = EMA_ALPHA * (float)period
                                + (1.0f - EMA_ALPHA) * motors[i].ema_period_us;
      }
      float rpm_f = 60000000.0f / (motors[i].ema_period_us * PPR);
      motors[i].rpm = (uint8_t)constrain((int)rpm_f, 0, 255);
    }
  }
}

// ============================================================================
// UART STATE MACHINE
// ============================================================================

// Battery reading cached here and refreshed every BATTERY_UPDATE_MS milliseconds.
// Keeps readBattery() (4x blocking analogRead) off the response hot-path.
#define BATTERY_UPDATE_MS 5
static uint8_t  cached_battery     = 0;
static uint32_t battery_next_ms    = 0;

// Comms serial — Serial (UART0) for USB, or UART2 for hardware pins
#if UART_USE_USB
  #define UARTComm Serial
#else
  HardwareSerial UARTComm(2);
#endif

enum UartState { WAIT_START, COLLECT };
static UartState uart_state = WAIT_START;
static uint8_t   rx_buf[PACKET_SIZE];
static int       rx_count = 0;

// Watchdog state
static uint32_t last_valid_ms  = 0;
static bool     watchdog_fired = false;

void sendResponse(uint8_t status) {
  uint8_t resp[PACKET_SIZE];
  resp[0] = START_BYTE;
  resp[1] = status;
  for (int i = 0; i < 4; i++) resp[2 + i] = motors[i].rpm;
  resp[6] = cached_battery;   // updated every ~5 ms in loop(); never blocks
  resp[7] = crc8(resp, PACKET_SIZE - 1);
  UARTComm.write(resp, PACKET_SIZE);
}

void runUART() {
  while (UARTComm.available()) {
    uint8_t b = (uint8_t)UARTComm.read();

    if (uart_state == WAIT_START) {
      if (b != START_BYTE) continue;     // skip until framed
      rx_buf[0]  = b;
      rx_count   = 1;
      uart_state = COLLECT;
      continue;
    }

    // COLLECT state
    rx_buf[rx_count++] = b;

    if (rx_count < PACKET_SIZE) continue;  // not yet a full packet

    // ── Full packet received ──────────────────────────────────────────────
    uart_state = WAIT_START;
    rx_count   = 0;

    // Validate CRC
    if (crc8(rx_buf, PACKET_SIZE - 1) != rx_buf[PACKET_SIZE - 1]) {
      // Flush anything still in the FIFO before responding.
      // Without this, a 0x55 byte inside the corrupted packet (or the next
      // in-flight packet) would be picked up as a false start byte, causing
      // the *next* packet to be misaligned and also fail CRC — cascading errors.
      while (UARTComm.available()) UARTComm.read();
      uart_state = WAIT_START;
      sendResponse(STATUS_CHECKSUM_ERR);
      break;
    }

    // Valid packet
    last_valid_ms  = millis();
    watchdog_fired = false;

    uint8_t cmd  = rx_buf[1];
    uint8_t stat = STATUS_OK;

    switch (cmd) {
      case CMD_SET_ALL: {
        uint8_t dirs = rx_buf[6];
        setMotor(0, rx_buf[2],  dirs        & 0x01);
        setMotor(1, rx_buf[3], (dirs >> 1)  & 0x01);
        setMotor(2, rx_buf[4], (dirs >> 2)  & 0x01);
        setMotor(3, rx_buf[5], (dirs >> 3)  & 0x01);
        break;
      }
      case CMD_STOP_ALL:
        stopAllMotors();
        break;
      case CMD_PING:
        break;  // just respond with current telemetry
      default:
        stat = STATUS_INVALID_CMD;
        break;
    }

    sendResponse(stat);
    break;  // one packet per drain pass — keep latency predictable
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialise motors
  for (int i = 0; i < 4; i++) {
    motors[i] = {0, 0, 0.0f, 0};
    pinMode(MOTOR_CONFIG[i].dir_pin, OUTPUT);
    pinMode(MOTOR_CONFIG[i].sig_pin, INPUT_PULLUP);
    digitalWrite(MOTOR_CONFIG[i].dir_pin, LOW);
    ledcAttach(MOTOR_CONFIG[i].pwm_pin, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTOR_CONFIG[i].pwm_pin, PWM_MAX);  // stopped
  }

  // Attach encoder ISRs
  attachInterrupt(digitalPinToInterrupt(MOTOR_CONFIG[0].sig_pin), motorISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_CONFIG[1].sig_pin), motorISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_CONFIG[2].sig_pin), motorISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_CONFIG[3].sig_pin), motorISR3, RISING);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_ATTENDB_MAX);  // 0-3.3V range

  // UART init — wake on first byte, flush partial packets after ~174 µs idle
#if UART_USE_USB
  UARTComm.begin(UART_BAUDRATE);  // USB: pins fixed by hardware
#else
  UARTComm.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
#endif
  UARTComm.setRxFIFOFull(1);
  UARTComm.setRxTimeout(2);

  last_valid_ms = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // 1. Update EMA-filtered RPM from latest ISR data
  updateRPM();

  // 2. Drain UART and dispatch any complete packet
  runUART();

  // 3. Comms watchdog — stop motors after 5 ms of silence
  if (!watchdog_fired && (millis() - last_valid_ms > WATCHDOG_MS)) {
    stopAllMotors();
    watchdog_fired = true;
  }

  // 4. Refresh battery cache every BATTERY_UPDATE_MS — time-based so the loop
  //    can run as fast as possible without hammering the ADC.
  uint32_t now_ms = millis();
  if (now_ms >= battery_next_ms) {
    battery_next_ms = now_ms + BATTERY_UPDATE_MS;
    cached_battery = readBattery();
  }
}
