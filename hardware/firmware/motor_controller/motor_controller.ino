/*
 * ESP32 Motor Controller. See wiki: Hardware-Firmware-MotorController.
 *
 * Single-loop architecture.
 * ISRs write volatile period data; loop() reads, filters, and dispatches.
 *
 * Protocol (8 bytes). See wiki: Allocator/Protocol.
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
 *   0x02: Checksum error
 */

#include <HardwareSerial.h>

// --- Configuration ---

// UART — USB (UART0) by default. Switch UART_USE_USB to 0 and set UART_TX/RX_PIN to use UART2.
#define UART_USE_USB  1
#define UART_TX_PIN   17
#define UART_RX_PIN   16
#define UART_BAUDRATE 921600

// Battery ADC
#define VOLTAGE_PIN           13
#define VOLTAGE_DIVIDER_RATIO 0.17543859649123f  // 100k / (470k + 100k)
#define ADC_VREF              3.3f
#define ADC_CAL_FACTOR        (16.0f / 10.0f)    // empirical non-linearity correction
#define VBATT_FULL_SCALE_V    16.0f              // byte 0-255 maps to 0..VBATT_FULL_SCALE_V volts

// Motor PWM
#define PWM_FREQ       20000
#define PWM_RESOLUTION 10      // 10-bit -> 0-1023
#define PWM_MAX        1023

// Encoder
#define PPR                 270
#define FEEDBACK_TIMEOUT_US 6000                                // RPM -> 0 if no pulse within this window
#define DEBOUNCE_MIN_US     50                                  // ignore pulses shorter than this
#define DEBOUNCE_MAX_US     ((uint32_t)(FEEDBACK_TIMEOUT_US * 4))

// EMA filter (0.0-1.0: higher = faster response, more noise)
#define EMA_ALPHA 0.4f

// Comms watchdog — stop motors if no valid packet within this window.
// Must be >= 3-5x the host's inter-packet interval. Too short and the watchdog
// fires between packets, keeping motors stopped.
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

// Battery cache refresh — keeps 4x blocking analogRead() off the response hot-path.
#define BATTERY_UPDATE_MS 5

// --- Motor pin map ---

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

// --- Motor state ---

struct MotorState {
  volatile uint32_t last_rise_us;  // written by ISR
  volatile uint32_t period_us;     // written by ISR (latest raw period)
  float    ema_period_us;          // EMA-filtered period (loop() only)
  uint8_t  rpm;                    // computed RPM (loop() only)
};

MotorState motors[4];

// --- Encoder ISRs ---
// No locks needed: loop() is the sole reader of ema_period_us / rpm, and 32-bit
// aligned volatile reads are atomic on Xtensa LX6.
// attachInterrupt() needs distinct function symbols, so one macro per motor.

#define DEFINE_MOTOR_ISR(N)                                                \
  void IRAM_ATTR motorISR##N() {                                           \
    uint32_t now = micros();                                               \
    uint32_t p = now - motors[N].last_rise_us;                             \
    if (p > DEBOUNCE_MIN_US && p < DEBOUNCE_MAX_US) motors[N].period_us = p; \
    motors[N].last_rise_us = now;                                          \
  }

DEFINE_MOTOR_ISR(0)
DEFINE_MOTOR_ISR(1)
DEFINE_MOTOR_ISR(2)
DEFINE_MOTOR_ISR(3)

// --- Motor control ---

void setMotor(int id, int speed, bool dir) {
  speed = constrain(speed, 0, 255);
  digitalWrite(MOTOR_CONFIG[id].dir_pin, dir ^ MOTOR_CONFIG[id].invert_dir);
  // PWM is inverted: 0 speed -> full PWM_MAX (stopped).
  int duty = ((255 - speed) * PWM_MAX) / 255;
  ledcWrite(MOTOR_CONFIG[id].pwm_pin, duty);
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) setMotor(i, 0, 0);
}

// --- CRC-8/MAXIM (polynomial 0x31) ---

uint8_t crc8(const uint8_t* data, int len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
}

// --- Battery ADC (4-sample oversampling + voltage divider + calibration) ---

uint8_t readBattery() {
  int sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(VOLTAGE_PIN);
  float v_meas = ((sum / 4) / 4095.0f) * ADC_VREF;
  float v_batt = (v_meas / VOLTAGE_DIVIDER_RATIO) * ADC_CAL_FACTOR;
  return (uint8_t)constrain((int)(v_batt * (255.0f / VBATT_FULL_SCALE_V)), 0, 255);
}

// --- RPM update (EMA filter applied every loop() iteration) ---

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

// --- UART state machine ---

#if UART_USE_USB
  #define UARTComm Serial
#else
  HardwareSerial UARTComm(2);
#endif

enum UartState { WAIT_START, COLLECT };
static UartState uart_state      = WAIT_START;
static uint8_t   rx_buf[PACKET_SIZE];
static int       rx_count        = 0;
static uint8_t   cached_battery  = 0;
static uint32_t  battery_next_ms = 0;
static uint32_t  last_valid_ms   = 0;
static bool      watchdog_fired  = false;

void sendResponse(uint8_t status) {
  uint8_t resp[PACKET_SIZE];
  resp[0] = START_BYTE;
  resp[1] = status;
  for (int i = 0; i < 4; i++) resp[2 + i] = motors[i].rpm;
  resp[6] = cached_battery;
  resp[7] = crc8(resp, PACKET_SIZE - 1);
  UARTComm.write(resp, PACKET_SIZE);
}

void runUART() {
  while (UARTComm.available()) {
    uint8_t b = (uint8_t)UARTComm.read();

    if (uart_state == WAIT_START) {
      if (b != START_BYTE) continue;
      rx_buf[0]  = b;
      rx_count   = 1;
      uart_state = COLLECT;
      continue;
    }

    rx_buf[rx_count++] = b;
    if (rx_count < PACKET_SIZE) continue;

    uart_state = WAIT_START;
    rx_count   = 0;

    if (crc8(rx_buf, PACKET_SIZE - 1) != rx_buf[PACKET_SIZE - 1]) {
      // Flush the FIFO before responding — a 0x55 byte inside the corrupted
      // packet (or the next in-flight packet) would be picked up as a false
      // start byte, cascading misalignment into the next frames.
      while (UARTComm.available()) UARTComm.read();
      sendResponse(STATUS_CHECKSUM_ERR);
      break;
    }

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
        break;
      default:
        stat = STATUS_INVALID_CMD;
        break;
    }

    sendResponse(stat);
    break;  // one packet per drain pass — keep latency predictable
  }
}

// --- Setup ---

void setup() {
  // motors[4] is zero-initialized at file scope; no explicit reset needed.
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_CONFIG[i].dir_pin, OUTPUT);
    pinMode(MOTOR_CONFIG[i].sig_pin, INPUT_PULLUP);
    digitalWrite(MOTOR_CONFIG[i].dir_pin, LOW);
    ledcAttach(MOTOR_CONFIG[i].pwm_pin, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(MOTOR_CONFIG[i].pwm_pin, PWM_MAX);  // stopped
  }

  attachInterrupt(digitalPinToInterrupt(MOTOR_CONFIG[0].sig_pin), motorISR0, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_CONFIG[1].sig_pin), motorISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_CONFIG[2].sig_pin), motorISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_CONFIG[3].sig_pin), motorISR3, RISING);

  analogReadResolution(12);
  analogSetAttenuation(ADC_ATTENDB_MAX);  // 0-3.3V range

#if UART_USE_USB
  UARTComm.begin(UART_BAUDRATE);
#else
  UARTComm.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
#endif
  UARTComm.setRxFIFOFull(1);   // wake on first byte
  UARTComm.setRxTimeout(2);    // flush partial packets after ~174 µs idle

  last_valid_ms = millis();
}

// --- Main loop ---

void loop() {
  updateRPM();
  runUART();

  if (!watchdog_fired && (millis() - last_valid_ms > WATCHDOG_MS)) {
    stopAllMotors();
    watchdog_fired = true;
  }

  uint32_t now_ms = millis();
  if (now_ms >= battery_next_ms) {
    battery_next_ms = now_ms + BATTERY_UPDATE_MS;
    cached_battery = readBattery();
  }
}
