// ESP32-C3 potentiometer reader. See wiki: Hardware-Firmware-Encoder.
// - 11 dB attenuation, calibrated mV reads
// - Trimmed-mean + EMA filter
// - Binary framed packets @ fixed rate, CRC8 + sequence number
// - GND/VCC reference auto-calibrated at boot and periodically

#include <Arduino.h>

// --- Pins ---
#define PWM_PIN     8
#define PIN_WIPER   2
#define PIN_GND     0
#define PIN_VCC     4
#define UART_TX_PIN 20

// --- PWM ---
#define PWM_FREQ    1000
#define PWM_RES     12

// --- UART ---
#define UART_BAUD   921600
#define SYNC_BYTE   0xAA

// --- Sampling / filter ---
#define SAMPLE_N        32       // samples per trimmed-mean batch
#define TRIM_EACH_SIDE  4        // drop N lowest + N highest
#define EMA_ALPHA_Q15   6554     // 0.20 * 32768 (exponential smoothing in Q15)
#define OUTPUT_HZ       200      // fixed packet rate
#define CAL_INTERVAL_MS 5000     // re-calibrate refs every 5 s
#define CAL_SAMPLES     256      // averaging count for ref cal

// --- Packet (little-endian, 7 bytes; must match lidar3d.py UartAngleReader) ---
// [0]    sync   0xAA
// [1]    seq    uint8 (wraps)
// [2..3] angle  int16, centi-degrees (e.g. 12345 = 123.45°)
// [4..5] wiper  uint16, millivolts
// [6]    crc8   poly 0x07, init 0x00, over bytes 1..5
struct __attribute__((packed)) Packet {
    uint8_t  sync;
    uint8_t  seq;
    int16_t  angle_cdeg;
    uint16_t wiper_mv;
    uint8_t  crc;
};

// --- State ---
static uint16_t refGndMv = 0;
static uint16_t refVccMv = 3300;
static int32_t  emaQ15   = 0;
static bool     emaInit  = false;
static uint8_t  seq      = 0;
static uint32_t lastCalMs    = 0;
static uint32_t nextSendUs   = 0;
static const uint32_t SEND_PERIOD_US = 1000000UL / OUTPUT_HZ;

// --- CRC8 (poly 0x07, init 0x00) ---

static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t c = 0;
    while (len--) {
        c ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            c = (c & 0x80) ? (c << 1) ^ 0x07 : (c << 1);
    }
    return c;
}

// --- ADC helpers ---

static int cmpU16(const void *a, const void *b) {
    return (int)(*(const uint16_t*)a) - (int)(*(const uint16_t*)b);
}

static uint16_t readTrimmedMeanMv(uint8_t pin) {
    uint16_t buf[SAMPLE_N];
    for (uint8_t i = 0; i < SAMPLE_N; i++) {
        buf[i] = (uint16_t)analogReadMilliVolts(pin);
    }
    qsort(buf, SAMPLE_N, sizeof(uint16_t), cmpU16);
    uint32_t sum = 0;
    for (uint8_t i = TRIM_EACH_SIDE; i < SAMPLE_N - TRIM_EACH_SIDE; i++) sum += buf[i];
    return (uint16_t)(sum / (SAMPLE_N - 2 * TRIM_EACH_SIDE));
}

static uint16_t calibrateRef(uint8_t pin) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < CAL_SAMPLES; i++)
        sum += (uint16_t)analogReadMilliVolts(pin);
    return (uint16_t)(sum / CAL_SAMPLES);
}

static void recalibrateRefs() {
    refGndMv = calibrateRef(PIN_GND);
    refVccMv = calibrateRef(PIN_VCC);
    if (refVccMv <= refGndMv) refVccMv = refGndMv + 1;  // guard div/0
}

// --- Setup ---

void setup() {
    Serial.begin(115200);
    Serial1.begin(UART_BAUD, SERIAL_8N1, -1, UART_TX_PIN);

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_WIPER, ADC_11db);
    analogSetPinAttenuation(PIN_GND,   ADC_11db);
    analogSetPinAttenuation(PIN_VCC,   ADC_11db);

    ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);

    delay(50);
    recalibrateRefs();
    lastCalMs  = millis();
    nextSendUs = micros();

    Serial.printf("Boot done. GND=%u mV  VCC=%u mV  baud=%u  rate=%uHz\n",
                  refGndMv, refVccMv, UART_BAUD, OUTPUT_HZ);
}

// --- Main loop ---

void loop() {
    uint32_t nowMs = millis();
    if (nowMs - lastCalMs >= CAL_INTERVAL_MS) {
        recalibrateRefs();
        lastCalMs = nowMs;
    }

    uint16_t wiperMv = readTrimmedMeanMv(PIN_WIPER);

    // EMA smoothing in Q15.
    if (!emaInit) { emaQ15 = (int32_t)wiperMv << 15; emaInit = true; }
    else {
        int32_t target = (int32_t)wiperMv << 15;
        emaQ15 += ((int64_t)(target - emaQ15) * EMA_ALPHA_Q15) >> 15;
    }
    int32_t smoothMv = emaQ15 >> 15;

    // Map wiper voltage to angle in centi-degrees (0..27000 = 0..270°).
    int32_t span = (int32_t)refVccMv - (int32_t)refGndMv;
    int32_t rel  = smoothMv - (int32_t)refGndMv;
    if (rel < 0)    rel = 0;
    if (rel > span) rel = span;
    int16_t angle_cdeg = (int16_t)((rel * 27000) / span);

    int duty = (int)((rel * 4095) / span);
    ledcWrite(PWM_PIN, duty);

    // Fixed-rate transmit (signed cast handles uint32_t wraparound).
    uint32_t nowUs = micros();
    if ((int32_t)(nowUs - nextSendUs) >= 0) {
        nextSendUs += SEND_PERIOD_US;
        if ((int32_t)(nowUs - nextSendUs) > (int32_t)SEND_PERIOD_US)
            nextSendUs = nowUs + SEND_PERIOD_US;

        Packet p;
        p.sync       = SYNC_BYTE;
        p.seq        = seq++;
        p.angle_cdeg = angle_cdeg;
        p.wiper_mv   = (uint16_t)smoothMv;
        p.crc        = crc8(((uint8_t*)&p) + 1, sizeof(Packet) - 2);
        Serial1.write((uint8_t*)&p, sizeof(p));
    }
}
