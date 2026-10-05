/*
 * ESP32-S3 closed-loop speed control of a DC motor with quadrature encoder.
 *
 *  - Motor   : JGA25-370, 12 V, 60 rpm (output shaft), 1:103 gearbox, Hall encoder
 *  - Driver  : TB6612FNG (channel A)
 *  - Setpoint: potentiometer on ADC1 (0 .. MAX_SETPOINT_RPM)
 *  - Feedback: speed = encoder counts accumulated over a fixed sample period
 *  - Control : PI + feed-forward, anti-windup, fixed 20 ms sample time
 *
 * Wiring (change the pins below if needed):
 *   TB6612FNG  PWMA -> GPIO6     AIN1 -> GPIO4     AIN2 -> GPIO5    STBY -> GPIO7
 *              VM   -> 12 V      VCC  -> 3.3 V     GND  -> common GND
 *              AO1/AO2 -> motor terminals
 *   Encoder    A -> GPIO15, B -> GPIO16, VCC -> 3.3 V (keeps signals 3.3 V safe), GND
 *   Pot        wiper -> GPIO1 (ADC1_CH0), ends -> 3.3 V and GND
 *
 * Serial (115200) prints "setpoint_rpm,measured_rpm,duty_percent" -> usable
 * with the Arduino / VS Code serial plotter for tuning.
 *
 * Serial commands (end each line with LF or CR+LF, case-insensitive):
 *   kp <value>      set proportional gain (duty per rpm)
 *   ki <value>      set integral gain (duty per rpm*s)
 *   get             print current gains
 *   save            store gains in flash (restored on boot)
 *   default         restore compile-time default gains
 *   stream on|off   enable/disable the CSV telemetry stream
 *   help            list commands
 */

#include <Arduino.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "PiController.h"

// ------------------------------- Pin map ----------------------------------
constexpr int PIN_PWMA = 6;
constexpr int PIN_AIN1 = 4;
constexpr int PIN_AIN2 = 5;
constexpr int PIN_STBY = 7;

constexpr int PIN_ENC_A = 15;
constexpr int PIN_ENC_B = 16;

constexpr int PIN_POT = 1;  // ADC1 channel (ADC2 is avoided on purpose)

// ----------------------------- Motor / encoder ----------------------------
// JGA25-370: 11 pulses/rev per channel on the motor shaft and a 103:1 gearbox
// (60 rpm at 12 V) -> 11 * 4 * 103 = 4532 counts per output rev.
// If necessary, calibrate by turning the output shaft exactly one revolution
// and reading the counter.
constexpr float ENCODER_PPR_MOTOR = 11.0f;  // pulses per channel per motor rev
constexpr float GEAR_RATIO        = 103.0f;
constexpr float COUNTS_PER_REV    = ENCODER_PPR_MOTOR * 4.0f * GEAR_RATIO;  // x4 decoding
constexpr bool  ENCODER_INVERT    = false;  // flip if speed reads negative

constexpr float MOTOR_NOLOAD_RPM  = 60.0f;   // at 12 V
// Keep the setpoint below the no-load speed to leave control headroom.
constexpr float MAX_SETPOINT_RPM  = 54.0f;
constexpr float MIN_SETPOINT_RPM  = 2.0f;    // below this the motor is stopped

// ------------------------------- PWM --------------------------------------
constexpr uint32_t PWM_FREQ_HZ = 20000;  // inaudible, TB6612FNG supports up to 100 kHz
constexpr uint8_t  PWM_BITS    = 10;
constexpr uint32_t PWM_MAX     = (1u << PWM_BITS) - 1;
constexpr uint8_t  PWM_CHANNEL = 0;      // used by LEDC API of arduino-esp32 2.x

// ------------------------------ Control -----------------------------------
constexpr uint32_t CONTROL_PERIOD_US = 20000;  // 20 ms -> 50 Hz loop (~91 counts/sample at full speed)
constexpr uint32_t PRINT_PERIOD_MS   = 50;

constexpr float KP_DEFAULT = 0.007f;  // duty per rpm of error (starting point, tune over serial)
constexpr float KI_DEFAULT = 0.050f;  // duty per (rpm * s) of error
constexpr float GAIN_MAX   = PiController::kGainMax;  // sanity limit for values typed over serial
constexpr float INTEGRAL_LIMIT = 0.5f;                // anti-windup clamp, duty units

// PI controller (output = duty 0..1). Gains are adjustable via the serial interface.
static PiController pi(KP_DEFAULT, KI_DEFAULT, 0.0f, 1.0f, INTEGRAL_LIMIT);

constexpr float SPEED_FILTER_ALPHA    = 0.35f;  // EMA on measured speed (1 = no filter)
constexpr float SETPOINT_FILTER_ALPHA = 0.10f;  // EMA on pot reading (noise rejection)
constexpr float SETPOINT_RAMP_RPM_S   = 170.0f; // max setpoint change rate (soft start)

// ---------------------------- Encoder decoding ----------------------------
static volatile int32_t encoderCount = 0;
static volatile uint8_t encoderState = 0;

// Index = (previousState << 2) | currentState, state = (A << 1) | B
static const int8_t QUAD_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static void IRAM_ATTR encoderISR() {
    const uint8_t a = gpio_get_level((gpio_num_t)PIN_ENC_A);
    const uint8_t b = gpio_get_level((gpio_num_t)PIN_ENC_B);
    const uint8_t curr = (a << 1) | b;
    encoderCount += QUAD_TABLE[(encoderState << 2) | curr];
    encoderState = curr;
}

// ------------------------------ PWM helpers -------------------------------
static void pwmInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_PWMA, PWM_FREQ_HZ, PWM_BITS);
#else
    ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_BITS);
    ledcAttachPin(PIN_PWMA, PWM_CHANNEL);
#endif
}

static void pwmWrite(uint32_t value) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PIN_PWMA, value);
#else
    ledcWrite(PWM_CHANNEL, value);
#endif
}

// Drive motor forward with duty 0..1 (single direction).
static void motorSetDuty(float duty) {
    duty = constrain(duty, 0.0f, 1.0f);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    pwmWrite((uint32_t)(duty * PWM_MAX + 0.5f));
}

// Short brake (AIN1 = AIN2 = HIGH) and zero PWM.
static void motorBrake() {
    pwmWrite(0);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
}

// ------------------------------ Potentiometer -----------------------------
static float readPotNormalized() {
    uint32_t sum = 0;
    constexpr int N = 16;
    for (int i = 0; i < N; ++i) sum += analogRead(PIN_POT);
    return (float)sum / (N * 4095.0f);  // 0..1
}

// ------------------------------ State -------------------------------------
static float setpointFiltered = 0.0f;  // rpm, filtered pot value
static float setpointRamped   = 0.0f;  // rpm, rate-limited value used by the PI
static float measuredRpm      = 0.0f;
static float dutyOut          = 0.0f;
static int32_t lastCount      = 0;
static uint32_t lastControlUs = 0;
static uint32_t lastPrintMs   = 0;
static bool motorStopped      = true;
static bool streamEnabled     = true;

// ------------------------------ Serial tuning -----------------------------
static Preferences prefs;

static void printGains() {
    Serial.printf("# Kp=%.5f Ki=%.5f\n", pi.kp, pi.ki);
}

static void loadGains() {
    prefs.begin("pi", true);  // read-only
    const float kp = prefs.getFloat("kp", KP_DEFAULT);
    const float ki = prefs.getFloat("ki", KI_DEFAULT);
    prefs.end();
    if (!pi.setGains(kp, ki)) pi.setGains(KP_DEFAULT, KI_DEFAULT);  // ignore corrupt values
}

static void saveGains() {
    prefs.begin("pi", false);
    prefs.putFloat("kp", pi.kp);
    prefs.putFloat("ki", pi.ki);
    prefs.end();
}

static void printHelp() {
    Serial.println("# Commands: kp <v> | ki <v> | get | save | default | stream on|off | help");
}

// Parses and validates a gain; returns true on success.
static bool parseGain(const String& arg, float& out) {
    if (arg.length() == 0) return false;
    char* end = nullptr;
    const float v = strtof(arg.c_str(), &end);
    if (end == arg.c_str() || *end != '\0') return false;
    if (!isfinite(v) || v < 0.0f || v > GAIN_MAX) return false;
    out = v;
    return true;
}

static void handleCommand(String line) {
    line.trim();
    line.toLowerCase();
    if (line.length() == 0) return;

    const int sp = line.indexOf(' ');
    const String cmd = (sp < 0) ? line : line.substring(0, sp);
    String arg = (sp < 0) ? "" : line.substring(sp + 1);
    arg.trim();

    float v;
    if (cmd == "kp" || cmd == "ki") {
        if (arg.length() == 0) {
            printGains();
        } else if (parseGain(arg, v) &&
                   (cmd == "kp" ? pi.setGains(v, pi.ki) : pi.setGains(pi.kp, v))) {
            printGains();
        } else {
            Serial.printf("# Error: value must be a number in 0..%.1f\n", GAIN_MAX);
        }
    } else if (cmd == "get") {
        printGains();
    } else if (cmd == "save") {
        saveGains();
        Serial.println("# Gains saved");
    } else if (cmd == "default") {
        pi.setGains(KP_DEFAULT, KI_DEFAULT);
        printGains();
    } else if (cmd == "stream") {
        if (arg == "on") streamEnabled = true;
        else if (arg == "off") streamEnabled = false;
        else { Serial.println("# Usage: stream on|off"); return; }
        Serial.printf("# Stream %s\n", streamEnabled ? "on" : "off");
    } else if (cmd == "help" || cmd == "?") {
        printHelp();
    } else {
        Serial.println("# Unknown command (type 'help')");
    }
}

// Non-blocking line reader; call every loop iteration.
static void serialTuningPoll() {
    static String buf;
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            handleCommand(buf);
            buf = "";
        } else if (buf.length() < 64) {
            buf += c;
        }
    }
}

void setup() {
    Serial.begin(115200);
    loadGains();

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);
    pwmInit();
    motorBrake();
    digitalWrite(PIN_STBY, HIGH);  // enable TB6612FNG

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_POT, ADC_11db);  // 0..~3.1 V range

    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    encoderState = (gpio_get_level((gpio_num_t)PIN_ENC_A) << 1) |
                    gpio_get_level((gpio_num_t)PIN_ENC_B);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

    setpointFiltered = readPotNormalized() * MAX_SETPOINT_RPM;
    lastCount        = encoderCount;
    lastControlUs    = micros();

    Serial.println("setpoint_rpm,measured_rpm,duty_percent");
    printGains();
    printHelp();
}

void loop() {
    serialTuningPoll();

    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;
    const float dt = (nowUs - lastControlUs) * 1e-6f;  // actual period, seconds
    lastControlUs = nowUs;

    // ---- 1. Speed measurement: counts over the sample period -------------
    const int32_t count = encoderCount;  // 32-bit aligned read is atomic
    int32_t delta = count - lastCount;
    lastCount = count;
    if (ENCODER_INVERT) delta = -delta;

    const float rpmRaw = ((float)delta / COUNTS_PER_REV) / dt * 60.0f;
    measuredRpm += SPEED_FILTER_ALPHA * (rpmRaw - measuredRpm);

    // ---- 2. Setpoint from the potentiometer ------------------------------
    const float potRpm = readPotNormalized() * MAX_SETPOINT_RPM;
    setpointFiltered += SETPOINT_FILTER_ALPHA * (potRpm - setpointFiltered);

    // Hysteresis around the stop threshold to avoid chatter near zero.
    if (motorStopped && setpointFiltered > MIN_SETPOINT_RPM + 1.0f) {
        motorStopped = false;
    } else if (!motorStopped && setpointFiltered < MIN_SETPOINT_RPM) {
        motorStopped = true;
    }

    if (motorStopped) {
        pi.reset();
        dutyOut       = 0.0f;
        setpointRamped = 0.0f;
        motorBrake();
    } else {
        // Rate-limit the setpoint for a smooth start / speed change.
        const float maxStep = SETPOINT_RAMP_RPM_S * dt;
        setpointRamped += constrain(setpointFiltered - setpointRamped, -maxStep, maxStep);

        // ---- 3. PI + feed-forward with anti-windup -----------------------
        const float ff = setpointRamped / MOTOR_NOLOAD_RPM;  // nominal duty for this speed
        dutyOut = pi.update(setpointRamped, measuredRpm, ff, dt);
        motorSetDuty(dutyOut);
    }

    // ---- 4. Telemetry ------------------------------------------------------
    const uint32_t nowMs = millis();
    if (streamEnabled && nowMs - lastPrintMs >= PRINT_PERIOD_MS) {
        lastPrintMs = nowMs;
        Serial.printf("%.1f,%.1f,%.1f\n", motorStopped ? 0.0f : setpointRamped,
                      measuredRpm, dutyOut * 100.0f);
    }
}
