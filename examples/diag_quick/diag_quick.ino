// ======================================================
// MAX6675 + ESP32 deep sleep — quick self-heating diagnostic
// ======================================================
//
// What it tests
//   Whether a MAX6675 heats up while the ESP32 is in deep sleep with
//   its CS/SCK pins left floating, which makes the first readings after
//   wake come out too high (the MAX6675 uses its own die temperature
//   for cold-junction compensation).
//
// How it works (~1.5 min per cycle)
//   - Sleeps SLEEP_S seconds. Cycles alternate: pins FLOATING during
//     sleep (no hold), then pins HELD (CS HIGH, SCK LOW).
//   - After waking, reads all channels at 0.3, 1, 2, 5, 10, 20, 30 s
//     and prints a table on Serial (115200).
//   - A channel that drops more than DROP_LIMIT_C during those 30 s was
//     warmer than its terminals when it woke up: marked with "!".
//   - Keeps a summary of the last 24 cycles in flash (NVS) and prints
//     it every cycle, so it can also run on battery and be read later.
//
// Expected if pin floating is the cause: "!" only on FLOATING cycles.
//
// Does not use WiFi, BLE or any network.
// ======================================================

#include "max6675.h"          // Adafruit MAX6675 library
#include <Preferences.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>

// ---------- settings ----------
#define SLEEP_MODE     2       // 0 = always floating · 1 = always held · 2 = alternate
#define SLEEP_S        60      // seconds asleep between tests
#define DROP_LIMIT_C   1.0f    // drop above this after wake = chip was warm

// ---------- pins (shared SCK/SO, one CS per module) ----------
#define PIN_SCK  18
#define PIN_SO   19
const int CS_PINS[] = {5, 32, 33};
#define N_TC  (int)(sizeof(CS_PINS) / sizeof(CS_PINS[0]))

// LilyGO T-SIM7000G only: keeps the SIM7000 modem off and the board LED
// (GPIO 12) isolated during sleep. Set to 0 on other boards.
#define LILYGO_T_SIM7000G  1
#define MODEM_PWRKEY       4
#define MODEM_RST          23
#define MODEM_PWRKEY_IDLE  LOW

MAX6675 tc1(PIN_SCK, CS_PINS[0], PIN_SO);
MAX6675 tc2(PIN_SCK, CS_PINS[1], PIN_SO);
MAX6675 tc3(PIN_SCK, CS_PINS[2], PIN_SO);
MAX6675* tcs[N_TC] = {&tc1, &tc2, &tc3};

const uint32_t MARKS_MS[] = {300, 1000, 2000, 5000, 10000, 20000, 30000};
const int NM = sizeof(MARKS_MS) / sizeof(MARKS_MS[0]);

// ---------- history in NVS (hundredths of °C) ----------
#define HIST 24
#define NO_READING -32768
struct Entry {
    uint32_t cycle;
    uint8_t  held;          // pins held during the sleep that just ended
    uint8_t  afterSleep;    // 1 = woke from deep sleep · 0 = power-on/reset
    int16_t  first[3];      // reading at 0.3 s
    int16_t  last[3];       // reading at 30 s
};
Entry hist[HIST];
uint8_t  histN = 0, histPos = 0;
uint32_t cycle = 0;
RTC_DATA_ATTR uint8_t heldDuringSleep = 0;
Preferences prefs;

int16_t toCent(float t) { return isnan(t) ? NO_READING : (int16_t)lroundf(t * 100.0f); }
float   fromCent(int16_t c) { return c == NO_READING ? NAN : c / 100.0f; }

void loadHistory() {
    prefs.begin("tcdiag", false);
    cycle   = prefs.getUInt("cycle", 0);
    histN   = prefs.getUChar("n", 0);
    histPos = prefs.getUChar("pos", 0);
    if (prefs.getBytes("hist", hist, sizeof(hist)) != sizeof(hist)) { histN = 0; histPos = 0; }
}

void saveHistory(const Entry& e) {
    hist[histPos] = e;
    histPos = (histPos + 1) % HIST;
    if (histN < HIST) histN++;
    prefs.putUInt("cycle", cycle);
    prefs.putUChar("n", histN);
    prefs.putUChar("pos", histPos);
    prefs.putBytes("hist", hist, sizeof(hist));
}

void printHistory() {
    Serial.println("\n===== HISTORY =====");
    Serial.print("cycle | sleep | pins     |");
    for (int s = 0; s < N_TC; s++) Serial.printf("  TC%d first→last (chg)  |", s + 1);
    Serial.println();
    for (int k = 0; k < histN; k++) {
        const Entry& e = hist[(histPos - histN + k + HIST) % HIST];
        Serial.printf("%5lu | %-5s | %-8s |", (unsigned long)e.cycle,
                      e.afterSleep ? "sleep" : "boot",
                      e.afterSleep ? (e.held ? "held" : "floating") : "-");
        for (int s = 0; s < N_TC; s++) {
            float a = fromCent(e.first[s]), b = fromCent(e.last[s]);
            if (isnan(a) || isnan(b)) { Serial.print("        no reading       |"); continue; }
            Serial.printf(" %6.2f→%6.2f (%+6.2f)%s|", a, b, b - a, (a - b) > DROP_LIMIT_C ? "!" : " ");
        }
        Serial.println();
    }
    Serial.println("(\"!\" = dropped more than 1 °C within 30 s after wake: chip was warm)");
}

void holdSensorPins(bool hold) {
    if (hold) {
        for (int s = 0; s < N_TC; s++) { pinMode(CS_PINS[s], OUTPUT); digitalWrite(CS_PINS[s], HIGH); }
        pinMode(PIN_SCK, OUTPUT); digitalWrite(PIN_SCK, LOW);
        for (int s = 0; s < N_TC; s++) gpio_hold_en((gpio_num_t)CS_PINS[s]);
        gpio_hold_en((gpio_num_t)PIN_SCK);
    } else {
        for (int s = 0; s < N_TC; s++) gpio_hold_dis((gpio_num_t)CS_PINS[s]);
        gpio_hold_dis((gpio_num_t)PIN_SCK);
    }
}

void boardPinsForSleep(bool sleeping) {
#if LILYGO_T_SIM7000G
    pinMode(MODEM_PWRKEY, OUTPUT); digitalWrite(MODEM_PWRKEY, MODEM_PWRKEY_IDLE);
    pinMode(MODEM_RST,    OUTPUT); digitalWrite(MODEM_RST,    HIGH);
    if (sleeping) {
        gpio_hold_en((gpio_num_t)MODEM_PWRKEY);
        gpio_hold_en((gpio_num_t)MODEM_RST);
        digitalWrite(12, HIGH);              // board LED off
        rtc_gpio_isolate(GPIO_NUM_12);
    } else {
        gpio_hold_dis((gpio_num_t)MODEM_PWRKEY);
        gpio_hold_dis((gpio_num_t)MODEM_RST);
        rtc_gpio_hold_dis(GPIO_NUM_12); rtc_gpio_deinit(GPIO_NUM_12);
        pinMode(12, OUTPUT); digitalWrite(12, LOW);   // LED on while awake
    }
#else
    (void)sleeping;
#endif
}

void setup() {
    Serial.begin(115200);

    // Same levels first, then release the hold: no floating transition.
    for (int s = 0; s < N_TC; s++) { pinMode(CS_PINS[s], OUTPUT); digitalWrite(CS_PINS[s], HIGH); }
    holdSensorPins(false);
    boardPinsForSleep(false);
    gpio_deep_sleep_hold_dis();

    bool afterSleep = esp_reset_reason() == ESP_RST_DEEPSLEEP;
    loadHistory();
    cycle++;

    float t[NM][N_TC];
    for (int m = 0; m < NM; m++) {
        while (millis() < MARKS_MS[m]) delay(5);
        for (int s = 0; s < N_TC; s++) {
            float v = tcs[s]->readCelsius();
            t[m][s] = (!isnan(v) && v > 0.0f && v <= 200.0f) ? v : NAN;
            delay(5);
        }
        if (m == 0) {   // print after the first reading so Serial does not delay it
            Serial.printf("\n===== cycle %lu · %s · pins during sleep: %s =====\n",
                          (unsigned long)cycle,
                          afterSleep ? "woke from deep sleep" : "boot (no sleep before, reference)",
                          afterSleep ? (heldDuringSleep ? "HELD" : "FLOATING") : "-");
            Serial.print("  time  |");
            for (int s = 0; s < N_TC; s++) Serial.printf("   TC%d   |", s + 1);
            Serial.println();
        }
        Serial.printf("%5.1f s |", MARKS_MS[m] / 1000.0f);
        for (int s = 0; s < N_TC; s++) {
            if (isnan(t[m][s])) Serial.print("  error  |");
            else Serial.printf(" %6.2f  |", t[m][s]);
        }
        Serial.println();
    }

    Entry e = {};
    e.cycle = cycle; e.held = heldDuringSleep; e.afterSleep = afterSleep;
    Serial.println("\nChange from 0.3 s to 30 s (negative = cooled down):");
    for (int s = 0; s < N_TC; s++) {
        e.first[s] = toCent(t[0][s]);
        e.last[s]  = toCent(t[NM - 1][s]);
        if (isnan(t[0][s]) || isnan(t[NM - 1][s])) { Serial.printf("  TC%d: no reading\n", s + 1); continue; }
        float drop = t[0][s] - t[NM - 1][s];
        Serial.printf("  TC%d: %+.2f °C%s\n", s + 1, -drop,
                      drop > DROP_LIMIT_C ? "  <-- cooled after wake: chip was WARM" : "");
    }
    saveHistory(e);
    printHistory();

    bool hold = SLEEP_MODE == 2 ? (cycle % 2 == 0) : SLEEP_MODE == 1;
    heldDuringSleep = hold;
    if (hold) holdSensorPins(true);
    boardPinsForSleep(true);
    gpio_deep_sleep_hold_en();

    Serial.printf("\nSleeping %d s (sensor pins %s)\n", SLEEP_S, hold ? "held" : "floating");
    Serial.flush();
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_S * 1000000ULL);
    esp_deep_sleep_start();
}

void loop() {}
