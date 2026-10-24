// ======================================================
// MAX6675 + ESP32 deep sleep — correct pin handling
// ======================================================
//
// Minimal pattern: read the MAX6675 modules, then deep sleep with every
// CS held HIGH and SCK held LOW, so no module is selected while asleep.
// The MAX6675 stays powered during sleep (as in most simple builds).
//
// Timing after wake: the MAX6675 needs up to 220 ms per conversion and
// every read restarts the conversion, so wait 300 ms, discard one read
// and keep >= 250 ms between reads of the same module.
// ======================================================

#include "max6675.h"          // Adafruit MAX6675 library
#include <esp_sleep.h>
#include <driver/gpio.h>

#define SLEEP_S   600
#define PIN_SCK   18
#define PIN_SO    19
const int CS_PINS[] = {5, 32, 33};
#define N_TC  (int)(sizeof(CS_PINS) / sizeof(CS_PINS[0]))

MAX6675 tc1(PIN_SCK, CS_PINS[0], PIN_SO);
MAX6675 tc2(PIN_SCK, CS_PINS[1], PIN_SO);
MAX6675 tc3(PIN_SCK, CS_PINS[2], PIN_SO);
MAX6675* tcs[N_TC] = {&tc1, &tc2, &tc3};

// Drive CS HIGH / SCK LOW. Called before releasing the hold on wake
// (so the pin goes straight from the held level to the same level)
// and before sleeping.
void sensorPinsIdle() {
    for (int s = 0; s < N_TC; s++) { pinMode(CS_PINS[s], OUTPUT); digitalWrite(CS_PINS[s], HIGH); }
    pinMode(PIN_SCK, OUTPUT); digitalWrite(PIN_SCK, LOW);
    pinMode(PIN_SO, INPUT);
}

void holdSensorPins(bool hold) {
    for (int s = 0; s < N_TC; s++) {
        if (hold) gpio_hold_en((gpio_num_t)CS_PINS[s]);
        else      gpio_hold_dis((gpio_num_t)CS_PINS[s]);
    }
    if (hold) gpio_hold_en((gpio_num_t)PIN_SCK);
    else      gpio_hold_dis((gpio_num_t)PIN_SCK);
}

float median(float* v, int n) {
    for (int i = 1; i < n; i++) { float x = v[i]; int j = i - 1; while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; } v[j + 1] = x; }
    return (n % 2) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2.0f;
}

void setup() {
    Serial.begin(115200);

    sensorPinsIdle();             // 1) same levels as during sleep
    holdSensorPins(false);        // 2) then release the hold
    gpio_deep_sleep_hold_dis();

    delay(300);                                          // conversion done (max 220 ms)
    for (int s = 0; s < N_TC; s++) tcs[s]->readCelsius(); // discard first read
    delay(300);

    const int N = 5;
    for (int s = 0; s < N_TC; s++) {
        float v[N]; int n = 0;
        for (int i = 0; i < N; i++) {
            float t = tcs[s]->readCelsius();
            if (!isnan(t) && t > 0.0f) v[n++] = t;
            delay(300);
        }
        if (n) Serial.printf("TC%d: %.2f °C\n", s + 1, median(v, n));
        else   Serial.printf("TC%d: no reading\n", s + 1);
    }

    sensorPinsIdle();             // no module selected while asleep
    holdSensorPins(true);
    gpio_deep_sleep_hold_en();    // needed for digital (non-RTC) pads such as 5 and 18

    Serial.flush();
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_S * 1000000ULL);
    esp_deep_sleep_start();
}

void loop() {}
