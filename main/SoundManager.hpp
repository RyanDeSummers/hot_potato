#pragma once
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

extern "C" {
    #include "esp_timer.h"
    #include "driver/dac_oneshot.h"
}

class SoundManager {
public:
    bool init(int dac_gpio = 25);                 // GPIO25 (DAC1) on M5Stack FIRE
    void beep(uint32_t freq_hz, uint32_t ms);     // enqueue a simple beep
    void death_jingle();                           // enqueue a short sequence
    void shutdown();                               // optional cleanup

private:
    // command queue
    enum class CmdType { Beep, DeathJingle, Stop };
    struct Cmd { CmdType type; uint32_t freq; uint32_t ms; };

    static void taskEntry(void* arg);
    void taskLoop();
    void playBeepBlocking(uint32_t freq_hz, uint32_t ms);
    void playDeathJingleBlocking();

    // timer callback that toggles DAC value (square wave)
    static void IRAM_ATTR timerCb(void* arg);

    // helpers
    bool startTone(uint32_t freq_hz);
    void stopTone();

private:
    dac_oneshot_handle_t dac_ = nullptr;
    esp_timer_handle_t   timer_ = nullptr;
    volatile uint8_t     level_ = 0;        // toggled by timer ISR
    QueueHandle_t        q_ = nullptr;
    TaskHandle_t         task_ = nullptr;
};
