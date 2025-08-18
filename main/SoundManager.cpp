#include "SoundManager.hpp"
#include <cstdio>

bool SoundManager::init(int /*dac_gpio*/)
{
    // Create DAC oneshot channel on DAC1 (GPIO25) for M5Stack FIRE
    // Note: channel is selected by enum, not raw GPIO
    dac_oneshot_config_t cfg = {};
    cfg.chan_id = DAC_CHAN_0; // DAC1 = GPIO25

    if (dac_oneshot_new_channel(&cfg, &dac_) != ESP_OK || !dac_) {
        printf("[Sound] dac_oneshot_new_channel failed\n");
        return false;
    }

    // Command queue + worker task
    q_ = xQueueCreate(8, sizeof(Cmd));
    if (!q_) return false;

    xTaskCreatePinnedToCore(taskEntry, "snd_task", 4096, this, 4, &task_, tskNO_AFFINITY);
    return task_ != nullptr;
}

void SoundManager::shutdown()
{
    if (q_) {
        Cmd c{CmdType::Stop, 0, 0};
        xQueueSend(q_, &c, 0);
    }
}


void SoundManager::beep(uint32_t freq_hz, uint32_t ms)
{
    if (!q_) return;
    Cmd c{CmdType::Beep, freq_hz, ms};
    xQueueSend(q_, &c, 0);
}


void SoundManager::death_jingle()
{
    if (!q_) return;
    Cmd c{CmdType::DeathJingle, 0, 0};
    xQueueSend(q_, &c, 0);
}

void SoundManager::taskEntry(void* arg)
{
    static_cast<SoundManager*>(arg)->taskLoop();
}

void SoundManager::taskLoop()
{
    Cmd cmd{};
    while (xQueueReceive(q_, &cmd, portMAX_DELAY) == pdTRUE) {
        switch (cmd.type) {
            case CmdType::Beep:
                playBeepBlocking(cmd.freq, cmd.ms);
                break;
            case CmdType::DeathJingle:
                playDeathJingleBlocking();
                break;
            case CmdType::Stop:
                stopTone();
                vTaskDelete(nullptr);
                return;
        }
    }
}

// ---- Tone generation via esp_timer toggling DAC (square wave) ----

bool SoundManager::startTone(uint32_t freq_hz)
{
    stopTone(); // ensure clean state

    if (freq_hz == 0 || !dac_) return false;

    // Create periodic timer at half-period to toggle the level
    const int64_t half_period_us = static_cast<int64_t>(500000) / (freq_hz); // 1e6/2 = 500000
    esp_timer_create_args_t args = {};
    args.callback = &SoundManager::timerCb;
    args.arg = this;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "snd_tone";

    if (esp_timer_create(&args, &timer_) != ESP_OK) {
        printf("[Sound] esp_timer_create failed\n");
        timer_ = nullptr;
        return false;
    }

    level_ = 0;
    // Start toggling
    esp_err_t err = esp_timer_start_periodic(timer_, half_period_us);
    if (err != ESP_OK) {
        printf("[Sound] esp_timer_start_periodic failed\n");
        esp_timer_delete(timer_);
        timer_ = nullptr;
        return false;
    }
    return true;
}

void SoundManager::stopTone()
{
    if (timer_) {
        esp_timer_stop(timer_);
        esp_timer_delete(timer_);
        timer_ = nullptr;
    }
    if (dac_) {
        // Drive to 0 to avoid lingering DC level (click/pop reduction)
        dac_oneshot_output_voltage(dac_, 0);
    }
}

void IRAM_ATTR SoundManager::timerCb(void* arg)
{
    auto* self = static_cast<SoundManager*>(arg);
    // Toggle between low and high output (simple square wave)
    uint8_t next = self->level_ ? 0 : 255;
    self->level_ = next;
    // Keep ISR short: just write the sample
    dac_oneshot_output_voltage(self->dac_, next);
}

// ---- Blocking sequences (executed in sound task) ----

void SoundManager::playBeepBlocking(uint32_t freq_hz, uint32_t ms)
{
    if (!startTone(freq_hz)) return;
    vTaskDelay(pdMS_TO_TICKS(ms));
    stopTone();
}

void SoundManager::playDeathJingleBlocking()
{
    // Simple descending tri-tone (tweak to taste)
    const struct { uint32_t f; uint32_t d; } seq[] = {
        { 1200, 160 }, { 900, 160 }, { 600, 240 },
    };
    for (auto &s : seq) {
        playBeepBlocking(s.f, s.d);
        vTaskDelay(pdMS_TO_TICKS(40)); // tiny gap
    }
}
