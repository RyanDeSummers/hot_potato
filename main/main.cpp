extern "C" {
    #include "led_strip.h"
    }
    #include <stdio.h>
    #include "freertos/FreeRTOS.h"
    #include "ir_simple_test.h"     // Mode A: holder sends IR beacons
#include "ir_uplink_test.h"     // Mode B: holder receives IR uplinks (new)
#include "ir_downlink_test.h"   // Mode A condensed: holder TX / runners RX
    #include "freertos/task.h"
    #include "game_logic.h"
    #include "M5GFX.h"
    #include "SoundManager.hpp"
    
    PlayerState myState;
    M5GFX display;
    led_strip_handle_t led_strip = NULL;
    SoundManager speaker;
    
    static void pulse_red_led()
    {
        if (!led_strip) return;
        for (int b = 0; b <= 255; b += 25) { led_strip_set_pixel(led_strip, 0, b, 0, 0); led_strip_refresh(led_strip); vTaskDelay(pdMS_TO_TICKS(20)); }
        vTaskDelay(pdMS_TO_TICKS(100));
        for (int b = 255; b >= 0; b -= 25) { led_strip_set_pixel(led_strip, 0, b, 0, 0); led_strip_refresh(led_strip); vTaskDelay(pdMS_TO_TICKS(20)); }
        led_strip_clear(led_strip);
        led_strip_refresh(led_strip);
    }
    
    extern "C" void app_main(void)
    {
            printf("Starting Hot Potato Game\n");

    // TEMP: choose test variant
#if defined(HP_TEST_UPLINK) && (HP_TEST_UPLINK==1)
    printf("Running Mode B: IR Uplink Test (holder RX / runners IR TX)\n");
    ir_uplink_test_main();
#else
    printf("Running Mode A: IR Downlink Test (holder TX / runners IR RX)\n");
    ir_downlink_test_main();
#endif
    
    // Original switch (currently disabled):
    // #if defined(HP_TEST_UPLINK) && (HP_TEST_UPLINK==1)
    //     printf("Running Mode B: IR Uplink Test (holder RX / runners IR TX)\n");
    //     ir_uplink_test_main();
    // #else
    //     printf("Running Mode A: IR Simple Test (holder TX / runners RX)\n");
    //     ir_simple_test_main();
    // #endif
        return;
    
        game_init(&myState);
    
        printf("Initializing display...\n");
        
        // Try to detect the board type
        auto board = display.getBoard();
        printf("Detected board type: %d\n", (int)board);
        
        if (!display.begin()) {
            printf("ERROR: Display initialization failed!\n");
            // Continue without display
        } else {
            printf("Display begin completed\n");
            display.setRotation(1);
            display.setTextSize(2);
            display.setTextColor(TFT_WHITE, TFT_BLACK);
            printf("Display setup completed\n");
            
            // Test display with a simple message
            display.clear();
            display.setCursor(10, 10);
            display.println("Hot Potato Starting...");
            printf("Initial display test completed\n");
        }
    
        // LED init (GPIO15, 1 LED)
        led_strip_config_t strip_config = {}; strip_config.strip_gpio_num = 15; strip_config.max_leds = 1;
        led_strip_rmt_config_t rmt_config = {}; rmt_config.resolution_hz = 10 * 1000 * 1000;
        if (led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip) != ESP_OK) { printf("LED strip init failed\n"); led_strip = NULL; }
    
        // Sound init (GPIO25 DAC)
        if (!speaker.init(25)) { printf("Sound init failed\n"); }
    
        // Start with a potato
        uint8_t fake_sender_mac[6] = {1,2,3,4,5,6};
        give_potato(&myState, fake_sender_mac);
    
        while (1) {
            int prev_health = myState.health;
    
            game_tick(&myState);
    
            // On health tick: pulse LED and beep
            if (myState.health < prev_health && myState.health > 0) {
                pulse_red_led();
                speaker.beep(600, 40);
            }
    
            // On elimination: LED off + jingle
            if (is_eliminated(&myState)) {
                if (led_strip) { led_strip_clear(led_strip); led_strip_refresh(led_strip); }
                speaker.death_jingle();
            }
    
            // LCD
            printf("Updating display...\n");
            if (display.getPanel()) {  // Check if display is initialized
                display.clear();
                display.setCursor(10, 20); display.printf("Health: %d", myState.health);
                display.setCursor(10, 50);
                if (myState.has_potato) { display.setTextColor(TFT_RED, TFT_BLACK); display.println("You have the potato!"); display.setTextColor(TFT_WHITE, TFT_BLACK); }
                else { display.println("No potato"); }
                display.setCursor(10, 80);
                if (!myState.alive) { display.setTextColor(TFT_RED, TFT_BLACK); display.println("ELIMINATED"); display.setTextColor(TFT_WHITE, TFT_BLACK); }
            }
            display.setTextSize(2);
    
            if (is_eliminated(&myState)) { printf("Game Over - Player Eliminated\n"); break; }
    
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    
        speaker.shutdown();
    }