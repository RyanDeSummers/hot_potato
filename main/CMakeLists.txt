idf_component_register(
    SRCS
        "main.cpp"
        "SoundManager.cpp"
        "game_logic.cpp"        # <-- use .cpp here
        "ir_simple_test.cpp"
        "ir_uplink_test.cpp"

    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        M5GFX
        espressif__led_strip
        esp_timer
        esp_driver_dac
        freertos
        driver
        esp_wifi
    )












