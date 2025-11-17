idf_component_register(
    SRCS "esp_lcd_touch_xpt2046.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES "driver" "esp_lcd"
    REQUIRES "esp_lcd_touch"
)
