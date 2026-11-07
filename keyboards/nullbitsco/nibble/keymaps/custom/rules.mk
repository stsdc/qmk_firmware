OLED_ENABLE = yes
OLED_DRIVER = ssd1306
WPM_ENABLE = yes

ifeq ($(strip $(OLED_ENABLE)), yes)
    SRC += oled_display.c
endif