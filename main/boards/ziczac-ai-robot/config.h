#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// ===================== AUDIO (Micro I2S & Loa I2S) =====================
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_I2S_METHOD_SIMPLEX

// Micro I2S
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4    // WS  -> GPIO 4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5    // SCK -> GPIO 5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6    // SD  -> GPIO 6

// Loa I2S
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16   // LRC  -> GPIO 16
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15   // BCLK -> GPIO 15
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7    // DIN  -> GPIO 7

// ===================== NÚT BẤM / LED TRẠNG THÁI =====================
#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define BOOT_BUTTON_GPIO        GPIO_NUM_47
#define TOUCH_BUTTON_GPIO       GPIO_NUM_NC

// ===================== MÀN HÌNH TFT 2.0" ST7789 SPI =====================
#define DISPLAY_MOSI_PIN      GPIO_NUM_10   // SDA -> GPIO 10
#define DISPLAY_CLK_PIN       GPIO_NUM_9    // SCL -> GPIO 9
#define DISPLAY_DC_PIN        GPIO_NUM_8    // DC  -> GPIO 8
#define DISPLAY_RST_PIN       GPIO_NUM_18   // RST -> GPIO 18
#define DISPLAY_CS_PIN        GPIO_NUM_NC   // CS nối thẳng GND

// Đèn nền: nối chân BLK vào 3V3 (KHÔNG để hở).
// Nếu muốn chỉnh độ sáng bằng phần mềm: nối BLK vào GPIO 14 rồi đổi thành GPIO_NUM_14.
#define DISPLAY_BACKLIGHT_PIN           GPIO_NUM_NC
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

#define LCD_TYPE_ST7789

// Nằm ngang 320x240 (kiểu mắt robot xiaozhi)
#define DISPLAY_WIDTH     320
#define DISPLAY_HEIGHT    240
#define DISPLAY_SWAP_XY   true
#define DISPLAY_MIRROR_X  true
#define DISPLAY_MIRROR_Y  false
#define DISPLAY_INVERT_COLOR  true
#define DISPLAY_RGB_ORDER     LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0

// Module không có CS (CS nối GND) thường chỉ chạy đúng ở SPI mode 3.
// Nếu vẫn không lên hình thì thử lại mode 0.
#define DISPLAY_SPI_MODE      3
#define DISPLAY_SPI_CLOCK_HZ  (10 * 1000 * 1000)

// 1 = lúc khởi động tô lần lượt đen/trắng/đỏ/xanh lá/xanh dương để kiểm tra đường truyền
// Khi màn hình đã chạy tốt thì đổi thành 0.
#define DISPLAY_TEST_PATTERN  0

// ===================== MÀN HÌNH OLED I2C SSD1306 128x32 =====================
// Hiện: tên Wi-Fi, giờ, ngày, trạng thái (đang nghe / đang nói...)
#define OLED_SDA_GPIO         GPIO_NUM_1
#define OLED_SCL_GPIO         GPIO_NUM_2
#define OLED_WIDTH            128
#define OLED_HEIGHT           32
#define OLED_I2C_ADDR         0x3C          // một số module dùng 0x3D
#define OLED_MIRROR_X         false         // nếu OLED bị ngược thì đổi cả 2 thành true
#define OLED_MIRROR_Y         false

// ===================== KHÔNG SỬ DỤNG =====================
#define ULTRASONIC_TRIG_GPIO GPIO_NUM_NC
#define ULTRASONIC_ECHO_GPIO GPIO_NUM_NC
#define EYE_LED_LEFT_GPIO    GPIO_NUM_NC
#define EYE_LED_RIGHT_GPIO   GPIO_NUM_NC
#define MOTOR_L_IN1          GPIO_NUM_NC
#define MOTOR_L_IN2          GPIO_NUM_NC
#define MOTOR_R_IN3          GPIO_NUM_NC
#define MOTOR_R_IN4          GPIO_NUM_NC
#define SERVO_NECK_GPIO      GPIO_NUM_NC
#define SERVO_ARM_LEFT_GPIO  GPIO_NUM_NC
#define SERVO_ARM_RIGHT_GPIO GPIO_NUM_NC

#endif // _BOARD_CONFIG_H_