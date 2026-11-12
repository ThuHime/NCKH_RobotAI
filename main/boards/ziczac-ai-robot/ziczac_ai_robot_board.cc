#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "assets/lang_config.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/single_led.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <esp_rom_sys.h>
#include <esp_wifi.h>
#include <esp_lvgl_port.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_ssd1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#define TAG "ZiczacAiRobot"

// Font nhỏ cho OLED 128x32 (2 dòng, mỗi dòng 16px). Chọn font nhỏ nhất có sẵn trong LVGL.
// Các font này chỉ có ký tự ASCII (không có dấu tiếng Việt).
static const lv_font_t* PickOledFont() {
#if LV_FONT_MONTSERRAT_12
    return &lv_font_montserrat_12;
#elif LV_FONT_UNSCII_8
    return &lv_font_unscii_8;
#else
    return LV_FONT_DEFAULT;
#endif
}

static bool IsAscii(const char* s) {
    for (; *s; s++) {
        if ((unsigned char)*s >= 0x80) return false;
    }
    return true;
}

static inline uint64_t safe_gpio_mask(gpio_num_t pin) {
    return (pin >= 0) ? (1ULL << pin) : 0ULL;
}

// =====================================================================
// LED MẮT
// =====================================================================
class EyeLeds {
public:
    enum Mode { kOff, kOn, kBlinkSlow, kBlinkFast, kBreathe };

    EyeLeds() {
        if (EYE_LED_LEFT_GPIO < 0 || EYE_LED_RIGHT_GPIO < 0) return;

        ledc_timer_config_t t = {};
        t.speed_mode = LEDC_LOW_SPEED_MODE;
        t.duty_resolution = LEDC_TIMER_10_BIT;
        t.timer_num = LEDC_TIMER_2;
        t.freq_hz = 5000;
        t.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));

        const gpio_num_t pins[2] = {EYE_LED_LEFT_GPIO, EYE_LED_RIGHT_GPIO};
        for (int i = 0; i < 2; i++) {
            ledc_channel_config_t c = {};
            c.gpio_num = pins[i];
            c.speed_mode = LEDC_LOW_SPEED_MODE;
            c.channel = (ledc_channel_t)(LEDC_CHANNEL_3 + i);
            c.timer_sel = LEDC_TIMER_2;
            c.duty = 0;
            c.hpoint = 0;
            ESP_ERROR_CHECK(ledc_channel_config(&c));
        }

        esp_timer_create_args_t args = {};
        args.callback = &EyeLeds::TimerCallback;
        args.arg = this;
        args.name = "eye_led";
        ESP_ERROR_CHECK(esp_timer_create(&args, &timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(timer_, 20000));
        enabled_ = true;
    }

    void SetMode(int side, Mode mode) {
        if (!enabled_) return;
        if (side == 0 || side == 2) mode_[0] = mode;
        if (side == 1 || side == 2) mode_[1] = mode;
    }

private:
    esp_timer_handle_t timer_ = nullptr;
    volatile Mode mode_[2] = {kOff, kOff};
    uint32_t tick_ = 0;
    bool enabled_ = false;

    static void TimerCallback(void* arg) {
        static_cast<EyeLeds*>(arg)->Tick();
    }

    void Tick() {
        tick_++;
        for (int i = 0; i < 2; i++) {
            uint32_t duty = 0;
            switch (mode_[i]) {
                case kOff: duty = 0; break;
                case kOn: duty = 1023; break;
                case kBlinkSlow: duty = ((tick_ / 50) % 2 == 0) ? 1023 : 0; break;
                case kBlinkFast: duty = ((tick_ / 10) % 2 == 0) ? 1023 : 0; break;
                case kBreathe: {
                    uint32_t phase = tick_ % 100;
                    uint32_t level = phase < 50 ? phase : 100 - phase;
                    duty = level * 1023 / 50;
                    break;
                }
            }
            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(LEDC_CHANNEL_3 + i), duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(LEDC_CHANNEL_3 + i));
        }
    }
};

// =====================================================================
// SERVO
// =====================================================================
class Servos {
public:
    enum Part { kNeck = 0, kArmLeft = 1, kArmRight = 2 };

    Servos() {
        if (SERVO_NECK_GPIO < 0) return;

        ledc_timer_config_t t = {};
        t.speed_mode = LEDC_LOW_SPEED_MODE;
        t.duty_resolution = LEDC_TIMER_14_BIT;
        t.timer_num = LEDC_TIMER_3;
        t.freq_hz = 50;
        t.clk_cfg = LEDC_AUTO_CLK;
        ESP_ERROR_CHECK(ledc_timer_config(&t));

        const gpio_num_t pins[3] = {SERVO_NECK_GPIO, SERVO_ARM_LEFT_GPIO, SERVO_ARM_RIGHT_GPIO};
        for (int i = 0; i < 3; i++) {
            if (pins[i] < 0) continue;
            ledc_channel_config_t c = {};
            c.gpio_num = pins[i];
            c.speed_mode = LEDC_LOW_SPEED_MODE;
            c.channel = (ledc_channel_t)(LEDC_CHANNEL_5 + i);
            c.timer_sel = LEDC_TIMER_3;
            c.duty = 0;
            c.hpoint = 0;
            ESP_ERROR_CHECK(ledc_channel_config(&c));
        }
    }

    void SetAngle(Part part, int angle) {
        const gpio_num_t pins[3] = {SERVO_NECK_GPIO, SERVO_ARM_LEFT_GPIO, SERVO_ARM_RIGHT_GPIO};
        if (pins[(int)part] < 0) return;

        angle = std::clamp(angle, 0, 180);
        uint32_t pulse_us = 500 + angle * 2000 / 180;
        uint32_t duty = pulse_us * 16383 / 20000;
        auto ch = (ledc_channel_t)(LEDC_CHANNEL_5 + (int)part);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    }
};

// =====================================================================
// MOTOR L298N
// =====================================================================
class Motors {
public:
    Motors() {
        if (MOTOR_L_IN1 < 0 || MOTOR_L_IN2 < 0 || MOTOR_R_IN3 < 0 || MOTOR_R_IN4 < 0) return;

        gpio_config_t io = {};
        io.pin_bit_mask = safe_gpio_mask(MOTOR_L_IN1) | safe_gpio_mask(MOTOR_L_IN2) |
                          safe_gpio_mask(MOTOR_R_IN3) | safe_gpio_mask(MOTOR_R_IN4);
        io.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&io));
        enabled_ = true;
        Stop();
    }

    void Stop() {
        if (!enabled_) return;
        gpio_set_level(MOTOR_L_IN1, 0);
        gpio_set_level(MOTOR_L_IN2, 0);
        gpio_set_level(MOTOR_R_IN3, 0);
        gpio_set_level(MOTOR_R_IN4, 0);
    }

    void Move(const std::string& dir, int duration_ms) {
        if (!enabled_) return;
        // TODO: điều khiển motor theo dir
    }

private:
    bool enabled_ = false;
};

// =====================================================================
// CẢM BIẾN SIÊU ÂM HC-SR04
// =====================================================================
class Ultrasonic {
public:
    Ultrasonic() {
        if (ULTRASONIC_TRIG_GPIO < 0 || ULTRASONIC_ECHO_GPIO < 0) return;

        gpio_config_t out = {};
        out.pin_bit_mask = safe_gpio_mask(ULTRASONIC_TRIG_GPIO);
        out.mode = GPIO_MODE_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&out));

        gpio_config_t in = {};
        in.pin_bit_mask = safe_gpio_mask(ULTRASONIC_ECHO_GPIO);
        in.mode = GPIO_MODE_INPUT;
        ESP_ERROR_CHECK(gpio_config(&in));
        gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);
        enabled_ = true;
    }

    int MeasureCm() {
        if (!enabled_) return -1;
        gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);
        esp_rom_delay_us(2);
        gpio_set_level(ULTRASONIC_TRIG_GPIO, 1);
        esp_rom_delay_us(10);
        gpio_set_level(ULTRASONIC_TRIG_GPIO, 0);

        int64_t t0 = esp_timer_get_time();
        while (gpio_get_level(ULTRASONIC_ECHO_GPIO) == 0) {
            if (esp_timer_get_time() - t0 > 30000) return -1;
        }
        int64_t start = esp_timer_get_time();
        while (gpio_get_level(ULTRASONIC_ECHO_GPIO) == 1) {
            if (esp_timer_get_time() - start > 30000) return -1;
        }
        int64_t width = esp_timer_get_time() - start;
        return (int)(width / 58);
    }

private:
    bool enabled_ = false;
};

// =====================================================================
// MẶT ROBOT TRÊN TFT
//   - Nền đen, 2 mắt tròn to màu trắng, chớp mắt ngẫu nhiên như người
//   - Khi LẮNG NGHE: hiện thêm miệng (thanh nhỏ)
//   - Khi ĐANG NÓI: miệng mở/đóng theo nhịp
// Toàn bộ vẽ bằng LVGL, không cần ảnh/emoji nào.
// Mọi hàm tạo (constructor) phải được gọi khi đang giữ khóa LVGL.
// =====================================================================
class RobotFace {
public:
    enum Mode { kIdle, kListening, kSpeaking };

    explicit RobotFace(lv_obj_t* screen) {
        lv_display_t* d = lv_obj_get_display(screen);
        const int W = lv_display_get_horizontal_resolution(d);
        const int H = lv_display_get_vertical_resolution(d);

        eye_d_ = std::min(H * 42 / 100, W * 40 / 100);   // đường kính mắt
        const int gap = eye_d_ * 35 / 100;               // khoảng cách giữa 2 mắt
        eye_cy_ = H * 38 / 100;                          // tâm dọc của mắt
        mouth_cy_ = H * 78 / 100;                        // tâm dọc của miệng
        mouth_w_ = eye_d_ * 90 / 100;
        mouth_x_ = (W - mouth_w_) / 2;

        root_ = lv_obj_create(screen);
        lv_obj_remove_style_all(root_);
        lv_obj_set_size(root_, W, H);
        lv_obj_set_pos(root_, 0, 0);
        lv_obj_set_style_bg_color(root_, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(root_, LV_OPA_COVER, 0);
        lv_obj_remove_flag(root_, LV_OBJ_FLAG_SCROLLABLE);

        eye_l_ = MakePill(root_);
        eye_r_ = MakePill(root_);
        lv_obj_set_width(eye_l_, eye_d_);
        lv_obj_set_width(eye_r_, eye_d_);
        lv_obj_set_x(eye_l_, W / 2 - gap / 2 - eye_d_);
        lv_obj_set_x(eye_r_, W / 2 + gap / 2);
        SetEyeHeight(eye_d_);

        mouth_ = MakePill(root_);
        lv_obj_set_width(mouth_, mouth_w_);
        lv_obj_set_x(mouth_, mouth_x_);
        lv_obj_add_flag(mouth_, LV_OBJ_FLAG_HIDDEN);

        next_blink_ms_ = NowMs() + 1500;
        lv_timer_create(&RobotFace::TimerCb, 33, this);
    }

    void SetMode(Mode mode) {
        if (!lvgl_port_lock(0)) return;
        if (mode != mode_) {
            mode_ = mode;
            last_mouth_h_ = -1;
            if (mode_ == kIdle) {
                lv_obj_add_flag(mouth_, LV_OBJ_FLAG_HIDDEN);
            } else {
                SetMouthHeight(kMouthMinH);
                lv_obj_remove_flag(mouth_, LV_OBJ_FLAG_HIDDEN);
            }
        }
        lvgl_port_unlock();
    }

private:
    static constexpr int kMouthMinH = 10;

    lv_obj_t* root_ = nullptr;
    lv_obj_t* eye_l_ = nullptr;
    lv_obj_t* eye_r_ = nullptr;
    lv_obj_t* mouth_ = nullptr;

    int eye_d_ = 100;
    int eye_cy_ = 90;
    int mouth_cy_ = 190;
    int mouth_w_ = 90;
    int mouth_x_ = 0;

    Mode mode_ = kIdle;
    bool blinking_ = false;
    uint32_t blink_start_ms_ = 0;
    uint32_t next_blink_ms_ = 0;
    int last_eye_h_ = -1;
    int last_mouth_h_ = -1;

    static uint32_t NowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

    static lv_obj_t* MakePill(lv_obj_t* parent) {
        lv_obj_t* o = lv_obj_create(parent);
        lv_obj_remove_style_all(o);
        lv_obj_set_style_bg_color(o, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
        lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
        return o;
    }

    void SetEyeHeight(int h) {
        if (h == last_eye_h_) return;
        last_eye_h_ = h;
        int y = eye_cy_ - h / 2;
        lv_obj_set_height(eye_l_, h);
        lv_obj_set_height(eye_r_, h);
        lv_obj_set_y(eye_l_, y);
        lv_obj_set_y(eye_r_, y);
    }

    void SetMouthHeight(int h) {
        if (h == last_mouth_h_) return;
        last_mouth_h_ = h;
        lv_obj_set_height(mouth_, h);
        lv_obj_set_y(mouth_, mouth_cy_ - h / 2);
    }

    static void TimerCb(lv_timer_t* t) {
        static_cast<RobotFace*>(lv_timer_get_user_data(t))->Tick();
    }

    void Tick() {
        const uint32_t now = NowMs();

        // ---- Chớp mắt: nhắm 70ms, giữ 40ms, mở 90ms; đôi khi chớp đúp ----
        int eye_h = eye_d_;
        const int min_h = std::max(6, eye_d_ / 14);
        if (blinking_) {
            const uint32_t kClose = 70, kHold = 40, kOpen = 90;
            uint32_t e = now - blink_start_ms_;
            if (e < kClose) {
                eye_h = eye_d_ - (eye_d_ - min_h) * (int)e / (int)kClose;
            } else if (e < kClose + kHold) {
                eye_h = min_h;
            } else if (e < kClose + kHold + kOpen) {
                eye_h = min_h + (eye_d_ - min_h) * (int)(e - kClose - kHold) / (int)kOpen;
            } else {
                blinking_ = false;
                bool double_blink = (esp_random() % 5 == 0);
                next_blink_ms_ = now + (double_blink ? 260 : 2200 + esp_random() % 3500);
            }
        } else if ((int32_t)(now - next_blink_ms_) >= 0) {
            blinking_ = true;
            blink_start_ms_ = now;
        }
        SetEyeHeight(eye_h);

        // ---- Miệng ----
        if (mode_ == kListening) {
            SetMouthHeight(kMouthMinH);
        } else if (mode_ == kSpeaking) {
            int p = (int)((now / 4) % 120);          // chu kỳ ~480ms
            int tri = p < 60 ? p : 120 - p;          // 0..60
            int h = kMouthMinH + tri * 32 / 60 + (int)((now / 97) % 3) * 4;
            SetMouthHeight(h);
        }
    }
};

// =====================================================================
// OLED 128x32 (chỉ đủ 2 dòng)
//   Bình thường:   dòng 1 = tên Wi-Fi
//                  dòng 2 = thứ + ngày/tháng + giờ   (vd: "T4 04/11 18:05")
//   Lắng nghe / đang nói:
//                  dòng 1 = "Đang nghe..." / "Đang nói..."
//                  dòng 2 = sóng âm nhấp nhô
// Không dùng lớp OledDisplay (vì nó gọi lvgl_port_init lần 2).
// Chỉ thêm 1 màn hình nữa vào LVGL đã được TFT khởi tạo sẵn.
// =====================================================================
class OledStatusPanel {
public:
    OledStatusPanel(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                    int width, int height, bool mirror_x, bool mirror_y)
        : width_(width), height_(height) {
        const lvgl_port_display_cfg_t display_cfg = {
            .io_handle = panel_io,
            .panel_handle = panel,
            .control_handle = nullptr,
            .buffer_size = static_cast<uint32_t>(width * height),
            .double_buffer = false,
            .trans_size = 0,
            .hres = static_cast<uint32_t>(width),
            .vres = static_cast<uint32_t>(height),
            .monochrome = true,
            .rotation =
                {
                    .swap_xy = false,
                    .mirror_x = mirror_x,
                    .mirror_y = mirror_y,
                },
            .flags =
                {
                    .buff_dma = 1,
                    .buff_spiram = 0,
                    .sw_rotate = 0,
                    .full_refresh = 0,
                    .direct_mode = 0,
                },
        };
        disp_ = lvgl_port_add_disp(&display_cfg);
        if (disp_ == nullptr) {
            ESP_LOGE(TAG, "Không thêm được màn hình OLED vào LVGL");
            return;
        }
        BuildUi();
    }

    void SetStatus(const char* status) {
        if (disp_ == nullptr || status == nullptr) return;
        if (!lvgl_port_lock(0)) return;
        status_ = status;
        if (strcmp(status, Lang::Strings::LISTENING) == 0) {
            mode_ = kListening;
        } else if (strcmp(status, Lang::Strings::SPEAKING) == 0) {
            mode_ = kSpeaking;
        } else if (strcmp(status, Lang::Strings::STANDBY) == 0) {
            mode_ = kIdle;
        } else {
            mode_ = kOther;
        }
        Refresh();
        lvgl_port_unlock();
    }

    void ShowNotification(const char* text, int duration_ms) {
        if (disp_ == nullptr || text == nullptr) return;
        if (!IsAscii(text)) return;  // font nhỏ không có dấu tiếng Việt
        if (duration_ms <= 0) duration_ms = 3000;
        if (!lvgl_port_lock(0)) return;
        notif_ = text;
        notif_active_ = true;
        notif_expire_us_ = esp_timer_get_time() + (int64_t)duration_ms * 1000;
        Refresh();
        lvgl_port_unlock();
    }

private:
    enum Mode { kIdle, kOther, kListening, kSpeaking };
    static constexpr int kBars = 16;

    int width_ = 128;
    int height_ = 32;
    int line_h_ = 14;
    int row_h_ = 14;
    lv_display_t* disp_ = nullptr;

    lv_obj_t* line1_label_ = nullptr;
    lv_obj_t* date_label_ = nullptr;
    lv_obj_t* bars_[kBars] = {};
    int bar_h_[kBars] = {};

    Mode mode_ = kOther;
    std::string status_;
    std::string notif_;
    std::string line1_text_;
    std::string ssid_text_ = "Wi-Fi...";
    std::string date_text_;
    bool notif_active_ = false;
    int64_t notif_expire_us_ = 0;
    uint32_t tick_ = 0;

    void BuildUi() {
        if (!lvgl_port_lock(0)) return;

        const lv_font_t* font = PickOledFont();
        // Mỗi dòng cao đúng 1/2 màn hình (16px với màn 32px), chữ canh giữa trong dòng
        row_h_ = height_ / 2;
        const int font_h = std::min((int)lv_font_get_line_height(font), row_h_);
        const int text_off = (row_h_ - font_h) / 2;
        line_h_ = row_h_;  // y bắt đầu của dòng 2

        lv_obj_t* scr = lv_display_get_screen_active(disp_);
        lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
        lv_obj_set_style_text_color(scr, lv_color_black(), 0);
        lv_obj_set_style_text_font(scr, font, 0);
        lv_obj_set_style_pad_all(scr, 0, 0);
        lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);

        // Dòng 1
        line1_label_ = lv_label_create(scr);
        lv_obj_set_size(line1_label_, width_ - 2, font_h);
        lv_label_set_long_mode(line1_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_label_set_text(line1_label_, ssid_text_.c_str());
        line1_text_ = ssid_text_;
        lv_obj_align(line1_label_, LV_ALIGN_TOP_LEFT, 1, text_off);

        // Dòng 2: ngày giờ
        date_label_ = lv_label_create(scr);
        lv_obj_set_size(date_label_, width_ - 2, font_h);
        lv_label_set_long_mode(date_label_, LV_LABEL_LONG_CLIP);
        lv_label_set_text(date_label_, "");
        lv_obj_align(date_label_, LV_ALIGN_TOP_LEFT, 1, line_h_ + text_off);

        // Dòng 2: sóng âm (ẩn sẵn)
        for (int i = 0; i < kBars; i++) {
            bar_h_[i] = 2;
            lv_obj_t* b = lv_obj_create(scr);
            lv_obj_remove_style_all(b);
            lv_obj_set_style_bg_color(b, lv_color_black(), 0);
            lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
            lv_obj_remove_flag(b, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_set_size(b, 5, 2);
            lv_obj_set_pos(b, 1 + i * 8, line_h_ + (row_h_ - 2) / 2);
            lv_obj_add_flag(b, LV_OBJ_FLAG_HIDDEN);
            bars_[i] = b;
        }

        lv_timer_create(&OledStatusPanel::SecondCb, 1000, this);
        lv_timer_create(&OledStatusPanel::WaveCb, 100, this);
        Tick();
        Refresh();

        lvgl_port_unlock();
    }

    static void SecondCb(lv_timer_t* t) {
        static_cast<OledStatusPanel*>(lv_timer_get_user_data(t))->Tick();
    }
    static void WaveCb(lv_timer_t* t) {
        static_cast<OledStatusPanel*>(lv_timer_get_user_data(t))->WaveTick();
    }

    bool WaveActive() const {
        return (mode_ == kListening || mode_ == kSpeaking) && !notif_active_;
    }

    // Cập nhật nội dung 2 dòng theo trạng thái hiện tại
    void Refresh() {
        if (line1_label_ == nullptr) return;

        std::string line1 = ssid_text_;
        if (notif_active_) {
            line1 = notif_;
        } else if (mode_ == kListening) {
            line1 = "Dang nghe...";
        } else if (mode_ == kSpeaking) {
            line1 = "Dang noi...";
        }
        if (line1 != line1_text_) {
            line1_text_ = line1;
            lv_label_set_text(line1_label_, line1.c_str());
        }

        bool wave = WaveActive();
        if (wave) {
            lv_obj_add_flag(date_label_, LV_OBJ_FLAG_HIDDEN);
            for (int i = 0; i < kBars; i++) lv_obj_remove_flag(bars_[i], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_remove_flag(date_label_, LV_OBJ_FLAG_HIDDEN);
            for (int i = 0; i < kBars; i++) lv_obj_add_flag(bars_[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Sóng âm nhấp nhô: cao ở giữa, thấp hai bên. Đang nói thì biên độ lớn hơn lắng nghe.
    void WaveTick() {
        if (!WaveActive()) return;
        int amp = row_h_ - 1;
        if (mode_ == kListening) amp = std::max(3, amp * 55 / 100);
        amp = std::max(3, amp);

        for (int i = 0; i < kBars; i++) {
            int env = 100 - abs(2 * i - (kBars - 1)) * 60 / (kBars - 1);  // 40..100 %
            int target = (2 + (int)(esp_random() % (uint32_t)(amp - 1))) * env / 100;
            target = std::max(2, target);
            int h = (bar_h_[i] * 2 + target) / 3;
            h = std::clamp(h, 2, row_h_);
            if (h == bar_h_[i]) continue;
            bar_h_[i] = h;
            lv_obj_set_height(bars_[i], h);
            lv_obj_set_y(bars_[i], line_h_ + (row_h_ - h) / 2);
        }
    }

    // Chạy mỗi giây: ngày giờ, tên Wi-Fi, hết hạn thông báo
    void Tick() {
        tick_++;

        // Thứ + ngày/tháng + giờ, ví dụ "T4 04/11 18:05"
        static const char* kWday[7] = {"CN", "T2", "T3", "T4", "T5", "T6", "T7"};
        char buf[32];
        std::string d = "-- --/-- --:--";
        time_t now = time(nullptr);
        struct tm tm_info = {};
        localtime_r(&now, &tm_info);
        if (tm_info.tm_year + 1900 >= 2025) {
            snprintf(buf, sizeof(buf), "%s %02d/%02d %02d:%02d", kWday[tm_info.tm_wday % 7],
                     tm_info.tm_mday, tm_info.tm_mon + 1, tm_info.tm_hour, tm_info.tm_min);
            d = buf;
        }
        if (d != date_text_) {
            date_text_ = d;
            lv_label_set_text(date_label_, d.c_str());
        }

        // Tên Wi-Fi (kiểm tra mỗi 5 giây)
        if (tick_ % 5 == 1) {
            wifi_ap_record_t ap = {};
            std::string s = "No Wi-Fi";
            if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                s = reinterpret_cast<const char*>(ap.ssid);
            }
            if (s != ssid_text_) {
                ssid_text_ = s;
                Refresh();
            }
        }

        // Hết hạn thông báo thì quay lại hiện trạng thái
        if (notif_active_ && esp_timer_get_time() >= notif_expire_us_) {
            notif_active_ = false;
            Refresh();
        }
    }
};

// =====================================================================
// DISPLAY KÉP
//   TFT: mặt robot (2 mắt chớp + miệng)    OLED: Wi-Fi / ngày giờ / đang nghe + sóng
// Kế thừa SpiLcdDisplay nên phần còn lại của dự án vẫn coi đây là LcdDisplay.
// =====================================================================
class DualDisplay : public SpiLcdDisplay {
public:
    DualDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                int width, int height, int offset_x, int offset_y,
                bool mirror_x, bool mirror_y, bool swap_xy,
                esp_lcd_panel_io_handle_t oled_io, esp_lcd_panel_handle_t oled_panel)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y,
                        mirror_x, mirror_y, swap_xy) {
        lv_display_t* tft_disp = display_;  // màn hình TFT (được tạo đầu tiên)
        oled_ = new OledStatusPanel(oled_io, oled_panel, OLED_WIDTH, OLED_HEIGHT,
                                    OLED_MIRROR_X, OLED_MIRROR_Y);
        // Đảm bảo màn hình mặc định của LVGL vẫn là TFT
        if (tft_disp != nullptr && lvgl_port_lock(0)) {
            lv_display_set_default(tft_disp);
            lvgl_port_unlock();
        }
    }

    virtual void SetupUI() override {
        // Giữ nguyên giao diện gốc (để các hàm khác của dự án không bị lỗi con trỏ rỗng),
        // rồi ẩn nó đi và đặt mặt robot lên trên.
        SpiLcdDisplay::SetupUI();
        SetHideSubtitle(true);

        DisplayLockGuard lock(this);
        if (container_ != nullptr) lv_obj_add_flag(container_, LV_OBJ_FLAG_HIDDEN);
        if (status_bar_ != nullptr) lv_obj_add_flag(status_bar_, LV_OBJ_FLAG_HIDDEN);
        if (top_bar_ != nullptr) lv_obj_add_flag(top_bar_, LV_OBJ_FLAG_HIDDEN);
        if (bottom_bar_ != nullptr) lv_obj_add_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);

        lv_obj_t* scr = lv_screen_active();
        lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
        if (face_ == nullptr) face_ = new RobotFace(scr);
    }

    virtual void SetStatus(const char* status) override {
        SpiLcdDisplay::SetStatus(status);
        if (status == nullptr) return;
        if (face_ != nullptr) {
            RobotFace::Mode m = RobotFace::kIdle;
            if (strcmp(status, Lang::Strings::LISTENING) == 0) {
                m = RobotFace::kListening;
            } else if (strcmp(status, Lang::Strings::SPEAKING) == 0) {
                m = RobotFace::kSpeaking;
            }
            face_->SetMode(m);
        }
        if (oled_ != nullptr) oled_->SetStatus(status);
    }

    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override {
        SpiLcdDisplay::ShowNotification(notification, duration_ms);
        if (oled_ != nullptr) oled_->ShowNotification(notification, duration_ms);
    }

    virtual void ShowNotification(const std::string& notification, int duration_ms = 3000) override {
        SpiLcdDisplay::ShowNotification(notification, duration_ms);
        if (oled_ != nullptr) oled_->ShowNotification(notification.c_str(), duration_ms);
    }

private:
    OledStatusPanel* oled_ = nullptr;
    RobotFace* face_ = nullptr;
};

// =====================================================================
// BOARD MAIN
// =====================================================================
class ZiczacAiRobotBoard : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_ = nullptr;
    EyeLeds* eyes_ = nullptr;
    Servos* servos_ = nullptr;
    Motors* motors_ = nullptr;
    Ultrasonic* ultrasonic_ = nullptr;

    i2c_master_bus_handle_t oled_i2c_bus_ = nullptr;
    esp_lcd_panel_io_handle_t oled_io_ = nullptr;
    esp_lcd_panel_handle_t oled_panel_ = nullptr;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    // Khởi tạo phần cứng OLED SSD1306 (I2C). Trả về false nếu không thấy OLED,
    // khi đó board vẫn chạy bình thường chỉ với màn TFT.
    bool InitializeOledHardware() {
        if (OLED_SDA_GPIO < 0 || OLED_SCL_GPIO < 0) return false;

        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_NUM_0;
        bus_cfg.sda_io_num = OLED_SDA_GPIO;
        bus_cfg.scl_io_num = OLED_SCL_GPIO;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = true;
        if (i2c_new_master_bus(&bus_cfg, &oled_i2c_bus_) != ESP_OK) {
            ESP_LOGE(TAG, "Không tạo được bus I2C cho OLED");
            return false;
        }

        if (i2c_master_probe(oled_i2c_bus_, OLED_I2C_ADDR, 200) != ESP_OK) {
            ESP_LOGW(TAG, "Không thấy OLED ở địa chỉ 0x%02X, bỏ qua OLED", OLED_I2C_ADDR);
            return false;
        }

        esp_lcd_panel_io_i2c_config_t io_config = {};
        io_config.dev_addr = OLED_I2C_ADDR;
        io_config.control_phase_bytes = 1;
        io_config.dc_bit_offset = 6;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        io_config.scl_speed_hz = 400 * 1000;
        if (esp_lcd_new_panel_io_i2c(oled_i2c_bus_, &io_config, &oled_io_) != ESP_OK) {
            ESP_LOGE(TAG, "Lỗi tạo panel io cho OLED");
            return false;
        }

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;
        esp_lcd_panel_ssd1306_config_t ssd1306_config = {};
        ssd1306_config.height = static_cast<uint8_t>(OLED_HEIGHT);
        panel_config.vendor_config = &ssd1306_config;
        if (esp_lcd_new_panel_ssd1306(oled_io_, &panel_config, &oled_panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Lỗi tạo panel SSD1306");
            return false;
        }
        if (esp_lcd_panel_reset(oled_panel_) != ESP_OK || esp_lcd_panel_init(oled_panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Lỗi khởi tạo SSD1306");
            return false;
        }
        esp_lcd_panel_invert_color(oled_panel_, false);
        esp_lcd_panel_disp_on_off(oled_panel_, true);
        return true;
    }

#if DISPLAY_TEST_PATTERN
    void FillScreen(esp_lcd_panel_handle_t panel, uint16_t color) {
        static uint16_t line[DISPLAY_WIDTH];
        for (int i = 0; i < DISPLAY_WIDTH; i++) line[i] = color;
        for (int y = 0; y < DISPLAY_HEIGHT; y++) {
            esp_lcd_panel_draw_bitmap(panel, 0, y, DISPLAY_WIDTH, y + 1, line);
        }
    }

    void RunDisplayTest(esp_lcd_panel_handle_t panel) {
        const uint16_t colors[] = {0x0000, 0xFFFF, 0xF800, 0x07E0, 0x001F};
        for (uint16_t c : colors) {
            FillScreen(panel, c);
            vTaskDelay(pdMS_TO_TICKS(600));
        }
        FillScreen(panel, 0x0000);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
#endif

    void InitializeLcdDisplay(bool with_oled) {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = DISPLAY_SPI_CLOCK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        vTaskDelay(pdMS_TO_TICKS(120));

        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));

#if DISPLAY_TEST_PATTERN
        FillScreen(panel, 0x0000);
        vTaskDelay(pdMS_TO_TICKS(50));
#endif

        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

#if DISPLAY_TEST_PATTERN
        RunDisplayTest(panel);
#endif

        if (with_oled) {
            display_ = new DualDisplay(panel_io, panel,
                                       DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                       DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                       oled_io_, oled_panel_);
        } else {
            display_ = new SpiLcdDisplay(panel_io, panel,
                                         DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                         DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        }
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTools() {
        eyes_ = new EyeLeds();
        servos_ = new Servos();
        motors_ = new Motors();
        ultrasonic_ = new Ultrasonic();

        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool("self.robot.get_distance",
            "Đọc khoảng cách tới vật cản phía trước bằng cảm biến siêu âm, đơn vị cm.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                int cm = ultrasonic_->MeasureCm();
                return std::string("{\"distance_cm\": ") + std::to_string(cm) + "}";
            });
    }

public:
    ZiczacAiRobotBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        bool oled_ok = InitializeOledHardware();
        InitializeLcdDisplay(oled_ok);
        InitializeButtons();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(ZiczacAiRobotBoard);