// ファンの風量切り替え ＋ PG（PD交渉成功）表示プログラム（Pico SDK版）
//
// 動作:
//   ・PG（PD交渉成功）が来ていると LED が点灯
//   ・PGが来ていない（12Vがもらえていない）と LED が速く点滅（エラー表示）
//       ※電源を入れた直後は交渉中なので、最初の2秒間はエラーと判定しない（消灯）
//       CH221K の PG = GPIO0、CH224K の PG = GPIO1（どちらかが Low なら成功）
//   ・各ファンには「可変抵抗モード」「弱」「中」「強」の4つのモードがある
//   ・電源を入れた直後は、4台とも「可変抵抗モード」（今までどおり可変抵抗で風量を変えられる）
//   ・スイッチを押すたびに、そのファンのモードが次の順に切り替わる
//         可変抵抗 → 弱 → 中 → 強 → 可変抵抗 → …
//       SW1 = J3のファン、SW2 = J4、SW3 = J5、SW4 = J6
//   ・切り替えたとき、LEDの点滅で今のモードを知らせる
//         弱 = 1回、中 = 2回、強 = 3回、可変抵抗 = 長く1回
//   ・VS Codeのデバッグで g_rpm[]（回転数）、g_duty[]（各ファンのデューティ%）を確認できる
//
// 接続（回路図より）:
//   PWM    : GPIO15(J3), GPIO16(J4), GPIO17(J5), GPIO18(J6)  ※MOSFETで反転
//   センサ : GPIO19(J3), GPIO20(J4), GPIO21(J5), GPIO22(J6)
//   PG     : GPIO0(CH221K), GPIO1(CH224K)  ※外付け10kΩでプルアップ済み、Lowで成功
//   LED    : GPIO2
//   ボタン : SW1=GPIO12, SW2=GPIO11, SW3=GPIO14, SW4=GPIO13（押すとGND）
//   可変抵抗 : GPIO26(ADC0)

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"

#define LED_PIN        2
#define PG_221K        0
#define PG_224K        1
#define POT_PIN        26
#define POT_ADC_INPUT  0
#define FAN_COUNT      4
#define PWM_FREQ_HZ    25000   // ファンの指定（25kHz）
#define PULSES_PER_REV 2       // 1回転あたりのパルス数（一般的な値）

// 各モードのデューティ（%）。好みに合わせて変更してください
#define DUTY_LOW   30
#define DUTY_MID   60
#define DUTY_HIGH  100

// PGエラー表示の設定
#define PG_STARTUP_WAIT_MS  2000   // 電源投入直後、交渉を待つ時間
#define PG_ERROR_BLINK_MS   250    // エラー時の点滅の速さ（点灯・消灯それぞれの時間）

typedef enum {
    MODE_POT = 0,   // 可変抵抗モード
    MODE_LOW,       // 弱
    MODE_MID,       // 中
    MODE_HIGH,      // 強
    MODE_COUNT
} fan_mode_t;

static const uint FAN_PWM_PINS[FAN_COUNT]  = {15, 16, 17, 18};
static const uint FAN_TACH_PINS[FAN_COUNT] = {19, 20, 21, 22};
static const uint BUTTON_PINS[FAN_COUNT]   = {14, 13, 11, 12};

static fan_mode_t g_mode[FAN_COUNT];
static volatile uint32_t g_pulse_count[FAN_COUNT];
volatile uint32_t g_rpm[FAN_COUNT];    // デバッグで見る用
volatile uint32_t g_duty[FAN_COUNT];   // デバッグで見る用
static uint16_t g_pwm_wrap;

// ---------- 回転センサ ----------
static void tach_callback(uint gpio, uint32_t events) {
    (void)events;
    for (int i = 0; i < FAN_COUNT; i++) {
        if (gpio == FAN_TACH_PINS[i]) {
            g_pulse_count[i]++;
        }
    }
}

// ---------- PWM ----------
static void fan_pwm_init(void) {
    uint32_t sys_hz = clock_get_hz(clk_sys);
    g_pwm_wrap = (uint16_t)(sys_hz / PWM_FREQ_HZ - 1);  // 125MHzなら4999

    for (int i = 0; i < FAN_COUNT; i++) {
        uint pin = FAN_PWM_PINS[i];
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pin);
        pwm_set_clkdiv(slice, 1.0f);
        pwm_set_wrap(slice, g_pwm_wrap);
        // MOSFETで反転するので、出力も反転して「設定値＝ファンのデューティ」にする
        if (pwm_gpio_to_channel(pin) == PWM_CHAN_A) {
            pwm_set_output_polarity(slice, true, pwm_hw->slice[slice].csr & PWM_CH0_CSR_B_INV_BITS);
        } else {
            pwm_set_output_polarity(slice, pwm_hw->slice[slice].csr & PWM_CH0_CSR_A_INV_BITS, true);
        }
        pwm_set_gpio_level(pin, 0);
        pwm_set_enabled(slice, true);
    }
}

static void fan_set_duty(int fan, uint32_t percent) {
    if (percent > 100) percent = 100;
    g_duty[fan] = percent;
    uint32_t level = (uint32_t)(g_pwm_wrap + 1) * percent / 100;
    pwm_set_gpio_level(FAN_PWM_PINS[fan], (uint16_t)level);
}

// ---------- PG（PD交渉成功）の確認 ----------
static bool is_power_good(void) {
    // どちらかのPGがLowなら成功（使っていないほうのICは電源が入らず、プルアップでHighのまま）
    return !gpio_get(PG_221K) || !gpio_get(PG_224K);
}

// ---------- LED ----------
static void led_flash(int count, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(on_ms);
        gpio_put(LED_PIN, 0);
        sleep_ms(off_ms);
    }
}

// 今のモードをLEDで知らせる
static void led_show_mode(fan_mode_t mode) {
    gpio_put(LED_PIN, 0);
    sleep_ms(200);
    switch (mode) {
        case MODE_LOW:  led_flash(1, 150, 150); break;
        case MODE_MID:  led_flash(2, 150, 150); break;
        case MODE_HIGH: led_flash(3, 150, 150); break;
        default:        led_flash(1, 800, 200); break;  // 可変抵抗モード：長く1回
    }
}

// ---------- メイン ----------
int main(void) {

    /* ======================================= */
    /*初期化処理                                */
    /* ======================================= */

    //PINの初期化
    gpio_init(LED_PIN);

    //PINの入出力を設定
    gpio_set_dir(LED_PIN, GPIO_OUT);

    //出力ピンをHigh/LOWにする 0:LOW 1:High
    gpio_put(LED_PIN, 0);

    // PG（外付けプルアップがあるので内部プルは使わない）
    gpio_init(PG_221K);

    gpio_set_dir(PG_221K, GPIO_IN);
    
    gpio_disable_pulls(PG_221K);
    
    gpio_init(PG_224K);
    
    gpio_set_dir(PG_224K, GPIO_IN);
    
    gpio_disable_pulls(PG_224K);

    // ボタン（内部プルアップ）
    for (int i = 0; i < FAN_COUNT; i++) {
        gpio_init(BUTTON_PINS[i]);
        gpio_set_dir(BUTTON_PINS[i], GPIO_IN);
        gpio_pull_up(BUTTON_PINS[i]);
        g_mode[i] = MODE_POT;
    }

    // 可変抵抗（ADC）
    adc_init();
    adc_gpio_init(POT_PIN);
    adc_select_input(POT_ADC_INPUT);

    // ファンのPWM
    fan_pwm_init();

    // 回転センサ（外付けプルアップあり、立ち下がりで数える）
    for (int i = 0; i < FAN_COUNT; i++) {
        uint pin = FAN_TACH_PINS[i];
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_disable_pulls(pin);
        if (i == 0) {
            gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, &tach_callback);
        } else {
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, true);
        }
    }

    bool prev_pressed[FAN_COUNT] = {false, false, false, false};
    absolute_time_t next_rpm_update = make_timeout_time_ms(1000);
    absolute_time_t pg_wait_end     = make_timeout_time_ms(PG_STARTUP_WAIT_MS);
    absolute_time_t next_blink      = get_absolute_time();
    bool blink_on = false;

    while (true) {
        // 可変抵抗の値（0〜100%）
        uint32_t pot_percent = (uint32_t)adc_read() * 100 / 4095;

        // ボタンの確認（押した瞬間だけ反応）
        for (int i = 0; i < FAN_COUNT; i++) {
            bool pressed = !gpio_get(BUTTON_PINS[i]);
            if (pressed && !prev_pressed[i]) {
                sleep_ms(20);                          // チャタリング対策
                if (!gpio_get(BUTTON_PINS[i])) {
                    g_mode[i] = (fan_mode_t)((g_mode[i] + 1) % MODE_COUNT);
                    led_show_mode(g_mode[i]);
                }
            }
            prev_pressed[i] = pressed;
        }

        // 各ファンのデューティを設定
        for (int i = 0; i < FAN_COUNT; i++) {
            switch (g_mode[i]) {
                case MODE_LOW:  fan_set_duty(i, DUTY_LOW);  break;
                case MODE_MID:  fan_set_duty(i, DUTY_MID);  break;
                case MODE_HIGH: fan_set_duty(i, DUTY_HIGH); break;
                default:        fan_set_duty(i, pot_percent); break;
            }
        }
        /* ======================================= */
        /*モータからくるセンサーの値をカウントして計算 */
        /* ======================================= */

        // 1秒ごとに回転数(rpm)を計算
        if (absolute_time_diff_us(get_absolute_time(), next_rpm_update) <= 0) {
            for (int i = 0; i < FAN_COUNT; i++) {
                uint32_t pulses = g_pulse_count[i];
                g_pulse_count[i] = 0;
                g_rpm[i] = pulses * 60 / PULSES_PER_REV;
            }
            next_rpm_update = make_timeout_time_ms(1000);
        }

        // PGの表示
        if (is_power_good()) {
            // 交渉成功：点灯
            gpio_put(LED_PIN, 1);
        } else if (absolute_time_diff_us(get_absolute_time(), pg_wait_end) > 0) {
            // 電源投入直後（交渉中）：消灯して待つ
            gpio_put(LED_PIN, 0);
        } else {
            // PGエラー：速く点滅
            if (absolute_time_diff_us(get_absolute_time(), next_blink) <= 0) {
                blink_on = !blink_on;
                next_blink = make_timeout_time_ms(PG_ERROR_BLINK_MS);
            }
            gpio_put(LED_PIN, blink_on);
        }

        sleep_ms(10);
    }
}