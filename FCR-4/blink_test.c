
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#define POT_ADC_PIN    26
#define PWM_OUTPUT_PIN 15

#define PWM_FREQUENCY  25000
#define PWM_TOP        4999

int main(void)
{
    // ADC初期化
    adc_init();
    adc_gpio_init(POT_ADC_PIN);
    adc_select_input(0);

    // PWM初期化
    gpio_set_function(PWM_OUTPUT_PIN, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(PWM_OUTPUT_PIN);
    uint channel = pwm_gpio_to_channel(PWM_OUTPUT_PIN);

    pwm_set_wrap(slice_num, PWM_TOP);
    pwm_set_clkdiv(slice_num, 1.0f);

    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);

    while (true)
    {
        // 可変抵抗の電圧を読み取る（0～4095）
        uint16_t adc_value = adc_read();

        // ADC値をPWMデューティ比に変換
        // MOSFET駆動用のため、PWM出力を反転する
        uint16_t fan_duty =
            (uint32_t)adc_value * 1000 / 4095;

        uint16_t mosfet_duty = 1000 - fan_duty;

        // PWM出力値に変換
        uint16_t pwm_level =
            (uint32_t)mosfet_duty * (PWM_TOP + 1) / 1000;

        pwm_set_chan_level(slice_num, channel, pwm_level);

        sleep_ms(10);
    }

    return 0;
}
