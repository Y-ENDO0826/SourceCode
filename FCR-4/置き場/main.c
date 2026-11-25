/*
 * ファンコントローラ (RCF-4) ファームウェア
 * マイコン : AVR32LA32 (TQFP-32)
 * 開発環境 : MPLAB X + XC8 (avr-gcc) / Microchip Studio
 *            ※ AVR-LA_DFP (デバイスパック) が必要です
 *
 * ---- 機能 ----------------------------------------------------------
 *  1. 起動時に毎回ゼロ点補正（ファン電源OFFの状態で電流を測る）
 *  2. PG(PD-PG)がLow かつ VBUS が11V以上を確認してからファン電源ON
 *  3. 可変抵抗(VR)でファン4系統のPWMデューティを設定（25kHz）
 *  4. 電流が1.9Aを0.5秒以上超え続けたらファン電源OFF（過電流）
 *  5. ファン8台の回転数を測定、止まっているファンがあれば警告
 *  6. スイッチ：運転中→停止 / 停止中・異常時→再起動（ゼロ点も取り直す）
 *  7. デバッグ用UART出力（PA0=TX, 115200bps）
 *
 * ---- LED表示 --------------------------------------------------------
 *  青 点滅      : ゼロ点補正中 / 12V待ち
 *  緑 点灯      : 運転中
 *  緑+青 点灯   : 運転中だが回転していないファンがある
 *  緑 点滅      : スイッチで停止中
 *  赤 点灯      : 過電流で停止
 *  赤 点滅      : ゼロ点異常（ファン電源OFFなのに電流が流れている）
 *
 * ---- ピン割り当て ---------------------------------------------------
 *  PA0 : UART TX (J2-1)       PA2 : SW (Lowで押下)
 *  PA3 : PD-PG (Lowで12V OK)  PA7 : VSNS (AIN27, VBUS 1/11)
 *  PC0 : LED赤  PC1 : LED緑  PC2 : LED青  PC3 : MOTOR_POWER (High=ON)
 *  PD0-PD7 : ファン回転センサ1〜8
 *  PF0-PF2 : PWM1〜3 (TCE0)   PF4 : PWM4 (TCB0)
 *  PF3 : VR (AIN19)           PF5 : ISNS (AIN21)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

/* =====================================================================
 *  設定（ここを変更して調整する）
 * ===================================================================== */

/* 電流検出アンプ(U2)を使う場合は1にする（JP1を2-3側にした場合） */
#define USE_CURRENT_AMP      0

/* デバッグ用UART出力を使う場合は1 */
#define USE_DEBUG_UART       1

/* 過電流判定 */
#define TRIP_CURRENT_MA      1900u   /* これを超えたら過電流とみなす [mA] */
#define TRIP_TIME_MS         500u    /* この時間超え続けたら停止 [ms] */
#define STARTUP_BLANK_MS     1000u   /* 電源ON直後は突入電流を無視 [ms] */

/* 12V確認 */
#define VBUS_START_MV        11000u  /* これ以上でファン電源ON [mV] */
#define VBUS_STOP_MV         10500u  /* これを下回ったらファン電源OFF [mV] */
#define VBUS_OK_TIME_MS      200u    /* 12Vがこの時間安定したらON [ms] */

/* ゼロ点補正 */
#define ZERO_SETTLE_MS       500u    /* 電源OFF後、電流が0になるまで待つ [ms] */
#define ZERO_MAX_MA          150u    /* ゼロ点がこれを超えたら異常 [mA] */

/* ファン */
#define MIN_DUTY_PERCENT     20u     /* VRを最小にしたときのデューティ [%] */
#define FAN_PRESENT_MASK     0xFFu   /* 回転監視するファン (bit0=ファン1) */
#define STALL_RPM            300u    /* これ未満なら停止とみなす [rpm] */
#define STALL_GRACE_MS       3000u   /* 電源ON後、回転監視を始めるまで [ms] */
#define PULSES_PER_REV       2u      /* 1回転あたりのパルス数（一般的なファンは2） */

/* 電流の傾き補正（DP100で校正して調整する。1000=補正なし） */
#define CAL_GAIN_PERMILLE    1000u

/* =====================================================================
 *  クロック・レジスタ設定値（データシート DS40002684A より）
 * ===================================================================== */
#define F_CPU_HZ             10000000UL   /* 20MHz / 2 = 10MHz */

/* CLKCTRL */
#define CLK_PDIV_DIV2        (0x0 << 1)
#define CLK_PEN              (1 << 0)

/* ADC0 */
#define ADC_ENABLE           (1 << 0)
#define ADC_PRESC_DIV8       0x06         /* 10MHz/8 = 1.25MHz */
#define ADC_REF_VDD          0x0
#define ADC_REF_1V024        0x4
#define ADC_REF_2V048        0x5
#define ADC_REF_2V500        0x7
#define ADC_SAMPNUM_ACC32    0x5          /* 32回の合計 */
#define ADC_CMD_BURST        ((0x3 << 4) | 0x1)   /* 32回連続変換して合計 */
#define ADC_CMD_SINGLE10     ((0x1 << 4) | 0x1)   /* 1回だけ変換 */
#define ADC_RESRDY           (1 << 0)
#define ADC_SAMPDUR          16           /* サンプル時間 17クロック */

#define MUX_ISNS             0x15         /* AIN21 = PF5 */
#define MUX_VR               0x13         /* AIN19 = PF3 */
#define MUX_VSNS             0x1B         /* AIN27 = PA7 */

/* TCE0 (PWM1〜3) */
#define TCE_ENABLE           (1 << 0)
#define TCE_CLK_DIV1         (0x0 << 1)
#define TCE_CMP0EN           (1 << 4)
#define TCE_CMP1EN           (1 << 5)
#define TCE_CMP2EN           (1 << 6)
#define TCE_WG_SINGLESLOPE   0x3
#define TCE_PER_VALUE        399u         /* 10MHz / 400 = 25kHz */

/* TCB (PWM4 / 1msタイマ) */
#define TCB_ENABLE           (1 << 0)
#define TCB_CLK_DIV1         (0x0 << 1)
#define TCB_CLK_DIV2         (0x1 << 1)
#define TCB_CCMPEN           (1 << 4)
#define TCB_MODE_INT         0x0
#define TCB_MODE_PWM8        0x7
#define TCB_CAPT             (1 << 0)
#define TCB0_PER_VALUE       199u         /* 5MHz / 200 = 25kHz */

/* PORTMUX */
#define PMUX_TCE0_PORTF      0x05         /* TCE0 WO0-2 → PF0-PF2 */
#define PMUX_TCB0_ALT1       (1 << 0)     /* TCB0 WO → PF4 */

/* PORT */
#define PINCTRL_PULLUP       (1 << 3)
#define PINCTRL_FALLING      0x3
#define PINCTRL_INPUT_DIS    0x4

/* USART0 */
#define USART_ENABLE         (1 << 0)
#define USART_TXEN           (1 << 0)
#define USART_CHSIZE_8BIT    0x3
#define USART_DRE            (1 << 0)
#define USART_BAUD_115200    347u         /* 64*10MHz/(16*115200) */

/* WDT */
#define WDT_PERIOD_1S        0x8

/* =====================================================================
 *  電流・電圧の換算（ADCの32回合計値 S から計算）
 *    32回合計のフルスケール = 1024 * 32 = 32768
 * ===================================================================== */
#if USE_CURRENT_AMP
  /* アンプ20倍・シャント0.05Ω → 1A = 1V。基準2.5V。I[mA] = S * 2500 / 32768 */
  #define ISNS_REF           ADC_REF_2V500
  #define ISNS_NUM           2500UL
#else
  /* アンプなし・シャント0.05Ω → 1A = 50mV。基準1.024V。
     I[mA] = (S * 1024 / 32768) / 0.05 = S * 20480 / 32768 */
  #define ISNS_REF           ADC_REF_1V024
  #define ISNS_NUM           20480UL
#endif
#define ADC_SUM_FULL         32768UL

/* VBUS: 100kΩ/10kΩ = 1/11。基準2.048V。VBUS[mV] = S * 2048 * 11 / 32768 */
#define VSNS_REF             ADC_REF_2V048
#define VSNS_NUM             (2048UL * 11UL)

/* =====================================================================
 *  ピン操作
 * ===================================================================== */
#define LED_RED              PIN0_bm      /* PC0 */
#define LED_GREEN            PIN1_bm      /* PC1 */
#define LED_BLUE             PIN2_bm      /* PC2 */
#define MOTOR_POWER          PIN3_bm      /* PC3 */

static inline void motor_on(void)  { PORTC.OUTSET = MOTOR_POWER; }
static inline void motor_off(void) { PORTC.OUTCLR = MOTOR_POWER; }
static inline bool pd_good(void)   { return (PORTA.IN & PIN3_bm) == 0; }  /* PGはLowで有効 */
static inline bool sw_pressed(void){ return (PORTA.IN & PIN2_bm) == 0; }

/* =====================================================================
 *  状態
 * ===================================================================== */
typedef enum {
    ST_ZERO,        /* ゼロ点補正（電源OFFで待ってから測る） */
    ST_WAIT_POWER,  /* 12V待ち */
    ST_RUN,         /* 運転中 */
    ST_STOPPED,     /* スイッチで停止中 */
    ST_FAULT_OC,    /* 過電流で停止 */
    ST_FAULT_ZERO   /* ゼロ点異常 */
} state_t;

static volatile uint32_t g_ms;              /* 1msカウンタ */
static volatile uint16_t g_tach_cnt[8];     /* 回転パルスのカウント */

static uint16_t g_zero_sum;                 /* ゼロ点（ADC 32回合計値） */
static uint16_t g_rpm[8];
static uint16_t g_current_ma;
static uint16_t g_vbus_mv;
static uint8_t  g_duty;                     /* 0〜100 [%] */
static bool     g_stall;

/* =====================================================================
 *  割り込み
 * ===================================================================== */
ISR(TCB1_INT_vect)                          /* 1msごと */
{
    TCB1.INTFLAGS = TCB_CAPT;
    g_ms++;
}

ISR(PORTD_PORT_vect)                        /* ファン回転パルス */
{
    uint8_t flags = PORTD.INTFLAGS;
    PORTD.INTFLAGS = flags;
    for (uint8_t i = 0; i < 8; i++) {
        if (flags & (1u << i)) {
            g_tach_cnt[i]++;
        }
    }
}

static uint32_t millis(void)
{
    uint32_t t;
    cli();
    t = g_ms;
    sei();
    return t;
}

/* =====================================================================
 *  初期化
 * ===================================================================== */
static void clock_init(void)
{
    /* 20MHz / 2 = 10MHz */
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLK_PDIV_DIV2 | CLK_PEN);
    CLKCTRL.MCLKTIMEBASE = 10;              /* 1µs = 10クロック（ADC用） */
}

static void port_init(void)
{
    /* ファン電源は真っ先にOFFにしておく */
    PORTC.OUTCLR = MOTOR_POWER | LED_RED | LED_GREEN | LED_BLUE;
    PORTC.DIRSET = MOTOR_POWER | LED_RED | LED_GREEN | LED_BLUE;

    /* SW(PA2): 内部プルアップ / PG(PA3): 外部プルアップあり */
    PORTA.PIN2CTRL = PINCTRL_PULLUP;
    PORTA.DIRCLR   = PIN2_bm | PIN3_bm;

    /* アナログ入力はデジタル入力を切る（ノイズ・消費電流対策） */
    PORTA.PIN7CTRL = PINCTRL_INPUT_DIS;     /* VSNS */
    PORTF.PIN3CTRL = PINCTRL_INPUT_DIS;     /* VR */
    PORTF.PIN5CTRL = PINCTRL_INPUT_DIS;     /* ISNS */

    /* 回転センサ(PD0-7): 外部10kプルアップあり、立ち下がりで割り込み */
    PORTD.DIRCLR = 0xFF;
    PORTD.PIN0CTRL = PINCTRL_FALLING;
    PORTD.PIN1CTRL = PINCTRL_FALLING;
    PORTD.PIN2CTRL = PINCTRL_FALLING;
    PORTD.PIN3CTRL = PINCTRL_FALLING;
    PORTD.PIN4CTRL = PINCTRL_FALLING;
    PORTD.PIN5CTRL = PINCTRL_FALLING;
    PORTD.PIN6CTRL = PINCTRL_FALLING;
    PORTD.PIN7CTRL = PINCTRL_FALLING;

    /* PWM出力(PF0-2, PF4) */
    PORTF.OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN4_bm;
    PORTF.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN4_bm;
}

static void pwm_init(void)
{
    PORTMUX.TCEROUTEA = PMUX_TCE0_PORTF;
    PORTMUX.TCBROUTEA = PMUX_TCB0_ALT1;

    /* TCE0: PWM1〜3、シングルスロープ 25kHz */
    TCE0.PER = TCE_PER_VALUE;
    TCE0.CMP0 = 0;
    TCE0.CMP1 = 0;
    TCE0.CMP2 = 0;
    TCE0.CTRLB = TCE_CMP0EN | TCE_CMP1EN | TCE_CMP2EN | TCE_WG_SINGLESLOPE;
    TCE0.CTRLA = TCE_CLK_DIV1 | TCE_ENABLE;

    /* TCB0: PWM4、8bit PWM 25kHz */
    TCB0.CCMPL = TCB0_PER_VALUE;
    TCB0.CCMPH = 0;
    TCB0.CTRLB = TCB_CCMPEN | TCB_MODE_PWM8;
    TCB0.CTRLA = TCB_CLK_DIV2 | TCB_ENABLE;
}

static void tick_init(void)
{
    /* TCB1: 1msごとの割り込み */
    TCB1.CCMP    = (uint16_t)(F_CPU_HZ / 1000UL - 1UL);
    TCB1.CTRLB   = TCB_MODE_INT;
    TCB1.INTCTRL = TCB_CAPT;
    TCB1.CTRLA   = TCB_CLK_DIV1 | TCB_ENABLE;
}

static void adc_init(void)
{
    ADC0.CTRLB = ADC_PRESC_DIV8;
    ADC0.CTRLE = ADC_SAMPDUR;
    ADC0.CTRLF = ADC_SAMPNUM_ACC32;         /* 32回合計（右詰め） */
    ADC0.CTRLA = ADC_ENABLE;
}

#if USE_DEBUG_UART
static void uart_init(void)
{
    PORTA.OUTSET = PIN0_bm;
    PORTA.DIRSET = PIN0_bm;                 /* PA0 = TX */
    USART0.BAUD  = USART_BAUD_115200;
    USART0.CTRLD = USART_CHSIZE_8BIT;       /* 8bit, パリティなし, ストップ1 */
    USART0.CTRLB = USART_TXEN;
    USART0.CTRLA = USART_ENABLE;
}

static void uart_putc(char c)
{
    while (!(USART0.INTFLAGS & USART_DRE)) { }
    USART0.TXDATAL = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s) uart_putc(*s++);
}

static void uart_putu(uint32_t v)           /* 整数を10進で表示（printfは使わない） */
{
    char buf[11];
    uint8_t i = 0;
    do { buf[i++] = (char)('0' + (v % 10)); v /= 10; } while (v);
    while (i) uart_putc(buf[--i]);
}
#endif

/* =====================================================================
 *  ADC
 * ===================================================================== */
static uint16_t adc_read_sum(uint8_t mux, uint8_t ref)
{
    if (ADC0.CTRLC != ref) {
        /* 基準電圧を切り替えたら、最初の1回は捨てる */
        ADC0.CTRLC   = ref;
        ADC0.MUXPOS  = mux;
        ADC0.COMMAND = ADC_CMD_SINGLE10;
        while (!(ADC0.INTFLAGS & ADC_RESRDY)) { }
        (void)ADC0.RESULT;
    }
    ADC0.MUXPOS  = mux;
    ADC0.COMMAND = ADC_CMD_BURST;           /* 32回連続で変換して合計 */
    while (!(ADC0.INTFLAGS & ADC_RESRDY)) { }
    return ADC0.RESULT;                     /* 0〜32736 */
}

static uint16_t sum_to_ma(uint16_t sum)
{
    uint32_t ma = (uint32_t)sum * ISNS_NUM / ADC_SUM_FULL;
    ma = ma * CAL_GAIN_PERMILLE / 1000UL;
    return (ma > 0xFFFFUL) ? 0xFFFFu : (uint16_t)ma;
}

static uint16_t read_current_ma(void)
{
    uint16_t sum = adc_read_sum(MUX_ISNS, ISNS_REF);
    uint16_t net = (sum > g_zero_sum) ? (uint16_t)(sum - g_zero_sum) : 0;  /* ゼロ点を引く */
    return sum_to_ma(net);
}

static uint16_t read_vbus_mv(void)
{
    uint16_t sum = adc_read_sum(MUX_VSNS, VSNS_REF);
    return (uint16_t)((uint32_t)sum * VSNS_NUM / ADC_SUM_FULL);
}

static uint8_t read_vr_duty(void)
{
    uint16_t sum = adc_read_sum(MUX_VR, ADC_REF_VDD);
    uint16_t v = sum / 32u;                 /* 0〜1023 */
    return (uint8_t)(MIN_DUTY_PERCENT + (uint32_t)(100u - MIN_DUTY_PERCENT) * v / 1023u);
}

/* ゼロ点補正：ファン電源OFFの状態で呼ぶこと */
static bool calibrate_zero(void)
{
    uint32_t total = 0;
    for (uint8_t i = 0; i < 8; i++) {       /* 32回×8 = 256回の平均 */
        total += adc_read_sum(MUX_ISNS, ISNS_REF);
    }
    g_zero_sum = (uint16_t)(total / 8u);
    return sum_to_ma(g_zero_sum) <= ZERO_MAX_MA;   /* 大きすぎたら異常 */
}

/* =====================================================================
 *  PWM
 * ===================================================================== */
static void pwm_set_all(uint8_t percent)
{
    if (percent > 100u) percent = 100u;
    uint16_t ce = (uint16_t)((uint32_t)(TCE_PER_VALUE + 1u) * percent / 100u);
    uint8_t  cb = (uint8_t)((uint16_t)(TCB0_PER_VALUE + 1u) * percent / 100u);
    TCE0.CMP0BUF = ce;                      /* バッファ経由で波形を乱さず更新 */
    TCE0.CMP1BUF = ce;
    TCE0.CMP2BUF = ce;
    TCB0.CCMPH   = cb;
}

/* =====================================================================
 *  メイン
 * ===================================================================== */
int main(void)
{
    clock_init();
    port_init();
    pwm_init();
    tick_init();
    adc_init();
#if USE_DEBUG_UART
    uart_init();
#endif
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_1S);   /* 固まったら1秒でリセット */
    sei();

    state_t  state      = ST_ZERO;
    uint32_t state_ms   = millis();       /* 状態に入った時刻 */
    uint32_t power_ms   = 0;              /* ファン電源ONの時刻 */
    uint32_t last_tick  = millis();
    uint32_t last_sec   = millis();
    uint16_t oc_ms      = 0;              /* 過電流が続いている時間 */
    uint16_t vok_ms     = 0;              /* 12Vが続いている時間 */
    uint16_t vng_ms     = 0;              /* 電圧不足が続いている時間 */
    uint8_t  sw_hist    = 0;
    bool     sw_prev    = false;

    for (;;) {
        __asm__ __volatile__("wdr");      /* ウォッチドッグをリセット */

        uint32_t now = millis();
        if ((uint32_t)(now - last_tick) < 10u) continue;   /* 10msごとに処理 */
        last_tick = now;

        /* ---- 測定 ---- */
        g_vbus_mv    = read_vbus_mv();
        g_current_ma = read_current_ma();
        g_duty       = read_vr_duty();

        /* ---- スイッチ（50msチャタリング除去） ---- */
        sw_hist = (uint8_t)((sw_hist << 1) | (sw_pressed() ? 1u : 0u));
        bool sw_now = ((sw_hist & 0x1Fu) == 0x1Fu);
        bool sw_click = sw_now && !sw_prev;
        if ((sw_hist & 0x1Fu) == 0u) sw_now = false; else if (!sw_now) sw_now = sw_prev;
        sw_prev = sw_now;

        bool blink = ((now / 500u) & 1u) != 0;
        uint32_t in_state = now - state_ms;

        /* ---- 状態ごとの処理 ---- */
        switch (state) {
        case ST_ZERO:
            motor_off();
            pwm_set_all(0);
            if (in_state >= ZERO_SETTLE_MS) {
                if (calibrate_zero()) {
                    state = ST_WAIT_POWER;
                } else {
                    state = ST_FAULT_ZERO;
                }
                state_ms = now;
                vok_ms = 0;
            }
            break;

        case ST_WAIT_POWER:
            motor_off();
            if (pd_good() && g_vbus_mv >= VBUS_START_MV) {
                if (vok_ms < VBUS_OK_TIME_MS) vok_ms += 10u;
            } else {
                vok_ms = 0;
            }
            if (vok_ms >= VBUS_OK_TIME_MS) {
                pwm_set_all(g_duty);
                motor_on();
                power_ms = now;
                oc_ms = 0;
                vng_ms = 0;
                g_stall = false;
                state = ST_RUN;
                state_ms = now;
            }
            break;

        case ST_RUN:
            pwm_set_all(g_duty);

            /* 過電流：一定時間超え続けたら停止（電源ON直後の突入電流は無視） */
            if ((now - power_ms) >= STARTUP_BLANK_MS && g_current_ma > TRIP_CURRENT_MA) {
                if (oc_ms < TRIP_TIME_MS) oc_ms += 10u;
            } else {
                oc_ms = 0;
            }
            if (oc_ms >= TRIP_TIME_MS) {
                motor_off();
                pwm_set_all(0);
                state = ST_FAULT_OC;
                state_ms = now;
                break;
            }

            /* 電圧不足・PG解除：止めて最初からやり直す */
            if (!pd_good() || g_vbus_mv < VBUS_STOP_MV) {
                vng_ms += 10u;
            } else {
                vng_ms = 0;
            }
            if (vng_ms >= 100u) {
                motor_off();
                state = ST_ZERO;
                state_ms = now;
                break;
            }

            if (sw_click) {                 /* スイッチで停止 */
                motor_off();
                pwm_set_all(0);
                state = ST_STOPPED;
                state_ms = now;
            }
            break;

        case ST_STOPPED:
        case ST_FAULT_OC:
        case ST_FAULT_ZERO:
            motor_off();
            pwm_set_all(0);
            if (sw_click) {                 /* スイッチで再起動（ゼロ点も取り直す） */
                state = ST_ZERO;
                state_ms = now;
            }
            break;
        }

        /* ---- 1秒ごと：回転数計算・デバッグ出力 ---- */
        if ((uint32_t)(now - last_sec) >= 1000u) {
            last_sec = now;
            uint16_t cnt[8];
            cli();
            for (uint8_t i = 0; i < 8; i++) { cnt[i] = g_tach_cnt[i]; g_tach_cnt[i] = 0; }
            sei();

            bool stall = false;
            for (uint8_t i = 0; i < 8; i++) {
                g_rpm[i] = (uint16_t)(cnt[i] * (60u / PULSES_PER_REV));
                if ((FAN_PRESENT_MASK & (1u << i)) && g_rpm[i] < STALL_RPM) stall = true;
            }
            g_stall = (state == ST_RUN) && ((now - power_ms) >= STALL_GRACE_MS) && stall;

#if USE_DEBUG_UART
            static const char *const names[] = { "ZERO", "WAIT", "RUN", "STOP", "OC", "ZERR" };
            uart_puts(names[state]);
            uart_puts(" VBUS="); uart_putu(g_vbus_mv);
            uart_puts("mV I=");  uart_putu(g_current_ma);
            uart_puts("mA Z=");  uart_putu(g_zero_sum);
            uart_puts(" D=");    uart_putu(g_duty);
            uart_puts("% RPM=");
            for (uint8_t i = 0; i < 8; i++) {
                uart_putu(g_rpm[i]);
                uart_putc(i < 7 ? ',' : '\r');
            }
            uart_putc('\n');
#endif
        }

        /* ---- LED ---- */
        uint8_t led = 0;
        switch (state) {
        case ST_ZERO:
        case ST_WAIT_POWER: if (blink) led |= LED_BLUE;                 break;
        case ST_RUN:        led |= LED_GREEN; if (g_stall) led |= LED_BLUE; break;
        case ST_STOPPED:    if (blink) led |= LED_GREEN;                break;
        case ST_FAULT_OC:   led |= LED_RED;                             break;
        case ST_FAULT_ZERO: if (blink) led |= LED_RED;                  break;
        }
        PORTC.OUTCLR = (uint8_t)(LED_RED | LED_GREEN | LED_BLUE) & (uint8_t)~led;
        PORTC.OUTSET = led;
    }
}