// SC2004C (20x4 キャラクタ液晶) を Raspberry Pi Pico で動かす C プログラム (Pico SDK)
// 4ビットモード / R/W は GND 直結(書き込み専用)
//
// 配線:
//   RS  -> GP16 (物理21)   E   -> GP17 (物理22)
//   DB4 -> GP18 (物理24)   DB5 -> GP19 (物理25)
//   DB6 -> GP20 (物理26)   DB7 -> GP21 (物理27)
//   VSS, R/W -> GND (Pico と外部電源の GND を共通に)

#include <stdio.h>
#include "pico/stdlib.h"

#define PIN_RS 16
#define PIN_E  17
static const uint PIN_DATA[4] = {18, 19, 20, 21};  // DB4, DB5, DB6, DB7

// 各行の先頭 DDRAM アドレス (20x4 液晶)
static const uint8_t ROW_OFFSETS[4] = {0x00, 0x40, 0x14, 0x54};

// ---- 低レベル処理 ----
static void lcd_pulse(void) {
    // E を H→L にした瞬間に液晶がデータを取り込む
    gpio_put(PIN_E, 1);
    sleep_us(1);
    gpio_put(PIN_E, 0);
    sleep_us(50);  // 通常の命令は約37us で完了
}

static void lcd_write4(uint8_t nibble) {
    // 4ビットを DB4〜DB7 に出力
    for (int i = 0; i < 4; i++) {
        gpio_put(PIN_DATA[i], (nibble >> i) & 1);
    }
    lcd_pulse();
}

static void lcd_write8(uint8_t value, bool rs) {
    // 1バイトを上位→下位の順に2回に分けて送る
    gpio_put(PIN_RS, rs);
    lcd_write4(value >> 4);
    lcd_write4(value & 0x0F);
}

void lcd_command(uint8_t cmd) {
    lcd_write8(cmd, false);           // RS=0: 命令
    if (cmd == 0x01 || cmd == 0x02) { // クリア/ホームは時間がかかる
        sleep_ms(2);
    }
}

void lcd_write_char(uint8_t code) {
    lcd_write8(code, true);           // RS=1: 文字データ
}

// ---- 初期化 ----
void lcd_init(void) {
    gpio_init(PIN_RS); gpio_set_dir(PIN_RS, GPIO_OUT); gpio_put(PIN_RS, 0);
    gpio_init(PIN_E);  gpio_set_dir(PIN_E,  GPIO_OUT); gpio_put(PIN_E,  0);
    for (int i = 0; i < 4; i++) {
        gpio_init(PIN_DATA[i]);
        gpio_set_dir(PIN_DATA[i], GPIO_OUT);
        gpio_put(PIN_DATA[i], 0);
    }

    sleep_ms(50);  // 電源投入後の安定待ち

    // 8ビットモードで起動しているので、4ビットモードへ切り替える
    lcd_write4(0x03); sleep_ms(5);
    lcd_write4(0x03); sleep_us(150);
    lcd_write4(0x03); sleep_us(150);
    lcd_write4(0x02); sleep_us(150);

    lcd_command(0x28);  // 4ビット, 2ライン(4行液晶もこの設定), 5x8ドット
    lcd_command(0x08);  // 表示OFF
    lcd_command(0x01);  // 画面クリア
    lcd_command(0x06);  // 書き込み後カーソルを右へ移動
    lcd_command(0x0C);  // 表示ON, カーソル非表示
}

// ---- 便利関数 ----
void lcd_clear(void) {
    lcd_command(0x01);
}

void lcd_move_to(uint8_t col, uint8_t row) {
    lcd_command(0x80 | (ROW_OFFSETS[row & 3] + col));
}

// 文字列表示 (UTF-8 の半角カタカナ ｱｲｳ… は液晶の文字コード 0xA1〜0xDF に変換)
void lcd_print(const char *s) {
    const uint8_t *p = (const uint8_t *)s;
    while (*p) {
        if (p[0] == 0xEF && p[1] == 0xBD && p[2] >= 0xA1 && p[2] <= 0xBF) {
            lcd_write_char(p[2]);            // U+FF61〜FF7F → 0xA1〜0xBF
            p += 3;
        } else if (p[0] == 0xEF && p[1] == 0xBE && p[2] >= 0x80 && p[2] <= 0x9F) {
            lcd_write_char(p[2] + 0x40);     // U+FF80〜FF9F → 0xC0〜0xDF
            p += 3;
        } else {
            lcd_write_char(*p);
            p++;
        }
    }
}

// CGRAM に自作文字を登録 (location: 0〜7, pattern: 8行分)
void lcd_create_char(uint8_t location, const uint8_t pattern[8]) {
    lcd_command(0x40 | ((location & 0x07) << 3));
    for (int i = 0; i < 8; i++) {
        lcd_write_char(pattern[i] & 0x1F);
    }
    lcd_move_to(0, 0);  // DDRAM に戻す
}

// ---------------- デモ ----------------
int main(void) {
    stdio_init_all();
    lcd_init();

    // 自作文字: ハートを 0 番に登録
    static const uint8_t heart[8] = {
        0b00000,
        0b01010,
        0b11111,
        0b11111,
        0b01110,
        0b00100,
        0b00000,
        0b00000,
    };
    lcd_create_char(0, heart);

    lcd_move_to(0, 0);
    lcd_print("Hello, Pico!");
    lcd_move_to(0, 1);
    lcd_print("SC2004C 4bit mode");
    lcd_move_to(0, 2);
    lcd_print("ｶﾀｶﾅ ﾓ ﾋｮｳｼﾞ ﾃﾞｷﾏｽ ");
    lcd_write_char(0);  // 自作文字(ハート)を表示

    char buf[21];
    uint32_t count = 0;
    while (true) {
        snprintf(buf, sizeof(buf), "Count: %-13lu", (unsigned long)count);
        lcd_move_to(0, 3);
        lcd_print(buf);
        count++;
        sleep_ms(1000);
    }
}