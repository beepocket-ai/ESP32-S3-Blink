/*
 * ESP32-S3 板载 WS2812 灯珠（GPIO48）多效果演示
 *
 * 按板载 BOOT 键（GPIO0）或外接红色按键（GPIO10）切换到下一个效果，
 * 串口打印当前效果序号与名称。
 *
 * 时序由 RMT 外设产生：
 *   0 码：高 0.3us + 低 0.9us
 *   1 码：高 0.9us + 低 0.3us
 *   数据按 GRB 顺序、MSB 优先发送。
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"

#define WS2812_GPIO                48           /* 板载 RGB 灯珠的数据引脚 */
#define WS2812_LED_COUNT           1            /* 板载只有 1 颗灯珠 */
#define WS2812_RESOLUTION_HZ       10000000     /* RMT 分辨率 10MHz，1 tick = 0.1us */

#define BTN_GPIO_BOOT              0            /* 板载 BOOT 按键 */
#define BTN_GPIO_EXT               10           /* 外接红色按键模块的 OUT，VCC 接 3V3 */
#define BTN_DEBOUNCE_MS            30

#define MAX_LEVEL                  80           /* 单颗灯珠的最亮值，别开满 255；峰值亮度 20% 衰减 */
#define FRAME_MS                   10           /* 每帧刷新间隔 */

static const char *TAG = "ws2812";

static rmt_channel_handle_t s_tx_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
/* 每颗灯珠 3 字节，顺序为 G、R、B */
static uint8_t s_pixels[WS2812_LED_COUNT * 3];

/* ---------------- WS2812 驱动 ---------------- */

static void ws2812_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = WS2812_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = WS2812_RESOLUTION_HZ,
        .mem_block_symbols = 48,       /* ESP32-S3 每通道 48 个 symbol，无需 DMA */
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_tx_chan));

    /* 一个字节 -> 8 个 RMT symbol，高位先发 */
    rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3,            /* 0.3us 高电平 */
            .level1 = 0,
            .duration1 = 9,            /* 0.9us 低电平 */
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9,            /* 0.9us 高电平 */
            .level1 = 0,
            .duration1 = 3,            /* 0.3us 低电平 */
        },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc_cfg, &s_encoder));
    ESP_ERROR_CHECK(rmt_enable(s_tx_chan));

    memset(s_pixels, 0, sizeof(s_pixels));
    ESP_LOGI(TAG, "RMT 初始化完成：GPIO%d，灯珠 %d 颗", WS2812_GPIO, WS2812_LED_COUNT);
}

static void ws2812_set_pixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LED_COUNT) {
        return;
    }
    s_pixels[index * 3 + 0] = g;
    s_pixels[index * 3 + 1] = r;
    s_pixels[index * 3 + 2] = b;
}

static void ws2812_refresh(void)
{
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    ESP_ERROR_CHECK(rmt_transmit(s_tx_chan, s_encoder, s_pixels, sizeof(s_pixels), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_tx_chan, 1000));
    /* 发送后数据线保持低电平，这段低电平即复位信号，至少 280us */
    esp_rom_delay_us(300);
}

/* ---------------- 通用工具 ---------------- */

/* HSV -> RGB，h 单位度，v 为最大亮度 */
static void hsv_to_rgb(uint16_t h, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    h %= 360;
    uint8_t region = (uint8_t)(h / 60);
    uint8_t frac = (uint8_t)((h % 60) * 255 / 60);
    uint8_t q = (uint8_t)((uint32_t)v * (255 - frac) / 255);
    uint8_t t = (uint8_t)((uint32_t)v * frac / 255);

    switch (region) {
    case 0: *r = v; *g = t; *b = 0; break;
    case 1: *r = q; *g = v; *b = 0; break;
    case 2: *r = 0; *g = v; *b = t; break;
    case 3: *r = 0; *g = q; *b = v; break;
    case 4: *r = t; *g = 0; *b = v; break;
    default: *r = v; *g = 0; *b = q; break;
    }
}

/* 三角波：返回 0~255，period 为一个完整往返的毫秒数 */
static uint8_t tri(uint32_t t, uint32_t period)
{
    uint32_t half = period / 2;
    uint32_t x = t % period;
    if (x < half) {
        return (uint8_t)(x * 255 / half);
    }
    return (uint8_t)((period - x) * 255 / half);
}

/* 呼吸用的亮度：三角波 + 二次曲线，低亮度段过渡更柔和 */
static uint8_t breath_level(uint32_t t, uint32_t period)
{
    uint32_t lv = (uint32_t)tri(t, period) * MAX_LEVEL / 255;
    return (uint8_t)(lv * lv / MAX_LEVEL);
}

/* 无状态伪随机哈希，保证同一时间片内颜色稳定 */
static uint32_t hash32(uint32_t x)
{
    x ^= x >> 15;
    x *= 0x2c1b3c6dU;
    x ^= x >> 12;
    x *= 0x297a2d39U;
    x ^= x >> 15;
    return x;
}

/* ---------------- 1. 亮度类 ---------------- */

static void fx_breathing(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t lv = breath_level(t, 1600);
    *r = lv;
    *g = lv;
    *b = 0;                             /* 黄色 */
}

static void fx_solid(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    (void)t;
    *r = 80;
    *g = 80;
    *b = 0;                             /* 黄色常亮 */
}

static void fx_slow_blink(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t lv = ((t % 2000) < 1000) ? 80 : 0;
    *r = lv;
    *g = lv;
    *b = 0;
}

static void fx_fast_blink(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t lv = ((t % 50) < 25) ? 80 : 0;   /* 10Hz */
    *r = lv;
    *g = lv;
    *b = 0;
}

static void fx_heartbeat(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t x = t % 900;
    uint32_t lv = 0;

    if (x < 120) {
        lv = tri(x, 120);               /* 第一下较强 */
    } else if (x >= 260 && x < 380) {
        lv = tri(x - 260, 120);         /* 第二下稍弱 */
    }
    *r = (uint8_t)(lv * 90 / 255);
    *g = 0;
    *b = 0;                             /* 红色心跳 */
}

static void fx_fade_out(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t x = t % 3000;
    uint32_t p;

    if (x < 1200) {
        p = 255 - x * 255 / 1200;       /* 渐暗 */
    } else if (x < 2400) {
        p = 0;                          /* 熄灭停留 */
    } else {
        p = (x - 2400) * 255 / 600;     /* 渐亮 */
    }
    uint8_t lv = (uint8_t)(p * 90 / 255);
    *r = lv;
    *g = lv;
    *b = 0;
}

/* ---------------- 2. 颜色类 ---------------- */

static const uint8_t s_fixed_colors[][3] = {
    {255, 0, 0},     /* 红 */
    {0, 255, 0},     /* 绿 */
    {0, 0, 255},     /* 蓝 */
    {255, 255, 0},   /* 黄 */
    {0, 255, 255},   /* 青 */
    {160, 0, 255},   /* 紫 */
    {255, 255, 255}, /* 白 */
};

static void fx_fixed_colors(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    size_t count = sizeof(s_fixed_colors) / sizeof(s_fixed_colors[0]);
    size_t i = (size_t)((t / 1500) % count);

    *r = (uint8_t)((uint32_t)s_fixed_colors[i][0] * 80 / 255);
    *g = (uint8_t)((uint32_t)s_fixed_colors[i][1] * 80 / 255);
    *b = (uint8_t)((uint32_t)s_fixed_colors[i][2] * 80 / 255);
}

static void fx_hue_cycle(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint16_t h = (uint16_t)((t % 5000) * 360 / 5000);   /* 5 秒一圈 */
    hsv_to_rgb(h, 80, r, g, b);
}

static void fx_color_jump(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t h = hash32(t / 500) % 360;                 /* 每 0.5 秒跳一次 */
    hsv_to_rgb((uint16_t)h, 80, r, g, b);
}

static void fx_two_color(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t f = tri(t, 2400);                          /* 红 <-> 蓝往返 */

    *r = (uint8_t)(90 * (255 - f) / 255);
    *g = 0;
    *b = (uint8_t)(90 * f / 255);
}

static void fx_random_color(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t h = hash32(t / 800 + 0x9e37U) % 360;       /* 每 0.8 秒随机换色 */
    hsv_to_rgb((uint16_t)h, 80, r, g, b);
}

/* ---------------- 3. 组合类 ---------------- */

static void fx_breath_color(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t lv = breath_level(t, 1600);
    uint16_t h = (uint16_t)((t % 6000) * 360 / 6000);   /* 色相缓慢漂移 */
    hsv_to_rgb(h, lv, r, g, b);
}

static void fx_fire(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t slot = t / 60;
    uint32_t h = hash32(slot);
    uint8_t base = (uint8_t)(50 + (h >> 24) * 50 / 255);            /* 50~100 抖动 */
    uint8_t green = (uint8_t)((uint32_t)base * (90 + ((h >> 16) & 0x3f)) / 255);

    *r = base;
    *g = green;                                                     /* 橙黄色焰心 */
    *b = 0;
}

static void fx_police(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if ((t % 1000) < 500) {
        *r = 90;
        *g = 0;
        *b = 0;
    } else {
        *r = 0;
        *g = 0;
        *b = 90;
    }
}

static void fx_lightning(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t x = t % 4000;
    uint8_t lv = 4;                     /* 平时保留一点微光 */

    if (x < 60) {
        lv = 100;
    } else if (x >= 150 && x < 210) {
        lv = 70;
    } else if (x >= 320 && x < 360) {
        lv = 85;
    }
    *r = lv;
    *g = lv;
    *b = lv;
}

static void fx_rainbow_breath(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t lv = breath_level(t, 1200);
    uint16_t h = (uint16_t)((t % 3000) * 360 / 3000);   /* 3 秒一圈彩虹 */
    hsv_to_rgb(h, lv, r, g, b);
}

/* ---------------- 4. 信息指示类 ---------------- */

static void fx_status(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint32_t x = t % 9000;              /* 正常 3s -> 警告 3s -> 故障 3s */

    if (x < 3000) {
        *r = 0;
        *g = 80;
        *b = 0;
    } else if (x < 6000) {
        *r = 80;
        *g = 60;
        *b = 0;
    } else {
        *r = 80;
        *g = 0;
        *b = 0;
    }
}

/* SOS：S = ...  O = ---  S = ... ，每个单位 150ms */
static const uint8_t s_sos_units[] = {
    /* 成对出现：亮几个单位、灭几个单位 */
    1, 1, 1, 1, 1, 3,
    3, 1, 3, 1, 3, 3,
    1, 1, 1, 1, 1, 7,
};

static void fx_sos(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    const uint32_t unit = 150;
    const size_t pairs = sizeof(s_sos_units) / (2 * sizeof(s_sos_units[0]));
    uint32_t total = 0;

    for (size_t i = 0; i < pairs; i++) {
        total += (s_sos_units[i * 2] + s_sos_units[i * 2 + 1]) * unit;
    }

    uint32_t x = t % total;
    uint32_t pos = 0;

    for (size_t i = 0; i < pairs; i++) {
        uint32_t on = s_sos_units[i * 2] * unit;
        uint32_t off = s_sos_units[i * 2 + 1] * unit;

        if (x < pos + on) {
            *r = 70;
            *g = 70;
            *b = 70;
            return;
        }
        pos += on;
        if (x < pos + off) {
            *r = 0;
            *g = 0;
            *b = 0;
            return;
        }
        pos += off;
    }
    *r = 0;
    *g = 0;
    *b = 0;
}

static void fx_pomodoro(uint32_t t, uint8_t *r, uint8_t *g, uint8_t *b)
{
    const uint32_t work_ms = 25 * 60 * 1000;    /* 工作 25 分钟，红色 */
    const uint32_t break_ms = 5 * 60 * 1000;    /* 休息 5 分钟，绿色 */
    uint32_t x = t % (work_ms + break_ms);

    if (x < work_ms) {
        *r = 80;
        *g = 0;
        *b = 0;
    } else {
        *r = 0;
        *g = 80;
        *b = 0;
    }
}

/* ---------------- 效果表 ---------------- */

typedef void (*fx_fn_t)(uint32_t t_ms, uint8_t *r, uint8_t *g, uint8_t *b);

typedef struct {
    const char *name;
    fx_fn_t fn;
} effect_t;

static const effect_t s_effects[] = {
    {"呼吸灯",        fx_breathing},
    {"常亮",          fx_solid},
    {"慢闪",          fx_slow_blink},
    {"快闪",          fx_fast_blink},
    {"心跳",          fx_heartbeat},
    {"渐暗熄灭",      fx_fade_out},
    {"固定色",        fx_fixed_colors},
    {"色相循环",      fx_hue_cycle},
    {"颜色跳变",      fx_color_jump},
    {"双色渐变往返",  fx_two_color},
    {"随机换色",      fx_random_color},
    {"呼吸+变色",     fx_breath_color},
    {"火焰",          fx_fire},
    {"警灯",          fx_police},
    {"闪电",          fx_lightning},
    {"彩虹呼吸",      fx_rainbow_breath},
    {"状态灯",        fx_status},
    {"摩斯码 SOS",    fx_sos},
    {"番茄钟",        fx_pomodoro},
};

#define EFFECT_COUNT (sizeof(s_effects) / sizeof(s_effects[0]))

/* ---------------- 按键 ---------------- */

static void button_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << BTN_GPIO_BOOT) | (1ULL << BTN_GPIO_EXT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));
}

/* ---------------- 主程序 ---------------- */

void app_main(void)
{
    ws2812_init();
    button_init();

    size_t idx = 0;
    int64_t start_ms = esp_timer_get_time() / 1000;

    /* 两个按键各自独立记录状态，互不影响 */
    bool boot_prev;
    bool ext_prev;
    int64_t boot_change_ms;
    int64_t ext_change_ms;

    /* 等按键电平稳定后再建立初始状态，避免上电瞬间被误判成一次按下 */
    vTaskDelay(pdMS_TO_TICKS(100));
    boot_prev = (gpio_get_level(BTN_GPIO_BOOT) == 0);
    ext_prev = (gpio_get_level(BTN_GPIO_EXT) == 0);
    boot_change_ms = esp_timer_get_time() / 1000;
    ext_change_ms = boot_change_ms;

    ESP_LOGI(TAG, "共 %d 个效果，按 BOOT 或红色按键 切换", (int)EFFECT_COUNT);
    ESP_LOGI(TAG, "效果 %d/%d：%s", (int)(idx + 1), (int)EFFECT_COUNT, s_effects[idx].name);

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        uint32_t t_ms = (uint32_t)(now_ms - start_ms);

        uint8_t r = 0, g = 0, b = 0;
        s_effects[idx].fn(t_ms, &r, &g, &b);
        ws2812_set_pixel(0, r, g, b);
        ws2812_refresh();

        /* 按键：两个引脚分开判断，任一出现高 -> 低跳变就切换 */
        bool boot_low = (gpio_get_level(BTN_GPIO_BOOT) == 0);
        bool ext_low = (gpio_get_level(BTN_GPIO_EXT) == 0);
        bool advance = false;

        if (boot_low != boot_prev && (now_ms - boot_change_ms) >= BTN_DEBOUNCE_MS) {
            boot_change_ms = now_ms;
            boot_prev = boot_low;
            ESP_LOGI(TAG, "BOOT 键电平=%d", (int)boot_low);
            if (boot_low) {
                advance = true;
            }
        }

        if (ext_low != ext_prev && (now_ms - ext_change_ms) >= BTN_DEBOUNCE_MS) {
            ext_change_ms = now_ms;
            ext_prev = ext_low;
            ESP_LOGI(TAG, "红色按键电平=%d", (int)ext_low);
            if (ext_low) {
                advance = true;
            }
        }

        if (advance) {
            idx = (idx + 1) % EFFECT_COUNT;
            start_ms = now_ms;
            ESP_LOGI(TAG, "效果 %d/%d：%s", (int)(idx + 1), (int)EFFECT_COUNT, s_effects[idx].name);
        }

        vTaskDelay(pdMS_TO_TICKS(FRAME_MS));
    }
}
