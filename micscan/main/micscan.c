// Mic pin scanner for Spotpear ESP32-S3 Otto robot.
// Plays a 1 kHz tone on the speaker (I2S1) and tries mic pin combos on I2S0,
// ranking them by how much of the captured signal is the 1 kHz tone.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define RATE        16000
#define TONE_HZ     1000
#define TONE_AMP    6000
#define N_FRAMES    2048
#define DISCARD_FR  960     // 60 ms
#define HIT_RATIO   0.2

#define SPK_DOUT 7
#define SPK_BCLK 15
#define SPK_WS   16

static const int cand[] = {1, 2, 4, 5, 6, 13, 14, 40, 41, 42, 45, 47, 48};
#define NCAND (sizeof(cand) / sizeof(cand[0]))

typedef enum { M_PDM, M_STD } mode_t_;

typedef struct {
    mode_t_ mode;
    int a, b, c;          // PDM: a=clk b=din ; STD: a=bclk b=ws c=din
    double rms[2], ratio[2];
} result_t;

#define MAX_RESULTS 2200
static result_t results[MAX_RESULTS];
static int nresults;

static i2s_chan_handle_t tx_chan;
static volatile bool tone_on = true;

static int32_t rxbuf[N_FRAMES * 2];    // big enough for 32-bit stereo
static double chbuf[N_FRAMES];

// ---------------- speaker tone ----------------
static void tone_task(void *arg)
{
    static int16_t buf[RATE / TONE_HZ * 10 * 2];   // 10 periods, stereo
    int frames = sizeof(buf) / 4;
    for (int i = 0; i < frames; i++) {
        int16_t v = (int16_t)(TONE_AMP * sinf(2 * M_PI * TONE_HZ * i / RATE));
        buf[2 * i] = v;
        buf[2 * i + 1] = v;
    }
    size_t w;
    while (tone_on) {
        i2s_channel_write(tx_chan, buf, sizeof(buf), &w, portMAX_DELAY);
    }
    memset(buf, 0, sizeof(buf));
    for (int i = 0; i < 20; i++) i2s_channel_write(tx_chan, buf, sizeof(buf), &w, portMAX_DELAY);
    i2s_channel_disable(tx_chan);
    vTaskDelete(NULL);
}

static void start_tone(void)
{
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    cc.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&cc, &tx_chan, NULL));
    i2s_std_config_t sc = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, .bclk = SPK_BCLK, .ws = SPK_WS,
            .dout = SPK_DOUT, .din = I2S_GPIO_UNUSED,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &sc));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    xTaskCreatePinnedToCore(tone_task, "tone", 4096, NULL, 10, NULL, 1);
}

// ---------------- analysis ----------------
static void analyse(const double *x, int n, double *rms_out, double *ratio_out)
{
    double mean = 0;
    for (int i = 0; i < n; i++) mean += x[i];
    mean /= n;
    double e = 0, s1 = 0, s2 = 0;
    double coeff = 2 * cos(2 * M_PI * TONE_HZ / RATE);
    for (int i = 0; i < n; i++) {
        double v = x[i] - mean;
        e += v * v;
        double s0 = v + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    double p = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    *rms_out = sqrt(e / n);
    *ratio_out = e > 0 ? 2 * p / (n * e) : 0;
}

static void reset_pins(int a, int b, int c)
{
    if (a >= 0) gpio_reset_pin(a);
    if (b >= 0) gpio_reset_pin(b);
    if (c >= 0) gpio_reset_pin(c);
}

static i2s_chan_handle_t open_rx(mode_t_ m, int a, int b, int c)
{
    i2s_chan_handle_t rx = NULL;
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    cc.dma_frame_num = 256;
    if (i2s_new_channel(&cc, NULL, &rx) != ESP_OK) return NULL;
    esp_err_t err;
    if (m == M_PDM) {
        i2s_pdm_rx_config_t pc = {
            .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(RATE),
            .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = { .clk = a, .din = b },
        };
        err = i2s_channel_init_pdm_rx_mode(rx, &pc);
    } else {
        i2s_std_config_t sc = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(RATE),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED, .bclk = a, .ws = b,
                .dout = I2S_GPIO_UNUSED, .din = c,
            },
        };
        err = i2s_channel_init_std_mode(rx, &sc);
    }
    if (err == ESP_OK) err = i2s_channel_enable(rx);
    if (err != ESP_OK) {
        i2s_del_channel(rx);
        return NULL;
    }
    return rx;
}

static void close_rx(i2s_chan_handle_t rx, int a, int b, int c)
{
    i2s_channel_disable(rx);
    i2s_del_channel(rx);
    reset_pins(a, b, c);
}

// Reads n frames into rxbuf; returns bytes per sample.
static int read_frames(i2s_chan_handle_t rx, mode_t_ m, int n)
{
    int bps = (m == M_PDM) ? 2 : 4;
    size_t want = (size_t)n * 2 * bps, got = 0;
    uint8_t *p = (uint8_t *)rxbuf;
    while (got < want) {
        size_t r = 0;
        if (i2s_channel_read(rx, p + got, want - got, &r, pdMS_TO_TICKS(500)) != ESP_OK) break;
        got += r;
    }
    if (got < want) memset(p + got, 0, want - got);
    return bps;
}

static void extract(mode_t_ m, int ch, int n)
{
    if (m == M_PDM) {
        int16_t *s = (int16_t *)rxbuf;
        for (int i = 0; i < n; i++) chbuf[i] = s[2 * i + ch];
    } else {
        for (int i = 0; i < n; i++) chbuf[i] = rxbuf[2 * i + ch] / 65536.0;   // scale to ~16-bit
    }
}

static void describe(const result_t *r, char *out, size_t len)
{
    if (r->mode == M_PDM)
        snprintf(out, len, "PDM clk=%-2d din=%-2d        ", r->a, r->b);
    else
        snprintf(out, len, "STD bclk=%-2d ws=%-2d din=%-2d", r->a, r->b, r->c);
}

static double score(const result_t *r)
{
    return fmax(r->ratio[0], r->ratio[1]);
}

static void test_combo(mode_t_ m, int a, int b, int c)
{
    i2s_chan_handle_t rx = open_rx(m, a, b, c);
    if (!rx) {
        printf("  init failed: %s %d %d %d\n", m == M_PDM ? "PDM" : "STD", a, b, c);
        reset_pins(a, b, c);
        return;
    }
    read_frames(rx, m, DISCARD_FR);
    read_frames(rx, m, N_FRAMES);
    close_rx(rx, a, b, c);

    if (nresults >= MAX_RESULTS) return;
    result_t *r = &results[nresults++];
    r->mode = m; r->a = a; r->b = b; r->c = c;
    for (int ch = 0; ch < 2; ch++) {
        extract(m, ch, N_FRAMES);
        analyse(chbuf, N_FRAMES, &r->rms[ch], &r->ratio[ch]);
    }
    if ((r->ratio[0] > HIT_RATIO && r->rms[0] > 1) || (r->ratio[1] > HIT_RATIO && r->rms[1] > 1)) {
        char d[64];
        describe(r, d, sizeof(d));
        printf("  HIT %s  L: rms=%8.1f ratio=%.3f  R: rms=%8.1f ratio=%.3f\n",
               d, r->rms[0], r->ratio[0], r->rms[1], r->ratio[1]);
    }
}

static int cmp_result(const void *x, const void *y)
{
    const result_t *a = x, *b = y;
    double sa = score(a), sb = score(b);
    if (sa != sb) return sa < sb ? 1 : -1;
    double ra = fmax(a->rms[0], a->rms[1]), rb = fmax(b->rms[0], b->rms[1]);
    return ra < rb ? 1 : (ra > rb ? -1 : 0);
}

static void print_top(void)
{
    printf("\n===== TOP 10 (score = max 1 kHz ratio of L/R) =====\n");
    for (int i = 0; i < nresults && i < 10; i++) {
        char d[64];
        describe(&results[i], d, sizeof(d));
        printf("%2d. %s  L: rms=%8.1f ratio=%.3f  R: rms=%8.1f ratio=%.3f\n", i + 1, d,
               results[i].rms[0], results[i].ratio[0], results[i].rms[1], results[i].ratio[1]);
    }
    printf("====================================================\n\n");
}

static bool have_hit(void)
{
    for (int i = 0; i < nresults; i++) {
        const result_t *r = &results[i];
        if ((r->ratio[0] > HIT_RATIO && r->rms[0] > 1) || (r->ratio[1] > HIT_RATIO && r->rms[1] > 1)) return true;
    }
    return false;
}

void app_main(void)
{
    esp_log_level_set("i2s_common", ESP_LOG_NONE);
    esp_log_level_set("i2s_std", ESP_LOG_NONE);
    esp_log_level_set("i2s_pdm", ESP_LOG_NONE);
    esp_log_level_set("gpio", ESP_LOG_NONE);

    for (int i = 5; i > 0; i--) {
        printf("micscan: starting in %d s...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    start_tone();
    vTaskDelay(pdMS_TO_TICKS(300));

    printf("\n--- Phase 1: PDM RX (clk x din), clk=5 first ---\n");
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < NCAND; i++) {
            int clk = cand[i];
            if ((pass == 0) != (clk == 5)) continue;
            printf(" clk=%d\n", clk);
            for (int j = 0; j < NCAND; j++)
                if (cand[j] != clk) test_combo(M_PDM, clk, cand[j], -1);
        }
    }

    printf("\n--- Phase 2a: STD, GPIO5 = BCLK ---\n");
    for (int i = 0; i < NCAND; i++)
        for (int j = 0; j < NCAND; j++) {
            int ws = cand[i], din = cand[j];
            if (ws == 5 || din == 5 || ws == din) continue;
            test_combo(M_STD, 5, ws, din);
        }
    printf("\n--- Phase 2b: STD, GPIO5 = WS ---\n");
    for (int i = 0; i < NCAND; i++)
        for (int j = 0; j < NCAND; j++) {
            int bclk = cand[i], din = cand[j];
            if (bclk == 5 || din == 5 || bclk == din) continue;
            test_combo(M_STD, bclk, 5, din);
        }
    printf("\n--- Phase 3: STD, GPIO5 = DIN ---\n");
    for (int i = 0; i < NCAND; i++)
        for (int j = 0; j < NCAND; j++) {
            int bclk = cand[i], ws = cand[j];
            if (bclk == 5 || ws == 5 || bclk == ws) continue;
            test_combo(M_STD, bclk, ws, 5);
        }

    if (!have_hit()) {
        printf("\n--- Phase 4: no hit yet, full STD sweep (~8 min) ---\n");
        for (int i = 0; i < NCAND; i++) {
            printf(" bclk=%d\n", cand[i]);
            for (int j = 0; j < NCAND; j++)
                for (int k = 0; k < NCAND; k++) {
                    int bclk = cand[i], ws = cand[j], din = cand[k];
                    if (bclk == 5 || ws == 5 || din == 5) continue;   // already covered
                    if (bclk == ws || bclk == din || ws == din) continue;
                    test_combo(M_STD, bclk, ws, din);
                }
        }
    }

    tone_on = false;
    vTaskDelay(pdMS_TO_TICKS(500));

    qsort(results, nresults, sizeof(result_t), cmp_result);
    printf("\nScan done: %d combos tested.\n", nresults);
    print_top();

    // Live monitor on the best combo: talk / clap to confirm.
    result_t best = results[0];
    char d[64];
    describe(&best, d, sizeof(d));
    int ch = best.ratio[1] > best.ratio[0] ? 1 : 0;
    printf("LIVE on %s (%s channel). Talk/clap now.\n", d, ch ? "RIGHT" : "LEFT");
    i2s_chan_handle_t rx = open_rx(best.mode, best.a, best.b, best.mode == M_STD ? best.c : -1);
    if (!rx) {
        printf("could not reopen best combo\n");
        return;
    }
    const int n = RATE / 10;   // 100 ms
    int tick = 0;
    while (1) {
        read_frames(rx, best.mode, n);
        double rl, rr, dummy;
        extract(best.mode, 0, n);
        analyse(chbuf, n, &rl, &dummy);
        extract(best.mode, 1, n);
        analyse(chbuf, n, &rr, &dummy);
        double r = ch ? rr : rl;
        int bar = (int)(20 * log10(r + 1));   // ~dB
        if (bar > 90) bar = 90;
        char bars[91];
        memset(bars, '#', bar);
        bars[bar] = 0;
        printf("L=%8.1f R=%8.1f |%s\n", rl, rr, bars);
        if (++tick % 100 == 0) {   // re-print the ranking every ~10 s for late listeners
            print_top();
            printf("LIVE on %s (%s channel)\n", d, ch ? "RIGHT" : "LEFT");
        }
    }
}
