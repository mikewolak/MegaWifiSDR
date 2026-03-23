/**
 * @file main.c
 * @brief MegaWifi SDR Receiver for Sega Genesis
 *
 * Copyright (c) 2026 Mike Wolak <mikewolak@gmail.com>
 * All rights reserved.
 *
 * KiwiSDR front end with VU meters, reverb, pixel-smooth marquee.
 * Audio decoded and mixed on ESP32-C3, only controls over LSD.
 */

#include <genesis.h>
#include <task.h>
#include <string.h>
#include "ext/mw/megawifi.h"
#include "ext/mw/mw-msg.h"
#include "reverb_ctrl.h"

extern void mw_set_draw_hook(void (*hook)(void));

extern enum mw_err mw_aud_play(void);
extern enum mw_err mw_aud_stop(void);
extern enum mw_err mw_aud_pause(void);
extern enum mw_err mw_aud_resume(void);
extern enum mw_err mw_aud_status(uint32_t *vu_word, uint32_t *pos_word);
extern enum mw_err mw_aud_set_vol(uint8_t vol);
extern enum mw_err mw_aud_cpu_pct(uint8_t *pct);

/* SDR commands */
extern enum mw_err mw_sdr_connect(const char *host, uint16_t port);
extern enum mw_err mw_sdr_tune(uint32_t freq_hz, uint8_t mode, uint16_t bandwidth);
extern enum mw_err mw_sdr_disconnect(void);
extern enum mw_err mw_sdr_status_poll(uint32_t *w0, uint32_t *w1);

#define FPS 60

/* WiFi — uses credentials already stored in flash slot 0 */
#define AP_SLOT     0

/* Default KiwiSDR server — try multiple if first is down */
#define SDR_DEFAULT_HOST  "sdr.hfunderground.com"
#define SDR_DEFAULT_PORT  8073

/* ── Layout ──────────────────────────────────────────────────────────────── */
#define ROW_TITLE       0
#define ROW_REVERB      1
#define ROW_HOST        2
#define ROW_STATUS      3
#define ROW_PRESET      4
#define ROW_VU_LABEL    5
#define ROW_VU_START    6
#define ROW_VU_END      21
#define ROW_VOL_LABEL   22
#define ROW_VOL_BAR     23
#define ROW_HELP        24
#define ROW_CPU         25
#define ROW_MARQUEE     26
#define ROW_FOOTER      27

#define VU_NUM_ROWS     16
#define VU_TOTAL_PX     128
#define VU_BOTTOM_Y     (ROW_VU_END * 8 + 7)
#define VU_CH_PX_WIDTH  32
#define VU_CH_X(ch)     (24 + (ch) * 72)
#define VOL_BAR_OFFSET  7
#define VOL_BAR_WIDTH   26

#define T_BLACK         (TILE_USER_INDEX + 0)
#define T_BLUE          (TILE_USER_INDEX + 1)
#define T_SPR_GREEN     (TILE_USER_INDEX + 2)
#define T_SPR_YELLOW    (TILE_USER_INDEX + 6)
#define T_SPR_RED       (TILE_USER_INDEX + 10)
#define T_SPR_WHITE     (TILE_USER_INDEX + 14)

#define VU_SPR_PER_CH   16
#define VU_SPR_TOTAL    (4 * (VU_SPR_PER_CH + 1))

/* ── SDR modes ───────────────────────────────────────────────────────────── */
#define MODE_AM     0
#define MODE_USB    1
#define MODE_LSB    2
#define MODE_CW     3
#define MODE_FM     4
#define NUM_MODES   5

static const char *mode_names[NUM_MODES] = { "AM", "USB", "LSB", "CW", "FM" };
static const u16 mode_default_bw[NUM_MODES] = { 6000, 2800, 2800, 500, 15000 };
static const u32 mode_coarse_step[NUM_MODES] = { 5000, 1000, 1000, 100, 100000 };
static const u32 mode_fine_step[NUM_MODES]   = { 1000,  100,  100,  10,  10000 };

/* ── Band presets ────────────────────────────────────────────────────────── */
typedef struct { u32 freq_hz; u8 mode; char desc[30]; } band_preset_t;

static const band_preset_t bands[] = {
    {  5000000, MODE_AM,  "WWV Time Signal 5 MHz"     },
    { 10000000, MODE_AM,  "WWV Time Signal 10 MHz"    },
    { 15000000, MODE_AM,  "WWV Time Signal 15 MHz"    },
    {  3550000, MODE_CW,  "80m CW / Morse Code"       },
    {  3750000, MODE_LSB, "80m Voice LSB"              },
    {  7030000, MODE_CW,  "40m CW / Morse Code"       },
    {  7200000, MODE_LSB, "40m Voice LSB"              },
    {  7290000, MODE_LSB, "40m Ragchew"                },
    { 14060000, MODE_CW,  "20m CW / Morse Code"       },
    { 14200000, MODE_USB, "20m Voice USB"              },
    { 14300000, MODE_USB, "20m Emergency Net"          },
    { 21060000, MODE_CW,  "15m CW / Morse Code"       },
    { 21200000, MODE_USB, "15m Voice USB"              },
    { 28400000, MODE_USB, "10m Voice USB"              },
    {  9420000, MODE_AM,  "BBC World Service 31m"      },
    {  6195000, MODE_AM,  "Shortwave Broadcast 49m"   },
    { 11735000, MODE_AM,  "Shortwave Broadcast 25m"   },
    {  4840000, MODE_AM,  "WWCR Nashville"             },
};
#define NUM_BANDS   18

/* ── State ───────────────────────────────────────────────────────────────── */
static u32 g_freq_hz = 10000000;   /* WWV 10 MHz — 24/7 time signal */
static u8  g_mode = MODE_AM;
static u16 g_bandwidth = 6000;
static u8  g_band = 1;             /* WWV 10 MHz */
static bool g_fine_tune = FALSE;
static bool g_tune_pending = FALSE;   /* set by input, sent by main loop */

/* ── Scan state ──────────────────────────────────────────────────────────── */
static bool g_scanning = FALSE;
static u16  g_scan_timer = 0;
static u16  g_scan_pause = 0;         /* pause countdown when signal found */
#define SCAN_DWELL_FRAMES   (3 * 60)  /* 3 seconds per preset */
#define SCAN_PAUSE_FRAMES   (5 * 60)  /* 5 seconds pause on signal */
#define SCAN_SQUELCH        300       /* S-meter threshold (raw) */
static u8  g_master_vol = 255;
static u8  g_vu_raw[4] = {0};
static u8  g_vu_px[4] = {0};
static u8  g_vu_peak[4] = {0};
static u8  g_vu_decay[4] = {0};
static u8  g_cpu_pct = 0;
static bool g_mw_connected = FALSE;

static uint16_t cmd_buf[MW_BUFLEN / 2];

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static char *itoa_simple(u32 val, char *buf)
{
    char tmp[12]; int i = 0;
    if (val == 0) { *buf++ = '0'; *buf = '\0'; return buf; }
    while (val > 0) { tmp[i++] = '0' + (val % 10); val /= 10; }
    while (i > 0) *buf++ = tmp[--i];
    *buf = '\0'; return buf;
}

static void format_freq(u32 hz, char *buf)
{
    u32 mhz = hz / 1000000;
    u32 khz = (hz / 1000) % 1000;
    char *p = buf;
    p = itoa_simple(mhz, p);
    *p++ = '.';
    if (khz < 100) *p++ = '0';
    if (khz < 10) *p++ = '0';
    p = itoa_simple(khz, p);
    memcpy(p, " MHz", 4); p[4] = '\0';
}

static void format_bw(u16 bw, char *buf)
{
    char *p = buf;
    memcpy(p, "BW:", 3); p += 3;
    if (bw >= 1000) {
        p = itoa_simple(bw / 1000, p); *p++ = '.';
        p = itoa_simple((bw % 1000) / 100, p); *p++ = 'k';
    } else { p = itoa_simple(bw, p); }
    *p = '\0';
}

/* ── Tiles + palettes ────────────────────────────────────────────────────── */

static void make_solid(u32 *t, u8 c)
{
    u32 r = 0; u8 i;
    for (i = 0; i < 8; i++) r |= ((u32)c << (28 - i * 4));
    for (i = 0; i < 8; i++) t[i] = r;
}

static void load_tiles(void)
{
    u32 t[8]; u8 i;
    make_solid(t, 0); VDP_loadTileData(t, T_BLACK, 1, CPU);
    make_solid(t, 5); VDP_loadTileData(t, T_BLUE, 1, CPU);
    make_solid(t, 1);
    for (i = 0; i < 4; i++) VDP_loadTileData(t, T_SPR_GREEN + i, 1, CPU);
    make_solid(t, 2);
    for (i = 0; i < 4; i++) VDP_loadTileData(t, T_SPR_YELLOW + i, 1, CPU);
    make_solid(t, 3);
    for (i = 0; i < 4; i++) VDP_loadTileData(t, T_SPR_RED + i, 1, CPU);
    make_solid(t, 4);
    for (i = 0; i < 4; i++) VDP_loadTileData(t, T_SPR_WHITE + i, 1, CPU);
}

static void setup_palettes(void)
{
    PAL_setColor(0,  RGB24_TO_VDPCOLOR(0x000000));
    PAL_setColor(15, RGB24_TO_VDPCOLOR(0xFFFFFF));
    PAL_setColor(16, RGB24_TO_VDPCOLOR(0x000000));
    PAL_setColor(17, RGB24_TO_VDPCOLOR(0x00DD44));
    PAL_setColor(18, RGB24_TO_VDPCOLOR(0xDDCC00));
    PAL_setColor(19, RGB24_TO_VDPCOLOR(0xEE2222));
    PAL_setColor(20, RGB24_TO_VDPCOLOR(0xFFFFFF));
    PAL_setColor(21, RGB24_TO_VDPCOLOR(0x2288EE));
    PAL_setColor(31, RGB24_TO_VDPCOLOR(0x00FF66));
    PAL_setColor(32, RGB24_TO_VDPCOLOR(0x000000));
    PAL_setColor(47, RGB24_TO_VDPCOLOR(0xFF4444));
    PAL_setColor(48, RGB24_TO_VDPCOLOR(0x000000));
    PAL_setColor(63, RGB24_TO_VDPCOLOR(0x00CCFF));
}

/* ── VU sprites ──────────────────────────────────────────────────────────── */

/* ── Volume ──────────────────────────────────────────────────────────────── */

static void draw_volume_bar(void)
{
    u8 filled = (g_master_vol * VOL_BAR_WIDTH + 127) / 255; u8 c;
    for (c = 0; c < VOL_BAR_WIDTH; c++) {
        u16 tile = (c < filled)
            ? TILE_ATTR_FULL(PAL1, TRUE, FALSE, FALSE, T_BLUE)
            : TILE_ATTR_FULL(PAL1, TRUE, FALSE, FALSE, T_BLACK);
        VDP_setTileMapXY(BG_A, tile, VOL_BAR_OFFSET + c, ROW_VOL_BAR);
    }
}

static void draw_volume_text(void)
{
    char buf[8]; char *p = buf;
    u8 pct = (g_master_vol * 100 + 127) / 255;
    p = itoa_simple(pct, p); *p++ = '%'; *p++ = ' '; *p++ = ' '; *p = '\0';
    VDP_setTextPalette(PAL3);
    VDP_drawText(buf, VOL_BAR_OFFSET + VOL_BAR_WIDTH + 1, ROW_VOL_LABEL);
    VDP_setTextPalette(PAL0);
}

/* ── Reverb + CPU ────────────────────────────────────────────────────────── */

static void draw_reverb_status(void)
{
    bool on = reverb_ctrl_is_enabled();
    VDP_setTextPalette(on ? PAL1 : PAL3);
    VDP_drawText(on ? "  Reverb: ON   [START] Settings " :
                      "  Reverb: OFF  [START] Settings ", 0, ROW_REVERB);
    VDP_setTextPalette(PAL0);
}

/* ── SDR state display ────────────────────────────────────────────────────── */

static u8  g_sdr_state = 0;
static u16 g_smeter = 0;
static u8  g_sdr_error = 0;
static u8  s_prev_band = 0xFF;

static void draw_preset_info(void)
{
    char buf[41];
    u8 pos;
    u8 idx;

    if (g_band == s_prev_band) return;
    s_prev_band = g_band;

    /* Bounds check */
    idx = g_band;
    if (idx >= NUM_BANDS) idx = 0;

    /* Build string safely — never write past buf[39] */
    memset(buf, ' ', 40);
    buf[40] = '\0';

    /* "[N/M] description" starting at column 2 */
    pos = 2;
    buf[pos++] = '[';
    if (idx + 1 >= 10) buf[pos++] = '0' + ((idx + 1) / 10);
    buf[pos++] = '0' + ((idx + 1) % 10);
    buf[pos++] = '/';
    if (NUM_BANDS >= 10) buf[pos++] = '0' + (NUM_BANDS / 10);
    buf[pos++] = '0' + (NUM_BANDS % 10);
    buf[pos++] = ']';
    buf[pos++] = ' ';

    /* Copy description, truncate at column 39 */
    {
        const char *d = bands[idx].desc;
        u8 i;
        for (i = 0; d[i] && pos < 39; i++)
            buf[pos++] = d[i];
    }

    VDP_setTextPalette(PAL3);
    VDP_drawText(buf, 0, ROW_PRESET);
    VDP_setTextPalette(PAL0);
}



/* ── Scan logic ──────────────────────────────────────────────────────────── */

static void update_scan(void)
{
    if (!g_scanning) return;

    /* If paused on a signal, count down */
    if (g_scan_pause > 0) {
        g_scan_pause--;
        /* If signal drops below threshold, resume immediately */
        if (g_smeter < SCAN_SQUELCH)
            g_scan_pause = 0;
        return;
    }

    g_scan_timer++;
    if (g_scan_timer < SCAN_DWELL_FRAMES) {
        /* Check if we found a signal — pause */
        if (g_smeter >= SCAN_SQUELCH) {
            g_scan_pause = SCAN_PAUSE_FRAMES;
            g_scan_timer = 0;
        }
        return;
    }

    /* Dwell time expired — advance to next preset */
    g_scan_timer = 0;
    g_band++;
    if (g_band >= NUM_BANDS) g_band = 0;
    g_freq_hz = bands[g_band].freq_hz;
    g_mode = bands[g_band].mode;
    if (g_mode >= NUM_MODES) g_mode = MODE_AM;
    g_bandwidth = mode_default_bw[g_mode];
    g_tune_pending = TRUE;
}

static void draw_scan_indicator(void)
{
    if (g_scanning) {
        VDP_setTextPalette(g_scan_pause > 0 ? PAL1 : PAL3);
        if (g_scan_pause > 0)
            VDP_drawText("SCAN:SIGNAL ", 28, ROW_STATUS);
        else
            VDP_drawText("SCAN:       ", 28, ROW_STATUS);
        VDP_setTextPalette(PAL0);
    } else {
        VDP_drawText("            ", 28, ROW_STATUS);
    }
}

static u8 s_prev_cpu = 0xFF;
static void draw_cpu(void)
{
    char buf[41]; u16 pal; char *p;
    if (g_cpu_pct == s_prev_cpu) return;
    s_prev_cpu = g_cpu_pct;
    if (g_cpu_pct >= 90) pal = PAL2;
    else if (g_cpu_pct >= 76) pal = PAL3;
    else pal = PAL1;
    memset(buf, ' ', 40); buf[40] = '\0';
    p = buf + 2; memcpy(p, "ESP32-C3 CPU: ", 14); p += 14;
    p = itoa_simple(g_cpu_pct, p); *p++ = '%';
    VDP_setTextPalette(pal); VDP_drawText(buf, 0, ROW_CPU); VDP_setTextPalette(PAL0);
}

/* ── Static UI ───────────────────────────────────────────────────────────── */

static void draw_static_ui(void)
{
    u8 ch; char label[41];
    VDP_setTextPalette(PAL3);
    VDP_drawText("====== MEGAWIFI SDR RECEIVER ======", 2, ROW_TITLE);
    draw_reverb_status();
    (void)ch; (void)label;
    VDP_drawText("  Vol:", 0, ROW_VOL_LABEL);
    VDP_drawText(" [A]Scan [B]Mode [C]Band [LR]Vol   ", 1, ROW_HELP);
    VDP_drawText("==================================", 2, ROW_FOOTER);
    VDP_setTextPalette(PAL0);
}

/* ── SDR status ──────────────────────────────────────────────────────────── */

static u32 s_prev_freq = 0;
static u8  s_prev_mode = 0xFF;
static u8  s_prev_fine = 0xFF;

static void draw_sdr_status(void)
{
    char buf[41]; char freq_str[16]; char bw_str[12]; char *p;
    if (g_freq_hz == s_prev_freq && g_mode == s_prev_mode &&
        (u8)g_fine_tune == s_prev_fine) return;
    s_prev_freq = g_freq_hz; s_prev_mode = g_mode; s_prev_fine = (u8)g_fine_tune;
    format_freq(g_freq_hz, freq_str);
    format_bw(g_bandwidth, bw_str);
    memset(buf, ' ', 40); buf[40] = '\0';
    p = buf + 2; memcpy(p, "> ", 2); p += 2;
    { u8 len = strlen(freq_str); memcpy(p, freq_str, len); p += len; }
    *p++ = ' '; *p++ = ' ';
    { u8 len = strlen(mode_names[g_mode]); memcpy(p, mode_names[g_mode], len); p += len; }
    *p++ = ' '; *p++ = ' ';
    { u8 len = strlen(bw_str); memcpy(p, bw_str, len); p += len; }
    if (g_fine_tune) { *p++ = ' '; *p++ = 'F'; }
    VDP_setTextPalette(PAL1);
    VDP_drawText(buf, 0, ROW_STATUS);
    VDP_setTextPalette(PAL0);
}

static void draw_host(void)
{
    char buf[41];
    u8 pos;
    memset(buf, ' ', 40);
    buf[40] = '\0';

    pos = 2;
    if (g_sdr_state >= 2) {
        /* Host name — truncate safely */
        const char *h = SDR_DEFAULT_HOST;
        u8 i;
        for (i = 0; h[i] && pos < 27; i++)
            buf[pos++] = h[i];
        /* S-meter at right side */
        {
            s16 dbm = (s16)((g_smeter / 10) - 127);
            u16 abs_dbm = (dbm < 0) ? (u16)(-dbm) : (u16)dbm;
            pos = 30;
            buf[pos++] = 'S';
            buf[pos++] = ':';
            if (dbm < 0) buf[pos++] = '-';
            if (abs_dbm >= 100) buf[pos++] = '0' + (abs_dbm / 100);
            if (abs_dbm >= 10) buf[pos++] = '0' + ((abs_dbm / 10) % 10);
            buf[pos++] = '0' + (abs_dbm % 10);
        }
    } else if (g_sdr_state == 1) {
        memcpy(buf + pos, "Authenticating...", 17);
    } else if (g_sdr_error > 0) {
        memcpy(buf + pos, "Error code: ", 12);
        pos += 14;
        buf[pos++] = '0' + g_sdr_error;
    } else {
        memcpy(buf + pos, "Connecting...", 13);
    }

    VDP_setTextPalette(g_sdr_state >= 2 ? PAL1 :
                       g_sdr_state == 1 ? PAL3 : PAL2);
    VDP_drawText(buf, 0, ROW_HOST);
    VDP_setTextPalette(PAL0);
}

/* ── Poll ────────────────────────────────────────────────────────────────── */

static void poll_status(void)
{
    uint32_t vu_word, pos_word;
    if (!g_mw_connected) return;
    if (mw_aud_status(&vu_word, &pos_word) == MW_ERR_NONE) {
        g_vu_raw[3] = (vu_word >> 23) & 0x7F;
        g_vu_raw[2] = (vu_word >> 16) & 0x7F;
        g_vu_raw[1] = (vu_word >> 9)  & 0x7F;
        g_vu_raw[0] = (vu_word >> 2)  & 0x7F;
    }
}

/* ── Marquee ─────────────────────────────────────────────────────────────── */

static const char *marquee_text = "  ~~~  MEGAWIFI SDR RECEIVER  ~~~  "
    "KiwiSDR WebSocket protocol  ~~~  "
    "IMA-ADPCM decode + Freeverb reverb  ~~~  "
    "8-ch stereo mixer with per-channel send  ~~~  "
    "Tune shortwave from your Genesis!  ~~~  ";
static u16 marquee_scroll_x = 0;
static u16 marquee_str_pos = 0;
static bool marquee_needs_init = TRUE;

static void marquee_write_tile(u16 tc, u16 sp)
{
    u16 sl = (u16)strlen(marquee_text);
    u8 ch = marquee_text[sp % sl];
    VDP_setTileMapXY(BG_A,
        TILE_ATTR_FULL(PAL3, TRUE, FALSE, FALSE, TILE_FONT_INDEX + (ch - 32)),
        tc, ROW_MARQUEE);
}

static void marquee_init(void)
{
    u16 c;
    for (c = 0; c < 64; c++) marquee_write_tile(c, c);
    marquee_str_pos = 64; marquee_scroll_x = 0; marquee_needs_init = FALSE;
}

static void update_marquee(void)
{
    s16 neg;
    if (marquee_needs_init) marquee_init();
    marquee_scroll_x = (marquee_scroll_x + 1) & 0x1FF;
    neg = -(s16)marquee_scroll_x;
    VDP_setHorizontalScrollTile(BG_A, ROW_MARQUEE, &neg, 1, CPU);
    if ((marquee_scroll_x & 7) == 0) {
        u16 tc = (u16)((marquee_scroll_x / 8 + 63) % 64);
        marquee_write_tile(tc, marquee_str_pos);
        marquee_str_pos++;
    }
}

static void frame_draw_hook(void)
{
    static u32 lf = 0;
    u32 f = vtimer;
    if (f == lf) return;
    lf = f;
    update_marquee();
}

/* ── Input ───────────────────────────────────────────────────────────────── */

static void main_redraw(void)
{
    VDP_clearPlane(BG_A, TRUE);
    draw_static_ui();
    draw_reverb_status();
    draw_sdr_status();
    draw_preset_info();
    draw_host();
    draw_volume_bar();
    draw_volume_text();
    draw_cpu();
    s_prev_freq = 0; s_prev_mode = 0xFF; s_prev_cpu = 0xFF; s_prev_band = 0xFF;
}

static void handle_input(void)
{
    u16 joy, pressed;
    static u16 prev_joy = 0;
    JOY_update();
    joy = JOY_readJoypad(JOY_1);
    pressed = joy & ~prev_joy;
    prev_joy = joy;
    if (!g_mw_connected) return;

    if (reverb_ctrl_active()) { reverb_ctrl_frame(pressed); return; }
    if (pressed & BUTTON_START) { reverb_ctrl_open(); return; }

    /* Up/Down: manual tune — stops scan */
    if (pressed & BUTTON_UP) {
        u32 step = g_fine_tune ? mode_fine_step[g_mode] : mode_coarse_step[g_mode];
        g_freq_hz += step;
        if (g_freq_hz > 30000000 && g_mode != MODE_FM) g_freq_hz = 30000000;
        if (g_freq_hz > 108000000) g_freq_hz = 108000000;
        g_tune_pending = TRUE;
        g_scanning = FALSE;
    }
    if (pressed & BUTTON_DOWN) {
        u32 step = g_fine_tune ? mode_fine_step[g_mode] : mode_coarse_step[g_mode];
        if (g_freq_hz > step) g_freq_hz -= step; else g_freq_hz = step;
        g_tune_pending = TRUE;
        g_scanning = FALSE;
    }
    /* A: toggle scan on/off */
    if (pressed & BUTTON_A) {
        g_scanning = !g_scanning;
        g_scan_timer = 0;
        g_scan_pause = 0;
    }
    /* B: cycle mode */
    if (pressed & BUTTON_B) {
        g_mode = (g_mode + 1) % NUM_MODES;
        g_bandwidth = mode_default_bw[g_mode];
        g_tune_pending = TRUE;
    }
    /* C: cycle band preset */
    if (pressed & BUTTON_C) {
        g_band++;
        if (g_band >= NUM_BANDS) g_band = 0;
        g_freq_hz = bands[g_band].freq_hz;
        g_mode = bands[g_band].mode;
        if (g_mode >= NUM_MODES) g_mode = MODE_AM;
        g_bandwidth = mode_default_bw[g_mode];
        g_tune_pending = TRUE;
    }
    /* Left/Right: volume */
    if (pressed & BUTTON_LEFT) {
        g_master_vol = (g_master_vol >= 10) ? g_master_vol - 10 : 0;
        mw_aud_set_vol(g_master_vol);
        draw_volume_bar(); draw_volume_text();
    }
    if (pressed & BUTTON_RIGHT) {
        g_master_vol = (g_master_vol <= 245) ? g_master_vol + 10 : 255;
        mw_aud_set_vol(g_master_vol);
        draw_volume_bar(); draw_volume_text();
    }
}

/* ── MegaWifi ────────────────────────────────────────────────────────────── */

static void user_tsk(void) { while (1) mw_process(); }

static bool megawifi_init(void)
{
    uint8_t fw_major, fw_minor;
    char *variant = NULL;
    enum mw_err err;

    if (mw_init(cmd_buf, MW_BUFLEN) != MW_ERR_NONE) return FALSE;
    TSK_userSet(user_tsk);

    err = mw_detect(&fw_major, &fw_minor, &variant);
    if (err != MW_ERR_NONE) return FALSE;

    { char buf[41]; char *p;
      memset(buf, ' ', 40); buf[40] = '\0';
      p = buf + 2; memcpy(p, "FW: v", 5); p += 5;
      p = itoa_simple(fw_major, p); *p++ = '.';
      p = itoa_simple(fw_minor, p);
      if (variant) { *p++ = '-'; strcpy(p, variant); }
      VDP_drawText(buf, 0, ROW_HOST);
    }

    /* Join WiFi using credentials already in flash slot 0 */
    VDP_setTextPalette(PAL3);
    VDP_drawText("  Joining WiFi...               ", 0, ROW_HOST);
    VDP_setTextPalette(PAL0);

    err = mw_ap_assoc(AP_SLOT);
    if (err != MW_ERR_NONE) return FALSE;

    err = mw_ap_assoc_wait(30 * FPS);
    if (err != MW_ERR_NONE) return FALSE;

    mw_sleep(3 * 60);

    VDP_setTextPalette(PAL1);
    VDP_drawText("  WiFi OK                       ", 0, ROW_HOST);
    VDP_setTextPalette(PAL0);

    return TRUE;
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(bool hard)
{
    VDP_setScreenWidth320();
    VDP_setScrollingMode(HSCROLL_TILE, VSCROLL_COLUMN);
    VDP_setTextPalette(PAL0);
    VDP_clearPlane(BG_A, TRUE);
    VDP_clearPlane(BG_B, TRUE);

    setup_palettes();
    load_tiles();
    draw_static_ui();
    draw_sdr_status();
    draw_preset_info();
    draw_host();
    draw_volume_bar();
    draw_volume_text();
    /* No VU sprites in SDR mode */

    JOY_init();
    reverb_ctrl_init(main_redraw);

    VDP_setTextPalette(PAL3);
    VDP_drawText("  Connecting...                 ", 0, ROW_HOST);
    VDP_setTextPalette(PAL0);

    g_mw_connected = megawifi_init();
    if (g_mw_connected) {
        reverb_ctrl_send_defaults();
        draw_reverb_status();

        /* Auto-connect to default KiwiSDR */
        VDP_setTextPalette(PAL3);
        VDP_drawText("  SDR: connecting...            ", 0, ROW_HOST);
        VDP_setTextPalette(PAL0);

        mw_sdr_connect(SDR_DEFAULT_HOST, SDR_DEFAULT_PORT);
        mw_sdr_tune(g_freq_hz, g_mode, g_bandwidth);

        { char buf[41]; char *p;
          memset(buf, ' ', 40); buf[40] = '\0';
          p = buf + 2; memcpy(p, "SDR: ", 5); p += 5;
          strcpy(p, SDR_DEFAULT_HOST);
          VDP_setTextPalette(PAL1);
          VDP_drawText(buf, 0, ROW_HOST);
          VDP_setTextPalette(PAL0);
        }
    } else {
        VDP_setTextPalette(PAL2);
        VDP_drawText("  MegaWifi not found            ", 0, ROW_HOST);
        VDP_setTextPalette(PAL0);
    }

    mw_set_draw_hook(frame_draw_hook);

    { u8 cpu_poll_ctr = 0;
      while (1) {
          VDP_waitVSync();
          update_marquee();
          draw_sdr_status();
          draw_preset_info();
          draw_cpu();
          draw_scan_indicator();
          handle_input();
          update_scan();
          /* Deferred tune — send after input, skip poll this frame */
          if (g_tune_pending && g_mw_connected) {
              g_tune_pending = FALSE;
              mw_sdr_tune(g_freq_hz, g_mode, g_bandwidth);
          } else {
              poll_status();
          }
          if (g_mw_connected && ++cpu_poll_ctr >= 30) {
              cpu_poll_ctr = 0;
              mw_aud_cpu_pct(&g_cpu_pct);
              /* Poll SDR status */
              { uint32_t sw0, sw1;
                if (mw_sdr_status_poll(&sw0, &sw1) == MW_ERR_NONE) {
                    g_sdr_state = (sw0 >> 30) & 0x03;
                    g_smeter    = (sw0 >> 20) & 0x3FF;
                    g_sdr_error = (sw0 >> 4) & 0x0F;
                }
              }
              draw_host();
          }
      }
    }
    return 0;
}
