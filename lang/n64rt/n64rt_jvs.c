/* A JVS I/O board as the console's controllers: the Analogizer SNAC adapter's JVS wiring (rtl/analogizer/an_jvs.sv).
 *
 * The hardware frames, escapes and checks the bus's bytes and resends one frame every PERIOD ms by itself, keeping the last good answer in its
 * buffer (128..255). This file is the runtime's side of it:
 *   - the switch. The menu's "Analogizer controllers" (INTERACT2 bits 14:10, interact.json id 249) with bit 14 set gives player 1 to the JVS's
 *     player 1 (16), and player 2 to its player 2 as well (17: bit 10). The SNAC port's Joybus is off then (n64rt_snac.c). The pins are the JVS's
 *     only while the adapter owns the cartridge port (INTERACT2 bit 31), as the Joybus's.
 *   - the init, a step at each look (n64rt_jvs_step, from each run of the PIF's commands and each read of the pads), so nothing ever waits for
 *     the bus: reset twice a second apart, 3 s for the board to come back, set the address 1, then the identity (logged), and the features (players, switches, analog
 *     channels), each frame 10 ms or more after the previous answer (a Namco board ignores one sent sooner). Then the poll frame
 *     01 20 <players> <bytes> [22 <channels>] is the hardware's, every POLL_MS. Answers stopping for RESTART_MS (the board off, the cable out): the init again.
 *   - the switches. A player's two bytes, as JVS lays them out (Start, Service, Up, Down, Left, Right, Push 1, Push 2; Push 3 .. Push 8), are
 *     the Pocket's keys (n64rt_jvs_keys) and go through n64rt_pad_map with the menu's settings like the Pocket's own:
 *       Start Start   Up Down Left Right the D-pad   Push 1 A   Push 2 B   Push 3 L1 (Z)   Push 4 R1 (R)   Push 5 L2 (L)
 *       Push 6 R2 (C-up)   Push 7 Y (C-left)   Push 8 X (C-right)   Service Select (C-down)
 *   - the analog channels. 0 and 1 are player 1's left stick (X, Y), 2 and 3 player 2's, as a dock pad's stick bytes (KEY_TYPE 2: n64rt_pad_map
 *     takes the stick where it is deflected, the D-pad keys where it is centred; the menu's "Stick" says which). A board's unconnected channels
 *     float, so a channel counts only once it has moved over a quarter of its range (AN_SPAN); its centre is where it was at the init, and its
 *     range each side of it what it has reached (AN_MIN at least), so a pot that does not go end to end still gives the whole stick. */
#include <generated/csr.h>
#include <generated/soc.h>
#include "n64rt.h"
#include "n64rt_pad.h"

#define JVS_ID        0x4A560001u
#define AN_OWN        0x80000000u              /* INTERACT2 bit 31: the adapter owns the cartridge port (n64rt_pak.c) */
#define AN_JVS(w)     (((w) >> 14) & 1u)       /* INTERACT2 bit 14: the menu's JVS choices */
#define AN_JVS_P2(w)  (((w) >> 10) & 1u)       /*   and bit 10 with it: player 2 too */
#define MS            (CONFIG_CLOCK_FREQUENCY / 1000u)
#define INIT_PERIOD   20u                      /* ms an init frame's answer is waited for (an identity is ~55 bytes: 5 ms) */
#define POLL_MS       14u                      /* the poll's period: a 2-player answer with 4 channels (~20 bytes) and the frame take ~3 ms, and a
                                                * Sega 838-13683B wants 10 ms between an answer and the next poll (JVS_Debugger's figure) */
#define GAP_MS        10u
#define RESET_MS      1000u                    /* between the two resets */
#define SETTLE_MS     3000u                    /* after the second: a 838-13683B does not answer a SETADDR 100 ms after it */
#define RESTART_MS    500u
#define AN_SPAN       0x4000u                  /* a channel's move (of 0..FFFF) before it is believed */
#define AN_MIN        0x1000u

enum { J_OFF, J_RESET1, J_RESET2, J_SETADDR, J_IDENT, J_FEAT, J_RUN };
static u8  s_probed, s_hw, s_st, s_sent, s_players, s_bytes, s_good, s_p2;
static u64 s_at, s_seen;                        /* the next step's earliest time; the last time GOOD moved */
static u8  s_sw[2][2];                          /* the players' first two switch bytes, from the last good answer */
static u8  s_nch;                               /* analog channels polled (0..4) */
static struct { u16 v, ctr, lo, hi; u8 init, on; } s_an[4];

static u64 now(void) { timer0_uptime_latch_write(1); return timer0_uptime_cycles_read(); }

static int jvs_hw(void)
{
    if (!s_probed) { s_probed = 1; s_hw = jvs_id_read() == JVS_ID; }
    return s_hw;
}

static void send(const u8 *f, u32 n, u32 ctrl)
{
    jvs_ctrl_write(ctrl);
    jvs_ptr_write(0);
    for (u32 i = 0; i < n; i++) jvs_data_write(f[i]);
    jvs_xfer_write(n);
    s_sent = 1;
}
static u32 answer(u8 *b, u32 max)              /* the last good answer (node .. sum), up to max bytes */
{
    u32 n = jvs_xfer_read() & 0xFFu;
    if (n > max) n = max;
    jvs_ptr_write(128);
    for (u32 i = 0; i < n; i++) b[i] = (u8)jvs_data_read();
    return n;
}
static void restart(u32 after_ms, const char *why)
{
    if (why) n64rt_log("n64rt: JVS: %s, the init again\n", why);
    jvs_ctrl_write(1u | INIT_PERIOD << 8);       /* (POLL off) */
    s_st = J_RESET1; s_sent = 0; s_players = 0; s_at = now() + (u64)after_ms * MS;
    for (int p = 0; p < 2; p++) s_sw[p][0] = s_sw[p][1] = 0;
    s_nch = 0;
    for (int c = 0; c < 4; c++) s_an[c].init = s_an[c].on = 0;
}
static void an_take(int c, u16 v)
{
    if (!s_an[c].init) { s_an[c].init = 1; s_an[c].ctr = s_an[c].lo = s_an[c].hi = v; }
    if (v < s_an[c].lo) s_an[c].lo = v;
    if (v > s_an[c].hi) s_an[c].hi = v;
    s_an[c].v = v;
    if (!s_an[c].on && (u32)(s_an[c].hi - s_an[c].lo) > AN_SPAN) {
        s_an[c].on = 1;
        n64rt_log("n64rt: JVS: analog channel %d in use (centre %04x)\n", c, s_an[c].ctr);
    }
}
static u32 an_byte(int c)                       /* channel c as a stick byte, 0x80 centred */
{
    if (c >= s_nch || !s_an[c].on) return 0x80u;
    const int d = (int)s_an[c].v - (int)s_an[c].ctr;
    int r = d >= 0 ? (int)s_an[c].hi - (int)s_an[c].ctr : (int)s_an[c].ctr - (int)s_an[c].lo;
    if (r < (int)AN_MIN) r = (int)AN_MIN;
    int o = d * 127 / r;
    if (o > 127) o = 127;
    if (o < -127) o = -127;
    return (u32)(128 + o);
}

/* the features (14): players and switches, the only function the poll needs */
static void features(const u8 *b, u32 n)
{
    s_players = 0; s_bytes = 0;
    for (u32 i = 4; i + 3 < n; i += 4) {        /* (00 len 01 01, then 4 bytes a function, 00 at the end) */
        if (b[i] == 0x00u) break;
        if (b[i] == 0x01u) { s_players = b[i + 1]; s_bytes = (u8)((b[i + 2] + 7u) / 8u); }
        if (b[i] == 0x03u) s_nch = b[i + 1] > 4u ? 4u : b[i + 1];
        n64rt_log("n64rt: JVS: function %02x: %u %u %u\n", b[i], b[i + 1], b[i + 2], b[i + 3]);
    }
    if (s_players > 2) s_players = 2;           /* (the poll asks for these only) */
    if (s_bytes > 4) s_bytes = 4;
}

void n64rt_jvs_step(void)
{
    if (!jvs_hw()) return;
    const u32 i2 = apf_interact_interact2_read();
    const int want = AN_JVS(i2) && (i2 & AN_OWN);
    s_p2 = (u8)AN_JVS_P2(i2);
    if (!want) {
        if (s_st != J_OFF) { jvs_ctrl_write(0); s_st = J_OFF; n64rt_log("n64rt: JVS: off\n"); }
        return;
    }
    if (s_st == J_OFF) { n64rt_log("n64rt: JVS: on\n"); restart(0, 0); }
    const u64 t = now();
    if (s_st == J_RUN) {
        const u8 g = (u8)jvs_stat_good_read();
        if (g != s_good) {
            /* a new answer: 00 len 01 | 01 system | the players' bytes [| 01 the channels, 2 bytes each]. Read it again if another came
             * while it was read (GOOD moved) */
            for (int k = 0; k < 3; k++) {
                u8 b[32];
                const u32 n = answer(b, sizeof b);
                const u8 g2 = (u8)jvs_stat_good_read();
                if (g2 != g) continue;
                const u32 a = 5u + (u32)s_players * s_bytes;        /* (the channels' report byte) */
                if (n >= a && b[2] == 0x01u && b[3] == 0x01u)
                    for (u32 p = 0; p < s_players; p++) { s_sw[p][0] = b[5 + p * s_bytes]; s_sw[p][1] = s_bytes > 1 ? b[6 + p * s_bytes] : 0; }
                if (s_nch && n >= a + 1u + 2u * s_nch && b[a] == 0x01u)
                    for (int c = 0; c < s_nch; c++) an_take(c, (u16)(b[a + 1 + 2 * c] << 8 | b[a + 2 + 2 * c]));
                break;
            }
            s_good = g; s_seen = t;
        } else if (t > s_seen + (u64)RESTART_MS * MS) restart(RESET_MS, "the board stopped answering");
        return;
    }
    if (t < s_at) return;
    if (s_sent) {                               /* an init frame is out: its answer */
        const u32 x = jvs_xfer_read();
        if (x >> 31) return;                    /* (BUSY) */
        s_sent = 0;
        u8 b[112];
        const u32 n = (x & (3u << 29)) ? 0 : answer(b, sizeof b);
        s_at = t + (u64)GAP_MS * MS;
        switch (s_st) {
        case J_RESET1: s_st = J_RESET2; s_at = t + (u64)RESET_MS * MS; break;
        case J_RESET2: s_st = J_SETADDR; s_at = t + (u64)SETTLE_MS * MS; break;
        case J_SETADDR:
            if (n >= 4 && b[2] == 0x01u && b[3] == 0x01u) { n64rt_log("n64rt: JVS: a board at address 1 (SENSE %u)\n", (unsigned)jvs_stat_sense_read()); s_st = J_IDENT; }
            else restart(RESET_MS, n ? "SETADDR refused" : "no board answered SETADDR");
            break;
        case J_IDENT:
            if (n > 5) { b[n - 1] = 0; n64rt_log("n64rt: JVS: %s\n", (const char *)&b[4]); }      /* (the string, the sum's byte made its end) */
            s_st = J_FEAT;
            break;
        case J_FEAT:
            if (n > 4) features(b, n);
            if (s_players == 0) { restart(RESET_MS, "no switches"); break; }
            {
                const u8 f[6] = { 0x01, 0x20, s_players, s_bytes, 0x22, s_nch };
                send(f, s_nch ? 6 : 4, 3u | POLL_MS << 8);   /* (ON, POLL) */
                s_sent = 0; s_st = J_RUN; s_good = (u8)jvs_stat_good_read(); s_seen = t;
                n64rt_log("n64rt: JVS: polling %u players, %u bytes each, %u analog channels, every %u ms\n", s_players, s_bytes, s_nch, POLL_MS);
            }
            break;
        default: break;
        }
        return;
    }
    static const u8 f_reset[] = { 0xFF, 0xF0, 0xD9 }, f_addr[] = { 0xFF, 0xF1, 0x01 }, f_ident[] = { 0x01, 0x10 }, f_feat[] = { 0x01, 0x14 };
    const u32 c = 1u | INIT_PERIOD << 8;
    switch (s_st) {
    case J_RESET1: case J_RESET2: send(f_reset, 3, c); break;
    case J_SETADDR: send(f_addr, 3, c); break;
    case J_IDENT: send(f_ident, 2, c); break;
    case J_FEAT: send(f_feat, 2, c); break;
    default: break;
    }
}

int n64rt_jvs_player(int ch)
{
    return s_st != J_OFF && (ch == 0 || (ch == 1 && s_p2));
}

/* player ch's switches as the Pocket's keys (KEY_TYPE 2 with analog channels: the stick bytes are believed where deflected; else 1) */
u32 n64rt_jvs_keys(int ch)
{
    u32 k = (s_nch ? 2u : 1u) << 28;
    if (s_st != J_RUN || ch < 0 || ch >= s_players) return k;
    const u8 a = s_sw[ch][0], b = s_sw[ch][1];
    if (a & 0x80u) k |= KEY_START;
    if (a & 0x40u) k |= KEY_SELECT;
    if (a & 0x20u) k |= KEY_DPAD_UP;
    if (a & 0x10u) k |= KEY_DPAD_DOWN;
    if (a & 0x08u) k |= KEY_DPAD_LEFT;
    if (a & 0x04u) k |= KEY_DPAD_RIGHT;
    if (a & 0x02u) k |= KEY_A;
    if (a & 0x01u) k |= KEY_B;
    if (b & 0x80u) k |= KEY_L1;
    if (b & 0x40u) k |= KEY_R1;
    if (b & 0x20u) k |= KEY_L2;
    if (b & 0x10u) k |= KEY_R2;
    if (b & 0x08u) k |= KEY_Y;
    if (b & 0x04u) k |= KEY_X;
    return k;
}

/* player ch's stick bytes (LX LY RX RY from the low byte up): channels 2 ch and 2 ch + 1 as the left stick */
u32 n64rt_jvs_joy(int ch)
{
    if (s_st != J_RUN || ch < 0 || ch > 1) return 0x80808080u;
    return 0x80800000u | an_byte(2 * ch + 1) << 8 | an_byte(2 * ch);
}
