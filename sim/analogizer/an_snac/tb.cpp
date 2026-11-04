// an_snac against models of the pads' shift registers: a NES and a SNES pad (4021s: the latch active high loads, a rising clock shifts),
// a DB15 board (74HC165s: the latch active low loads) with the DB15 data on IO3 or, for a cable that crosses TX and RX, on IO6.
// Every pad type, one and two players, many button sets (each given as the Pocket's keys that should come out, and turned into the
// pad's bits by the pad's own bit order), then the waveforms (latch width, clock period, the clock edges of a poll), the pins' ownership,
// a change of pad in mid-poll, and that a key word is always one poll's: only ever the old or the new set. Exit status 1 on a failure.
#include "Van_snac.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>

static Van_snac *t;
static long cyc = 0;

// ---- the pads
struct Pad {
    int nbits;
    bool lat_low;           // DB15: the latch loads when low
    uint32_t sr, raw;       // raw: what a load takes (1: released)
    bool pclk;
};
static Pad p1, p2;
static bool crossed;

static void model()
{
    bool lat = t->lat_o, ck = t->clk_o;
    for (Pad *p : {&p1, &p2}) {
        bool load = p->lat_low ? !lat : lat;
        if (load) p->sr = p->raw;
        else if (ck && !p->pclk) p->sr = (p->sr >> 1) | (1u << (p->nbits - 1));
        p->pclk = ck;
    }
    int d1 = p1.sr & 1, d2 = p2.sr & 1;
    t->dat1 = crossed ? 1 : d1;
    t->dat1b = crossed ? d1 : 1;
    t->dat2 = d2;
}
static void step() { t->clk = 0; t->eval(); t->clk = 1; t->eval(); model(); cyc++; }
static void run(long n) { while (n-- > 0) step(); }

// ---- the bit orders (physical: the pads' documented orders), as Pocket key bit -> the pad's bit
enum { UP = 0, DOWN, LEFT, RIGHT, KA, KB, KX, KY, KL, KR, KSEL = 14, KSTART = 15 };
struct Map { int key, bit; };
static const Map NES[]  = { {KA, 0}, {KB, 1}, {KSEL, 2}, {KSTART, 3}, {UP, 4}, {DOWN, 5}, {LEFT, 6}, {RIGHT, 7} };
static const Map SNES[] = { {KB, 0}, {KY, 1}, {KSEL, 2}, {KSTART, 3}, {UP, 4}, {DOWN, 5}, {LEFT, 6}, {RIGHT, 7}, {KA, 8}, {KX, 9}, {KL, 10}, {KR, 11} };
// DB15, player 1: D C B A RIGHT LEFT DOWN UP, then (after player 2's d-pad) F E SELECT START; player 2 in the second half of the chain
// (D is the Pocket's Y, C its X, E its L, F its R)
static const Map DB1[]  = { {KY, 0}, {KX, 1}, {KB, 2}, {KA, 3}, {RIGHT, 4}, {LEFT, 5}, {DOWN, 6}, {UP, 7}, {KR, 12}, {KL, 13}, {KSEL, 14}, {KSTART, 15} };
static const Map DB2[]  = { {RIGHT, 8}, {LEFT, 9}, {DOWN, 10}, {UP, 11}, {KR, 16}, {KL, 17}, {KSEL, 18}, {KSTART, 19}, {KY, 20}, {KX, 21}, {KB, 22}, {KA, 23} };
template <size_t N> static uint32_t mask_of(const Map (&m)[N]) { uint32_t k = 0; for (auto &e : m) k |= 1u << e.key; return k; }
template <size_t N> static uint32_t raw_of(const Map (&m)[N], uint32_t keys, int nbits)
{
    uint32_t raw = (nbits == 32 ? 0xFFFFFFFFu : (1u << nbits) - 1u);          // all released
    for (auto &e : m) if (keys >> e.key & 1u) raw &= ~(1u << e.bit);          // pressed: 0
    return raw;
}

static uint32_t rng = 12345;
static uint32_t rnd() { rng = rng * 1664525u + 1013904223u; return rng >> 8; }

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { if (fails++ < 20) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// the pad of `ty` (1 DB15, 2 NES, 3 SNES), its two players' keys given. `fresh`: a pad plugged in (its register is what it loads);
// not: the buttons change while the register shifts, as they do, and the new set is what the next latch takes
static void set_pads(int ty, uint32_t k1, uint32_t k2, bool fresh = true)
{
    Pad n1, n2;
    if (ty == 1) {
        n1 = {24, true, 0, raw_of(DB1, k1, 24) & raw_of(DB2, k2, 24), false};      // one chain: the two players' bits together
        n2 = {1, true, 1, 1, false};
    } else {
        int n = ty == 2 ? 8 : 16;
        const Map *m = ty == 2 ? NES : SNES;
        size_t cnt = ty == 2 ? sizeof(NES) / sizeof(Map) : sizeof(SNES) / sizeof(Map);
        auto rw = [&](uint32_t k) { uint32_t raw = (1u << n) - 1u; for (size_t i = 0; i < cnt; i++) if (k >> m[i].key & 1u) raw &= ~(1u << m[i].bit); return raw; };
        n1 = {n, false, 0, rw(k1), false};
        n2 = {n, false, 0, rw(k2), false};
    }
    if (fresh) { n1.sr = n1.raw; n2.sr = n2.raw; p1 = n1; p2 = n2; }
    else { p1.raw = n1.raw; p2.raw = n2.raw; }
}
static uint32_t valid(int ty, int player)
{
    return ty == 2 ? mask_of(NES) : ty == 3 ? mask_of(SNES) : player == 1 ? mask_of(DB1) : mask_of(DB2);
}
static const char *NAME[] = { "off", "DB15", "NES", "SNES" };

int main()
{
    t = new Van_snac;
    t->own = 1; t->kind = 0; t->dat1 = 1; t->dat1b = 1; t->dat2 = 1;
    run(10);

    // ---- every pad, one and two players, crossed cable or not: random button sets
    for (int ty = 1; ty <= 3; ty++)
        for (int two = 0; two <= 1; two++)
            for (int cr = 0; cr <= (ty == 1 ? 1 : 0); cr++) {
                crossed = cr;
                for (int rep = 0; rep < 8; rep++) {
                    uint32_t k1 = rep == 0 ? 0 : rep == 1 ? valid(ty, 1) : rnd() & valid(ty, 1);
                    uint32_t k2 = rep == 0 ? 0 : rep == 1 ? valid(ty, 2) : rnd() & valid(ty, 2);
                    t->kind = ty | (two << 2);
                    set_pads(ty, k1, k2);
                    run(60000);                                                   // a few polls (the slowest, SNES, takes 16300 clocks)
                    CHECK(t->drive == 1, "%s: pins not driven", NAME[ty]);
                    CHECK(t->key1 == k1, "%s%s%s set %d: key1 %04x, wanted %04x", NAME[ty], two ? " 2P" : "", cr ? " crossed" : "", rep, t->key1, k1);
                    CHECK(t->key2 == (two ? k2 : 0u), "%s%s%s set %d: key2 %04x, wanted %04x", NAME[ty], two ? " 2P" : "", cr ? " crossed" : "", rep, t->key2, two ? k2 : 0u);
                }
                printf("%-4s %s%s: 8 button sets done\n", NAME[ty], two ? "2 players" : "1 player ", cr ? ", data on IO6" : "");
            }
    crossed = false;

    // ---- the waveforms: the latch, the clock period and the clock edges of one poll, from the pins
    for (int ty = 1; ty <= 3; ty++) {
        t->kind = ty; set_pads(ty, 0, 0); run(30000);
        const long tick = ty == 1 ? 156 : 625;                                   // clk_sys / 400 kHz, / 100 kHz (CLK_HZ 62.5 MHz)
        int pl = t->lat_o, pc = t->clk_o;
        long lat_on = -1, lat_off = -1, edges = 0, hi = -1, hi_t = 0, poll_start = -1, poll_len = 0;
        bool inpoll = false;
        long clk_hi_min = 1 << 30, clk_hi_max = 0;
        for (long i = 0; i < 80000; i++) {
            step();
            int l = t->lat_o, c = t->clk_o;
            bool lat_act = ty == 1 ? !l : l, lat_was = ty == 1 ? !pl : pl;
            if (lat_act && !lat_was) {
                if (inpoll && poll_start >= 0 && poll_len == 0) poll_len = cyc - poll_start;
                if (!inpoll) { inpoll = true; poll_start = cyc; edges = 0; }
                else break;                                                          // the next poll: one is measured
                lat_on = cyc;
            }
            if (!lat_act && lat_was && inpoll && lat_off < 0) lat_off = cyc;
            if (inpoll && lat_off >= 0 && c && !pc) edges++;
            if (c && !pc) hi_t = cyc;
            if (!c && pc && hi_t) { long w = cyc - hi_t; if (inpoll && lat_off >= 0) { if (w < clk_hi_min) clk_hi_min = w; if (w > clk_hi_max) clk_hi_max = w; } }
            pl = l; pc = c;
        }
        long poll_clocks = cyc - poll_start;
        // NES 7, SNES 11 shifts. DB15: 23 shifts after the latch, and one more edge, the idle level coming up after the poll's last bit
        // (the next /LAT reloads the register, so it shifts nothing that is read)
        int want_edges = ty == 1 ? 24 : ty == 2 ? 7 : 11;
        CHECK(lat_off - lat_on >= 2 * tick - 2 && lat_off - lat_on <= 2 * tick + 2, "%s: latch %ld clocks, wanted %ld", NAME[ty], lat_off - lat_on, 2 * tick);
        CHECK(edges == want_edges, "%s: %ld clock edges after the latch, wanted %d", NAME[ty], edges, want_edges);
        CHECK(clk_hi_min >= tick - 2 && clk_hi_max <= tick + 2, "%s: clock high %ld..%ld clocks, wanted %ld", NAME[ty], clk_hi_min, clk_hi_max, tick);
        printf("%-4s waveform: latch %ld, %ld edges, clock high %ld..%ld, a poll %ld clocks (%.0f us)\n", NAME[ty], lat_off - lat_on, edges, clk_hi_min, clk_hi_max,
               poll_clocks, poll_clocks / 62.5);
    }

    // ---- the port's owner and the setting: pins and keys
    t->kind = 3 | 4; set_pads(3, valid(3, 1), valid(3, 2)); t->own = 0; run(60000);
    CHECK(t->drive == 0 && t->key1 == 0 && t->key2 == 0, "adapter does not own the port: drive %d keys %04x %04x", t->drive, t->key1, t->key2);
    t->own = 1; run(60000);
    CHECK(t->drive == 1 && t->key1 == valid(3, 1) && t->key2 == valid(3, 2), "owned again: drive %d keys %04x %04x", t->drive, t->key1, t->key2);
    t->kind = 0; run(100);
    CHECK(t->drive == 0 && t->key1 == 0 && t->key2 == 0, "kind off: drive %d keys %04x %04x", t->drive, t->key1, t->key2);
    t->kind = 4; run(100);
    CHECK(t->drive == 0 && t->key1 == 0, "kind 4 (a second player with no pad): drive %d key1 %04x", t->drive, t->key1);
    printf("ownership and the setting: done\n");

    // ---- changes: SNES -> NES -> DB15 in mid-poll, and the key words only ever the old or the new set (a poll's, whole)
    {
        uint32_t ka = rnd() & mask_of(SNES), kb = rnd() & mask_of(SNES), kn = rnd() & mask_of(NES);
        t->kind = 3; set_pads(3, ka, 0); run(40000);
        CHECK(t->key1 == ka, "SNES set A: %04x wanted %04x", t->key1, ka);
        long odd = 0;
        for (int rep = 0; rep < 6; rep++) {
            uint32_t from = rep & 1 ? kb : ka, to = rep & 1 ? ka : kb;
            run(rnd() % 20000);                                                      // the change comes at any point of a poll
            set_pads(3, to, 0, false);
            for (long i = 0; i < 40000; i++) { step(); if (t->key1 != from && t->key1 != to) odd++; }
            CHECK(t->key1 == to, "SNES change %d: %04x wanted %04x", rep, t->key1, to);
        }
        CHECK(odd == 0, "a key word that was neither set, %ld clocks", odd);
        t->kind = 2; set_pads(2, kn, 0);
        run(2);
        CHECK(t->key1 == 0, "after a change of pad the old keys are cleared (%04x)", t->key1);
        run(40000);
        CHECK(t->key1 == kn, "NES after SNES: %04x wanted %04x", t->key1, kn);
        uint32_t kd = rnd() & mask_of(DB1);
        t->kind = 1; set_pads(1, kd, 0); run(40000);
        CHECK(t->key1 == kd, "DB15 after NES: %04x wanted %04x", t->key1, kd);
        printf("changes in mid-poll: done\n");
    }

    printf("%d checks, %d failed\n", checks, fails);
    printf(fails ? "SOME FAILED\n" : "ALL PASS\n");
    delete t;
    return fails ? 1 : 0;
}
