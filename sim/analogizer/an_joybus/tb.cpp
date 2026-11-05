// an_joybus against four models of N64 controllers on the Joybus: one wire each, a 1 kohm pull-up (high unless the master or the pad
// pulls it low), the master's `drive` pulling all four low together. A pad decodes the command from the widths of the low pulses
// (under 2 us a 1), and answers the buttons command (0x01) with 32 bits and a stop bit in 4 us cells after a delay of its own, at a speed
// of its own. Checks: every pad's buttons and stick; a pad taken out (gone after the MISS polls, not before), put back; pads with other
// speeds and delays; the master's waveform (the command's nine pulses, the cell, the poll period); no pad, a line stuck low, `en` low.
// Exit status 1 on a failure.
#include "Van_joybus.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

static Van_joybus *t;
static long cyc = 0;
static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails < 25) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static inline long us(double x) { return (long)(x * 62.5 + 0.5); }   // clk_sys 62.5 MHz

struct Pad {
    bool conn = false;
    bool stuck_low = false;
    uint32_t reply = 0;
    double speed = 1.0;         // its cells' length against 4 us
    double delay_us = 2.0;      // from the stop bit's end to its first bit
    // decoding
    bool prev = true;
    long fall_c = 0;
    int rxn = 0;
    uint32_t cmd = 0;
    // answering
    bool replying = false;
    std::vector<std::pair<long, long>> cells;   // (low cycles, high cycles)
    size_t cell = 0;
    long cell_c = 0, start_c = 0;
    bool low = false;
    void begin_reply() {
        cells.clear();
        for (int i = 31; i >= 0; i--) {
            bool b = (reply >> i) & 1;
            cells.push_back({us((b ? 1.0 : 3.0) * speed), us((b ? 3.0 : 1.0) * speed)});
        }
        cells.push_back({us(1.0 * speed), us(1.0 * speed)});          // stop bit
        cell = 0; cell_c = 0; replying = true; start_c = cyc + us(delay_us);
    }
    bool line_in(bool master_low) {
        bool l = !(master_low || low || stuck_low);
        if (conn && !replying) {
            if (!l && prev) fall_c = cyc;
            if (l && !prev && (cyc - fall_c) > us(20)) { rxn = 0; cmd = 0; }     // (a long low is no bit: the pad starts over)
            else if (l && !prev) {
                bool bit = (cyc - fall_c) < us(2.0);
                rxn++;
                if (rxn <= 8) cmd = (cmd << 1) | bit;
                if (rxn == 9) { if (cmd == 0x01) begin_reply(); rxn = 0; cmd = 0; }
            }
            prev = l;
        } else prev = true;
        return l;
    }
    void tick() {
        if (!replying) { low = false; return; }
        if (cyc < start_c) { low = false; return; }
        if (cell >= cells.size()) { replying = false; low = false; rxn = 0; cmd = 0; prev = true; return; }
        auto [lo, hi] = cells[cell];
        low = cell_c < lo;
        if (++cell_c >= lo + hi) { cell++; cell_c = 0; }
    }
};
static Pad pad[4];

// waveform recording
static bool rec = false;
static std::vector<long> fall_t, rise_t;
static bool prev_drive = false;

static void step()
{
    t->clk = 0; t->eval(); t->clk = 1; t->eval();
    bool d = t->drive;
    if (rec) { if (d && !prev_drive) fall_t.push_back(cyc); if (!d && prev_drive) rise_t.push_back(cyc); }
    prev_drive = d;
    uint32_t si = 0;
    for (int i = 0; i < 4; i++) { si |= (uint32_t)pad[i].line_in(d) << i; pad[i].tick(); }
    t->si = si;
    cyc++;
}
static void run(long n) { while (n-- > 0) step(); }
static void run_ms(double ms) { run(us(ms * 1000.0)); }

static uint16_t exp_btn(uint32_t w) { return (uint16_t)((w >> 16) & 0xFF3Fu); }
static uint16_t exp_stk(uint32_t w) { return (uint16_t)(((w & 0xFF) << 8) | ((w >> 8) & 0xFF)); }   // {Y, X}
static uint16_t got_btn(int i) { return (uint16_t)(t->btn >> (16 * i)); }
static uint16_t got_stk(int i) { return (uint16_t)(t->stick >> (16 * i)); }

static void check_pads(const char *what, uint32_t mask)
{
    CHECK(t->present == mask, "%s: present %x, wanted %x", what, (unsigned)t->present, mask);
    for (int i = 0; i < 4; i++) {
        if (mask & (1u << i)) {
            CHECK(got_btn(i) == exp_btn(pad[i].reply), "%s: pad %d buttons %04x, wanted %04x", what, i + 1, got_btn(i), exp_btn(pad[i].reply));
            CHECK(got_stk(i) == exp_stk(pad[i].reply), "%s: pad %d stick %04x, wanted %04x", what, i + 1, got_stk(i), exp_stk(pad[i].reply));
        } else {
            CHECK(got_btn(i) == 0 && got_stk(i) == 0, "%s: pad %d absent but its words are %04x %04x", what, i + 1, got_btn(i), got_stk(i));
        }
    }
}

int main()
{
    t = new Van_joybus;
    t->en = 0; t->si = 0xF;
    for (int i = 0; i < 4; i++) pad[i].conn = true;
    run(100);
    // en low: the lines are let go, no pad
    pad[0].reply = 0x8000007F; run_ms(3);
    CHECK(t->drive == 0 && t->present == 0, "en low: drive %d present %x", (int)t->drive, (unsigned)t->present);
    t->en = 1;

    // four pads, four different states: A+Start with the stick right/up, nothing, everything, the stick at the ends
    pad[0].reply = 0x9000'507F;
    pad[1].reply = 0x0000'0000;
    pad[2].reply = 0xFFFF'3F3F;          // all buttons, the reset flag (bit 23 of the reply: 0x0080 in the buttons) set below
    pad[3].reply = 0x0123'80B0;          // Z, B-less ... a mixed word, the stick at X=-128, Y=-80
    pad[2].reply |= 0x00800000;          // the reset flag
    run_ms(3);
    check_pads("four pads", 0xF);
    CHECK((got_btn(2) & 0x00C0) == 0, "the reset flag and bit 6 are out: %04x", got_btn(2));

    // buttons and stick change
    pad[0].reply = 0x0800'FF01; pad[3].reply = 0x2000'0080;
    run_ms(3);
    check_pads("changed", 0xF);

    // ports 2 and 3 only
    pad[0].conn = false; pad[3].conn = false;
    run_ms(2);
    CHECK((t->present & 0x6) == 0x6, "ports 2, 3 there: %x", (unsigned)t->present);
    CHECK((t->present & 0x1) == 0x1 && (t->present & 0x8) == 0x8, "a pad just out is still there (the poll's miss count): %x", (unsigned)t->present);
    run_ms(10);
    CHECK((t->present & 0x9) == 0x9, "after 10 ms (fewer than MISS polls) they are still there: %x", (unsigned)t->present);
    run_ms(35);
    check_pads("ports 2, 3", 0x6);
    // put back
    pad[0].conn = true;
    run_ms(2.5);
    check_pads("port 1 back", 0x7);

    // other speeds and delays: +-4 % cells, a reply 1 us, 6 us and 30 us after the command
    double sp[4] = {0.96, 1.04, 1.0, 0.98}, dl[4] = {1.0, 6.0, 30.0, 2.0};
    for (int i = 0; i < 4; i++) { pad[i].conn = true; pad[i].speed = sp[i]; pad[i].delay_us = dl[i]; }
    pad[0].reply = 0xA5C3'1234; pad[1].reply = 0x5A3C'EDCB; pad[2].reply = 0x1234'80FF; pad[3].reply = 0xF0F0'0F0F;
    run_ms(4);
    check_pads("speeds and delays", 0xF);
    for (int i = 0; i < 4; i++) { pad[i].speed = 1.0; pad[i].delay_us = 2.0; }

    // a line stuck low (a short): no falling edge to hear, so no pad
    pad[1].stuck_low = true;
    run_ms(45);
    CHECK((t->present & 0x2) == 0 && (t->present & 0xD) == 0xD, "port 2's line stuck low: present %x", (unsigned)t->present);
    pad[1].stuck_low = false; run_ms(3);
    check_pads("line free again", 0xF);

    // the master's waveform: nine low pulses a poll (the command 0x01 and the stop bit), a 4 us cell, a poll every ~1 ms
    fall_t.clear(); rise_t.clear(); rec = true; run_ms(2.2); rec = false;
    // find a poll: three consecutive falls 248 cycles apart begin one; take the first fall after a long gap
    size_t k0 = 0;
    for (size_t i = 1; i < fall_t.size(); i++) if (fall_t[i] - fall_t[i - 1] > us(500)) { k0 = i; break; }
    CHECK(k0 > 0 && fall_t.size() >= k0 + 9 && rise_t.size() >= k0 + 9, "no complete poll recorded (falls %zu, k0 %zu)", fall_t.size(), k0);
    if (k0 > 0 && fall_t.size() >= k0 + 10 && rise_t.size() >= k0 + 9) {
        // (rise_t has the same index as fall_t when the first event is a fall; shift if a rise came first)
        size_t r0 = 0; while (r0 < rise_t.size() && rise_t[r0] < fall_t[k0]) r0++;
        int bits[9] = {0, 0, 0, 0, 0, 0, 0, 1, 1};        // 0x01, the stop bit
        for (int i = 0; i < 9; i++) {
            long w = rise_t[r0 + i] - fall_t[k0 + i];
            long want = bits[i] ? 62 : 186;
            CHECK(labs(w - want) <= 2, "pulse %d: %ld cycles low, wanted %ld", i, w, want);
            if (i < 8) { long cell = fall_t[k0 + i + 1] - fall_t[k0 + i]; CHECK(labs(cell - 248) <= 2, "cell %d: %ld cycles, wanted 248", i, cell); }
        }
        long per = fall_t[k0 + 9] - fall_t[k0];
        CHECK(fall_t.size() > k0 + 9 && per > us(900) && per < us(1300), "poll period %ld us", (long)(per / 62.5));
    }

    // no pad at all
    for (int i = 0; i < 4; i++) pad[i].conn = false;
    run_ms(45);
    CHECK(t->present == 0 && t->btn == 0 && t->stick == 0, "no pads: present %x", (unsigned)t->present);
    for (int i = 0; i < 4; i++) pad[i].conn = true;
    run_ms(3);
    check_pads("all back", 0xF);

    // en low again: everything off at once
    t->en = 0; run(10);
    CHECK(t->drive == 0 && t->present == 0 && t->btn == 0 && t->stick == 0, "en low: not cleared (present %x)", (unsigned)t->present);

    printf("%d checks, %d failed\n", checks, fails);
    printf(fails ? "SOME FAILED\n" : "ALL PASS\n");
    return fails ? 1 : 0;
}
