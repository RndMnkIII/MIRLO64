// an_joybus against three models of N64 controllers on the Joybus (a fourth, on the port the engine does not use, to hear nothing from it):
// one wire each, a 1 kohm pull-up (high unless the master or the pad pulls it low), the master's `drive` pulling the line of the port it
// works on low. A pad decodes a command from the widths of the low pulses (under 2 us a 1) and answers the commands of a controller:
//   0x00 / 0xFF  the status   (05 00 + 01 with an accessory, 02 without)       0x01  the buttons and the stick (4 bytes)
//   0x02 addr    a block of the accessory (32 bytes + the CRC)                  0x03 addr + 32 bytes   a block written (the CRC)
// in 4 us cells after a delay of its own, at a speed of its own.
// Checks: the polling (every pad's buttons and stick; a pad taken out -- gone after the MISS polls, not before -- put back; other speeds and delays;
// a line stuck low; the master's waveform: the command's nine pulses, the cell, the slot, one line at a time, the ports in turn) and the runtime's
// transactions through the registers (the status, a block written and read back, a pad that is not there, a reply that is short, the 4th port,
// a request in the middle of a poll, a second go while busy, the polling going on after, the time a block takes). Exit status 1 on a failure.
#include "Van_joybus.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <vector>

static Van_joybus *t;
static long cyc = 0;
static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; if (fails < 25) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static inline long us(double x) { return (long)(x * 62.5 + 0.5); }   // clk_sys 62.5 MHz

static uint8_t crc_of(const uint8_t *d) { uint8_t c = 0; for (int i = 0; i < 32; i++) c ^= d[i]; return (uint8_t)(c ^ 0xA5); }   // (a checksum of the model's own)

struct Pad {
    bool conn = false;
    bool stuck_low = false;
    bool pak = false;
    uint32_t reply = 0;                         // the buttons command's four bytes
    std::map<uint32_t, std::vector<uint8_t>> mem;   // the accessory: 32-byte blocks by address
    double speed = 1.0;         // its cells' length against 4 us
    double delay_us = 2.0;      // from the stop bit's end to its first bit
    int short_by = 0;           // bytes fewer than a command's reply (a device that says less)
    // decoding
    bool prev = true;
    long fall_c = 0, rise_c = 0;
    std::vector<uint8_t> bits;
    // answering
    bool replying = false;
    std::vector<std::pair<long, long>> cells;   // (low cycles, high cycles)
    size_t cell = 0;
    long cell_c = 0, start_c = 0;
    bool low = false;
    void begin_reply(const std::vector<uint8_t> &bytes) {
        cells.clear();
        for (uint8_t by : bytes)
            for (int i = 7; i >= 0; i--) {
                bool b = (by >> i) & 1;
                cells.push_back({us((b ? 1.0 : 3.0) * speed), us((b ? 3.0 : 1.0) * speed)});
            }
        cells.push_back({us(1.0 * speed), us(1.0 * speed)});          // stop bit
        cell = 0; cell_c = 0; replying = true; start_c = cyc + us(delay_us);
    }
    uint8_t byte_at(size_t k) const { uint8_t v = 0; for (int i = 0; i < 8; i++) v = (uint8_t)((v << 1) | bits[k * 8 + i]); return v; }
    int cmd_len(uint8_t c) const { return c == 0x00 || c == 0xFF || c == 0x01 ? 1 : c == 0x02 ? 3 : c == 0x03 ? 35 : 0; }
    void command_done() {
        uint8_t c = byte_at(0);
        std::vector<uint8_t> r;
        if (c == 0x00 || c == 0xFF) r = {0x05, 0x00, (uint8_t)(pak ? 0x01 : 0x02)};
        else if (c == 0x01) r = {(uint8_t)(reply >> 24), (uint8_t)(reply >> 16), (uint8_t)(reply >> 8), (uint8_t)reply};
        else if (c == 0x02 && pak) {
            uint32_t a = ((byte_at(1) << 8) | byte_at(2)) & 0xFFE0u;
            std::vector<uint8_t> d = mem.count(a) ? mem[a] : std::vector<uint8_t>(32, 0xFF);
            r = d; r.push_back(crc_of(d.data()));
        } else if (c == 0x03 && pak) {
            uint32_t a = ((byte_at(1) << 8) | byte_at(2)) & 0xFFE0u;
            std::vector<uint8_t> d(32);
            for (int i = 0; i < 32; i++) d[i] = byte_at(3 + i);
            mem[a] = d;
            r = {crc_of(d.data())};
        }
        for (int i = 0; i < short_by && !r.empty(); i++) r.pop_back();
        if (!r.empty()) begin_reply(r);
    }
    bool line_in(bool master_low) {
        bool l = !(master_low || low || stuck_low);
        if (conn && !replying) {
            if (!l && prev) { if (cyc - rise_c > us(20)) bits.clear(); fall_c = cyc; }   // (a quiet line this long: a command begins afresh)
            if (l && !prev) rise_c = cyc;
            if (l && !prev && (cyc - fall_c) > us(20)) bits.clear();     // (a long low is no bit: the pad starts over)
            else if (l && !prev) {
                bool bit = (cyc - fall_c) < us(2.0);
                bits.push_back(bit);
                if (bits.size() >= 8) {
                    int n = cmd_len(byte_at(0));
                    if (n == 0) bits.clear();
                    else if (bits.size() == (size_t)(8 * n + 1)) { bits.pop_back(); command_done(); bits.clear(); }   // (the stop bit)
                }
            }
            prev = l;
        } else prev = true;
        return l;
    }
    void tick() {
        if (!replying) { low = false; return; }
        if (cyc < start_c) { low = false; return; }
        if (cell >= cells.size()) { replying = false; low = false; bits.clear(); prev = true; return; }
        auto [lo, hi] = cells[cell];
        low = cell_c < lo;
        if (++cell_c >= lo + hi) { cell++; cell_c = 0; }
    }
};
static Pad pad[4];

// waveform recording
static bool rec = false;
static std::vector<long> fall_t, rise_t;
static std::vector<int> fall_port;
static bool prev_drive = false;
static long multi_drive = 0;        // cycles with more than one line pulled low, or the 4th line pulled

static void step()
{
    t->clk = 0; t->eval(); t->clk = 1; t->eval();
    uint32_t dv = t->drive & 0xF;
    bool d = dv != 0;
    if ((dv & (dv - 1)) || (dv & 8)) multi_drive++;
    if (rec) { if (d && !prev_drive) { fall_t.push_back(cyc); fall_port.push_back(__builtin_ctz(dv)); } if (!d && prev_drive) rise_t.push_back(cyc); }
    prev_drive = d;
    uint32_t si = 0;
    for (int i = 0; i < 4; i++) { si |= (uint32_t)pad[i].line_in((dv >> i) & 1) << i; pad[i].tick(); }
    t->si = si;
    cyc++;
}
static void run(long n) { while (n-- > 0) step(); }
static void run_ms(double ms) { run(us(ms * 1000.0)); }

// the runtime's registers
static void csw(int reg, uint32_t v) { t->cs_wr = 1; t->cs_wa = reg; t->cs_wd = v; step(); t->cs_wr = 0; }
static uint32_t csr(int reg, bool pop = false)
{
    t->cs_ra = reg; t->cs_rd = pop; t->eval();
    uint32_t v = t->cs_rdata;
    step(); t->cs_rd = 0;
    return v;
}
static uint32_t status() { return csr(3); }
enum { ST_BUSY = 1, ST_DONE = 2 };
static int st_res(uint32_t s) { return (s >> 2) & 3; }
static int st_got(uint32_t s) { return (s >> 4) & 63; }
static void go(int port, int txn, int rxn) { csw(0, (uint32_t)txn | (uint32_t)rxn << 6 | (uint32_t)port << 12 | 1u << 16); }
static void load(const std::vector<uint8_t> &b) { csw(1, 0); run(3); for (uint8_t x : b) csw(2, x); }
static std::vector<uint8_t> fetch(int n) { csw(1, 0); run(3); std::vector<uint8_t> r; for (int i = 0; i < n; i++) { run(3); r.push_back((uint8_t)csr(2, true)); } return r; }
// one whole transaction: returns the cycles it was BUSY; the status in `st`
static uint32_t st_last;
static long xfer(int port, const std::vector<uint8_t> &tx, int rxn, double limit_ms = 6)
{
    load(tx);
    go(port, (int)tx.size(), rxn);
    long c0 = cyc;
    long lim = us(limit_ms * 1000.0);
    while (cyc - c0 < lim) { if (!(status() & ST_BUSY)) break; }
    st_last = status();
    return cyc - c0;
}

static uint16_t exp_btn(uint32_t w) { return (uint16_t)((w >> 16) & 0xFF3Fu); }
static uint16_t exp_stk(uint32_t w) { return (uint16_t)(((w & 0xFF) << 8) | ((w >> 8) & 0xFF)); }   // {Y, X}
static uint16_t got_btn(int i) { return (uint16_t)(t->btn >> (16 * i)); }
static uint16_t got_stk(int i) { return (uint16_t)(t->stick >> (16 * i)); }

static void check_pads(const char *what, uint32_t mask)
{
    CHECK(t->present == mask, "%s: present %x, wanted %x", what, (unsigned)t->present, mask);
    for (int i = 0; i < 3; i++) {
        if (mask & (1u << i)) {
            CHECK(got_btn(i) == exp_btn(pad[i].reply), "%s: pad %d buttons %04x, wanted %04x", what, i + 1, got_btn(i), exp_btn(pad[i].reply));
            CHECK(got_stk(i) == exp_stk(pad[i].reply), "%s: pad %d stick %04x, wanted %04x", what, i + 1, got_stk(i), exp_stk(pad[i].reply));
        } else {
            CHECK(got_btn(i) == 0 && got_stk(i) == 0, "%s: pad %d absent but its words are %04x %04x", what, i + 1, got_btn(i), got_stk(i));
        }
    }
    CHECK(got_btn(3) == 0 && got_stk(3) == 0, "%s: port 4 has words %04x %04x", what, got_btn(3), got_stk(3));
}

int main()
{
    t = new Van_joybus;
    t->en = 0; t->si = 0xF; t->cs_wr = 0; t->cs_rd = 0; t->cs_wa = 0; t->cs_ra = 0; t->cs_wd = 0;
    for (int i = 0; i < 4; i++) pad[i].conn = true;
    run(100);
    // en low: the lines are let go, no pad, no request taken
    pad[0].reply = 0x8000007F; run_ms(3);
    CHECK(t->drive == 0 && t->present == 0, "en low: drive %x present %x", (unsigned)t->drive, (unsigned)t->present);
    go(0, 1, 3); run(10);
    CHECK((status() & ST_BUSY) == 0, "en low: a go is taken");
    t->en = 1;

    // three pads, three different states: A+Start with the stick right/up, nothing, everything; the 4th port is never heard
    pad[0].reply = 0x9000'507F;
    pad[1].reply = 0x0000'0000;
    pad[2].reply = 0xFFFF'3F3F;          // all buttons, the reset flag (bit 23 of the reply: 0x0080 in the buttons) set below
    pad[3].reply = 0x0123'80B0;
    pad[2].reply |= 0x00800000;          // the reset flag
    run_ms(3);
    check_pads("three pads", 0x7);
    CHECK((got_btn(2) & 0x00C0) == 0, "the reset flag and bit 6 are out: %04x", got_btn(2));
    CHECK((status() >> 16 & 7) == 7, "status: present %x", (unsigned)(status() >> 16 & 7));

    // buttons and stick change
    pad[0].reply = 0x0800'FF01; pad[2].reply = 0x2000'0080;
    run_ms(3);
    check_pads("changed", 0x7);

    // ports 2 and 3 only
    pad[0].conn = false;
    run_ms(2);
    CHECK((t->present & 0x1) == 0x1, "a pad just out is still there (the poll's miss count): %x", (unsigned)t->present);
    run_ms(20);
    CHECK((t->present & 0x1) == 0x1, "after 20 ms (fewer than MISS polls) it is still there: %x", (unsigned)t->present);
    run_ms(30);
    check_pads("ports 2, 3", 0x6);
    pad[0].conn = true;
    run_ms(2.5);
    check_pads("port 1 back", 0x7);

    // other speeds and delays: +-4 % cells, a reply 1 us, 6 us and 30 us after the command
    double sp[3] = {0.96, 1.04, 0.98}, dl[3] = {1.0, 6.0, 30.0};
    for (int i = 0; i < 3; i++) { pad[i].speed = sp[i]; pad[i].delay_us = dl[i]; }
    pad[0].reply = 0xA5C3'1234; pad[1].reply = 0x5A3C'EDCB; pad[2].reply = 0x1234'80FF;
    run_ms(4);
    check_pads("speeds and delays", 0x7);
    for (int i = 0; i < 3; i++) { pad[i].speed = 1.0; pad[i].delay_us = 2.0; }

    // a line stuck low (a short): no falling edge to hear, so no pad
    pad[1].stuck_low = true;
    run_ms(45);
    CHECK((t->present & 0x2) == 0 && (t->present & 0x5) == 0x5, "port 2's line stuck low: present %x", (unsigned)t->present);
    pad[1].stuck_low = false; run_ms(3);
    check_pads("line free again", 0x7);

    // the master's waveform: nine low pulses a poll (the command 0x01 and the stop bit), a 4 us cell, a slot of 250 us, the ports in turn
    fall_t.clear(); rise_t.clear(); fall_port.clear(); rec = true; run_ms(2.3); rec = false;
    size_t k0 = 0;
    for (size_t i = 1; i < fall_t.size(); i++) if (fall_t[i] - fall_t[i - 1] > us(100)) { k0 = i; break; }
    CHECK(k0 > 0 && fall_t.size() >= k0 + 10 && rise_t.size() >= k0 + 9, "no complete poll recorded (falls %zu, k0 %zu)", fall_t.size(), k0);
    if (k0 > 0 && fall_t.size() >= k0 + 10 && rise_t.size() >= k0 + 9) {
        size_t r0 = 0; while (r0 < rise_t.size() && rise_t[r0] < fall_t[k0]) r0++;
        int bits[9] = {0, 0, 0, 0, 0, 0, 0, 1, 1};        // 0x01, the stop bit
        for (int i = 0; i < 9; i++) {
            long w = rise_t[r0 + i] - fall_t[k0 + i];
            long want = bits[i] ? 62 : 186;
            CHECK(labs(w - want) <= 2, "pulse %d: %ld cycles low, wanted %ld", i, w, want);
            if (i < 8) { long cell = fall_t[k0 + i + 1] - fall_t[k0 + i]; CHECK(labs(cell - 248) <= 2, "cell %d: %ld cycles, wanted 248", i, cell); }
        }
        for (int i = 1; i < 9; i++) CHECK(fall_port[k0 + i] == fall_port[k0], "poll's fall %d on port %d, the poll on port %d", i, fall_port[k0 + i] + 1, fall_port[k0] + 1);
        long per = fall_t[k0 + 9] - fall_t[k0];
        CHECK(per > us(240) && per < us(260), "slot %ld us", (long)(per / 62.5));
        CHECK(fall_port[k0 + 9] == (fall_port[k0] + 1) % 3, "next slot's port %d after port %d", fall_port[k0 + 9] + 1, fall_port[k0] + 1);
        int seen = 0;
        for (size_t i = k0; i < fall_t.size(); i++) seen |= 1 << fall_port[i];
        CHECK(seen == 0x7, "ports polled in 2.3 ms: %x", seen);
    }
    CHECK(multi_drive == 0, "%ld cycles with more than one line pulled low (or the 4th)", multi_drive);

    // ---- the runtime's transactions
    // the status: a pad with no accessory, then with one (the command 0xFF too)
    xfer(0, {0x00}, 3);
    { auto r = fetch(3); CHECK(!(st_last & ST_BUSY) && (st_last & ST_DONE) && st_res(st_last) == 0 && st_got(st_last) == 3, "status: %x", (unsigned)st_last);
      CHECK(r[0] == 0x05 && r[1] == 0x00 && r[2] == 0x02, "status reply %02x %02x %02x", r[0], r[1], r[2]); }
    pad[1].pak = true;
    xfer(1, {0xFF}, 3);
    { auto r = fetch(3); CHECK(st_res(st_last) == 0 && r[0] == 0x05 && r[2] == 0x01, "pad 2 with an accessory: res %d reply %02x %02x %02x", st_res(st_last), r[0], r[1], r[2]); }
    // the buttons by the runtime too (the same pad, the same bytes)
    xfer(2, {0x01}, 4);
    { auto r = fetch(4); uint32_t w = pad[2].reply; CHECK(st_res(st_last) == 0 && r[0] == (w >> 24) && r[1] == ((w >> 16) & 255) && r[2] == ((w >> 8) & 255) && r[3] == (w & 255), "buttons by the runtime: %02x%02x%02x%02x", r[0], r[1], r[2], r[3]); }

    // a block written and read back (the accessory of pad 2): 35 bytes out, 1 in; then 3 out, 33 in
    {
        std::vector<uint8_t> w = {0x03, 0x80, 0x20};
        std::vector<uint8_t> d(32); for (int i = 0; i < 32; i++) d[i] = (uint8_t)(0x40 + 3 * i);
        w.insert(w.end(), d.begin(), d.end());
        long c = xfer(1, w, 1);
        auto r = fetch(1);
        CHECK(st_res(st_last) == 0 && st_got(st_last) == 1 && r[0] == crc_of(d.data()), "pak write: res %d got %d crc %02x", st_res(st_last), st_got(st_last), r[0]);
        CHECK(c > us(1100) && c < us(1500), "a pak write took %ld us (wanted 1100..1500)", (long)(c / 62.5));
        CHECK(pad[1].mem.count(0x8020) && pad[1].mem[0x8020] == d, "the accessory has not the block");
        c = xfer(1, {0x02, 0x80, 0x3F}, 33);       // (the address is a block's: the low 5 bits are the CRC in the real thing, and mean nothing here)
        auto rr = fetch(33);
        CHECK(st_res(st_last) == 0 && st_got(st_last) == 33, "pak read: res %d got %d", st_res(st_last), st_got(st_last));
        bool same = true; for (int i = 0; i < 32; i++) same &= rr[i] == d[i];
        CHECK(same && rr[32] == crc_of(d.data()), "pak read back differs (crc %02x)", rr[32]);
        CHECK(c > us(1100) && c < us(1500), "a pak read took %ld us", (long)(c / 62.5));
        run_ms(2);
        check_pads("after the pak", 0x7);
    }

    // a pad that is not there: nothing comes; the time it is waited for
    pad[0].conn = false;
    { long c = xfer(0, {0x00}, 3); CHECK(st_res(st_last) == 1 && st_got(st_last) == 0, "no pad: res %d got %d", st_res(st_last), st_got(st_last));
      CHECK(c > us(500) && c < us(800), "no pad: %ld us", (long)(c / 62.5)); }
    pad[0].conn = true;
    // a reply that is short (the pad says 3 bytes, 5 are asked, then 1 byte fewer than a status)
    xfer(0, {0x00}, 5);
    { auto r = fetch(3); CHECK(st_res(st_last) == 2 && st_got(st_last) == 3 && r[0] == 0x05, "short reply: res %d got %d", st_res(st_last), st_got(st_last)); }
    pad[0].short_by = 1;
    xfer(0, {0x00}, 3);
    CHECK(st_res(st_last) == 2 && st_got(st_last) == 2, "a status of 2 bytes: res %d got %d", st_res(st_last), st_got(st_last));
    pad[0].short_by = 0;
    // the 4th port: not there for the engine, however the model answers
    { long c = xfer(3, {0x00}, 3); CHECK(st_res(st_last) == 1 && st_got(st_last) == 0 && c < us(100), "the 4th port: res %d got %d in %ld us", st_res(st_last), st_got(st_last), (long)(c / 62.5)); }

    // a request at every moment of the polling: it waits for the poll that is on, runs, and the polling goes on
    for (int k = 0; k < 24; k++) {
        run(us(37 * k + 5));
        xfer(k % 3, {0x01}, 4);
        auto r = fetch(4); uint32_t w = pad[k % 3].reply;
        CHECK(st_res(st_last) == 0 && r[0] == (w >> 24) && r[3] == (w & 255), "request %d at +%d us: res %d %02x..%02x", k, 37 * k + 5, st_res(st_last), r[0], r[3]);
    }
    pad[0].reply = 0x1234'5678; run_ms(3);
    check_pads("polling after the requests", 0x7);

    // a second go while busy is no go
    {
        load({0x00}); go(0, 1, 3);
        run(20);
        load({0x01}); go(1, 1, 4);              // (the buffer is rewritten too: that is the runtime's own doing; the go is what is checked)
        long c0 = cyc; while ((status() & ST_BUSY) && cyc - c0 < us(3000)) {}
        CHECK(st_got(status()) == 3 && st_res(status()) == 0, "second go while busy: got %d res %d", st_got(status()), st_res(status()));
        run_ms(1);
        CHECK((status() & ST_BUSY) == 0, "a second go while busy left it busy");
    }

    // no pad at all
    for (int i = 0; i < 4; i++) pad[i].conn = false;
    run_ms(60);
    CHECK(t->present == 0 && t->btn == 0 && t->stick == 0, "no pads: present %x", (unsigned)t->present);
    for (int i = 0; i < 4; i++) pad[i].conn = true;
    run_ms(3);
    check_pads("all back", 0x7);
    CHECK(multi_drive == 0, "%ld cycles with more than one line pulled low (or the 4th)", multi_drive);

    // en low again: everything off at once
    t->en = 0; run(10);
    CHECK(t->drive == 0 && t->present == 0 && t->btn == 0 && t->stick == 0, "en low: not cleared (present %x)", (unsigned)t->present);

    printf("%d checks, %d failed\n", checks, fails);
    printf(fails ? "SOME FAILED\n" : "ALL PASS\n");
    return fails ? 1 : 0;
}
