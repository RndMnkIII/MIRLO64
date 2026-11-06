/* The runtime's map for a pad of KEY_TYPE 4 (an N64 pad on the Analogizer's SNAC port, hardware: rtl/analogizer/an_joybus.sv): the
 * buttons come out as they went in, the stick's two signed bytes as they are, whatever the settings say; and a pad of another type is
 * as it was (the Pocket's with the default settings: A is A, the D-pad the stick). Host test: cc native_map.c -I../../../lang/n64rt */
#include <stdio.h>
#include "n64rt_pad.h"
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)
int main(void)
{
    uint16_t b; int8_t x, y;
    /* every button word, with the settings at their defaults and with every setting field full */
    const uint32_t sets[4] = { 0u, 0xFFFFFFFFu, 0x55555555u, 0xAAAAAAAAu };
    for (int si = 0; si < 4; si++)
        for (uint32_t w = 0; w < 0x10000u; w += 0x1u) {
            uint32_t k = (4u << 28) | w, j = ((uint32_t)(uint8_t)-80 << 8) | 0x7Fu;     /* X = +127, Y = -80 */
            n64rt_pad_map(k, j, sets[si], sets[(si + 1) & 3], &b, &x, &y);
            if (b != (uint16_t)w || x != 127 || y != -80) { CHECK(0, "type 4: word %04x settings %d -> %04x %d %d", w, si, b, x, y); return 1; }
        }
    /* the stick at the ends and the middle */
    const int8_t v[5] = { -128, -80, 0, 80, 127 };
    for (int i = 0; i < 5; i++) for (int m = 0; m < 5; m++) {
        n64rt_pad_map(4u << 28, ((uint32_t)(uint8_t)v[m] << 8) | (uint8_t)v[i], 0, 0, &b, &x, &y);
        CHECK(x == v[i] && y == v[m] && b == 0, "stick %d %d -> %d %d", v[i], v[m], x, y);
    }
    /* the Pocket's own pad, the defaults: A is A, the D-pad's up is the stick up */
    n64rt_pad_map((1u << 28) | KEY_A, 0, 0, 0, &b, &x, &y);
    CHECK(b == CONT_A && x == 0 && y == 0, "type 1 A: %04x %d %d", b, x, y);
    n64rt_pad_map((1u << 28) | KEY_DPAD_UP, 0, 0, 0, &b, &x, &y);
    CHECK(b == 0 && x == 0 && y == 80, "type 1 up: %04x %d %d", b, x, y);
    /* the accessory's address word: every block against the checksum computed bit by bit by the polynomial (a division), and the two words every
     * accessory driver knows: 0x8000 (the Rumble Pak's probe and motor) is sent as 0x8001, 0xC000 as 0xC01B */
    for (uint32_t blk = 0; blk < 0x800u; blk++) {
        uint32_t r = blk << 5;                       /* the 16-bit word, its low 5 bits the remainder's place */
        for (int i = 15; i >= 5; i--) if (r & (1u << i)) { r ^= 0x35u << (i - 5); }
        CHECK(n64rt_pak_addr_word(blk) == ((blk << 5) | (r & 0x1Fu)), "address word of block %x: %04x", blk, n64rt_pak_addr_word(blk));
    }
    CHECK(n64rt_pak_addr_word(0x8000u >> 5) == 0x8001u, "0x8000 -> %04x", n64rt_pak_addr_word(0x8000u >> 5));
    CHECK(n64rt_pak_addr_word(0xC000u >> 5) == 0xC01Bu, "0xC000 -> %04x", n64rt_pak_addr_word(0xC000u >> 5));
    CHECK(n64rt_pak_addr_word(0) == 0u, "0 -> %04x", n64rt_pak_addr_word(0));
    printf(fails ? "SOME FAILED\n" : "ALL PASS\n");
    return fails != 0;
}
