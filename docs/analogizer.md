# Analogizer support

The Analogizer is an add-on that plugs into the Pocket's cartridge slot and
turns the core's video into analog signals (RGBS, RGsB, YPbPr, Y/C, RGBHV).
The slot is also where a Game Boy cartridge goes (the Transfer Pak setting
"Transfer Pak cartridge", rtl/soc/gb_cart.sv), so the two share the 32 pins:
at any time one of them owns the port.

The modules under `rtl/analogizer` are written for this core from the
adapter's pin map and from the video standards, with one exception:
`serlatch_gc.v`, the SNAC pad reader, is the author's own from the
Xain'd Sleena core (Analogizer_openFPGA-XaindSleena), where it runs on real pads.

## The port

Every pin goes through a level translator whose direction is the bank's `_dir`
(1: the FPGA drives). With the adapter in the slot the FPGA drives banks 1, 2
and 3 and the adapter's DAC reads them:

| bank | bit 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
|---|---|---|---|---|---|---|---|---|
| `cart_tran_bank3` | R5 | R4 | R3 | R2 | R1 | R0 | HS pin | VS pin |
| `cart_tran_bank2` | B0 | /BLANK | G5 | G4 | G3 | G2 | G1 | G0 |
| `cart_tran_bank1` | spare | spare | pixel clock | B5 | B4 | B3 | B2 | B1 |

The DAC takes six bits a channel (the top six of the core's eight). `HS pin`
and `VS pin` are the sync outputs; what each mode puts on them:

| mode | R | G | B | HS pin | VS pin | /BLANK |
|---|---|---|---|---|---|---|
| RGBS | R | G | B | csync | 1 | blank |
| RGsB (sync on green) | R | G | B | 1 | csync (the DAC's SYNC) | blank |
| YPbPr | Pr | Y | Pb | 1 | csync (the DAC's SYNC) | 1 |
| Y/C NTSC, PAL | C | Y | 0 | csync | 1 | 1 |
| RGBHV (scandoubled) | R | G | B | hsync | vsync | 1 |

`/BLANK` is active low; in the RGB modes the colour is also forced to 0 in the
blanking. Bank 0 (bits 7:4) and pins 30 and 31 belong to the SNAC controller
port and are not used by the video; bank 0 is driven high unless a SNAC pad is
chosen ([SNAC controllers](#snac-controllers)). The two spare bits of bank 1 (7 and 6) are
the SNAC port's latch and clock outputs.

## What the scan-out sends

`rtl/soc/n64_video.sv` sends the N64 VI's own timing: a 640-pixel line of 790
(NTSC) or 800 (PAL) pixel clocks, 264 / 313 lines a field, 59.94 / 50 Hz, a
pixel each two clocks of the 25 MHz `clk_vid` (12.5 MHz pixels, 15.8 kHz
lines), `vga_skip` marking the second clock of a pixel. That is a 240p signal
a CRT takes as it is: the 15 kHz modes (RGBS, RGsB, YPbPr, Y/C) use it
directly. A serrated hi-res picture is woven by the scan-out into 480 lines a
frame, 640 x 480 at 31.6 kHz, a pixel a clock: that is a VGA signal, which
only the RGBHV mode shows; for the 15 kHz modes the weave is switched off
(the scan-out's `no_weave` input), so a serrated picture goes out field by
field, 240 lines each, as 240p.

The RGBHV mode doubles the lines of the 240p signal (31.6 kHz, 25 MHz pixels)
and passes a woven frame as it is.

## Modules (rtl/analogizer)

- `an_csync`: composite sync from the scan-out's hsync and vsync (hs, with the vertical interval's broad pulses and
  serrations; active low, `CSYNC_ACTIVE_LOW`).
- `an_ypbpr`: the RGB to Y, Pb, Pr matrix (BT.601, studio levels): nine products by constants, which the synthesis puts in
  four DSP blocks, summed in two clocks.
- `an_yc`: the luma and chroma for S-Video and composite, NTSC and PAL (the adapter makes the composite by adding the
  two): a 22-bit carrier, two sine tables (U and V, with the weights and the gain in them, so no multiplier scales U and
  V), U and V low-passed, the burst in the back porch, the V axis switched every line in PAL (by reading the V table
  half a turn on). R carries the chroma, centred on 128; G the luma; B is 0; csync goes out on the HS pin. It also hands
  out its own matrix and sync for the YPbPr mode, so the top builds one of each.
- `an_scandbl`: the line doubler for RGBHV: 240p lines written once and read twice at the 25 MHz pixel rate (31.6 kHz,
  528 lines), the scanlines option dimming the second copy; a woven frame goes through as it is. The line buffer keeps
  six bits a channel, which is all the DAC takes.
- `an_snac`: the SNAC pad reader (below); it runs on the SoC's clock, not the video's.
- `an_out`: the mode multiplexer and the pin map above; everything registered on `clk_vid`. The colours of RGBS and RGsB
  go through `an_scandbl`'s pass-through (as the RGBHV woven frame does), a clock behind the scan-out's, so that one
  colour path serves the three RGB modes.

Tests: `sim/analogizer/run_tests.sh` (Verilator, against floating-point models in Python; `an_out` checks the pin map of
every mode on the scan-out's timing; `an_snac` is read against models of the pads' shift registers) and `sim/analogizer/scandbl/run_tests.sh`.

### Resources

The five modules together take about 180 ALMs, 5 DSP blocks (four in the matrix, one for the modulator) and 4 M10K blocks
(two for the line buffer, one for each sine table) on the Cyclone V (measured on `an_out` alone, with the core's synthesis
settings). The U and V weights are in the sine tables, the YPbPr matrix is in DSP blocks, and the RGB modes share the
colour path of the line doubler, which is what keeps them small. The matrix is a plain multiplication by constants: if
the DSP blocks run out the synthesis builds them in logic, at about 75 ALMs more (125 against 50).

### What a monitor has to confirm

The modules are checked in simulation against the standards, not against an adapter; these are the assumptions:

- the csync polarity (`CSYNC_ACTIVE_LOW`) and the RGBHV polarities (`HS_ACTIVE_LOW`, `VS_ACTIVE_LOW`);
- the Y and C levels at the adapter's resistor network (`CGAIN` of `an_yc`: 12/16) and the burst's level;
- the burst's place: 0.6 us (NTSC) after the sync; the scan-out's sync is 3.84 us wide (the standard's 4.7);
- the carrier's phase restarts every line (the line rate is the scan-out's 15.82 kHz: a receiver relocks on each burst);
- in RGBS and RGsB the colour is one clock (40 ns) behind the scan-out's and the sync two behind it (the YPbPr and Y/C
  modes have each their own delay, the sync and the signal aligned);
- the RGBHV frame is 528 lines at 59.94 Hz, vsync 8 lines wide (`VS_LINES`, `VS_DELAY` for the standard's 2);
- the DAC's clock (bank 1 bit 5) is `clk_vid_90deg`: its phase against the data may need the other edge.

## Who owns the port

There is no pin that tells an Analogizer from a cartridge, and no setting for
it: the runtime, at the core's start, brings the port up as a Game Boy cartridge's host and
reads its header: the Nintendo logo and the header checksum, the test the Transfer Pak
code already makes before it trusts a cartridge. If the header is a Game Boy's
the port stays the cartridge's; if not (an Analogizer, or an empty slot) the
Analogizer takes the port and shows the video mode of "Analogizer video". The
probe puts addresses and /RD on the pins the adapter reads as video for a few
milliseconds, once, at the core's start; the slot is not hot-pluggable, so
that is the only time it is looked at.

## SNAC controllers

A DB15, NES or SNES pad on the adapter's SNAC port (its USB 3 connector) plays as an N64 controller. `rtl/analogizer/an_snac.sv`
(the strobe, the port's ownership and the menu's pad type around `serlatch_gc.v`, the reader) polls the pad and gives its buttons as the Pocket's key bits, which `rtl/soc/mirlo_regs.sv` ORs into `CONT1_KEY` (and, with a second player,
`CONT2_KEY`); the runtime then sees one more source of the same keys, and the "Stick", "C buttons", "Z button" ... settings apply to it as to the
Pocket's own buttons. The Pocket's buttons and the dock's pads go on working at the same time. The SNAC port belongs to the adapter only
when it owns the cartridge port ("Who owns the port", below): with a Game Boy cartridge in the slot the setting is ignored.

The pins (the adapter's configuration A: its A/B switch on A):

| signal | cartridge port pin | direction | NES, SNES | DB15 |
|---|---|---|---|---|
| OUT1 | `cart_tran_bank1[6]` | out | clock, both ports | clock |
| OUT2 | `cart_tran_bank1[7]` | out | latch (active high) | latch (active low) |
| IO5 | `cart_tran_pin30` | out | clock of port 2, a copy of OUT1 | - |
| IO3 | `cart_tran_bank0[4]` | in | data, player 1 | data of the chain (both players) |
| IO6 | `cart_tran_pin31` | in | - | the same data, for a cable that crosses TX and RX |
| IN7 | `cart_tran_bank0[5]` | in | data, player 2 | - |

With a pad chosen bank 0 is an input (`cart_tran_bank0_dir` low) and bank 1 an output, also when the video is off (its bits 5:0 are then 0).
The pads send a pressed button as 0. A poll is a latch pulse and a clock pulse for each further bit, and goes on all the time: the NES's and the SNES's clock
is 50 kHz (a console's is 83 kHz), a poll of 360 us (NES and SNES: the reader clocks 16 bits, the buttons are the first 8 and the first 12), and the DB15's 200 kHz, 125 us
for its 24 bits. The bench measures the waveforms.

The buttons go to the Pocket's keys by where they sit (the SNES's diamond is the Pocket's):

| pad | buttons |
|---|---|
| SNES | D-pad, A, B, X, Y, L (the Pocket's L1), R (R1), Select, Start |
| NES | D-pad, A, B, Select, Start |
| DB15 | D-pad, A, B, C (X), D (Y), E (L1), F (R1), Select, Start; player 2's the same, from the second half of the chain |

The second player (`NES 2P`, `SNES 2P`, `DB15 2P`) is controller 2: the second port of a NES or SNES adapter, the second half of a DB15's chain. A controller
2 the dock does not report is told to the runtime as a dock pad without analog (`KEY_TYPE` 2), because the runtime takes a pad with type 0 for no controller
(`pad_there` in `lang/n64rt/n64rt_io.c`); the choice is the only thing that says a second pad is there, so with `NES`, `SNES` or `DB15` alone the N64 sees one.

## N64 controllers

Up to four N64 controllers on the SNAC port play as the N64's own four, through `rtl/analogizer/an_joybus.sv`: the menu's "Analogizer controller"
= "N64 pads (1-4)". The adapter is the N64 SNAC board (a 1 kohm pull-up to 3.3 V on each data line, an LDO for the pads) with its cable harness, on the
adapter's configuration A:

| port | data line | cartridge port pin | direction |
|---|---|---|---|
| 1 | GND_D (IO5) | `cart_tran_pin30` | its own |
| 2 | TX- (IO6) | `cart_tran_pin31` | its own |
| 3 | RX- (IO3) | `cart_tran_bank0[4]` | bank 0's, one for the four bits |
| 4 | RX+ (IN4) | `cart_tran_bank0[7]` | bank 0's |

The Joybus has one wire a pad and nobody drives a 1: the FPGA pulls the line low (the pin an output with the value 0) or lets it go (an input) and the
pull-up makes the 1. Bank 0 has one direction for its four bits, so the four lines are always driven together and let go together: one poll, the command
`0x01` (the buttons) in 4 us cells and a stop bit, goes to every pad at once and each pad's reply is heard on its own line (about 175 us; a poll every
millisecond). A pad is there after one good reply and until 32 polls in a row have none, so a poll lost to noise is not a controller taken out. The pins'
levels while a poll is not going on are the pull-ups' (all lines let go); bank 1's bits 7:6 (OUT2, OUT1) and the pins the harness does not take are not used.

The pad's buttons are the N64's own, so they do not go through the Pocket's keys: while a pad is there, its port's `KEY` word (`CONT1_KEY` ... `CONT4_KEY`)
is `KEY_TYPE` 4 with the 16 buttons in the CONT_ layout in bits 15:0 (A at bit 15 ... C-right at bit 0; the pad's reset flag is cleared) and its `JOY` word
the stick's two signed bytes (X in 7:0, Y in 15:8). The runtime takes a type 4 pad as it is (`n64rt_pad_map` in `lang/n64rt/n64rt_pad.h`): the Stick,
C buttons, Z and the other settings of the Pocket's keys do not touch it. A port with no pad is what it was (the Pocket's own controls for port 1, the dock's pad
for 2 to 4, nothing otherwise), so a controller 2 to 4 is there when its pad answers; while a pad 1 answers, the Pocket's own buttons do not play.

What the Joybus does not do here: the Controller Pak, the Rumble Pak and the Transfer Pak of a pad (the pak commands are not sent; the pad's accessory is not
seen), and the VRU. The N64 SNAC board's LDO has to feed four pads (and a rumble motor draws more than a pad); its datasheet's output current is the limit.

The bench (`sim/analogizer/an_joybus`) has four models of the pad on four wires with pull-ups: every button and stick, a pad taken out and put back, other
speeds and reply delays, a line stuck low, the command's pulses and cells and the poll's period; `native_map.c` checks the runtime's type 4 on the host.

## Settings

Two entries in the core's menu: "Analogizer controller": Off, DB15, NES, SNES, DB15 2P, NES 2P, SNES 2P
(`INTERACT2[9:5]`: bits 1:0 the pad, bit 2 the second player; the value 4 is off); and "Analogizer video": Off, RGBS, RGsB, YPbPr, Y/C NTSC,
Y/C PAL, RGBHV, and RGBHV with scanlines 25 %, 50 %, 75 % (the scanlines dim the odd lines, which only the doubled
RGBHV picture has). The Pocket's own picture stays on.

The Pocket's menu has two limits, both measured on a Pocket: it shows only 14 of the core's entries (a 15th loads without
an error and is silently left out, so "Restart Game" has to stay the 14th, the last), and it refuses an `interact.json`
whose lists have more than about 75 options in all ("Load error in 'interact'": 75 loads, 77 does not, 76 untried). The
core's menu has 14 entries and 71 options: the two sticks share one "Stick dead zone" (steps of 10 %, 0 to 70 %) to make
room for the video entry, and the Expansion Pak and the GameShark codes, two checks, are one list of four ("Exp. Pak / Gameshark":
`INTERACT1[25:24]`, bit 24 the Pak out, bit 25 the codes on) to make room for the controller entry. 71 is four below the
75 that loads: the next entry that needs options has to come out of one of these.
