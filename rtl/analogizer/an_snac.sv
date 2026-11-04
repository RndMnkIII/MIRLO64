// The Analogizer's SNAC port: a DB15, NES or SNES pad (one or two players) on the adapter's USB 3 connector, read into the
// Pocket's key words (docs/analogizer.md has the pin map and the settings). Written for this core from the adapter's pin map and from
// the pads' shift registers (4021s in the NES and SNES pads, 74HC165s on a DB15 board); none of it is taken from another project.
//
//   pin map (adapter configuration A)         NES / SNES                DB15
//     OUT1  bank 1 bit 6   output             CLK, both ports           CLK
//     OUT2  bank 1 bit 7   output             LAT (active high)         /LAT (active low)
//     IO5   pin 30         output             CLK, port 2 (a copy)      -
//     IO3   bank 0 bit 4   input              data, player 1            data (the chain of both players)
//     IO6   pin 31         input              -                         the same data, for a cable that crosses TX and RX
//     IN7   bank 0 bit 5   input              data, player 2            -
//
// A poll is a latch pulse and then a clock pulse for each further bit. A tick is half a clock period: 10 us for the NES and the SNES
// (50 kHz, a good deal slower than a console's 83 kHz), 2.5 us for a DB15 (200 kHz). Seen from the pad, with n the tick's number:
//   - the latch is active after ticks 0 and 1 (20 us / 5 us) and idle from tick 2 on;
//   - the clock idles low (NES, SNES: it stays low until tick 3) or high (DB15, which toggles from the start: the latch overrides
//     the register while it is active), and rises after each odd tick from 3 on: each rising edge shifts the pad's register one bit;
//   - the data pin is sampled at the end of each even tick from 2 on: the first bit is there with the latch (before any clock edge),
//     each further one at the end of the clock's high tick, which is where a console samples.
// The pads send a pressed button as 0. A poll ends with a tick that publishes all its bits at once (a key word never mixes two
// polls) and starts the next one at once.
//
// SNES pads have four more bits after the twelve buttons; they are not read.
//
// `kind`: bits 1:0 the pad (0 off, 1 DB15, 2 NES, 3 SNES), bit 2 a second player (DB15: the second player of the chain). The pins are
// the adapter's only when `own` says so (the cartridge port is a Game Boy cartridge's otherwise) and a pad is chosen: `drive`.
// The keys are the Pocket's bits (docs/control.md, APF_INPUT: 0 up, 1 down, 2 left, 3 right, 4 A, 5 B, 6 X, 7 Y, 8 L1, 9 R1,
// 14 Select, 15 Start; 1 is pressed), which a pad's buttons take by where they sit: the SNES pad's diamond is the Pocket's.
`default_nettype none
module an_snac #(
    parameter int CLK_HZ = 62_500_000       // clk's frequency (clk_sys)
) (
    input  wire         clk,
    input  wire         own,                // the adapter owns the cartridge port
    input  wire  [2:0]  kind,               // see above
    input  wire         dat1,               // IO3: player 1's data (a DB15's data)
    input  wire         dat1b,              // IO6: the DB15's data again (high, idle, when the cable does not cross)
    input  wire         dat2,               // IN7: player 2's data (NES, SNES)
    output logic        clk_o,              // OUT1 and IO5
    output logic        lat_o,              // OUT2
    output wire         drive,              // the SNAC pins are the pad's: outputs on, bank 0 an input
    output logic [15:0] key1,
    output logic [15:0] key2
);

localparam int TICK_SLOW = CLK_HZ / 100_000;    // 10 us: the NES's and the SNES's half period
localparam int TICK_FAST = CLK_HZ / 400_000;    // 2.5 us: a DB15's

wire [1:0] ty  = kind[1:0];
wire       en  = own && ty != 2'd0;
wire       two = kind[2];
wire       db  = ty == 2'd1;
assign drive = en;

// the data pins, in clk's domain
logic [1:0] s1, s1b, s2;
always_ff @(posedge clk) begin s1 <= {s1[0], dat1}; s1b <= {s1b[0], dat1b}; s2 <= {s2[0], dat2}; end
wire d1 = db ? (s1[1] & s1b[1]) : s1[1];
wire d2 = s2[1];

// the tick; a change of pad (or of the port's owner) starts it over
logic [1:0] ty_q;
logic       en_q;
logic [9:0] div;
logic       tick;
wire        restart = !en || !en_q || ty != ty_q;
always_ff @(posedge clk) begin
    ty_q <= ty;
    en_q <= en;
    tick <= 1'b0;
    if (restart) div <= 10'd0;
    else if (div == (db ? 10'(TICK_FAST - 1) : 10'(TICK_SLOW - 1))) begin div <= 10'd0; tick <= 1'b1; end
    else div <= div + 10'd1;
end

// the poll
logic        run;
logic [5:0]  n;
logic [23:0] b1;                            // what the data pins gave, bit by bit (1: released)
logic [11:0] b2;
wire  [5:0]  last = db ? 6'd48 : (ty == 2'd3) ? 6'd24 : 6'd16;      // 2 x the bits: 24 (DB15), 12 (SNES), 8 (NES)
wire  [4:0]  bit_ix = 5'(n[5:1] - 5'd1);

// a pad's bits to the Pocket's keys (1: pressed)
function automatic logic [15:0] nes_keys(input logic [11:0] b);     // A B SEL START UP DOWN LEFT RIGHT
    return ~{b[3], b[2], 8'hFF, b[1], b[0], b[7], b[6], b[5], b[4]};
endfunction
function automatic logic [15:0] snes_keys(input logic [11:0] b);    // B Y SEL START UP DOWN LEFT RIGHT A X L R
    return ~{b[3], b[2], 4'hF, b[11], b[10], b[1], b[9], b[0], b[8], b[7], b[6], b[5], b[4]};
endfunction
// a DB15's chain: P1 D C B A RIGHT LEFT DOWN UP, P2 RIGHT LEFT DOWN UP, P1 F E SELECT START, P2 F E SELECT START, P2 D C B A
function automatic logic [15:0] db15_keys1(input logic [23:0] b);
    return ~{b[15], b[14], 4'hF, b[12], b[13], b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]};
endfunction
function automatic logic [15:0] db15_keys2(input logic [23:0] b);
    return ~{b[19], b[18], 4'hF, b[16], b[17], b[20], b[21], b[22], b[23], b[8], b[9], b[10], b[11]};
endfunction

always_ff @(posedge clk) begin
    if (restart) begin
        run <= 1'b0; n <= 6'd0; clk_o <= 1'b0; lat_o <= 1'b0; key1 <= 16'd0; key2 <= 16'd0; b1 <= '1; b2 <= '1;
    end else if (tick) begin
        if (!run) begin                                     // the poll's first tick: the last one's keys, the idle levels
            case (ty)
            2'd1:    begin key1 <= db15_keys1(b1);         key2 <= two ? db15_keys2(b1) : 16'd0; end
            2'd2:    begin key1 <= nes_keys(b1[11:0]);     key2 <= two ? nes_keys(b2)  : 16'd0; end
            default: begin key1 <= snes_keys(b1[11:0]);    key2 <= two ? snes_keys(b2) : 16'd0; end
            endcase
            n <= 6'd0; run <= 1'b1; b1 <= '1; b2 <= '1;
            lat_o <= db; clk_o <= db;
        end else begin
            n <= n + 6'd1;
            lat_o <= (n <= 6'd1) ? !db : db;
            clk_o <= db ? n[0] : (n[0] && n >= 6'd3);
            if (n >= 6'd2 && !n[0]) begin
                b1[bit_ix] <= d1;
                if (bit_ix < 5'd12) b2[bit_ix[3:0]] <= d2;
            end
            if (n == last) run <= 1'b0;
        end
    end
end

endmodule
