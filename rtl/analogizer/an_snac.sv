// The Analogizer's SNAC port: a DB15, NES or SNES pad (one or two players) on the adapter's USB 3 connector, read into the
// Pocket's key words (docs/analogizer.md has the pin map and the settings). The pads are read by serlatch_gc.v (the Analogizer's
// serial-latch reader, by @RndMnkIII, as it runs in the Xain'd Sleena core on real pads); this module is what the Pocket core needs around it:
// the strobe (a tick is half a clock period), the port's ownership and the menu's pad type.
//
//   pin map (adapter configuration A)         NES / SNES                DB15
//     OUT1  bank 1 bit 6   output             CLK, both ports           CLK
//     OUT2  bank 1 bit 7   output             LAT (active high)         /LAT (active low)
//     IO5   pin 30         output             CLK, port 2 (a copy)      -
//     IO3   bank 0 bit 4   input              data, player 1            data (the chain of both players)
//     IO6   pin 31         input              -                         the same data, for a cable that crosses TX and RX
//     IN7   bank 0 bit 5   input              data, player 2            -
//
// A tick is 10 us for the NES and the SNES (50 kHz clock), 2.5 us for a DB15 (200 kHz). The pads send a pressed button as 0.
//
// `kind`: bits 1:0 the pad (0 off, 1 DB15, 2 NES, 3 SNES), bit 2 a second player (DB15: the second player of the chain). The pins are
// the adapter's only when `own` says so (the cartridge port is a Game Boy cartridge's otherwise) and a pad is chosen: `drive`.
// The keys are the Pocket's bits (docs/control.md, APF_INPUT: 0 up, 1 down, 2 left, 3 right, 4 A, 5 B, 6 X, 7 Y, 8 L1, 9 R1,
// 14 Select, 15 Start; 1 is pressed).
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
    output wire         clk_o,              // OUT1 and IO5
    output wire         lat_o,              // OUT2
    output wire         drive,              // the SNAC pins are the pad's: outputs on, bank 0 an input
    output wire  [15:0] key1,
    output wire  [15:0] key2
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

// the strobe; a change of pad (or of the port's owner) starts it over
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

wire [15:0] p1, p2;
wire        clk2_unused;
serlatch_game_controller #(.MASTER_CLK_FREQ(CLK_HZ)) slgc (
    .i_clk(clk), .i_rst(restart), .game_controller_type({2'b00, en ? ty : 2'b00}), .i_stb(tick),
    .p1_btn_state(p1), .p2_btn_state(p2), .busy(),
    .o_clk(clk_o), .o_clk2(clk2_unused), .o_lat(lat_o), .i_dat1(d1), .i_dat2(d2)
);

// the reader's words are the old pad's until its first poll after a change: cleared (not live) till then
logic seen;
always_ff @(posedge clk) if (restart) seen <= 1'b0; else if (tick) seen <= 1'b1;
wire live = en && en_q && seen;
assign key1 = live ? p1 : 16'd0;
assign key2 = (live && two) ? p2 : 16'd0;

endmodule
