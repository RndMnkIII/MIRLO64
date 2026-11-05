// Up to four N64 controllers on the Analogizer's SNAC port (docs/analogizer.md, "N64 controllers"): the Joybus, one wire a pad.
//
// The adapter is the N64 SNAC board with its cable harness (config. A of the adapter's switch): the four data lines are
//     port 1  pin 30 (IO5)          port 2  pin 31 (IO6)          port 3  bank 0 bit 4 (IO3)          port 4  bank 0 bit 7 (IN4)
// each with a 1 kohm pull-up to 3.3 V on the board. The FPGA never drives a 1: a line is pulled low (the pin an output with the value 0) or
// let go (an input), the pull-up making the 1. The pads are polled one at a time, a slot of SLOT_US each (a round of four is 1 ms): only the
// polled port's line is pulled low (`drive`, a bit a port) and only its receiver listens (an_jb_rx); every other line is let go. (Bank 0
// has one direction for its two data bits, so core_top.sv holds the other of the two high while one is polled: the level the pull-up
// gives.) A pad that does not answer is a port that has no controller.
//
// A poll is the command 0x01 (the buttons), sent bit by bit in a 4 us cell -- a 1 is 1 us low and 3 us high, a 0 is 3 us low and 1 us
// high -- and a stop bit, 1 us low; then every pad that is there answers with 32 bits in the same cells, high bit first:
//     A B Z Start Up Down Left Right | Reset 0 L R C-up C-down C-left C-right | X (signed byte) | Y (signed byte)
// A reply bit is read 2 us after its falling edge: high a 1, low a 0. A poll is about 175 us (a port that does not answer, about 140) and a port's slot starts every SLOT_US.
//
// A pad is `present` after one good reply and until MISS polls in a row have none (a poll lost to noise is not a pad taken out). The
// buttons come out as the N64's own 16 bits (libultra's CONT_ layout: A is bit 15 ... C-right bit 0; the pad's reset flag, bit 7,
// is cleared) and the stick as {Y, X}: what the runtime takes from a pad of KEY_TYPE 4 (lang/n64rt/n64rt_pad.h).
`default_nettype none

// one line's receiver: armed when the pad is let go, it takes 32 bits or gives up
module an_jb_rx #(
    parameter int US = 62                   // clock cycles in a microsecond
) (
    input  wire         clk,
    input  wire         si,                 // the line
    input  wire         arm,                // a pulse: the command is sent, listen
    output logic        fin,                // a reply is taken (ok) or given up (!ok); holds till the next arm
    output logic        ok,
    output logic [31:0] data                // the reply, first bit at 31
);
logic [1:0]  s;
logic        q;
logic [12:0] t;                             // cycles since the last falling edge (or the arm)
logic [5:0]  n;                             // bits taken
logic [31:0] sh;
typedef enum logic [1:0] {IDLE, WFALL, SAMP} st_t;
st_t st;
wire fall = q & ~s[1];
always_ff @(posedge clk) begin
    s <= {s[0], si};
    q <= s[1];
    if (arm) begin
        st <= WFALL; t <= 13'd0; n <= 6'd0; fin <= 1'b0; ok <= 1'b0;
    end else case (st)
        WFALL: begin                        // the first bit may take a while (a pad's delay); later ones come every 4 us
            t <= t + 13'd1;
            if (fall) begin t <= 13'd0; st <= SAMP; end
            else if (t == 13'(n == 6'd0 ? 100 * US : 6 * US)) begin fin <= 1'b1; ok <= 1'b0; st <= IDLE; end
        end
        SAMP: begin                         // 2 us after the edge: a 1 has let go (1 us low), a 0 has not (3 us)
            t <= t + 13'd1;
            if (t == 13'(2 * US)) begin
                sh <= {sh[30:0], s[1]};
                n <= n + 6'd1;
                if (n == 6'd31) begin data <= {sh[30:0], s[1]}; fin <= 1'b1; ok <= 1'b1; st <= IDLE; end
                else st <= WFALL;
            end
        end
        default: ;
    endcase
end
endmodule

module an_joybus #(
    parameter int CLK_HZ    = 62_500_000,   // clk's frequency (clk_sys)
    parameter int SLOT_US   = 250,          // a port's slot: one poll, a port every SLOT_US (a round of four ports: 1 ms)
    parameter int MISS      = 32            // polls without an answer before a pad is gone
) (
    input  wire         clk,
    input  wire         en,                 // the port is the adapter's and N64 pads are chosen
    input  wire  [3:0]  si,                 // the four lines, as the pins read them (port 1 at bit 0)
    output logic [3:0]  drive,              // a bit a port: 1 the line is pulled low (the pin an output, value 0); 0 let go (an input)
    output wire  [3:0]  present,
    output wire  [63:0] btn,                // a port 16 bits (port 1 at 15:0): the N64's buttons
    output wire  [63:0] stick               // a port 16 bits: {Y, X}
);
localparam int US = CLK_HZ / 1_000_000;
localparam int SLOT = SLOT_US * US;
localparam logic [7:0] CMD = 8'h01;         // "the buttons"

logic [3:0]  arm_v;
wire  [3:0]  fin, ok;
wire  [31:0] rx [4];
genvar gi;
generate for (gi = 0; gi < 4; gi++) begin : g_rx
    an_jb_rx #(.US(US)) rxp (.clk, .si(si[gi]), .arm(arm_v[gi]), .fin(fin[gi]), .ok(ok[gi]), .data(rx[gi]));
end endgenerate

typedef enum logic [1:0] {S_IDLE, S_TX, S_ARM, S_WAIT} s_t;
s_t          st;
logic [1:0]  sel;                           // the port in its slot
logic [20:0] per;
logic [3:0]  txn;                           // the bit being sent: 0..7 the command, 8 the stop bit
logic [9:0]  t;                             // cycles into the bit's cell
logic [3:0]  pres;
logic [5:0]  miss [4];
logic [15:0] btn_r [4];
logic [15:0] stk_r [4];
wire         bitv  = txn < 4'd8 ? CMD[3'(4'd7 - txn)] : 1'b1;
wire  [9:0]  lowt  = bitv ? 10'(US) : 10'(3 * US);
logic        lastfin;                       // (the polled port's `fin`, as of its slot)
assign lastfin = fin[sel];

always_ff @(posedge clk) begin
    arm_v <= 4'd0;
    drive <= 4'd0;
    if (!en) begin
        st <= S_IDLE; sel <= 2'd0; per <= '0; pres <= 4'd0; txn <= 4'd0; t <= 10'd0;
        for (int i = 0; i < 4; i++) begin miss[i] <= 6'd0; btn_r[i] <= 16'd0; stk_r[i] <= 16'd0; end
    end else begin
        if (per != '1) per <= per + 21'd1;
        case (st)
            S_IDLE: begin
                if (per >= 21'(SLOT - 1)) begin per <= '0; txn <= 4'd0; t <= 10'd0; st <= S_TX; end
            end
            S_TX: begin
                t <= t + 10'd1;
                if (txn == 4'd8) begin                          // the stop bit: 1 us low, then let go and listen
                    drive[sel] <= (t < 10'(US));
                    if (t == 10'(US)) begin arm_v[sel] <= 1'b1; st <= S_ARM; end
                end else begin
                    drive[sel] <= (t < lowt);
                    if (t == 10'(4 * US - 1)) begin t <= 10'd0; txn <= txn + 4'd1; end
                end
            end
            S_ARM: st <= S_WAIT;                                // (the receiver takes the arm: its `fin` is low from the next cycle)
            S_WAIT: begin
                if (lastfin) begin
                    if (ok[sel]) begin
                        pres[sel]  <= 1'b1;
                        miss[sel]  <= 6'd0;
                        btn_r[sel] <= rx[sel][31:16] & 16'hFF3F;        // (the reset flag and the always-0 bit: out)
                        stk_r[sel] <= {rx[sel][7:0], rx[sel][15:8]};    // {Y, X}
                    end else if (miss[sel] == 6'(MISS)) begin
                        pres[sel] <= 1'b0; btn_r[sel] <= 16'd0; stk_r[sel] <= 16'd0;
                    end else miss[sel] <= miss[sel] + 6'd1;
                    sel <= sel + 2'd1;
                    st <= S_IDLE;
                end
            end
            default: st <= S_IDLE;
        endcase
    end
end
assign present = pres;
assign btn   = {btn_r[3], btn_r[2], btn_r[1], btn_r[0]};
assign stick = {stk_r[3], stk_r[2], stk_r[1], stk_r[0]};
endmodule
