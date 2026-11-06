// Up to three N64 controllers on the Analogizer's SNAC port (docs/analogizer.md, "N64 controllers"): the Joybus, one wire a pad, and the
// engine the runtime's PIF drives (rtl/soc/mirlo_regs.sv JOYBUS, lang/n64rt/n64rt_io.c pif_run).
//
// The adapter is the N64 SNAC board with its cable harness (config. A of the adapter's switch): the data lines are
//     port 1  pin 30 (IO5)          port 2  pin 31 (IO6)          port 3  bank 0 bit 4 (IO3)          (port 4, bank 0 bit 7: not used)
// each with a 1 kohm pull-up to 3.3 V on the board. The FPGA never drives a 1: a line is pulled low (the pin an output with the value 0) or
// let go (an input), the pull-up making the 1. One transaction at a time, on one line (`drive`, a bit a port); every other line is let go.
// (Bank 0 has one direction for its bits, so core_top.sv holds the others high while port 3 is polled: the level the pull-up gives.)
//
// A transaction is what the PIF does for a channel of its 64-byte block: `tx` bytes out (the first the command), then `rx` bytes in. Bit by
// bit, a 4 us cell -- a 1 is 1 us low and 3 us high, a 0 is 3 us low and 1 us high -- the bytes high bit first, and a stop bit, 1 us low; then
// the line is let go and the device answers in the same cells. A reply bit is read 2 us after its falling edge: high a 1, low a 0.
//     result   0 the `rx` bytes came      1 nothing came (no device: the PIF's "no response")      2 some came, then no more
// There are two users of the one engine, one transaction at a time and the runtime's first:
//   * the engine itself polls the pads' buttons (the command 0x01, `rx` 4) one port in turn, a slot of SLOT_US each, so the runtime reads a pad
//     as a word and does not wait for the wire. A pad is `present` after one good reply and until MISS polls in a row have none (a poll lost to
//     noise is not a pad taken out). The buttons come out as the N64's own 16 bits (libultra's CONT_ layout: A is bit 15 ... C-right bit 0;
//     the pad's reset flag, bit 7, is cleared) and the stick as {Y, X}: what the runtime takes from a pad of KEY_TYPE 4 (lang/n64rt/n64rt_pad.h).
//   * the runtime's, for the commands it cannot answer from a word -- the pad's status (0x00, 0xFF), the accessory's blocks (0x02, 0x03),
//     the reset -- through four registers (JOYBUS): CTL (txlen, rxlen, the port, go), ADDR (the buffer's pointer), DATA (a byte of the
//     64-byte buffer, the pointer stepping), STATUS. The buffer is the PIF's block as it is: the command's bytes go in at 0, and the reply
//     comes back over them.
`default_nettype none

module an_joybus #(
    parameter int CLK_HZ       = 62_500_000,    // clk's frequency (clk_sys)
    parameter int SLOT_US      = 250,           // the polling: a port's slot (a round of three is 750 us)
    parameter int MISS         = 32,            // polls without an answer before a pad is gone
    parameter int AUTO_WAIT_US = 100,           // the polling: how long the first bit of a reply is waited for
    parameter int SW_WAIT_US   = 600            // the runtime's: the same (an accessory answers a write once it has it stored)
) (
    input  wire         clk,
    input  wire         en,                     // the port is the adapter's and N64 pads are chosen
    input  wire  [3:0]  si,                     // the lines, as the pins read them (port 1 at bit 0; bit 3 is not used)
    output logic [3:0]  drive,                  // a bit a port: 1 the line is pulled low (the pin an output, value 0); 0 let go (an input)
    output wire  [3:0]  present,
    output wire  [63:0] btn,                    // a port 16 bits (port 1 at 15:0): the N64's buttons
    output wire  [63:0] stick,                  // a port 16 bits: {Y, X}
    // the runtime's registers (mirlo_regs.sv: JOYBUS): a write, a read (cs_rd: a read of DATA, which steps the pointer), the register read
    input  wire         cs_wr,
    input  wire  [1:0]  cs_wa,
    input  wire  [31:0] cs_wd,
    input  wire         cs_rd,
    input  wire  [1:0]  cs_ra,
    output logic [31:0] cs_rdata
);
localparam int US   = CLK_HZ / 1_000_000;
localparam int SLOT = SLOT_US * US;
localparam int GAP  = 10 * US;                  // between transactions (a device's stop bit is over)

// ---- the buffer: the PIF's block as the runtime sees it (port A) and the engine (port B)
(* ramstyle = "M10K, no_rw_check" *) logic [7:0] mem [0:63];
logic [7:0] a_q, b_q;
logic [5:0] ptr, badr;
logic       b_we;
logic [7:0] b_wd;
wire        a_we = cs_wr && cs_wa == 2'd2;
initial for (int k = 0; k < 64; k++) mem[k] = 8'd0;
// (plain `always`: two processes write one memory, a true dual-port RAM, which `always_ff` does not allow)
always @(posedge clk) begin if (a_we) mem[ptr] <= cs_wd[7:0]; a_q <= mem[ptr]; end
always @(posedge clk) begin if (b_we) mem[badr] <= b_wd; b_q <= mem[badr]; end

// ---- the engine
typedef enum logic [2:0] {S_IDLE, S_PRE, S_TX, S_STOP, S_WF, S_SM, S_FIN} s_t;
s_t          st;
logic        swp, busy, done;                   // the runtime's request: waiting, running, over
logic [1:0]  res_q;
logic [5:0]  got_q;
logic [5:0]  sw_tx, sw_rx;
logic [1:0]  sw_port;
logic        cur_sw;                            // the transaction is the runtime's
logic [1:0]  port, sel;                         // its port; the polling's next
logic [5:0]  txlen, rxlen, txi, rxi;
logic [2:0]  bit_i, rb;
logic [15:0] t;
logic [7:0]  tsh, rsh;
logic [31:0] ash;                               // a poll's reply
logic [20:0] per;
logic [1:0]  res;
logic [1:0]  s;
logic        q;
logic [3:0]  pres;
logic [5:0]  miss [3];
logic [15:0] btn_r [3];
logic [15:0] stk_r [3];
wire         si_sel = port == 2'd0 ? si[0] : port == 2'd1 ? si[1] : si[2];
wire         fall   = q & ~s[1];
wire         first  = rxi == 6'd0 && rb == 3'd0;
wire  [15:0] wait_first = 16'(cur_sw ? SW_WAIT_US * US : AUTO_WAIT_US * US);

always_ff @(posedge clk) begin
    s <= {s[0], si_sel};
    q <= s[1];
    drive <= 4'd0;
    b_we  <= 1'b0;
    // the registers
    if (cs_wr && cs_wa == 2'd1) ptr <= cs_wd[5:0];
    else if (a_we || (cs_rd && cs_ra == 2'd2)) ptr <= ptr + 6'd1;
    if (!en) begin
        st <= S_IDLE; sel <= 2'd0; per <= '0; pres <= 4'd0; t <= '0; swp <= 1'b0; busy <= 1'b0; done <= 1'b0;
        for (int i = 0; i < 3; i++) begin miss[i] <= 6'd0; btn_r[i] <= 16'd0; stk_r[i] <= 16'd0; end
    end else begin
        if (per != '1) per <= per + 21'd1;
        if (cs_wr && cs_wa == 2'd0 && cs_wd[16] && !busy) begin         // go
            sw_tx <= cs_wd[5:0]; sw_rx <= cs_wd[11:6]; sw_port <= cs_wd[13:12];
            swp <= 1'b1; busy <= 1'b1; done <= 1'b0;
        end
        case (st)
            S_IDLE: begin
                if (t < 16'(GAP)) t <= t + 16'd1;
                else if (swp || per >= 21'(SLOT - 1)) begin
                    swp <= 1'b0;
                    if (swp) begin cur_sw <= 1'b1; port <= sw_port; txlen <= sw_tx; rxlen <= sw_rx; end
                    else begin cur_sw <= 1'b0; port <= sel; txlen <= 6'd1; rxlen <= 6'd4; per <= '0; end
                    txi <= 6'd0; rxi <= 6'd0; bit_i <= 3'd0; rb <= 3'd0; t <= '0; badr <= 6'd0;
                    st <= S_PRE;
                end
            end
            S_PRE: begin                                                // (the buffer's first byte comes two cycles after its address)
                t <= t + 16'd1;
                if (port == 2'd3 || txlen == 6'd0 || rxlen == 6'd0) begin res <= 2'd1; st <= S_FIN; end   // (no such port: nothing answers)
                else if (t == 16'd3) begin tsh <= cur_sw ? b_q : 8'h01; badr <= 6'd1; t <= '0; st <= S_TX; end
            end
            S_TX: begin
                t <= t + 16'd1;
                drive[port] <= (t < 16'(tsh[7] ? US : 3 * US));
                if (t == 16'(4 * US - 1)) begin
                    t <= '0; tsh <= {tsh[6:0], 1'b0};
                    if (bit_i == 3'd7) begin
                        bit_i <= 3'd0;
                        if (txi + 6'd1 == txlen) st <= S_STOP;
                        else begin txi <= txi + 6'd1; tsh <= cur_sw ? b_q : 8'h01; badr <= txi + 6'd2; end
                    end else bit_i <= bit_i + 3'd1;
                end
            end
            S_STOP: begin                                               // 1 us low, then let go and listen
                t <= t + 16'd1;
                drive[port] <= (t < 16'(US));
                if (t == 16'(US)) begin t <= '0; st <= S_WF; end
            end
            S_WF: begin                                                 // a bit's falling edge (the first may take a while: a device's delay)
                badr <= rxi;
                t <= t + 16'd1;
                if (fall) begin t <= '0; st <= S_SM; end
                else if (t == (first ? wait_first : 16'(6 * US))) begin res <= first ? 2'd1 : 2'd2; st <= S_FIN; end
            end
            S_SM: begin                                                 // 2 us after the edge: a 1 has let go (1 us low), a 0 has not (3 us)
                t <= t + 16'd1;
                if (t == 16'(2 * US)) begin
                    t <= '0;
                    rsh <= {rsh[6:0], s[1]};
                    if (rb == 3'd7) begin
                        rb <= 3'd0;
                        if (cur_sw) begin b_we <= 1'b1; b_wd <= {rsh[6:0], s[1]}; end
                        else ash <= {ash[23:0], rsh[6:0], s[1]};
                        rxi <= rxi + 6'd1;
                        if (rxi + 6'd1 == rxlen) begin res <= 2'd0; st <= S_FIN; end
                        else st <= S_WF;
                    end else begin rb <= rb + 3'd1; st <= S_WF; end
                end
            end
            S_FIN: begin
                st <= S_IDLE; t <= '0;
                if (cur_sw) begin busy <= 1'b0; done <= 1'b1; res_q <= res; got_q <= rxi; end
                else begin
                    if (res == 2'd0) begin
                        pres[sel]  <= 1'b1;
                        miss[sel]  <= 6'd0;
                        btn_r[sel] <= ash[31:16] & 16'hFF3F;            // (the reset flag and the always-0 bit: out)
                        stk_r[sel] <= {ash[7:0], ash[15:8]};            // {Y, X}
                    end else if (miss[sel] == 6'(MISS)) begin
                        pres[sel] <= 1'b0; btn_r[sel] <= 16'd0; stk_r[sel] <= 16'd0;
                    end else miss[sel] <= miss[sel] + 6'd1;
                    sel <= sel == 2'd2 ? 2'd0 : sel + 2'd1;
                end
            end
            default: st <= S_IDLE;
        endcase
    end
end

always_comb begin
    case (cs_ra)
    2'd0:    cs_rdata = {18'd0, sw_port, sw_rx, sw_tx};
    2'd1:    cs_rdata = {26'd0, ptr};
    2'd2:    cs_rdata = {24'd0, a_q};
    default: cs_rdata = {13'd0, pres[2:0], 6'd0, got_q, res_q, done, busy};
    endcase
end
assign present = {1'b0, pres[2:0]};
assign btn   = {16'd0, btn_r[2], btn_r[1], btn_r[0]};
assign stick = {16'd0, stk_r[2], stk_r[1], stk_r[0]};
endmodule
