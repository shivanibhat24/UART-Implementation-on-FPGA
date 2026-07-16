// =====================================================================
// uart_mvote_fifo.v  --  Method 4: majority-vote receiver + FIFOs
//
//   Format      : 8N1  (1 start, 8 data LSB-first, 1 stop, no parity)
//   TX timing   : 1x baud tick, mux serializer      (unchanged from M2/M3)
//   RX timing   : 16x oversample tick, phase reset on start edge   (M3)
//   RX sampling : 3 samples per bit at ticks 7/8/9, 2-of-3 MAJORITY VOTE
//   Buffering   : 16-deep FIFO both directions, distributed RAM
//
//   Target      : xc7s15ftgb196-1, 100 MHz
//
// ---------------------------------------------------------------------
// DELTA FROM METHOD 3 (uart_ovs16.v)
// ---------------------------------------------------------------------
//
//   1. MAJORITY VOTING. M3 sampled once at os_cnt==7. M4 samples at ticks
//      7, 8 and 9 and votes 2-of-3. Cost: 3 FFs + one LUT6 for the vote
//      term (a&b)|(b&c)|(a&c). This is the FIRST method that narrows the
//      long-glitch hole, and the first with any noise immunity at all.
//
//   2. rx_noise flag. When the three samples DISAGREE the vote still
//      produces a bit, but disagreement is itself information: on a clean
//      link all three samples are identical, because they all land inside
//      the settled region of the bit. Any disagreement means the line is
//      transitioning where it should be quiet -- marginal baud, reflection,
//      EMI, or an SET. Sticky per frame, cleared with the byte. M1-M3 had
//      no way to know the link was degrading until it started corrupting.
//
//   3. FIFOs, 16 deep, both directions. M3's single holding register made
//      the consumer collect each byte within one frame time (~87us at
//      115200) or lose it. 16 bytes buys ~1.4ms of slack.
//
// ---------------------------------------------------------------------
// WHY VOTING WORKS WHERE METHOD 3's START CHECK DID NOT
// ---------------------------------------------------------------------
//
//   M3 briefly carried a second start-bit check at os_cnt==15. It looked
//   like it rejected the long glitch. It was a bug: os_cnt==15 sits ON the
//   bit boundary, where the line is legitimately transitioning into data
//   bit 0, so it also rejected every valid frame from a fast far end.
//
//   The difference is WHERE the second opinion is taken:
//     - a different POSITION in the bit  -> boundary -> transitions are
//       legal there -> false rejections. This is what M3 got wrong.
//     - the SAME position, three times   -> all three land in the settled
//       region -> a clean signal CANNOT make them disagree.
//
//   Voting is not "more checking". It is checking somewhere it is valid to
//   check.
//
// ---------------------------------------------------------------------
// WHAT IS STILL NOT FIXED -- STATED HONESTLY
// ---------------------------------------------------------------------
//
//   Voting NARROWS the long-glitch hole; it does not close it. A
//   disturbance held low across ticks 7, 8 AND 9 is still accepted as a
//   start bit, because at every instant a receiver is permitted to look,
//   it is indistinguishable from a real start bit. Nothing local to the
//   start bit can fix that. Closing it needs wider context -- a minimum
//   idle-time qualifier before accepting a start edge, or a protocol-level
//   frame check. See the testbench: the 1-bit glitch case is still
//   accepted and is asserted as a KNOWN limitation, not papered over.
//
//   Also absent: parity, break detection, runtime-programmable baud, TMR.
//   Those are Method 5.
//
// ---------------------------------------------------------------------
// THE DIVISOR TRAP (unchanged from M3, restated because it still bites)
// ---------------------------------------------------------------------
//
//   100e6 / (115200*16) = 54.25 -> 54. Effective baud 115741, +0.47%.
//   Method 1 divided by 868 for 0.007%. Rounding a SMALLER number costs
//   ~65x more. Still inside a ~2% two-endpoint budget. An NCO is the fix
//   and it is Method 5's business.
// =====================================================================

`timescale 1ns / 1ps
`default_nettype none

// ---------------------------------------------------------------------
// baud_tick_gen_mv : clock-enable pulse generator (restartable)
// ---------------------------------------------------------------------
module baud_tick_gen_mv #(
    parameter integer DIV = 54
)(
    input  wire clk,
    input  wire rst_n,
    input  wire restart,      // 1-clk strobe: zero the phase NOW
    output reg  tick          // 1-clk strobe every DIV clocks
);

    localparam integer W = (DIV <= 2) ? 1 : $clog2(DIV);

    reg [W-1:0] cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            cnt  <= {W{1'b0}};
            tick <= 1'b0;
        end else if (restart) begin
            cnt  <= {W{1'b0}};
            tick <= 1'b0;          // suppress any tick on the restart clk
        end else if (cnt == DIV - 1) begin
            cnt  <= {W{1'b0}};
            tick <= 1'b1;
        end else begin
            cnt  <= cnt + 1'b1;
            tick <= 1'b0;
        end
    end

endmodule


// ---------------------------------------------------------------------
// uart_fifo : synchronous FIFO, distributed RAM
//
//   POINTER WIDTH IS THE WHOLE TRICK. Pointers are $clog2(DEPTH)+1 bits,
//   NOT $clog2(DEPTH). The extra MSB is the WRAP bit:
//
//     empty = (wr_ptr == rd_ptr)                    all bits equal
//     full  = (wr_ptr[MSB] != rd_ptr[MSB]) &&       wrapped an odd number
//             (wr_ptr[MSB-1:0] == rd_ptr[MSB-1:0])  of times, same slot
//
//   Size the pointers by address bits alone and full becomes
//   indistinguishable from empty -- both have wr_ptr == rd_ptr. The FIFO
//   then reports empty when it is full, the producer overwrites unread
//   data, and the bug only appears under sustained back-pressure. This is
//   the single most common FIFO defect and it is invisible in a lightly
//   loaded testbench.
//
//   NO BRAM. 16 x 8 = 128 bits. A BRAM18K holds 18,432 -- burning one to
//   store 0.7% of its capacity wastes 5% of an XC7S15's 20 tiles. The
//   coding style below (simple dual-port, async read) infers LUTRAM/SRL16
//   at ~8 LUTs. Do not add a registered read port "for timing": that turns
//   it into BRAM and costs a cycle of latency this design does not need.
// ---------------------------------------------------------------------
module uart_fifo #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 16          // must be a power of 2
)(
    input  wire              clk,
    input  wire              rst_n,

    input  wire [WIDTH-1:0]  wr_data,
    input  wire              wr_en,       // qualified internally by !full
    output wire              full,

    output wire [WIDTH-1:0]  rd_data,
    input  wire              rd_en,       // qualified internally by !empty
    output wire              empty,

    output wire [$clog2(DEPTH):0] count   // 0..DEPTH inclusive -> AW+1 bits
);

    localparam integer AW = $clog2(DEPTH);

    // AW+1 bits: AW address bits + 1 wrap bit. See header.
    reg [AW:0] wr_ptr;
    reg [AW:0] rd_ptr;

    (* ram_style = "distributed" *)
    reg [WIDTH-1:0] mem [0:DEPTH-1];

    assign empty = (wr_ptr == rd_ptr);
    assign full  = (wr_ptr[AW] != rd_ptr[AW]) &&
                   (wr_ptr[AW-1:0] == rd_ptr[AW-1:0]);

    // Subtraction in AW+1 bits wraps correctly and yields 0..DEPTH.
    assign count = wr_ptr - rd_ptr;

    // Async read: rd_data always shows the head. The consumer strobes rd_en
    // to advance. Keeps latency at zero and keeps this in LUTRAM.
    assign rd_data = mem[rd_ptr[AW-1:0]];

    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= {(AW+1){1'b0}};
        end else if (wr_en && !full) begin
            mem[wr_ptr[AW-1:0]] <= wr_data;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr <= {(AW+1){1'b0}};
        end else if (rd_en && !empty) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

endmodule


// ---------------------------------------------------------------------
// uart_tx_mvote : parallel-in, serial-out, mux serializer, FIFO-fed
//
//   The serializer is the Method 2 design unchanged. What is new is that
//   it pulls from a FIFO instead of a single register, so the producer can
//   burst up to DEPTH bytes and walk away.
// ---------------------------------------------------------------------
module uart_tx_mvote #(
    parameter integer CLKS_PER_BIT = 868,
    parameter integer FIFO_DEPTH   = 16
)(
    input  wire       clk,
    input  wire       rst_n,

    // Producer side
    input  wire [7:0] tx_data,
    input  wire       tx_we,        // 1-clk strobe: push a byte
    output wire       tx_full,
    output wire [$clog2(FIFO_DEPTH):0] tx_count,

    output reg        tx_active,    // a frame is on the wire right now
    output wire       txd
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0] state;
    reg [2:0] bit_idx;      // doubles as the mux select
    reg [7:0] tx_hold;      // NOT shifted -- the M2 property, preserved
    reg       txd_r;

    assign txd = txd_r;

    wire [7:0] fifo_dout;
    wire       fifo_empty;
    reg        fifo_rd;

    uart_fifo #(.WIDTH(8), .DEPTH(FIFO_DEPTH)) u_txfifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_data (tx_data),
        .wr_en   (tx_we),
        .full    (tx_full),
        .rd_data (fifo_dout),
        .rd_en   (fifo_rd),
        .empty   (fifo_empty),
        .count   (tx_count)
    );

    reg  tick_restart;
    wire tick;

    baud_tick_gen_mv #(.DIV(CLKS_PER_BIT)) u_tick (
        .clk     (clk),
        .rst_n   (rst_n),
        .restart (tick_restart),
        .tick    (tick)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            bit_idx      <= 3'd0;
            tx_hold      <= 8'd0;
            tx_active    <= 1'b0;
            txd_r        <= 1'b1;   // MUST reset high: low = a start bit, so
                                    // a low reset value transmits a garbage
                                    // frame on every reset release.
            tick_restart <= 1'b0;
            fifo_rd      <= 1'b0;
        end else begin
            tick_restart <= 1'b0;
            fifo_rd      <= 1'b0;

            case (state)

            S_IDLE: begin
                txd_r   <= 1'b1;
                bit_idx <= 3'd0;
                if (!fifo_empty) begin
                    tx_hold      <= fifo_dout;   // async read: head is here now
                    fifo_rd      <= 1'b1;        // pop it
                    tx_active    <= 1'b1;
                    tick_restart <= 1'b1;        // align phase to the frame
                    state        <= S_START;
                end else begin
                    tx_active <= 1'b0;
                end
            end

            S_START: begin
                txd_r <= 1'b0;
                if (tick) state <= S_DATA;
            end

            S_DATA: begin
                txd_r <= tx_hold[bit_idx];   // index, don't shift
                if (tick) begin
                    if (bit_idx == 3'd7) begin
                        bit_idx <= 3'd0;
                        state   <= S_STOP;
                    end else begin
                        bit_idx <= bit_idx + 3'd1;
                    end
                end
            end

            S_STOP: begin
                txd_r <= 1'b1;
                if (tick) begin
                    // Back to IDLE at the END of the stop bit. If the FIFO
                    // has more, IDLE will start the next frame on the very
                    // next clock -- back-to-back with exactly one stop bit
                    // of gap, which is legal 8N1.
                    state <= S_IDLE;
                end
            end

            default: state <= S_IDLE;

            endcase
        end
    end

endmodule


// ---------------------------------------------------------------------
// uart_rx_mvote : 16x oversampling receiver with 2-of-3 majority voting
//
//   THE METHOD.
//
//   Timing model:
//
//     bit period = 16 oversample ticks
//
//        os_cnt:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
//                 |                       ^  ^  ^                |
//                 |                       └──┼──┘                |
//              phase 0                    VOTE 2-of-3        end of bit
//           (set at start edge)
//
//   All three samples land inside the settled region of the bit. On a
//   clean link they are identical -- disagreement is therefore a direct
//   measurement of link quality, exported as rx_noise.
// ---------------------------------------------------------------------
module uart_rx_mvote #(
    parameter integer OS_DIV     = 54,    // clk / (baud * 16)
    parameter integer FIFO_DEPTH = 16
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       rxd,           // raw pin -- asynchronous to clk

    // Consumer side
    output wire [7:0] rx_data,       // head of FIFO, valid while !rx_empty
    input  wire       rx_re,         // 1-clk strobe: pop
    output wire       rx_empty,
    output wire [$clog2(FIFO_DEPTH):0] rx_count,

    output reg        frame_err,     // stop bit was not mark (sticky, see below)
    output reg        rx_overrun,    // byte arrived with the FIFO full
    output reg        rx_noise,      // the 3 samples disagreed somewhere in
                                     // this frame -- link is marginal
    input  wire       err_clear      // 1-clk strobe: clear the sticky flags
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    // The three vote points and the bit boundary, within the 16-tick window.
    localparam [3:0] VOTE_A  = 4'd7;
    localparam [3:0] VOTE_B  = 4'd8;
    localparam [3:0] VOTE_C  = 4'd9;
    localparam [3:0] BIT_END = 4'd15;

    // ---- CDC ------------------------------------------------------------
    // Two flops minimum. ASYNC_REG keeps the pair in one slice so routing
    // delay does not eat the metastability settling window. Reset HIGH --
    // a low reset value is a start bit.
    //
    // Note this is genuinely two flops. Some well-regarded cores ship with
    // ONE (a bare rxd_reg <= rxd); that is a finite-MTBF exposure whose
    // failure mode is a randomly corrupted byte, and no testbench that
    // drives rxd synchronously can detect it.
    (* ASYNC_REG = "TRUE" *) reg rxd_meta;
    (* ASYNC_REG = "TRUE" *) reg rxd_sync;

    always @(posedge clk) begin
        if (!rst_n) begin
            rxd_meta <= 1'b1;
            rxd_sync <= 1'b1;
        end else begin
            rxd_meta <= rxd;
            rxd_sync <= rxd_meta;
        end
    end

    reg [1:0] state;
    reg [3:0] os_cnt;      // position WITHIN the current bit, 0..15
    reg [2:0] bit_idx;     // which data bit, 0..7
    reg [7:0] shreg;       // RX assembles a byte that does not exist yet, so
                           // a shift register is correct here -- the M2 mux
                           // trick has no dual on the receive side.

    // The three captured samples for the bit in progress.
    reg s_a, s_b, s_c;

    // 2-of-3 majority. Synthesises to a single LUT6.
    wire vote = (s_a & s_b) | (s_b & s_c) | (s_a & s_c);

    // Disagreement detector: on a clean link all three are equal.
    wire disagree = (s_a ^ s_b) | (s_b ^ s_c);

    reg  os_restart;
    wire os_tick;

    baud_tick_gen_mv #(.DIV(OS_DIV)) u_os (
        .clk     (clk),
        .rst_n   (rst_n),
        .restart (os_restart),
        .tick    (os_tick)
    );

    // ---- RX FIFO --------------------------------------------------------
    reg        fifo_we;
    reg  [7:0] fifo_din;
    wire       fifo_full;

    uart_fifo #(.WIDTH(8), .DEPTH(FIFO_DEPTH)) u_rxfifo (
        .clk     (clk),
        .rst_n   (rst_n),
        .wr_data (fifo_din),
        .wr_en   (fifo_we),
        .full    (fifo_full),
        .rd_data (rx_data),
        .rd_en   (rx_re),
        .empty   (rx_empty),
        .count   (rx_count)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            os_cnt     <= 4'd0;
            bit_idx    <= 3'd0;
            shreg      <= 8'd0;
            s_a        <= 1'b1;
            s_b        <= 1'b1;
            s_c        <= 1'b1;
            frame_err  <= 1'b0;
            rx_overrun <= 1'b0;
            rx_noise   <= 1'b0;
            os_restart <= 1'b0;
            fifo_we    <= 1'b0;
            fifo_din   <= 8'd0;
        end else begin
            os_restart <= 1'b0;
            fifo_we    <= 1'b0;

            if (err_clear) begin
                frame_err  <= 1'b0;
                rx_overrun <= 1'b0;
                rx_noise   <= 1'b0;
            end

            case (state)

            // -------------------------------------------------------------
            S_IDLE: begin
                os_cnt  <= 4'd0;
                bit_idx <= 3'd0;
                if (rxd_sync == 1'b0) begin
                    // Falling edge -> candidate start bit. Restart the
                    // oversample phase AT THE EDGE: phase 0 is pinned to the
                    // signal, not to whatever the free-running counter was
                    // doing. Worst-case alignment error is one oversample
                    // period = 1/16 bit.
                    os_restart <= 1'b1;
                    state      <= S_START;
                end
            end

            // -------------------------------------------------------------
            S_START: begin
                if (os_tick) begin
                    // Capture the three samples as the ticks go by.
                    if (os_cnt == VOTE_A) s_a <= rxd_sync;
                    if (os_cnt == VOTE_B) s_b <= rxd_sync;

                    if (os_cnt == VOTE_C) begin
                        // s_c is being registered this cycle, so vote on the
                        // combinational value rather than waiting a clock.
                        // Voting on {s_a, s_b, rxd_sync} here is the same
                        // three samples, one cycle earlier.
                        if (((s_a & s_b) | (s_b & rxd_sync) | (s_a & rxd_sync))
                             != 1'b0) begin
                            // Majority says MARK -> not a start bit. A real
                            // start bit is space at all three points.
                            state <= S_IDLE;
                        end else begin
                            s_c    <= rxd_sync;
                            // Any disagreement among the three means the line
                            // was transitioning inside the settled region.
                            if ((s_a ^ s_b) | (s_b ^ rxd_sync))
                                rx_noise <= 1'b1;
                            os_cnt <= os_cnt + 4'd1;
                        end
                    end else if (os_cnt == BIT_END) begin
                        // NOTE: no level check here, deliberately. os_cnt==15
                        // sits ON the bit boundary where the line legitimately
                        // transitions into data bit 0. An earlier draft of
                        // Method 3 validated here and rejected every valid
                        // frame from a fast far end. Never sample a level at
                        // a bit boundary.
                        os_cnt <= 4'd0;
                        state  <= S_DATA;
                    end else begin
                        os_cnt <= os_cnt + 4'd1;
                    end
                end
            end

            // -------------------------------------------------------------
            S_DATA: begin
                if (os_tick) begin
                    if (os_cnt == VOTE_A) begin
                        s_a    <= rxd_sync;
                        os_cnt <= os_cnt + 4'd1;
                    end else if (os_cnt == VOTE_B) begin
                        s_b    <= rxd_sync;
                        os_cnt <= os_cnt + 4'd1;
                    end else if (os_cnt == VOTE_C) begin
                        s_c    <= rxd_sync;
                        if ((s_a ^ s_b) | (s_b ^ rxd_sync))
                            rx_noise <= 1'b1;
                        // Shift in the VOTED bit, not a raw sample. Shift in
                        // from the top: the first bit received is the LSB, so
                        // after 8 shifts it has walked down to shreg[0]. No
                        // reversal needed anywhere.
                        shreg  <= {((s_a & s_b) | (s_b & rxd_sync) |
                                    (s_a & rxd_sync)), shreg[7:1]};
                        os_cnt <= os_cnt + 4'd1;
                    end else if (os_cnt == BIT_END) begin
                        os_cnt <= 4'd0;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        os_cnt <= os_cnt + 4'd1;
                    end
                end
            end

            // -------------------------------------------------------------
            S_STOP: begin
                if (os_tick) begin
                    if (os_cnt == VOTE_A) begin
                        s_a    <= rxd_sync;
                        os_cnt <= os_cnt + 4'd1;
                    end else if (os_cnt == VOTE_B) begin
                        s_b    <= rxd_sync;
                        os_cnt <= os_cnt + 4'd1;
                    end else if (os_cnt == VOTE_C) begin
                        s_c <= rxd_sync;
                        if ((s_a ^ s_b) | (s_b ^ rxd_sync))
                            rx_noise <= 1'b1;

                        // Vote on the stop bit too. Sampling the stop bit at
                        // its CENTRE (not at the end) is what keeps the
                        // receiver tolerant of baud error: accumulated drift
                        // over 10 bits is worst at the last one.
                        if (((s_a & s_b) | (s_b & rxd_sync) |
                             (s_a & rxd_sync)) != 1'b1) begin
                            // Framing error: the byte is not trustworthy, so
                            // it does NOT go into the FIFO. M1-M3 pushed the
                            // byte anyway and set a flag beside it, which
                            // forces every consumer to remember to check.
                            frame_err <= 1'b1;
                        end else if (fifo_full) begin
                            // A good byte arrived with nowhere to put it.
                            // The byte is dropped and the FACT is recorded --
                            // silent loss is the thing being engineered out.
                            rx_overrun <= 1'b1;
                        end else begin
                            fifo_din <= shreg;
                            fifo_we  <= 1'b1;
                        end

                        os_cnt <= os_cnt + 4'd1;
                    end else if (os_cnt == BIT_END) begin
                        // Return to IDLE at the END of the stop bit, not at
                        // its centre. Going back early would let the tail of
                        // this stop bit be mistaken for the next start bit.
                        os_cnt <= 4'd0;
                        state  <= S_IDLE;
                    end else begin
                        os_cnt <= os_cnt + 4'd1;
                    end
                end
            end

            // -------------------------------------------------------------
            default: state <= S_IDLE;

            endcase
        end
    end

endmodule


// ---------------------------------------------------------------------
// uart_mvote_fifo : wrapper
// ---------------------------------------------------------------------
module uart_mvote_fifo #(
    parameter integer CLK_FREQ   = 100_000_000,
    parameter integer BAUD       = 115_200,
    parameter integer OVERSAMPLE = 16,
    parameter integer FIFO_DEPTH = 16,

    // TX: divide once, cleanly.  100e6/115200 = 868.06 -> 868, err 0.007%
    parameter integer CLKS_PER_BIT = CLK_FREQ / BAUD,

    // RX: divide by 16x more, so rounding hurts ~65x more.
    //     100e6/(115200*16) = 54.25 -> 54, err +0.47%
    parameter integer OS_DIV = CLK_FREQ / (BAUD * OVERSAMPLE)
)(
    input  wire       clk,
    input  wire       rst_n,

    // TX producer side
    input  wire [7:0] tx_data,
    input  wire       tx_we,
    output wire       tx_full,
    output wire [$clog2(FIFO_DEPTH):0] tx_count,
    output wire       tx_active,
    output wire       txd,

    // RX consumer side
    input  wire       rxd,
    output wire [7:0] rx_data,
    input  wire       rx_re,
    output wire       rx_empty,
    output wire [$clog2(FIFO_DEPTH):0] rx_count,

    // Status
    output wire       frame_err,
    output wire       rx_overrun,
    output wire       rx_noise,
    input  wire       err_clear
);

    // A silently-zero divisor is a design that "works" in sim at the wrong
    // rate and never on hardware. Catch it at elaboration.
    initial begin
        if (OS_DIV < 2) begin
            $display("FATAL: OS_DIV=%0d -- CLK_FREQ too low for BAUD*OVERSAMPLE",
                     OS_DIV);
            $finish;
        end
    end

    uart_tx_mvote #(
        .CLKS_PER_BIT (CLKS_PER_BIT),
        .FIFO_DEPTH   (FIFO_DEPTH)
    ) u_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_data   (tx_data),
        .tx_we     (tx_we),
        .tx_full   (tx_full),
        .tx_count  (tx_count),
        .tx_active (tx_active),
        .txd       (txd)
    );

    uart_rx_mvote #(
        .OS_DIV     (OS_DIV),
        .FIFO_DEPTH (FIFO_DEPTH)
    ) u_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .rxd        (rxd),
        .rx_data    (rx_data),
        .rx_re      (rx_re),
        .rx_empty   (rx_empty),
        .rx_count   (rx_count),
        .frame_err  (frame_err),
        .rx_overrun (rx_overrun),
        .rx_noise   (rx_noise),
        .err_clear  (err_clear)
    );

endmodule

`default_nettype wire
