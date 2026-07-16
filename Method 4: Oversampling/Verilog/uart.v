// =====================================================================
// uart_ovs16.v  --  Method 3: 16x oversampling receiver with start resync
//
//   Format      : 8N1  (1 start, 8 data LSB-first, 1 stop, no parity)
//   TX timing   : 1x baud tick, mux serializer     (unchanged from M2)
//   RX timing   : 16x oversample tick, phase reset on start edge
//   RX sampling : single sample at oversample tick 7 (bit centre)
//   Buffering   : single-byte holding register + overrun flag
//
//   Target      : xc7s15ftgb196-1, 100 MHz
//
//   DELTA FROM METHOD 2 (uart_muxser.v):
//     1. RX tick runs at 16x baud. Bit position is now COUNTED (os_cnt
//        0..15), not merely delayed. Phase resets on the start EDGE, not
//        half a bit later -> alignment error <= 1/16 bit instead of a
//        coarse counter's worth.
//     2. Baud tolerance ~ +/-0.5% -> VERIFIED CLEAN AT +/-2% against an
//        independent BFM transmitter (see tb_uart_ovs16.v sweep). This is
//        the method's real and only robustness claim.
//     3. rx_data now has a holding register + rx_overrun, so a byte that
//        is not collected before the next one arrives is FLAGGED rather
//        than silently lost.
//
//   TX IS DELIBERATELY NOT OVERSAMPLED:
//     There is nothing to recover on a transmitter -- you generate the
//     timing, you don't discover it. A 16x tick on TX would cost a wider
//     counter to do exactly what the 1x tick already does.
//
//   THE DIVISOR TRAP (read this before retargeting):
//     100e6 / (115200*16) = 54.25 -> 54. Effective baud 115741, error
//     +0.47%. Method 1 divided by 868 for 0.007% error. Rounding a
//     SMALLER number costs ~65x more. Still inside the ~2% two-endpoint
//     budget, but this is the first method where an NCO stops being
//     pointless. See OS_DIV assertion below.
//
//   WHAT IS STILL NOT FIXED -- STATED HONESTLY:
//     Still ONE sample per bit. A glitch one bit period long is still
//     accepted as a start bit and still produces a phantom byte with
//     frame_err=0, exactly as in M1 and M2. Oversampling bought PHASE
//     ACCURACY, not noise immunity -- those are different properties and
//     conflating them is easy.
//
//     During development this file briefly carried a second start-bit
//     check at os_cnt==BIT_END, which appeared to reject the glitch. It
//     was a bug: os_cnt==15 sits on the bit boundary where the line is
//     legitimately transitioning, so it also rejected every valid frame
//     from a fast far end, and the +/-2% sweep failed asymmetrically.
//     The apparent hardening was the bug. Noise immunity requires a
//     SECOND OPINION at the bit centre -- majority voting over ticks
//     7/8/9 -- which is Method 4.
// =====================================================================

`timescale 1ns / 1ps
`default_nettype none

// ---------------------------------------------------------------------
// baud_tick_gen_ovs : clock-enable pulse generator (restartable)
//
//   Identical in spirit to Method 2's baud_tick_gen, but 'restart' now
//   zeroes the phase to align with an EDGE rather than loading a half-bit
//   offset. Method 3's RX counts its own sub-bit position in os_cnt, so
//   the generator no longer needs a 'half' mode -- the half-bit delay is
//   expressed as "wait until os_cnt == 7", which is more precise and more
//   readable than a magic load value.
// ---------------------------------------------------------------------
module baud_tick_gen_ovs #(
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
// uart_tx_ovs16 : parallel-in, serial-out, mux serializer
//
//   Byte-for-byte the Method 2 transmitter. Reproduced here so each method
//   is a self-contained file that can be built and measured on its own --
//   the repo is a comparison, and cross-file dependencies between methods
//   would make the utilisation numbers ambiguous.
// ---------------------------------------------------------------------
module uart_tx_ovs16 #(
    parameter integer CLKS_PER_BIT = 868
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output reg        tx_busy,
    output wire [7:0] tx_hold_o,   // byte survives transmission (M2 property)
    output reg        txd
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0] state;
    reg [2:0] bit_idx;      // doubles as the mux select
    reg [7:0] tx_hold;      // NOT shifted -- read-only during transmission

    assign tx_hold_o = tx_hold;

    reg  tick_restart;
    wire tick;

    baud_tick_gen_ovs #(.DIV(CLKS_PER_BIT)) u_tick (
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
            tx_busy      <= 1'b0;
            txd          <= 1'b1;   // MUST reset high: low = start bit
            tick_restart <= 1'b0;
        end else begin
            tick_restart <= 1'b0;

            case (state)

            S_IDLE: begin
                txd     <= 1'b1;
                bit_idx <= 3'd0;
                if (tx_start) begin
                    tx_hold      <= tx_data;
                    tx_busy      <= 1'b1;
                    tick_restart <= 1'b1;
                    state        <= S_START;
                end else begin
                    tx_busy <= 1'b0;
                end
            end

            S_START: begin
                txd <= 1'b0;
                if (tick) state <= S_DATA;
            end

            S_DATA: begin
                txd <= tx_hold[bit_idx];    // index, don't shift
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
                txd <= 1'b1;
                if (tick) begin
                    tx_busy <= 1'b0;
                    state   <= S_IDLE;
                end
            end

            default: state <= S_IDLE;

            endcase
        end
    end

endmodule


// ---------------------------------------------------------------------
// uart_rx_ovs16 : 16x oversampling receiver
//
//   THE METHOD. Everything above this comment is Method 2 carried forward.
//
//   Timing model:
//
//     bit period = 16 oversample ticks
//
//        os_cnt:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
//                 |                       ^                    |
//                 |                    SAMPLE                  |
//              phase 0                (centre)             end of bit
//           (set at start edge)
//
//   On the start-bit falling edge the oversample generator is restarted,
//   putting phase 0 at the edge itself. From then on every bit centre is
//   at os_cnt==7 and every bit boundary at os_cnt==15. Worst-case initial
//   alignment error is one oversample period = 1/16 bit = 6.25%, versus
//   Method 2 where the half-bit delay inherited whatever phase the coarse
//   counter happened to be in.
// ---------------------------------------------------------------------
module uart_rx_ovs16 #(
    parameter integer OS_DIV = 54    // clk / (baud * 16)
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       rxd,           // raw pin -- asynchronous to clk

    output reg  [7:0] rx_data,       // holding register: stays valid until
                                     // overwritten by the NEXT byte
    output reg        rx_valid,      // 1-clk strobe: rx_data updated
    output reg        frame_err,     // stop bit was not mark
    output reg        rx_overrun,    // NEW: a byte arrived while the previous
                                     // one had not been collected. Sticky
                                     // until rx_clear.
    input  wire       rx_clear       // 1-clk strobe: consumer took the byte,
                                     // clears rx_overrun
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    // Sample point and bit boundary within the 16-tick window.
    localparam [3:0] SAMPLE_AT = 4'd7;    // centre
    localparam [3:0] BIT_END   = 4'd15;

    // ---- CDC ------------------------------------------------------------
    // Two flops minimum. ASYNC_REG keeps the pair in one slice so routing
    // delay doesn't eat the metastability settling window. Reset HIGH --
    // a low reset value is a start bit.
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
    reg [7:0] shreg;       // RX assembles a byte that does not exist yet,
                           // so a shift register is correct here -- the mux
                           // trick has no dual on the receive side.
    reg       byte_pending; // a byte is sitting in rx_data uncollected

    reg  os_restart;
    wire os_tick;

    baud_tick_gen_ovs #(.DIV(OS_DIV)) u_os (
        .clk     (clk),
        .rst_n   (rst_n),
        .restart (os_restart),
        .tick    (os_tick)
    );

    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            os_cnt       <= 4'd0;
            bit_idx      <= 3'd0;
            shreg        <= 8'd0;
            rx_data      <= 8'd0;
            rx_valid     <= 1'b0;
            frame_err    <= 1'b0;
            rx_overrun   <= 1'b0;
            byte_pending <= 1'b0;
            os_restart   <= 1'b0;
        end else begin
            rx_valid   <= 1'b0;    // default: 1-clk strobe
            os_restart <= 1'b0;

            // Consumer collected the byte.
            if (rx_clear) begin
                byte_pending <= 1'b0;
                rx_overrun   <= 1'b0;
            end

            case (state)

            // -------------------------------------------------------------
            S_IDLE: begin
                os_cnt  <= 4'd0;
                bit_idx <= 3'd0;
                if (rxd_sync == 1'b0) begin
                    // Falling edge -> candidate start bit.
                    // Restart the oversample phase AT THE EDGE. This is the
                    // core of Method 3: phase 0 is now pinned to the signal,
                    // not to whatever the free-running counter was doing.
                    os_restart <= 1'b1;
                    state      <= S_START;
                end
            end

            // -------------------------------------------------------------
            S_START: begin
                if (os_tick) begin
                    if (os_cnt == SAMPLE_AT) begin
                        // Centre of the start bit. Validate here and ONLY
                        // here.
                        //
                        // An earlier draft of this FSM also re-checked at
                        // os_cnt==BIT_END, reasoning that "the line must
                        // still be space at the end of the start bit". That
                        // is wrong and it cost a debug session: os_cnt==15
                        // sits ON the bit boundary, where the line is
                        // legitimately transitioning into data bit 0. The
                        // phase restart also costs one tick before os_cnt
                        // begins counting, pushing tick 15 slightly PAST the
                        // boundary. With a far end running fast (positive
                        // baud error) the check sampled the first data bit
                        // and rejected every valid frame whose LSB was 1.
                        // Negative offsets passed, which is what made the
                        // failure look like a tolerance problem rather than
                        // a logic error.
                        //
                        // Never validate a level at a bit boundary. Bit
                        // centres are the only place the line is guaranteed
                        // settled. Extra confidence about the start bit is
                        // Method 4's job (majority vote at ticks 7/8/9),
                        // not an extra sample at the edge.
                        if (rxd_sync != 1'b0) begin
                            state <= S_IDLE;      // glitch, abandon
                        end else begin
                            os_cnt <= os_cnt + 4'd1;
                        end
                    end else if (os_cnt == BIT_END) begin
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
                    if (os_cnt == SAMPLE_AT) begin
                        // Bit centre. Shift in from the top: first bit
                        // received is the LSB, so after 8 shifts it has
                        // walked down to shreg[0]. No reversal needed.
                        shreg  <= {rxd_sync, shreg[7:1]};
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
                    if (os_cnt == SAMPLE_AT) begin
                        // Sample the stop bit at ITS centre, not at the end.
                        // Sampling late here is what makes a receiver
                        // intolerant of baud error -- the accumulated drift
                        // over 10 bits is worst at the last one.
                        rx_data   <= shreg;
                        rx_valid  <= 1'b1;
                        frame_err <= (rxd_sync != 1'b1);

                        // Overrun: previous byte never collected.
                        if (byte_pending && !rx_clear) rx_overrun <= 1'b1;
                        byte_pending <= 1'b1;

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
// uart_ovs16 : wrapper
// ---------------------------------------------------------------------
module uart_ovs16 #(
    parameter integer CLK_FREQ  = 100_000_000,
    parameter integer BAUD      = 115_200,
    parameter integer OVERSAMPLE = 16,

    // TX: divide once, cleanly.  100e6/115200 = 868.06 -> 868, err 0.007%
    parameter integer CLKS_PER_BIT = CLK_FREQ / BAUD,

    // RX: divide by 16x more, so rounding hurts ~65x more.
    //     100e6/(115200*16) = 54.25 -> 54, err +0.47%
    // Still inside a ~2% two-endpoint budget, but this is where fractional
    // baud generation stops being a waste of an accumulator.
    parameter integer OS_DIV = CLK_FREQ / (BAUD * OVERSAMPLE)
)(
    input  wire       clk,
    input  wire       rst_n,

    // TX
    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output wire       tx_busy,
    output wire [7:0] tx_hold_o,
    output wire       txd,

    // RX
    input  wire       rxd,
    output wire [7:0] rx_data,
    output wire       rx_valid,
    output wire       frame_err,
    output wire       rx_overrun,
    input  wire       rx_clear
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

    uart_tx_ovs16 #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk       (clk),
        .rst_n     (rst_n),
        .tx_data   (tx_data),
        .tx_start  (tx_start),
        .tx_busy   (tx_busy),
        .tx_hold_o (tx_hold_o),
        .txd       (txd)
    );

    uart_rx_ovs16 #(.OS_DIV(OS_DIV)) u_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .rxd        (rxd),
        .rx_data    (rx_data),
        .rx_valid   (rx_valid),
        .frame_err  (frame_err),
        .rx_overrun (rx_overrun),
        .rx_clear   (rx_clear)
    );

endmodule

`default_nettype wire
