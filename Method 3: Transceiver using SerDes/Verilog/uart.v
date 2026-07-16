// =====================================================================
// uart_m1.v  --  Method 1: Minimal FSM UART (TX + RX + top wrapper)
//
//   Format      : 8N1  (1 start, 8 data LSB-first, 1 stop, no parity)
//   Baud timing : integer divider, CLKS_PER_BIT = CLK_FREQ / BAUD
//   RX sampling : SINGLE sample at nominal bit centre
//   Buffering   : none (single holding register each direction)
//
//   Target      : xc7s15ftgb196-1, 100 MHz  -> CLKS_PER_BIT = 868
//
//   This is the baseline. It is deliberately NOT robust:
//     * one glitch on rxd = one corrupted bit, undetected
//     * no start-bit re-validation -> line noise can fake a frame
//     * baud tolerance ~ +/-0.5% (fine FPGA-to-FPGA, marginal to a PC)
//     * no overrun detection -> a missed byte vanishes silently
//   Methods 3-5 fix these in order.
// =====================================================================

`timescale 1ns / 1ps
`default_nettype none            // catch typo'd nets at compile time

// ---------------------------------------------------------------------
// uart_tx_m1 : parallel-in, serial-out
// ---------------------------------------------------------------------
module uart_tx_m1 #(
    parameter integer CLKS_PER_BIT = 868
)(
    input  wire       clk,
    input  wire       rst_n,        // active-low, synchronous

    input  wire [7:0] tx_data,      // byte to send; captured on tx_start
    input  wire       tx_start,     // 1-clk strobe; ignored while tx_busy
    output reg        tx_busy,      // high from load until stop bit ends
    output reg        txd           // serial line (idles high)
);

    // ---- state encoding -------------------------------------------------
    // Binary encoding. Vivado will re-encode to one-hot if it prefers;
    // with 4 states it makes no measurable difference.
    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;      // 0 .. CLKS_PER_BIT-1   (868 needs 10 bits;
                             // 16 is roomy so CLK_FREQ can be raised later)
    reg [2:0]  bit_idx;      // 0..7, which data bit is on the wire
    reg [7:0]  shreg;        // shift register: LSB is always the live bit

    // Reusable "one bit period has elapsed" condition.
    wire bit_done = (clk_cnt == CLKS_PER_BIT - 1);

    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            clk_cnt <= 16'd0;
            bit_idx <= 3'd0;
            shreg   <= 8'd0;
            tx_busy <= 1'b0;
            txd     <= 1'b1;         // MUST reset high: low = a start bit,
                                     // so a low reset value transmits a
                                     // garbage frame on every release.
        end else begin
            case (state)

            // -------------------------------------------------------------
            S_IDLE: begin
                txd     <= 1'b1;     // mark / idle
                clk_cnt <= 16'd0;
                bit_idx <= 3'd0;
                if (tx_start) begin
                    shreg   <= tx_data;   // capture NOW; caller is free to
                                          // change tx_data next cycle
                    tx_busy <= 1'b1;
                    state   <= S_START;
                end else begin
                    tx_busy <= 1'b0;
                end
            end

            // -------------------------------------------------------------
            S_START: begin
                txd <= 1'b0;              // start bit = space
                if (bit_done) begin
                    clk_cnt <= 16'd0;
                    state   <= S_DATA;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // -------------------------------------------------------------
            S_DATA: begin
                txd <= shreg[0];          // LSB first, per the standard
                if (bit_done) begin
                    clk_cnt <= 16'd0;
                    shreg   <= {1'b0, shreg[7:1]};   // shift right
                    if (bit_idx == 3'd7) begin
                        bit_idx <= 3'd0;
                        state   <= S_STOP;
                    end else begin
                        bit_idx <= bit_idx + 3'd1;
                    end
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // -------------------------------------------------------------
            S_STOP: begin
                txd <= 1'b1;              // stop bit = mark
                if (bit_done) begin
                    clk_cnt <= 16'd0;
                    tx_busy <= 1'b0;      // drops as the stop bit completes,
                                          // so back-to-back sends are legal
                    state   <= S_IDLE;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // -------------------------------------------------------------
            // Explicit default. With a 2-bit reg and 4 states this is
            // unreachable, but leaving it out is how you get an inferred
            // latch the day someone adds a 5th state.
            default: state <= S_IDLE;

            endcase
        end
    end

endmodule


// ---------------------------------------------------------------------
// uart_rx_m1 : serial-in, parallel-out
// ---------------------------------------------------------------------
module uart_rx_m1 #(
    parameter integer CLKS_PER_BIT = 868
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       rxd,          // raw pin -- asynchronous to clk
    output reg  [7:0] rx_data,      // valid when rx_valid pulses
    output reg        rx_valid,     // 1-clk strobe, byte complete
    output reg        frame_err     // sticky-per-byte: stop bit was not 1
);

    localparam [1:0] S_IDLE  = 2'd0,
                     S_START = 2'd1,
                     S_DATA  = 2'd2,
                     S_STOP  = 2'd3;

    // ---- CDC: rxd crosses from the pin's (nonexistent) domain into clk ---
    // Two flops minimum. ASYNC_REG keeps the pair placed in the same slice
    // so the metastability-settling window isn't eaten by routing delay.
    (* ASYNC_REG = "TRUE" *) reg rxd_meta;
    (* ASYNC_REG = "TRUE" *) reg rxd_sync;

    always @(posedge clk) begin
        if (!rst_n) begin
            rxd_meta <= 1'b1;      // reset HIGH -- idle line, not a start bit
            rxd_sync <= 1'b1;
        end else begin
            rxd_meta <= rxd;
            rxd_sync <= rxd_meta;
        end
    end

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;

    // THE critical detail of this whole module:
    //   the RX counter is NOT free-running. It is zeroed on the start-bit
    //   edge, then counts CLKS_PER_BIT/2 to reach the middle of the start
    //   bit, and CLKS_PER_BIT per bit after that. Share a free-running tick
    //   with the TX and you sample at an arbitrary phase -- it will appear
    //   to work in loopback and fail against real hardware.
    localparam integer HALF_BIT = CLKS_PER_BIT / 2;   // 434

    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            clk_cnt   <= 16'd0;
            bit_idx   <= 3'd0;
            shreg     <= 8'd0;
            rx_data   <= 8'd0;
            rx_valid  <= 1'b0;
            frame_err <= 1'b0;
        end else begin
            rx_valid <= 1'b0;      // default: strobe is 1 clk wide

            case (state)

            // -------------------------------------------------------------
            S_IDLE: begin
                clk_cnt <= 16'd0;
                bit_idx <= 3'd0;
                if (rxd_sync == 1'b0) begin   // falling edge -> candidate start
                    state <= S_START;
                end
            end

            // -------------------------------------------------------------
            S_START: begin
                // Wait to the MIDDLE of the start bit and re-check. If the
                // line has gone back high it was a glitch, not a frame.
                // This is the only noise rejection Method 1 has, and it is
                // one sample wide.
                if (clk_cnt == HALF_BIT - 1) begin
                    if (rxd_sync == 1'b0) begin
                        clk_cnt <= 16'd0;     // restart: now aligned to
                                              // bit centres for the rest
                                              // of the frame
                        state   <= S_DATA;
                    end else begin
                        state <= S_IDLE;      // false start, abandon
                    end
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // -------------------------------------------------------------
            S_DATA: begin
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    // Sample and shift in from the top: first bit received
                    // is the LSB, so after 8 shifts it has walked down to
                    // shreg[0]. This is why no bit reversal is needed.
                    shreg   <= {rxd_sync, shreg[7:1]};
                    if (bit_idx == 3'd7) begin
                        bit_idx <= 3'd0;
                        state   <= S_STOP;
                    end else begin
                        bit_idx <= bit_idx + 3'd1;
                    end
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // -------------------------------------------------------------
            S_STOP: begin
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt   <= 16'd0;
                    rx_data   <= shreg;
                    rx_valid  <= 1'b1;
                    // Stop bit must be mark. If it isn't, the framing is
                    // wrong (baud mismatch, or we locked onto a data bit
                    // as a start bit). Flag it -- but note rx_valid still
                    // pulses, so the consumer must check frame_err.
                    frame_err <= (rxd_sync != 1'b1);
                    state     <= S_IDLE;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // -------------------------------------------------------------
            default: state <= S_IDLE;

            endcase
        end
    end

endmodule


// ---------------------------------------------------------------------
// uart_m1 : convenience wrapper -- one instance of each
// ---------------------------------------------------------------------
module uart_m1 #(
    parameter integer CLK_FREQ = 100_000_000,
    parameter integer BAUD     = 115_200,
    // 100e6 / 115200 = 868.06 -> 868. Error 0.007%, i.e. 0.06% cumulative
    // over a 10-bit frame. Comfortably inside the ~2% budget. An NCO buys
    // nothing here; it only earns its keep on awkward oscillators (e.g. the
    // 12 MHz crystal on an Arty S7, where 12e6/115200 = 104.17).
    parameter integer CLKS_PER_BIT = CLK_FREQ / BAUD
)(
    input  wire       clk,
    input  wire       rst_n,

    // TX side
    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output wire       tx_busy,
    output wire       txd,

    // RX side
    input  wire       rxd,
    output wire [7:0] rx_data,
    output wire       rx_valid,
    output wire       frame_err
);

    uart_tx_m1 #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (tx_data),
        .tx_start (tx_start),
        .tx_busy  (tx_busy),
        .txd      (txd)
    );

    uart_rx_m1 #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk       (clk),
        .rst_n     (rst_n),
        .rxd       (rxd),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .frame_err (frame_err)
    );

endmodule

`default_nettype wire
