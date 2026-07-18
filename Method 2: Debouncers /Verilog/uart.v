//============================================================================
// SIMPLE UART FOR FPGA - ALL-IN-ONE FILE (single-file VERILOG edition)
// Source design: jakubcabal/uart-for-fpga (MIT). Verilog-2001 translation of
// the all-in-one VHDL file; original per-unit headers retained.
//
// This one file contains all SIX design units in dependency order, so it
// analyzes in a single pass in Vivado/xsim/iverilog with no other files:
//   1. UART_CLK_DIV    2. UART_DEBOUNCER    3. UART_PARITY
//   4. UART_RX         5. UART_TX           6. UART (top level)
// Add this file as a design source and instantiate UART (module, port and
// parameter names are identical to the VHDL entities/generics, so it is a
// drop-in swap and the upstream VHDL testbench still binds to it in a
// mixed-language Vivado sim).
//
// TRANSLATION NOTES (differences forced by the language, not the design):
//  * Divider constants: VHDL integer(real/real) ROUNDS TO NEAREST; Verilog
//    '/' truncates. Both localparams therefore add half the divisor before
//    dividing, reproducing the VHDL rounding exactly (worked example below).
//  * ceil(log2(N)) -> $clog2(N), guarded against DIV_MAX_VAL=1 (a null
//    range in VHDL, an illegal [-1:0] range in Verilog).
//  * Registers the VHDL leaves un-reset are given power-up initial values
//    here. The design's own documentation relies on FPGA flops waking as 0
//    (the inverted-RXD idle trick); Verilog initializers state that intent
//    explicitly, synthesize to FF INIT values on 7-series, and keep X out
//    of Icarus simulation. Hardware behavior is unchanged.
//  * The ASYNC_REG property the VHDL comments ask for in the XDC is applied
//    inline on the two synchronizer flops.
//  * UART_PARITY: upstream leaves PARITY_OUT undriven for "none" (it is
//    never instantiated in that case); tied to 0 here to stay lint-clean.
//  * RST is an UNUSED port on UART_CLK_DIV upstream (reset reaches the
//    counters through CLEAR). Port kept for interface fidelity.
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

//============================================================================
// UNIT 1/6 : UART_CLK_DIV - programmable divider / tick generator
// WHAT: Counts CLK cycles and pulses DIV_MARK for one cycle at DIV_MARK_POS;
//       CLEAR re-phases the count.
// WHY : The single timing primitive reused three times: the ~16x oversampler
//       in the top, and the per-bit timers inside RX and TX (RX re-phases it
//       on each start bit - that is how bit sampling stays aligned).
//============================================================================
//------------------------------------------------------------------------------
// PROJECT: SIMPLE UART FOR FPGA
// AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
// LICENSE: The MIT License, please read LICENSE file
// WEBSITE: https://github.com/jakubcabal/uart-for-fpga
//------------------------------------------------------------------------------

module UART_CLK_DIV #(
    parameter integer DIV_MAX_VAL  = 16,
    parameter integer DIV_MARK_POS = 1
)(
    input  wire CLK,      // system clock
    input  wire RST,      // high active synchronous reset (unused upstream:
                          // the counter is re-phased through CLEAR instead)
    // USER INTERFACE
    input  wire CLEAR,    // clock divider counter clear
    input  wire ENABLE,   // clock divider counter enable
    output reg  DIV_MARK = 1'b0 // output divider mark (divided clock enable)
);

    // VHDL: integer(ceil(log2(real(DIV_MAX_VAL)))). $clog2 matches it for
    // every DIV_MAX_VAL >= 2; the guard avoids a zero-width vector at 1.
    localparam integer CLK_DIV_WIDTH = (DIV_MAX_VAL < 2) ? 1 : $clog2(DIV_MAX_VAL);

    reg  [CLK_DIV_WIDTH-1:0] clk_div_cnt = {CLK_DIV_WIDTH{1'b0}};
    wire                     clk_div_cnt_mark;

    // counter: CLEAR has priority over ENABLE, exactly as upstream
    always @(posedge CLK) begin
        if (CLEAR) begin
            clk_div_cnt <= {CLK_DIV_WIDTH{1'b0}};
        end else if (ENABLE) begin
            if (clk_div_cnt == DIV_MAX_VAL-1)
                clk_div_cnt <= {CLK_DIV_WIDTH{1'b0}};
            else
                clk_div_cnt <= clk_div_cnt + 1'b1;
        end
    end

    assign clk_div_cnt_mark = (clk_div_cnt == DIV_MARK_POS);

    // registered mark: one CLK cycle wide, only while ENABLE is high
    always @(posedge CLK) begin
        DIV_MARK <= ENABLE & clk_div_cnt_mark;
    end

endmodule

//============================================================================
// UNIT 2/6 : UART_DEBOUNCER - glitch filter
// WHAT: Output follows input only after it has been stable for LATENCY
//       consecutive CLK cycles.
// WHY : A sub-bit-period noise spike on an idle line would otherwise be taken
//       as a start bit and desynchronize a whole frame.
//============================================================================
//------------------------------------------------------------------------------
// PROJECT: SIMPLE UART FOR FPGA
// AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
// LICENSE: The MIT License, please read LICENSE file
// WEBSITE: https://github.com/jakubcabal/uart-for-fpga
//------------------------------------------------------------------------------

module UART_DEBOUNCER #(
    // latency of debouncer in clock cycles, minimum value is 2,
    // value also corresponds to the number of bits compared
    parameter integer LATENCY = 4
)(
    input  wire CLK,           // system clock
    input  wire DEB_IN,        // input of signal from outside FPGA
    output reg  DEB_OUT = 1'b0 // output of debounced (filtered) signal
);

    localparam integer SHREG_DEPTH = LATENCY - 1;

    reg  [SHREG_DEPTH-1:0] input_shreg = {SHREG_DEPTH{1'b0}};
    wire                   output_reg_rst;
    wire                   output_reg_set;

    // parameterized input shift register
    // (LATENCY=2 gives a 1-deep shreg; the VHDL handles that as a null slice,
    //  Verilog needs the explicit branch)
    generate
        if (SHREG_DEPTH > 1) begin : g_shreg_multi
            always @(posedge CLK)
                input_shreg <= {input_shreg[SHREG_DEPTH-2:0], DEB_IN};
        end else begin : g_shreg_single
            always @(posedge CLK)
                input_shreg <= DEB_IN;
        end
    endgenerate

    // output register will be reset when all compared bits are low
    assign output_reg_rst = ~(|{input_shreg, DEB_IN});
    // output register will be set when all compared bits are high
    assign output_reg_set =  &{input_shreg, DEB_IN};

    // output register: holds its value while the input is mid-transition
    always @(posedge CLK) begin
        if (output_reg_rst)
            DEB_OUT <= 1'b0;
        else if (output_reg_set)
            DEB_OUT <= 1'b1;
    end

endmodule

//============================================================================
// UNIT 3/6 : UART_PARITY - combinational parity generator
// WHAT: Reduces the data bits to one parity bit; mode (even/odd/mark/space)
//       selected by parameter.
// WHY : Shared by RX (to check) and TX (to generate) so both ends compute
//       parity identically; costs nothing when PARITY_BIT="none" because it
//       is never instantiated in that case.
//============================================================================
//------------------------------------------------------------------------------
// PROJECT: SIMPLE UART FOR FPGA
// AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
// LICENSE: The MIT License, please read LICENSE file
// WEBSITE: https://github.com/jakubcabal/uart-for-fpga
//------------------------------------------------------------------------------

module UART_PARITY #(
    parameter integer DATA_WIDTH  = 8,
    parameter         PARITY_TYPE = "none" // legal: "none","even","odd","mark","space"
)(
    input  wire [DATA_WIDTH-1:0] DATA_IN,
    output wire                  PARITY_OUT
);

    // -------------------------------------------------------------------------
    // PARITY BIT GENERATOR
    // -------------------------------------------------------------------------
    // VHDL loops XOR-accumulating over DATA_IN'range from a seed of '0' (even)
    // or '1' (odd); the reduction operators below are the same computation.
    generate
        if (PARITY_TYPE == "even") begin : g_even
            assign PARITY_OUT = ^DATA_IN;    // seed 0, XOR of all bits
        end else if (PARITY_TYPE == "odd") begin : g_odd
            assign PARITY_OUT = ~^DATA_IN;   // seed 1, XOR of all bits
        end else if (PARITY_TYPE == "mark") begin : g_mark
            assign PARITY_OUT = 1'b1;
        end else if (PARITY_TYPE == "space") begin : g_space
            assign PARITY_OUT = 1'b0;
        end else begin : g_none
            // upstream leaves the port undriven for "none"; tied off for lint
            assign PARITY_OUT = 1'b0;
        end
    endgenerate

endmodule

//============================================================================
// UNIT 4/6 : UART_RX - receiver engine
// WHAT: FSM detects the start edge, re-phases its bit timer to it, samples
//       8 data bits LSB-first, then checks parity and stop.
// WHY : The deserializer half. Contract: DOUT_VLD pulses one cycle ONLY for a
//       clean frame; a low stop bit pulses FRAME_ERROR instead.
//============================================================================
//------------------------------------------------------------------------------
// PROJECT: SIMPLE UART FOR FPGA
// AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
// LICENSE: The MIT License, please read LICENSE file
// WEBSITE: https://github.com/jakubcabal/uart-for-fpga
//------------------------------------------------------------------------------

module UART_RX #(
    parameter integer CLK_DIV_VAL = 16,
    parameter         PARITY_BIT  = "none" // "none","even","odd","mark","space"
)(
    input  wire       CLK,          // system clock
    input  wire       RST,          // high active synchronous reset
    // UART INTERFACE
    input  wire       UART_CLK_EN,  // oversampling (16x) UART clock enable
    input  wire       UART_RXD,     // serial receive data
    // USER DATA OUTPUT INTERFACE
    output wire [7:0] DOUT,                // output data received via UART
    output reg        DOUT_VLD     = 1'b0, // 1 for one cycle: DOUT valid, no errors
    output reg        FRAME_ERROR  = 1'b0, // 1 for one cycle: stop bit was invalid
    output reg        PARITY_ERROR = 1'b0  // 1 for one cycle: parity bit was invalid
);

    // FSM state encoding (VHDL enumerated type: idle, startbit, databits,
    // paritybit, stopbit)
    localparam [2:0] S_IDLE      = 3'd0,
                     S_STARTBIT  = 3'd1,
                     S_DATABITS  = 3'd2,
                     S_PARITYBIT = 3'd3,
                     S_STOPBIT   = 3'd4;

    wire       rx_clk_en;
    reg  [7:0] rx_data      = 8'h00;
    reg  [2:0] rx_bit_count = 3'd0;
    wire       rx_parity_error;
    wire       rx_done;
    reg        fsm_idle;
    reg        fsm_databits;
    reg        fsm_stopbit;
    reg  [2:0] fsm_pstate   = S_IDLE;
    reg  [2:0] fsm_nstate;

    // -------------------------------------------------------------------------
    // UART RECEIVER CLOCK DIVIDER AND CLOCK ENABLE FLAG
    // -------------------------------------------------------------------------
    // Cleared while idle, so the per-bit timer re-phases to every start edge.

    UART_CLK_DIV #(
        .DIV_MAX_VAL  (CLK_DIV_VAL),
        .DIV_MARK_POS (3)
    ) rx_clk_divider_i (
        .CLK      (CLK),
        .RST      (RST),
        .CLEAR    (fsm_idle),
        .ENABLE   (UART_CLK_EN),
        .DIV_MARK (rx_clk_en)
    );

    // -------------------------------------------------------------------------
    // UART RECEIVER BIT COUNTER
    // -------------------------------------------------------------------------

    always @(posedge CLK) begin
        if (RST) begin
            rx_bit_count <= 3'd0;
        end else if (rx_clk_en && fsm_databits) begin
            if (rx_bit_count == 3'b111)
                rx_bit_count <= 3'd0;
            else
                rx_bit_count <= rx_bit_count + 3'd1;
        end
    end

    // -------------------------------------------------------------------------
    // UART RECEIVER DATA SHIFT REGISTER  (LSB first: new bit enters at MSB)
    // -------------------------------------------------------------------------

    always @(posedge CLK) begin
        if (rx_clk_en && fsm_databits)
            rx_data <= {UART_RXD, rx_data[7:1]};
    end

    assign DOUT = rx_data;

    // -------------------------------------------------------------------------
    // UART RECEIVER PARITY GENERATOR AND CHECK
    // -------------------------------------------------------------------------
    // Note (upstream behavior, preserved): the check register updates on EVERY
    // rx_clk_en mark; only the last update before the stop-bit mark - i.e. the
    // parity-bit mark - is the one the output register consumes.

    generate
        if (PARITY_BIT != "none") begin : g_rx_parity
            wire rx_parity_bit;
            reg  rx_parity_error_r = 1'b0;

            UART_PARITY #(
                .DATA_WIDTH  (8),
                .PARITY_TYPE (PARITY_BIT)
            ) uart_rx_parity_gen_i (
                .DATA_IN    (rx_data),
                .PARITY_OUT (rx_parity_bit)
            );

            always @(posedge CLK) begin
                if (rx_clk_en)
                    rx_parity_error_r <= rx_parity_bit ^ UART_RXD;
            end

            assign rx_parity_error = rx_parity_error_r;
        end else begin : g_rx_noparity
            assign rx_parity_error = 1'b0;
        end
    endgenerate

    // -------------------------------------------------------------------------
    // UART RECEIVER OUTPUT REGISTER
    // -------------------------------------------------------------------------
    // The frame verdict, decided in one cycle at the stop-bit mark:
    //   stop high & parity ok -> DOUT_VLD ; stop low -> FRAME_ERROR ;
    //   parity bad            -> PARITY_ERROR (mutually exclusive strobes).

    assign rx_done = rx_clk_en & fsm_stopbit;

    always @(posedge CLK) begin
        if (RST) begin
            DOUT_VLD     <= 1'b0;
            FRAME_ERROR  <= 1'b0;
            PARITY_ERROR <= 1'b0;
        end else begin
            DOUT_VLD     <= rx_done & ~rx_parity_error &  UART_RXD;
            FRAME_ERROR  <= rx_done & ~UART_RXD;
            PARITY_ERROR <= rx_done &  rx_parity_error;
        end
    end

    // -------------------------------------------------------------------------
    // UART RECEIVER FSM
    // -------------------------------------------------------------------------

    // PRESENT STATE REGISTER
    always @(posedge CLK) begin
        if (RST)
            fsm_pstate <= S_IDLE;
        else
            fsm_pstate <= fsm_nstate;
    end

    // NEXT STATE AND OUTPUTS LOGIC (every output assigned in every branch:
    // no inferred latches; PARITY_BIT compare is a constant and folds away)
    always @(*) begin
        case (fsm_pstate)

            S_IDLE: begin
                fsm_stopbit  = 1'b0;
                fsm_databits = 1'b0;
                fsm_idle     = 1'b1;
                if (UART_RXD == 1'b0)
                    fsm_nstate = S_STARTBIT;
                else
                    fsm_nstate = S_IDLE;
            end

            S_STARTBIT: begin
                fsm_stopbit  = 1'b0;
                fsm_databits = 1'b0;
                fsm_idle     = 1'b0;
                if (rx_clk_en)
                    fsm_nstate = S_DATABITS;
                else
                    fsm_nstate = S_STARTBIT;
            end

            S_DATABITS: begin
                fsm_stopbit  = 1'b0;
                fsm_databits = 1'b1;
                fsm_idle     = 1'b0;
                if (rx_clk_en && (rx_bit_count == 3'b111)) begin
                    if (PARITY_BIT == "none")
                        fsm_nstate = S_STOPBIT;
                    else
                        fsm_nstate = S_PARITYBIT;
                end else begin
                    fsm_nstate = S_DATABITS;
                end
            end

            S_PARITYBIT: begin
                fsm_stopbit  = 1'b0;
                fsm_databits = 1'b0;
                fsm_idle     = 1'b0;
                if (rx_clk_en)
                    fsm_nstate = S_STOPBIT;
                else
                    fsm_nstate = S_PARITYBIT;
            end

            S_STOPBIT: begin
                fsm_stopbit  = 1'b1;
                fsm_databits = 1'b0;
                fsm_idle     = 1'b0;
                if (rx_clk_en)
                    fsm_nstate = S_IDLE;
                else
                    fsm_nstate = S_STOPBIT;
            end

            default: begin
                fsm_stopbit  = 1'b0;
                fsm_databits = 1'b0;
                fsm_idle     = 1'b0;
                fsm_nstate   = S_IDLE;
            end

        endcase
    end

endmodule

//============================================================================
// UNIT 5/6 : UART_TX - transmitter engine
// WHAT: Latches DIN on the DIN_VLD-and-DIN_RDY handshake, then shifts start +
//       8 data bits LSB-first + optional parity + stop.
// WHY : The serializer half. No FIFO: DIN_RDY low back-pressures the producer
//       until the frame completes. DIN_RDY re-asserts during the stop bit so
//       back-to-back frames run gapless (stopbit -> txsync keeps the line at
//       the stop level until the next bit boundary).
//============================================================================
//------------------------------------------------------------------------------
// PROJECT: SIMPLE UART FOR FPGA
// AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
// LICENSE: The MIT License, please read LICENSE file
// WEBSITE: https://github.com/jakubcabal/uart-for-fpga
//------------------------------------------------------------------------------

module UART_TX #(
    parameter integer CLK_DIV_VAL = 16,
    parameter         PARITY_BIT  = "none" // "none","even","odd","mark","space"
)(
    input  wire       CLK,             // system clock
    input  wire       RST,             // high active synchronous reset
    // UART INTERFACE
    input  wire       UART_CLK_EN,     // oversampling (16x) UART clock enable
    output reg        UART_TXD = 1'b1, // serial transmit data (idles high)
    // USER DATA INPUT INTERFACE
    input  wire [7:0] DIN,             // input data to be transmitted over UART
    input  wire       DIN_VLD,         // when 1, input data (DIN) are valid
    output wire       DIN_RDY          // when 1, a valid byte is accepted this cycle
);

    // FSM state encoding (VHDL: idle, txsync, startbit, databits, paritybit,
    // stopbit)
    localparam [2:0] S_IDLE      = 3'd0,
                     S_TXSYNC    = 3'd1,
                     S_STARTBIT  = 3'd2,
                     S_DATABITS  = 3'd3,
                     S_PARITYBIT = 3'd4,
                     S_STOPBIT   = 3'd5;

    wire       tx_clk_en;
    reg        tx_clk_div_clr;
    reg  [7:0] tx_data      = 8'h00;
    reg  [2:0] tx_bit_count = 3'd0;
    reg        tx_bit_count_en;
    reg        tx_ready;
    wire       tx_parity_bit;
    reg  [1:0] tx_data_out_sel;
    reg  [2:0] tx_pstate    = S_IDLE;
    reg  [2:0] tx_nstate;

    assign DIN_RDY = tx_ready;

    // -------------------------------------------------------------------------
    // UART TRANSMITTER CLOCK DIVIDER AND CLOCK ENABLE FLAG
    // -------------------------------------------------------------------------
    // Cleared only in idle: the mark grid is established when the first byte
    // is accepted and every bit thereafter is exactly one grid period.

    UART_CLK_DIV #(
        .DIV_MAX_VAL  (CLK_DIV_VAL),
        .DIV_MARK_POS (1)
    ) tx_clk_divider_i (
        .CLK      (CLK),
        .RST      (RST),
        .CLEAR    (tx_clk_div_clr),
        .ENABLE   (UART_CLK_EN),
        .DIV_MARK (tx_clk_en)
    );

    // -------------------------------------------------------------------------
    // UART TRANSMITTER INPUT DATA REGISTER (latched on the VLD & RDY handshake)
    // -------------------------------------------------------------------------

    always @(posedge CLK) begin
        if (DIN_VLD && tx_ready)
            tx_data <= DIN;
    end

    // -------------------------------------------------------------------------
    // UART TRANSMITTER BIT COUNTER
    // -------------------------------------------------------------------------

    always @(posedge CLK) begin
        if (RST) begin
            tx_bit_count <= 3'd0;
        end else if (tx_bit_count_en && tx_clk_en) begin
            if (tx_bit_count == 3'b111)
                tx_bit_count <= 3'd0;
            else
                tx_bit_count <= tx_bit_count + 3'd1;
        end
    end

    // -------------------------------------------------------------------------
    // UART TRANSMITTER PARITY GENERATOR
    // -------------------------------------------------------------------------

    generate
        if (PARITY_BIT != "none") begin : g_tx_parity
            UART_PARITY #(
                .DATA_WIDTH  (8),
                .PARITY_TYPE (PARITY_BIT)
            ) uart_tx_parity_gen_i (
                .DATA_IN    (tx_data),
                .PARITY_OUT (tx_parity_bit)
            );
        end else begin : g_tx_noparity
            assign tx_parity_bit = 1'b0;
        end
    endgenerate

    // -------------------------------------------------------------------------
    // UART TRANSMITTER OUTPUT DATA REGISTER
    // -------------------------------------------------------------------------

    always @(posedge CLK) begin
        if (RST) begin
            UART_TXD <= 1'b1;
        end else begin
            case (tx_data_out_sel)
                2'b01:   UART_TXD <= 1'b0;                      // START BIT
                2'b10:   UART_TXD <= tx_data[tx_bit_count];     // DATA BITS
                2'b11:   UART_TXD <= tx_parity_bit;             // PARITY BIT
                default: UART_TXD <= 1'b1;                      // STOP BIT OR IDLE
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // UART TRANSMITTER FSM
    // -------------------------------------------------------------------------

    // PRESENT STATE REGISTER
    always @(posedge CLK) begin
        if (RST)
            tx_pstate <= S_IDLE;
        else
            tx_pstate <= tx_nstate;
    end

    // NEXT STATE AND OUTPUTS LOGIC
    always @(*) begin
        case (tx_pstate)

            S_IDLE: begin
                tx_ready        = 1'b1;
                tx_data_out_sel = 2'b00;
                tx_bit_count_en = 1'b0;
                tx_clk_div_clr  = 1'b1;
                if (DIN_VLD)
                    tx_nstate = S_TXSYNC;
                else
                    tx_nstate = S_IDLE;
            end

            S_TXSYNC: begin
                tx_ready        = 1'b0;
                tx_data_out_sel = 2'b00;
                tx_bit_count_en = 1'b0;
                tx_clk_div_clr  = 1'b0;
                if (tx_clk_en)
                    tx_nstate = S_STARTBIT;
                else
                    tx_nstate = S_TXSYNC;
            end

            S_STARTBIT: begin
                tx_ready        = 1'b0;
                tx_data_out_sel = 2'b01;
                tx_bit_count_en = 1'b0;
                tx_clk_div_clr  = 1'b0;
                if (tx_clk_en)
                    tx_nstate = S_DATABITS;
                else
                    tx_nstate = S_STARTBIT;
            end

            S_DATABITS: begin
                tx_ready        = 1'b0;
                tx_data_out_sel = 2'b10;
                tx_bit_count_en = 1'b1;
                tx_clk_div_clr  = 1'b0;
                if (tx_clk_en && (tx_bit_count == 3'b111)) begin
                    if (PARITY_BIT == "none")
                        tx_nstate = S_STOPBIT;
                    else
                        tx_nstate = S_PARITYBIT;
                end else begin
                    tx_nstate = S_DATABITS;
                end
            end

            S_PARITYBIT: begin
                tx_ready        = 1'b0;
                tx_data_out_sel = 2'b11;
                tx_bit_count_en = 1'b0;
                tx_clk_div_clr  = 1'b0;
                if (tx_clk_en)
                    tx_nstate = S_STOPBIT;
                else
                    tx_nstate = S_PARITYBIT;
            end

            S_STOPBIT: begin
                tx_ready        = 1'b1;   // accepts the next byte DURING the stop bit
                tx_data_out_sel = 2'b00;
                tx_bit_count_en = 1'b0;
                tx_clk_div_clr  = 1'b0;
                if (DIN_VLD)
                    tx_nstate = S_TXSYNC;  // txsync holds the line high to the
                                           // next mark -> stop bit keeps full width
                else if (tx_clk_en)
                    tx_nstate = S_IDLE;
                else
                    tx_nstate = S_STOPBIT;
            end

            default: begin
                tx_ready        = 1'b0;
                tx_data_out_sel = 2'b00;
                tx_bit_count_en = 1'b0;
                tx_clk_div_clr  = 1'b0;
                tx_nstate       = S_IDLE;
            end

        endcase
    end

endmodule

//============================================================================
// UNIT 6/6 : UART - top level
// WHAT: Wires divider + synchronizer + debouncer + RX + TX into one module
//       with a byte-stream user interface.
// WHY : This is the unit you instantiate; everything above is internal
//       plumbing.
//
// Frame format is fixed: 1 start bit, 8 data bits (LSB first), optional
// parity bit, 1 stop bit. Everything else is set through the parameters.
//============================================================================
//------------------------------------------------------------------------------
// PROJECT: SIMPLE UART FOR FPGA
// AUTHORS: Jakub Cabal <jakubcabal@gmail.com>
// LICENSE: The MIT License, please read LICENSE file
// WEBSITE: https://github.com/jakubcabal/uart-for-fpga
//------------------------------------------------------------------------------
// UART FOR FPGA REQUIRES: 1 START BIT, 8 DATA BITS, 1 STOP BIT!!!
// OTHER PARAMETERS CAN BE SET USING PARAMETERS.

module UART #(
    parameter integer CLK_FREQ      = 50_000_000, // Hz; MUST match the real clock on CLK or the baud rate is wrong
    parameter integer BAUD_RATE     = 115200,     // keep CLK_FREQ/BAUD_RATE >= ~16 for the oversampler
    parameter         PARITY_BIT    = "none",     // "none","even","odd","mark","space"
    parameter         USE_DEBOUNCER = 1           // 1 = filter sub-4-cycle glitches on RXD (keep 1 on real cables)
)(
    // CLOCK AND RESET
    input  wire       CLK,          // system clock; single domain for every flop
    input  wire       RST,          // high active synchronous reset; hold >= 2 cycles at startup
    // UART INTERFACE (the physical serial pins)
    output wire       UART_TXD,     // serial transmit data; idles high
    input  wire       UART_RXD,     // serial receive data; ASYNC input, synchronized internally
    // USER DATA INPUT INTERFACE (parallel side, TX direction)
    input  wire [7:0] DIN,          // input data to be transmitted over UART
    input  wire       DIN_VLD,      // byte accepted on the edge where DIN_VLD and DIN_RDY are both 1
    output wire       DIN_RDY,      // no FIFO: wait for this before each byte
    // USER DATA OUTPUT INTERFACE (parallel side, RX direction)
    output wire [7:0] DOUT,         // received byte; stable until the next byte finishes
    output wire       DOUT_VLD,     // 1-cycle strobe, only on an error-free frame: capture DOUT on it
    output wire       FRAME_ERROR,  // 1-cycle strobe, fires INSTEAD of DOUT_VLD when the stop bit was low
    output wire       PARITY_ERROR  // 1-cycle strobe; only possible when PARITY_BIT != "none"
);

    // ==== BAUD-RATE DIVIDER CONSTANTS ========================================
    // WHY the "+ divisor/2": VHDL's integer(real/real) rounds to the NEAREST
    // divider, halving the worst-case baud error vs truncation; Verilog '/'
    // truncates, so the rounding is reproduced explicitly.
    // At 50 MHz / 115200: (50e6+921600)/1843200 = 27 (real 27.13 -> 27), then
    // (50e6+1555200)/3110400 = 16 (real 16.08 -> 16): effective 115740 baud
    // (+0.47%, well inside the ~2% UART tolerance).
    localparam integer OS_CLK_DIV_VAL   = (CLK_FREQ + (16*BAUD_RATE)/2)
                                          / (16*BAUD_RATE);              // CLK cycles per oversampling tick
    localparam integer UART_CLK_DIV_VAL = (CLK_FREQ + (OS_CLK_DIV_VAL*BAUD_RATE)/2)
                                          / (OS_CLK_DIV_VAL*BAUD_RATE); // oversampling ticks per bit (nominally 16)

    // ==== INTERNAL SIGNALS ===================================================
    // WHY the "_n" (inverted) naming: the RXD path is deliberately carried
    // INVERTED through the synchronizer/debouncer flops. Flops wake up as 0
    // after configuration (made explicit by the initializers), and
    // inverted-idle (0 = line idle-high) means the receiver sees a clean idle
    // line at power-up instead of a phantom start bit.
    wire os_clk_en;             // one-cycle enable at ~16x baud; distributed instead of a derived clock (single domain, no CDC)
    (* ASYNC_REG = "TRUE" *)
    reg  uart_rxd_meta_n   = 1'b0;   // synchronizer flop 1; may go metastable, read only by flop 2
    (* ASYNC_REG = "TRUE" *)
    reg  uart_rxd_synced_n = 1'b0;   // synchronizer flop 2; safe, still inverted
    wire uart_rxd_debounced_n;       // after glitch filtering, still inverted
    wire uart_rxd_debounced;         // final clean, true-polarity RXD used by the receiver

    // -------------------------------------------------------------------------
    //  UART OVERSAMPLING (~16X) CLOCK DIVIDER AND CLOCK ENABLE FLAG
    // -------------------------------------------------------------------------
    // Free-running timebase for everything serial. CLEAR tied to RST so the
    // tick phase is deterministic after reset.

    UART_CLK_DIV #(
        .DIV_MAX_VAL  (OS_CLK_DIV_VAL),
        .DIV_MARK_POS (OS_CLK_DIV_VAL-1)  // emit the tick on the last count value
    ) os_clk_divider_i (
        .CLK      (CLK),
        .RST      (RST),
        .CLEAR    (RST),
        .ENABLE   (1'b1),     // never pauses (RX/TX gate themselves)
        .DIV_MARK (os_clk_en)
    );

    // -------------------------------------------------------------------------
    //  UART RXD CROSS DOMAIN CROSSING
    // -------------------------------------------------------------------------
    // REQUIRED two-flop synchronizer: UART_RXD comes from an unrelated clock;
    // sampling it directly would feed metastable values into the RX FSM.
    // ASYNC_REG keeps the pair packed together (the VHDL asks for this in the
    // XDC; here it rides on the registers directly).

    always @(posedge CLK) begin
        uart_rxd_meta_n   <= ~UART_RXD;        // flop 1: capture async pin (inverted)
        uart_rxd_synced_n <= uart_rxd_meta_n;  // flop 2: now safe to use
    end

    // -------------------------------------------------------------------------
    //  UART RXD DEBOUNCER
    // -------------------------------------------------------------------------
    // Optional glitch filter; a generate pair so the disabled case costs zero
    // hardware. 4 CLK cycles of required stability is far below a bit period,
    // so it has no timing impact on the frame.

    generate
        if (USE_DEBOUNCER != 0) begin : use_debouncer_g
            UART_DEBOUNCER #(
                .LATENCY (4)
            ) debouncer_i (
                .CLK     (CLK),
                .DEB_IN  (uart_rxd_synced_n),
                .DEB_OUT (uart_rxd_debounced_n)
            );
        end else begin : not_use_debouncer_g
            assign uart_rxd_debounced_n = uart_rxd_synced_n;
        end
    endgenerate

    // undo the storage inversion: from here on, 1 = idle, 0 = start bit
    assign uart_rxd_debounced = ~uart_rxd_debounced_n;

    // -------------------------------------------------------------------------
    //  UART RECEIVER
    // -------------------------------------------------------------------------

    UART_RX #(
        .CLK_DIV_VAL (UART_CLK_DIV_VAL),
        .PARITY_BIT  (PARITY_BIT)
    ) uart_rx_i (
        .CLK          (CLK),
        .RST          (RST),
        // UART INTERFACE
        .UART_CLK_EN  (os_clk_en),
        .UART_RXD     (uart_rxd_debounced),
        // USER DATA OUTPUT INTERFACE
        .DOUT         (DOUT),
        .DOUT_VLD     (DOUT_VLD),
        .FRAME_ERROR  (FRAME_ERROR),
        .PARITY_ERROR (PARITY_ERROR)
    );

    // -------------------------------------------------------------------------
    //  UART TRANSMITTER
    // -------------------------------------------------------------------------

    UART_TX #(
        .CLK_DIV_VAL (UART_CLK_DIV_VAL), // same ticks-per-bit as RX: both ends agree on the baud rate
        .PARITY_BIT  (PARITY_BIT)
    ) uart_tx_i (
        .CLK         (CLK),
        .RST         (RST),
        // UART INTERFACE
        .UART_CLK_EN (os_clk_en),
        .UART_TXD    (UART_TXD),
        // USER DATA INPUT INTERFACE
        .DIN         (DIN),
        .DIN_VLD     (DIN_VLD),
        .DIN_RDY     (DIN_RDY)
    );

endmodule

`default_nettype wire
