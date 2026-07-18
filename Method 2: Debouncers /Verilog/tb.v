//============================================================================
// tb_uart.v - self-checking regression for the Verilog UART translation
//
// Three DUT configurations:
//   dut1: PARITY_BIT="none", USE_DEBOUNCER=1  (default configuration)
//   dut2: PARITY_BIT="even", USE_DEBOUNCER=1  (parity generate/check paths)
//   dut3: PARITY_BIT="odd",  USE_DEBOUNCER=0  (odd parity + debouncer bypass)
//
// The TB drives RXD with its own ideal 115200 bit timing (8680.6 ns/bit)
// while the DUT runs at its rounded effective rate (115740 baud, +0.47%),
// so every RX test also exercises baud-mismatch tolerance.
//
// Run:  iverilog -g2005 -Wall -o build/uart_sim uart.v tb_uart.v
//       vvp build/uart_sim            (add +dump for tb_uart.vcd)
//============================================================================
`timescale 1ns / 1ps
`default_nettype none

module tb_uart;

    localparam integer CLK_FREQ   = 50_000_000;
    localparam integer BAUD_RATE  = 115_200;
    localparam real    CLK_PERIOD = 1.0e9 / CLK_FREQ;   // 20 ns
    localparam real    BIT_PERIOD = 1.0e9 / BAUD_RATE;  // 8680.556 ns

    // ---- clock / reset ------------------------------------------------------
    reg clk = 1'b0;
    reg rst = 1'b1;
    always #(CLK_PERIOD/2.0) clk = ~clk;

    // ---- DUT1: no parity, debouncer on -------------------------------------
    reg        rxd1 = 1'b1;
    wire       txd1;
    reg  [7:0] din1 = 8'h00;
    reg        din1_vld = 1'b0;
    wire       din1_rdy;
    wire [7:0] dout1;
    wire       dout1_vld, ferr1, perr1;

    UART #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE),
        .PARITY_BIT("none"), .USE_DEBOUNCER(1)
    ) dut1 (
        .CLK(clk), .RST(rst),
        .UART_TXD(txd1), .UART_RXD(rxd1),
        .DIN(din1), .DIN_VLD(din1_vld), .DIN_RDY(din1_rdy),
        .DOUT(dout1), .DOUT_VLD(dout1_vld),
        .FRAME_ERROR(ferr1), .PARITY_ERROR(perr1)
    );

    // ---- DUT2: even parity, debouncer on ------------------------------------
    reg        rxd2 = 1'b1;
    wire       txd2;
    reg  [7:0] din2 = 8'h00;
    reg        din2_vld = 1'b0;
    wire       din2_rdy;
    wire [7:0] dout2;
    wire       dout2_vld, ferr2, perr2;

    UART #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE),
        .PARITY_BIT("even"), .USE_DEBOUNCER(1)
    ) dut2 (
        .CLK(clk), .RST(rst),
        .UART_TXD(txd2), .UART_RXD(rxd2),
        .DIN(din2), .DIN_VLD(din2_vld), .DIN_RDY(din2_rdy),
        .DOUT(dout2), .DOUT_VLD(dout2_vld),
        .FRAME_ERROR(ferr2), .PARITY_ERROR(perr2)
    );

    // ---- DUT3: odd parity, debouncer OFF ------------------------------------
    reg        rxd3 = 1'b1;
    wire       txd3;
    wire       din3_rdy;
    wire [7:0] dout3;
    wire       dout3_vld, ferr3, perr3;

    UART #(
        .CLK_FREQ(CLK_FREQ), .BAUD_RATE(BAUD_RATE),
        .PARITY_BIT("odd"), .USE_DEBOUNCER(0)
    ) dut3 (
        .CLK(clk), .RST(rst),
        .UART_TXD(txd3), .UART_RXD(rxd3),
        .DIN(8'h00), .DIN_VLD(1'b0), .DIN_RDY(din3_rdy),
        .DOUT(dout3), .DOUT_VLD(dout3_vld),
        .FRAME_ERROR(ferr3), .PARITY_ERROR(perr3)
    );

    // ---- strobe counters / last-byte capture --------------------------------
    integer   vld1_cnt = 0, ferr1_cnt = 0, perr1_cnt = 0;
    integer   vld2_cnt = 0, ferr2_cnt = 0, perr2_cnt = 0;
    integer   vld3_cnt = 0, ferr3_cnt = 0, perr3_cnt = 0;
    reg [7:0] rx1_last = 8'h00, rx2_last = 8'h00, rx3_last = 8'h00;

    always @(posedge clk) begin
        if (dout1_vld) begin vld1_cnt = vld1_cnt + 1; rx1_last = dout1; end
        if (ferr1)     ferr1_cnt = ferr1_cnt + 1;
        if (perr1)     perr1_cnt = perr1_cnt + 1;
        if (dout2_vld) begin vld2_cnt = vld2_cnt + 1; rx2_last = dout2; end
        if (ferr2)     ferr2_cnt = ferr2_cnt + 1;
        if (perr2)     perr2_cnt = perr2_cnt + 1;
        if (dout3_vld) begin vld3_cnt = vld3_cnt + 1; rx3_last = dout3; end
        if (ferr3)     ferr3_cnt = ferr3_cnt + 1;
        if (perr3)     perr3_cnt = perr3_cnt + 1;
    end

    // ---- TX handshake accept counters (samples pre-edge values, like the DUT)
    wire    tx1_hs = din1_vld & din1_rdy;
    wire    tx2_hs = din2_vld & din2_rdy;
    integer tx1_acc = 0, tx2_acc = 0;
    always @(posedge clk) begin
        if (tx1_hs) tx1_acc = tx1_acc + 1;
        if (tx2_hs) tx2_acc = tx2_acc + 1;
    end

    // ---- TXD line monitors ---------------------------------------------------
    // Resync on every start edge, sample at nominal bit centers; 0.47% baud
    // drift accumulates to <5% of a bit by the stop bit - safely inside.
    reg [7:0] tx1_bytes [0:15];
    integer   tx1_cnt = 0;
    reg       tx1_stop_ok = 1'b1;

    initial begin : tx1_monitor
        integer i;
        reg [7:0] b;
        forever begin
            @(negedge txd1);                      // start bit edge
            #(BIT_PERIOD*1.5);                    // center of data bit 0
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = txd1;                      // LSB first
                #(BIT_PERIOD);
            end
            if (txd1 !== 1'b1) tx1_stop_ok = 1'b0; // 9.5 bits: stop center
            tx1_bytes[tx1_cnt] = b;
            tx1_cnt = tx1_cnt + 1;
        end
    end

    reg [7:0] tx2_bytes [0:15];
    reg       tx2_par   [0:15];
    integer   tx2_cnt = 0;
    reg       tx2_stop_ok = 1'b1;

    initial begin : tx2_monitor
        integer i;
        reg [7:0] b;
        reg p;
        forever begin
            @(negedge txd2);
            #(BIT_PERIOD*1.5);
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = txd2;
                #(BIT_PERIOD);
            end
            p = txd2;                              // 9.5 bits: parity center
            #(BIT_PERIOD);
            if (txd2 !== 1'b1) tx2_stop_ok = 1'b0; // 10.5 bits: stop center
            tx2_bytes[tx2_cnt] = b;
            tx2_par[tx2_cnt]   = p;
            tx2_cnt = tx2_cnt + 1;
        end
    end

    // ---- serial frame drivers (TB -> DUT RXD, ideal timing) ------------------
    task uart1_drive_frame(input [7:0] data, input stop_bit);
        integer i;
        begin
            rxd1 = 1'b0;  #(BIT_PERIOD);                          // start
            for (i = 0; i < 8; i = i + 1) begin
                rxd1 = data[i];  #(BIT_PERIOD);                   // LSB first
            end
            rxd1 = stop_bit;  #(BIT_PERIOD);                      // stop (0 = frame error)
            rxd1 = 1'b1;  #(BIT_PERIOD);                          // inter-frame idle
        end
    endtask

    task uart2_drive_frame(input [7:0] data, input parity, input stop_bit);
        integer i;
        begin
            rxd2 = 1'b0;  #(BIT_PERIOD);
            for (i = 0; i < 8; i = i + 1) begin
                rxd2 = data[i];  #(BIT_PERIOD);
            end
            rxd2 = parity;    #(BIT_PERIOD);
            rxd2 = stop_bit;  #(BIT_PERIOD);
            rxd2 = 1'b1;      #(BIT_PERIOD);
        end
    endtask

    task uart3_drive_frame(input [7:0] data, input parity, input stop_bit);
        integer i;
        begin
            rxd3 = 1'b0;  #(BIT_PERIOD);
            for (i = 0; i < 8; i = i + 1) begin
                rxd3 = data[i];  #(BIT_PERIOD);
            end
            rxd3 = parity;    #(BIT_PERIOD);
            rxd3 = stop_bit;  #(BIT_PERIOD);
            rxd3 = 1'b1;      #(BIT_PERIOD);
        end
    endtask

    // ---- TX byte senders (drive on negedge, DUT samples mid-cycle) -----------
    task tx1_send(input [7:0] b);
        integer base;
        begin
            base = tx1_acc;
            @(negedge clk);
            din1 = b;  din1_vld = 1'b1;
            wait (tx1_acc == base + 1);
            @(negedge clk);
            din1_vld = 1'b0;
        end
    endtask

    // three bytes with DIN_VLD held high throughout: exercises the
    // stopbit -> txsync gapless back-to-back path
    task tx1_send_stream3(input [7:0] b0, input [7:0] b1, input [7:0] b2);
        integer base;
        begin
            base = tx1_acc;
            @(negedge clk);
            din1 = b0;  din1_vld = 1'b1;
            wait (tx1_acc == base + 1);  @(negedge clk);  din1 = b1;
            wait (tx1_acc == base + 2);  @(negedge clk);  din1 = b2;
            wait (tx1_acc == base + 3);  @(negedge clk);  din1_vld = 1'b0;
        end
    endtask

    task tx2_send(input [7:0] b);
        integer base;
        begin
            base = tx2_acc;
            @(negedge clk);
            din2 = b;  din2_vld = 1'b1;
            wait (tx2_acc == base + 1);
            @(negedge clk);
            din2_vld = 1'b0;
        end
    endtask

    // ---- scoreboard ----------------------------------------------------------
    integer pass_cnt = 0, fail_cnt = 0;

    task check(input cond, input [8*48-1:0] name);
        begin
            if (cond === 1'b1) begin
                pass_cnt = pass_cnt + 1;
                $display("  PASS  %0s", name);
            end else begin
                fail_cnt = fail_cnt + 1;
                $display("  FAIL  %0s", name);
            end
        end
    endtask

    // ---- global watchdog -----------------------------------------------------
    initial begin
        #10_000_000;  // 10 ms
        $display("GLOBAL TIMEOUT - TESTS FAILED");
        $finish;
    end

    // ---- main sequence -------------------------------------------------------
    initial begin : main
        integer base_v;

        if ($test$plusargs("dump")) begin
            $dumpfile("tb_uart.vcd");
            $dumpvars(0, tb_uart);
        end

        $display("== UART Verilog translation regression ==");
        repeat (10) @(posedge clk);
        rst = 1'b0;
        $display("dividers: os=%0d clk/tick, bit=%0d ticks -> effective %0d baud (nominal %0d)",
                 dut1.OS_CLK_DIV_VAL, dut1.UART_CLK_DIV_VAL,
                 CLK_FREQ/(dut1.OS_CLK_DIV_VAL*dut1.UART_CLK_DIV_VAL), BAUD_RATE);
        #(BIT_PERIOD*2);

        // ---- DUT1 RX: clean frames ------------------------------------------
        uart1_drive_frame(8'hA5, 1'b1);  #(BIT_PERIOD*2);
        check(vld1_cnt == 1 && rx1_last == 8'hA5 && ferr1_cnt == 0, "DUT1 RX 0xA5");
        uart1_drive_frame(8'h00, 1'b1);  #(BIT_PERIOD*2);
        check(vld1_cnt == 2 && rx1_last == 8'h00, "DUT1 RX 0x00");
        uart1_drive_frame(8'hFF, 1'b1);  #(BIT_PERIOD*2);
        check(vld1_cnt == 3 && rx1_last == 8'hFF, "DUT1 RX 0xFF");
        uart1_drive_frame(8'h3C, 1'b1);  #(BIT_PERIOD*2);
        check(vld1_cnt == 4 && rx1_last == 8'h3C, "DUT1 RX 0x3C");

        // ---- DUT1 debouncer: 2-CLK-cycle glitch must be swallowed ------------
        rxd1 = 1'b0;  #(2*CLK_PERIOD);  rxd1 = 1'b1;
        #(BIT_PERIOD*14);
        check(vld1_cnt == 4 && ferr1_cnt == 0 && perr1_cnt == 0,
              "DUT1 glitch filtered by debouncer");

        // ---- DUT1 frame error (broken stop bit) ------------------------------
        // Upstream-identical behavior, preserved on purpose: after the low
        // stop bit the FSM returns to idle while the line is still low,
        // immediately re-arms, and decodes the following idle-high line as a
        // phantom 0xFF frame with a good stop bit. Both effects are asserted.
        base_v = vld1_cnt;
        uart1_drive_frame(8'h5A, 1'b0);
        #(BIT_PERIOD*14);
        check(ferr1_cnt == 1, "DUT1 FRAME_ERROR on low stop bit");
        check(vld1_cnt == base_v + 1 && rx1_last == 8'hFF,
              "DUT1 phantom 0xFF after break (matches VHDL)");

        // ---- DUT1 TX: single byte then queued back-to-back -------------------
        tx1_send(8'h55);
        wait (tx1_cnt == 1);  #(BIT_PERIOD*2);
        check(tx1_bytes[0] == 8'h55 && tx1_stop_ok, "DUT1 TX 0x55");

        tx1_send_stream3(8'h11, 8'hE7, 8'h80);
        wait (tx1_cnt == 4);  #(BIT_PERIOD*2);
        check(tx1_bytes[1] == 8'h11 && tx1_bytes[2] == 8'hE7 &&
              tx1_bytes[3] == 8'h80 && tx1_stop_ok,
              "DUT1 TX back-to-back x3 (stopbit->txsync)");

        // ---- DUT2 (even parity) ----------------------------------------------
        uart2_drive_frame(8'h3C, 1'b0, 1'b1);  // even parity of 0x3C = 0
        #(BIT_PERIOD*2);
        check(vld2_cnt == 1 && rx2_last == 8'h3C && perr2_cnt == 0,
              "DUT2 RX even parity ok");
        uart2_drive_frame(8'h81, 1'b1, 1'b1);  // correct = 0, drive 1 -> error
        #(BIT_PERIOD*2);
        check(perr2_cnt == 1 && vld2_cnt == 1 && ferr2_cnt == 0,
              "DUT2 PARITY_ERROR on bad parity, VLD suppressed");

        tx2_send(8'h81);
        wait (tx2_cnt == 1);  #(BIT_PERIOD*2);
        check(tx2_bytes[0] == 8'h81 && tx2_par[0] == 1'b0 && tx2_stop_ok,
              "DUT2 TX data + even parity bit");

        // ---- DUT3 (odd parity, debouncer OFF) --------------------------------
        uart3_drive_frame(8'hA5, 1'b1, 1'b1);  // odd parity of 0xA5 (4 ones) = 1
        #(BIT_PERIOD*2);
        check(vld3_cnt == 1 && rx3_last == 8'hA5 && perr3_cnt == 0,
              "DUT3 RX odd parity, no debouncer");

        // same 2-cycle glitch as DUT1: with the debouncer bypassed it IS taken
        // as a start bit and the idle-high line decodes as a phantom 0xFF
        // (odd parity of 0xFF = 1 = idle level, so it even passes parity)
        rxd3 = 1'b0;  #(2*CLK_PERIOD);  rxd3 = 1'b1;
        #(BIT_PERIOD*14);
        check(vld3_cnt == 2 && rx3_last == 8'hFF,
              "DUT3 glitch NOT filtered (debouncer off)");

        // ---- summary ---------------------------------------------------------
        $display("------------------------------------------");
        $display("Result: %0d passed, %0d failed", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else               $display("TESTS FAILED");
        $finish;
    end

endmodule

`default_nettype wire
