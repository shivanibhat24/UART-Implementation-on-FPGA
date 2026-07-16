// =====================================================================
// tb_uart_axis.v -- self-checking testbench for the AXI4-Stream UART
//                   (alexforencich/verilog-uart: uart / uart_tx / uart_rx)
//
//   iverilog -g2005 -Wall -o sim_axis tb_uart_axis.v uart.v && vvp sim_axis
//
//   (adjust the RTL filename; the three modules may be in one or three files)
//
// ---------------------------------------------------------------------
// THINGS ABOUT THIS DUT THE TESTBENCH MUST RESPECT
// ---------------------------------------------------------------------
//
//  1. prescale = clk / (baud * 8), NOT clk / baud.
//     The RTL reconstructs a full bit with (prescale << 3) and finds the
//     start-bit centre with (prescale << 2)-2 (half a bit). Passing
//     clk/baud here makes everything run 8x slow and looks like a DUT bug.
//
//  2. The 8x is STRUCTURAL, not noise immunity. The RX still takes ONE
//     sample per bit. The x8 exists so a half-bit offset is expressible.
//     Do not write tests that assume majority voting.
//
//  3. frame_error and overrun_error are 1-CLOCK PULSES. Both are assigned
//     0 unconditionally at the top of the always block and only set on the
//     cycle they occur. A test that polls them after the fact reads 0 and
//     passes vacuously. They must be latched by the monitor below.
//
//  4. m_axis_tvalid has NO skid buffer. It clears only on a tvalid&&tready
//     handshake. Overrun is "a byte completed while tvalid was still high".
//
//  5. rxd is synchronised by exactly ONE flop (rxd_reg). That is a real
//     metastability exposure on a physical pin -- see the note at the end
//     of this file. This TB drives rxd synchronously to clk, so it CANNOT
//     detect that problem. Not a testbench oversight; stated so nobody
//     concludes "TB passes, therefore the CDC is fine".
//
//  6. rst is ACTIVE HIGH here (most of this repo's siblings use rst_n).
//
// =====================================================================

`timescale 1ns / 1ps

module tb_uart_axis;

    // -----------------------------------------------------------------
    // Timing
    //
    // Keep the numbers small so the sim is quick, but keep the RATIO
    // honest: prescale is clk/(baud*8), so one bit = prescale*8 clocks.
    // PRESCALE=4 -> 32 clocks per bit, 10 bits per frame = 320 clocks.
    //
    // Real target for reference (xc7s15, 100 MHz, 115200 baud):
    //   prescale = 100e6 / (115200*8) = 108.5 -> 108
    //   effective baud = 100e6/(108*8) = 115741, error +0.47%
    // -----------------------------------------------------------------
    localparam integer DATA_WIDTH = 8;
    localparam integer PRESCALE   = 4;
    localparam integer CLKS_PER_BIT = PRESCALE * 8;      // 32
    localparam integer BIT_NS       = CLKS_PER_BIT * 10; // 10ns clk -> 320 ns

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                                 // 100 MHz

    // -----------------------------------------------------------------
    // DUT connections
    // -----------------------------------------------------------------
    reg  [DATA_WIDTH-1:0] s_axis_tdata  = 0;
    reg                   s_axis_tvalid = 1'b0;
    wire                  s_axis_tready;

    wire [DATA_WIDTH-1:0] m_axis_tdata;
    wire                  m_axis_tvalid;
    reg                   m_axis_tready = 1'b0;

    wire                  txd;
    wire                  tx_busy;
    wire                  rx_busy;
    wire                  rx_overrun_error;
    wire                  rx_frame_error;

    reg  [15:0]           prescale = PRESCALE;

    // ---- rxd source select ------------------------------------------
    //   0 = loopback from the DUT's own txd
    //   1 = BFM line (independent bit rate, or deliberately malformed)
    reg  use_bfm  = 1'b0;
    reg  bfm_line = 1'b1;
    wire rxd = use_bfm ? bfm_line : txd;

    uart #(
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .clk              (clk),
        .rst              (rst),
        .s_axis_tdata     (s_axis_tdata),
        .s_axis_tvalid    (s_axis_tvalid),
        .s_axis_tready    (s_axis_tready),
        .m_axis_tdata     (m_axis_tdata),
        .m_axis_tvalid    (m_axis_tvalid),
        .m_axis_tready    (m_axis_tready),
        .rxd              (rxd),
        .txd              (txd),
        .tx_busy          (tx_busy),
        .rx_busy          (rx_busy),
        .rx_overrun_error (rx_overrun_error),
        .rx_frame_error   (rx_frame_error),
        .prescale         (prescale)
    );

    integer errors = 0;

    // -----------------------------------------------------------------
    // Monitor
    //
    // frame_error and overrun_error are single-cycle pulses. Latch them
    // here or they are gone before any test can look. clear_flags() resets
    // the latches between tests.
    // -----------------------------------------------------------------
    reg [DATA_WIDTH-1:0] rx_bytes [0:63];
    integer              n_rx        = 0;
    reg                  seen_ferr   = 1'b0;
    reg                  seen_overrun= 1'b0;

    always @(posedge clk) begin
        if (!rst) begin
            if (m_axis_tvalid && m_axis_tready) begin
                rx_bytes[n_rx] = m_axis_tdata;
                n_rx           = n_rx + 1;
            end
            if (rx_frame_error)   seen_ferr    = 1'b1;
            if (rx_overrun_error) seen_overrun = 1'b1;
        end
    end

    task clear_flags;
        begin
            n_rx         = 0;
            seen_ferr    = 1'b0;
            seen_overrun = 1'b0;
        end
    endtask

    // -----------------------------------------------------------------
    // AXI-Stream master: push one byte into the TX
    //
    // Proper handshake -- assert tvalid, hold until tready, then drop.
    // Do NOT wait for tready before asserting tvalid: that is a handshake
    // deadlock, and it is the single most common way to "prove" a working
    // AXI slave is broken.
    // -----------------------------------------------------------------
    task axis_send(input [DATA_WIDTH-1:0] b);
        begin
            @(posedge clk);
            s_axis_tdata  <= b;
            s_axis_tvalid <= 1'b1;
            @(posedge clk);
            while (!s_axis_tready) @(posedge clk);
            s_axis_tvalid <= 1'b0;
        end
    endtask

    // -----------------------------------------------------------------
    // BFM transmitter -- drives bfm_line at an arbitrary bit period,
    // independent of the DUT's clock. Loopback CANNOT test baud tolerance
    // because both ends share one crystal; a loopback-only suite will pass
    // a receiver that fails against every real UART.
    //
    // bit_x1000 is in picoseconds (milli-nanoseconds) so ppm offsets are
    // expressible without floating point in the caller.
    // -----------------------------------------------------------------
    task bfm_send(input [DATA_WIDTH-1:0] b, input integer bit_x1000);
        integer i;
        begin
            bfm_line = 1'b0;                       // start
            #(bit_x1000 / 1000.0);
            for (i = 0; i < DATA_WIDTH; i = i + 1) begin
                bfm_line = b[i];                   // LSB first
                #(bit_x1000 / 1000.0);
            end
            bfm_line = 1'b1;                       // stop
            #(bit_x1000 / 1000.0);
        end
    endtask

    // BFM frame with a DELIBERATELY BROKEN stop bit -> must set frame_error
    task bfm_send_badstop(input [DATA_WIDTH-1:0] b, input integer bit_x1000);
        integer i;
        begin
            bfm_line = 1'b0;
            #(bit_x1000 / 1000.0);
            for (i = 0; i < DATA_WIDTH; i = i + 1) begin
                bfm_line = b[i];
                #(bit_x1000 / 1000.0);
            end
            bfm_line = 1'b0;                       // stop bit held SPACE
            #(bit_x1000 / 1000.0);
            bfm_line = 1'b1;                       // release
            #(bit_x1000 / 1000.0);
        end
    endtask

    // -----------------------------------------------------------------
    // Checks
    // -----------------------------------------------------------------
    task check_byte(input [DATA_WIDTH-1:0] expect_b, input [255:0] label);
        begin
            if (n_rx != 1) begin
                $display("  FAIL: %0s -- expected 1 byte, got %0d", label, n_rx);
                errors = errors + 1;
            end else if (rx_bytes[0] !== expect_b) begin
                $display("  FAIL: %0s -- expected 0x%02h, got 0x%02h",
                         label, expect_b, rx_bytes[0]);
                errors = errors + 1;
            end else if (seen_ferr) begin
                $display("  FAIL: %0s -- frame_error on a good frame", label);
                errors = errors + 1;
            end else begin
                $display("  ok  : %0s  0x%02h", label, expect_b);
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Loopback: send a byte out txd, receive it back on rxd
    // -----------------------------------------------------------------
    task loopback_check(input [DATA_WIDTH-1:0] b);
        begin
            clear_flags;
            m_axis_tready = 1'b1;
            use_bfm       = 1'b0;
            axis_send(b);
            wait (tx_busy == 1'b1);
            wait (tx_busy == 1'b0);
            repeat (CLKS_PER_BIT * 3) @(posedge clk);
            check_byte(b, "loopback");
        end
    endtask

    // -----------------------------------------------------------------
    // BFM check at a given baud error
    // -----------------------------------------------------------------
    task bfm_check(input [DATA_WIDTH-1:0] b,
                   input integer          ppm_err,
                   input                  expect_ok);
        integer nominal_x1000;
        integer bit_x1000;
        begin
            nominal_x1000 = BIT_NS * 1000;
            // MULTIPLY BEFORE DIVIDE. (nominal/1000000)*ppm truncates to
            // zero for any nominal under 1e6 and silently turns the whole
            // sweep into N repeats of the 0% case -- passing, meaningless.
            bit_x1000 = nominal_x1000 + ((nominal_x1000 * ppm_err) / 1000000);

            use_bfm       = 1'b1;
            bfm_line      = 1'b1;
            m_axis_tready = 1'b1;
            // Idle and drain any byte still in flight from a previous call
            // BEFORE zeroing the counters, or each test scores the previous
            // test's byte.
            repeat (CLKS_PER_BIT * 3) @(posedge clk);
            clear_flags;

            bfm_send(b, bit_x1000);
            repeat (CLKS_PER_BIT * 3) @(posedge clk);
            use_bfm = 1'b0;

            if (expect_ok) begin
                if (n_rx == 1 && rx_bytes[0] === b && !seen_ferr) begin
                    $display("  ok  : %+0.2f%%  0x%02h clean",
                             ppm_err/10000.0, b);
                end else begin
                    $display("  FAIL: %+0.2f%%  expected 0x%02h clean; n_rx=%0d got=0x%02h ferr=%b",
                             ppm_err/10000.0, b, n_rx,
                             (n_rx > 0) ? rx_bytes[0] : 8'hxx, seen_ferr);
                    errors = errors + 1;
                end
            end else begin
                // Beyond spec: no pass/fail. We do not require a particular
                // failure mode, only visibility into what happens.
                if (n_rx == 1 && rx_bytes[0] === b && !seen_ferr)
                    $display("  note: %+0.2f%%  still worked (margin beats spec)",
                             ppm_err/10000.0);
                else
                    $display("  note: %+0.2f%%  broke (n_rx=%0d got=0x%02h ferr=%b)",
                             ppm_err/10000.0, n_rx,
                             (n_rx > 0) ? rx_bytes[0] : 8'hxx, seen_ferr);
            end
        end
    endtask

    // =================================================================
    // Test sequence
    // =================================================================
    initial begin
        $dumpfile("tb_uart_axis.vcd");
        $dumpvars(0, tb_uart_axis);

        m_axis_tready = 1'b0;
        repeat (8) @(posedge clk);
        rst = 1'b0;
        repeat (8) @(posedge clk);

        // -------------------------------------------------------------
        $display("--- 1. loopback, corner patterns ---");
        // 0x00: every data bit is a space -- looks like start bits
        // 0xFF: every data bit is a mark  -- looks like idle
        // 0x01/0x80: single bits at each end -- catches bit-order errors
        loopback_check(8'h00);
        loopback_check(8'hFF);
        loopback_check(8'hA5);
        loopback_check(8'h5A);
        loopback_check(8'h01);
        loopback_check(8'h80);
        loopback_check(8'h55);
        loopback_check(8'hAA);

        // -------------------------------------------------------------
        // Back-to-back with tvalid held high across frames. Exercises the
        // s_axis_tready_reg <= !s_axis_tready_reg toggle: tready is 1 in
        // that branch, so the toggle is a clear. If it were a genuine
        // toggle from an arbitrary state, this test would double-accept.
        // -------------------------------------------------------------
        $display("--- 2. AXI-Stream back-to-back (tready toggle-as-clear) ---");
        clear_flags;
        m_axis_tready = 1'b1;
        use_bfm       = 1'b0;
        begin : b2b
            integer i;
            reg [7:0] pat [0:3];
            pat[0] = 8'h11; pat[1] = 8'h22; pat[2] = 8'h33; pat[3] = 8'h44;
            for (i = 0; i < 4; i = i + 1) begin
                axis_send(pat[i]);
                wait (tx_busy == 1'b1);
                wait (tx_busy == 1'b0);
                repeat (CLKS_PER_BIT * 2) @(posedge clk);
            end
            repeat (CLKS_PER_BIT * 3) @(posedge clk);
            if (n_rx != 4) begin
                $display("  FAIL: expected 4 bytes, got %0d", n_rx);
                errors = errors + 1;
            end else begin
                for (i = 0; i < 4; i = i + 1) begin
                    if (rx_bytes[i] !== pat[i]) begin
                        $display("  FAIL: byte %0d expected 0x%02h got 0x%02h",
                                 i, pat[i], rx_bytes[i]);
                        errors = errors + 1;
                    end
                end
                if (errors == 0)
                    $display("  ok  : 4 bytes in order, no duplicates");
            end
        end

        // -------------------------------------------------------------
        // tready backpressure. Hold m_axis_tready low; tvalid must stay
        // asserted and the byte must survive until collected. This is the
        // AXI contract: once tvalid is high it cannot drop before a
        // handshake.
        // -------------------------------------------------------------
        $display("--- 3. m_axis_tready backpressure ---");
        clear_flags;
        m_axis_tready = 1'b0;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_send(8'h7E, BIT_NS * 1000);
        repeat (CLKS_PER_BIT * 2) @(posedge clk);

        if (!m_axis_tvalid) begin
            $display("  FAIL: tvalid low after a complete frame");
            errors = errors + 1;
        end else if (m_axis_tdata !== 8'h7E) begin
            $display("  FAIL: tdata=0x%02h, expected 0x7E", m_axis_tdata);
            errors = errors + 1;
        end else begin
            // Hold it a while: tvalid must not self-clear.
            repeat (CLKS_PER_BIT * 4) @(posedge clk);
            if (!m_axis_tvalid || m_axis_tdata !== 8'h7E) begin
                $display("  FAIL: tvalid/tdata did not hold under backpressure");
                errors = errors + 1;
            end else begin
                $display("  ok  : tvalid held, tdata stable = 0x7E");
            end
        end
        // Now collect it.
        @(posedge clk);
        m_axis_tready = 1'b1;
        @(posedge clk);
        @(posedge clk);
        m_axis_tready = 1'b0;
        @(posedge clk);
        if (m_axis_tvalid) begin
            $display("  FAIL: tvalid still high after handshake");
            errors = errors + 1;
        end else begin
            $display("  ok  : tvalid cleared on handshake");
        end
        use_bfm = 1'b0;

        // -------------------------------------------------------------
        // Overrun: two frames with tready held low. The second completes
        // while tvalid is still high -> overrun_error pulses for 1 clock.
        //
        // NOTE the DUT's semantics: overrun_error_reg <= m_axis_tvalid_reg,
        // i.e. it reports "the new byte landed on top of an uncollected
        // one". The NEW byte wins; the old one is lost. That is a design
        // choice, not a bug, but a consumer must know which byte survived.
        // -------------------------------------------------------------
        $display("--- 4. overrun (tready held low across two frames) ---");
        clear_flags;
        m_axis_tready = 1'b0;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_send(8'hC3, BIT_NS * 1000);
        bfm_send(8'h3C, BIT_NS * 1000);
        repeat (CLKS_PER_BIT * 3) @(posedge clk);
        if (!seen_overrun) begin
            $display("  FAIL: two uncollected frames did not raise overrun_error");
            errors = errors + 1;
        end else begin
            $display("  ok  : overrun_error pulsed");
        end
        if (m_axis_tdata !== 8'h3C) begin
            $display("  note: surviving byte = 0x%02h (expected the NEWER, 0x3C)",
                     m_axis_tdata);
        end else begin
            $display("  ok  : newer byte survived (0x3C), older one dropped");
        end
        // Drain
        @(posedge clk);
        m_axis_tready = 1'b1;
        @(posedge clk);
        @(posedge clk);
        use_bfm = 1'b0;

        // -------------------------------------------------------------
        // Frame error: stop bit driven SPACE instead of MARK.
        // Must pulse frame_error and must NOT emit a byte.
        // -------------------------------------------------------------
        $display("--- 5. frame error (stop bit = space) ---");
        clear_flags;
        m_axis_tready = 1'b1;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT * 2) @(posedge clk);
        bfm_send_badstop(8'h96, BIT_NS * 1000);
        repeat (CLKS_PER_BIT * 4) @(posedge clk);
        if (!seen_ferr) begin
            $display("  FAIL: bad stop bit did not raise frame_error");
            errors = errors + 1;
        end else if (n_rx != 0) begin
            $display("  FAIL: frame_error raised but %0d byte(s) still emitted", n_rx);
            errors = errors + 1;
        end else begin
            $display("  ok  : frame_error pulsed, no byte emitted");
        end
        use_bfm = 1'b0;
        repeat (CLKS_PER_BIT * 4) @(posedge clk);

        // -------------------------------------------------------------
        // Short glitch on an idle line. The RX re-checks at the start-bit
        // centre ((prescale<<2)-2 later); a glitch shorter than half a bit
        // must be rejected without emitting anything.
        // -------------------------------------------------------------
        $display("--- 6. short glitch on idle line ---");
        clear_flags;
        m_axis_tready = 1'b1;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_line = 1'b0;
        repeat (CLKS_PER_BIT / 4) @(posedge clk);
        bfm_line = 1'b1;
        repeat (CLKS_PER_BIT * 12) @(posedge clk);
        if (n_rx != 0) begin
            $display("  FAIL: glitch produced %0d phantom byte(s)", n_rx);
            errors = errors + 1;
        end else begin
            $display("  ok  : rejected");
        end
        use_bfm = 1'b0;

        // -------------------------------------------------------------
        // Long glitch: low for one full bit period.
        //
        // EXPECTED TO BE ACCEPTED as a start bit. This RX takes ONE sample
        // per bit, so a disturbance that is low at the sample point is
        // indistinguishable from a real start bit -- the signal is not
        // malformed at the only place the receiver may legitimately look.
        // Informational, not a pass/fail: it documents the design's noise
        // floor. Rejecting it needs majority voting (multiple samples per
        // bit) or an idle-time qualifier, neither of which this core has.
        // -------------------------------------------------------------
        $display("--- 7. long glitch (informational -- documents the noise floor) ---");
        clear_flags;
        m_axis_tready = 1'b1;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_line = 1'b0;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_line = 1'b1;
        repeat (CLKS_PER_BIT * 14) @(posedge clk);
        $display("  note: n_rx=%0d ferr=%b  (single-sample RX; expected)",
                 n_rx, seen_ferr);
        use_bfm = 1'b0;

        // -------------------------------------------------------------
        // Baud tolerance sweep, independent time base.
        // -------------------------------------------------------------
        $display("--- 8. baud tolerance sweep (independent BFM) ---");
        bfm_check(8'hA5,      0, 1);
        bfm_check(8'hA5,  10000, 1);   // +1.0%
        bfm_check(8'hA5, -10000, 1);   // -1.0%
        bfm_check(8'h5A,  20000, 1);   // +2.0%
        bfm_check(8'h5A, -20000, 1);   // -2.0%
        $display("  (beyond spec -- informational)");
        bfm_check(8'hFF,  40000, 0);   // +4.0%
        bfm_check(8'h00, -40000, 0);   // -4.0%

        // -------------------------------------------------------------
        // Runtime prescale change. prescale is a live input, not a
        // parameter -- confirm the core actually tracks it. This is the
        // main advantage of this core over a compile-time-divisor design.
        // -------------------------------------------------------------
        $display("--- 9. runtime prescale change ---");
        clear_flags;
        use_bfm       = 1'b0;
        m_axis_tready = 1'b1;
        prescale      = PRESCALE * 2;          // half the baud rate
        repeat (CLKS_PER_BIT * 4) @(posedge clk);
        axis_send(8'hD7);
        wait (tx_busy == 1'b1);
        wait (tx_busy == 1'b0);
        repeat (CLKS_PER_BIT * 8) @(posedge clk);
        check_byte(8'hD7, "loopback @ 2x prescale");
        prescale = PRESCALE;
        repeat (CLKS_PER_BIT * 4) @(posedge clk);

        // -------------------------------------------------------------
        // Reset mid-frame. Assert rst while a frame is on the wire; the
        // core must return to idle (txd high) and not emit a partial byte.
        // -------------------------------------------------------------
        $display("--- 10. reset mid-frame ---");
        clear_flags;
        use_bfm       = 1'b0;
        m_axis_tready = 1'b1;
        axis_send(8'h6B);
        wait (tx_busy == 1'b1);
        repeat (CLKS_PER_BIT * 3) @(posedge clk);   // mid-frame
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (CLKS_PER_BIT * 12) @(posedge clk);
        if (txd !== 1'b1) begin
            $display("  FAIL: txd=%b after reset -- must idle MARK", txd);
            errors = errors + 1;
        end else if (tx_busy !== 1'b0) begin
            $display("  FAIL: tx_busy still high after reset");
            errors = errors + 1;
        end else begin
            $display("  ok  : txd idles high, tx_busy clear");
        end

        // -------------------------------------------------------------
        $display("");
        if (errors == 0) $display("=== ALL TESTS PASSED ===");
        else             $display("=== %0d FAILURE(S) ===", errors);

        $display("");
        $display("NOTE (not a test failure): rxd is synchronised by ONE flop");
        $display("      (rxd_reg). On a physical pin that is a metastability");
        $display("      exposure. This TB drives rxd synchronously to clk, so");
        $display("      it cannot and does not test that. Add a second flop");
        $display("      with (* ASYNC_REG=\"TRUE\" *) before hardware bring-up.");
        $finish;
    end

    initial begin
        #20000000;
        $display("TIMEOUT -- check that prescale is clk/(baud*8), not clk/baud");
        $finish;
    end

endmodule// =====================================================================
// tb_uart_axis.v -- self-checking testbench for the AXI4-Stream UART
//                   (alexforencich/verilog-uart: uart / uart_tx / uart_rx)
//
//   iverilog -g2005 -Wall -o sim_axis tb_uart_axis.v uart.v && vvp sim_axis
//
//   (adjust the RTL filename; the three modules may be in one or three files)
//
// ---------------------------------------------------------------------
// THINGS ABOUT THIS DUT THE TESTBENCH MUST RESPECT
// ---------------------------------------------------------------------
//
//  1. prescale = clk / (baud * 8), NOT clk / baud.
//     The RTL reconstructs a full bit with (prescale << 3) and finds the
//     start-bit centre with (prescale << 2)-2 (half a bit). Passing
//     clk/baud here makes everything run 8x slow and looks like a DUT bug.
//
//  2. The 8x is STRUCTURAL, not noise immunity. The RX still takes ONE
//     sample per bit. The x8 exists so a half-bit offset is expressible.
//     Do not write tests that assume majority voting.
//
//  3. frame_error and overrun_error are 1-CLOCK PULSES. Both are assigned
//     0 unconditionally at the top of the always block and only set on the
//     cycle they occur. A test that polls them after the fact reads 0 and
//     passes vacuously. They must be latched by the monitor below.
//
//  4. m_axis_tvalid has NO skid buffer. It clears only on a tvalid&&tready
//     handshake. Overrun is "a byte completed while tvalid was still high".
//
//  5. rxd is synchronised by exactly ONE flop (rxd_reg). That is a real
//     metastability exposure on a physical pin -- see the note at the end
//     of this file. This TB drives rxd synchronously to clk, so it CANNOT
//     detect that problem. Not a testbench oversight; stated so nobody
//     concludes "TB passes, therefore the CDC is fine".
//
//  6. rst is ACTIVE HIGH here (most of this repo's siblings use rst_n).
//
// =====================================================================

`timescale 1ns / 1ps

module tb_uart_axis;

    // -----------------------------------------------------------------
    // Timing
    //
    // Keep the numbers small so the sim is quick, but keep the RATIO
    // honest: prescale is clk/(baud*8), so one bit = prescale*8 clocks.
    // PRESCALE=4 -> 32 clocks per bit, 10 bits per frame = 320 clocks.
    //
    // Real target for reference (xc7s15, 100 MHz, 115200 baud):
    //   prescale = 100e6 / (115200*8) = 108.5 -> 108
    //   effective baud = 100e6/(108*8) = 115741, error +0.47%
    // -----------------------------------------------------------------
    localparam integer DATA_WIDTH = 8;
    localparam integer PRESCALE   = 4;
    localparam integer CLKS_PER_BIT = PRESCALE * 8;      // 32
    localparam integer BIT_NS       = CLKS_PER_BIT * 10; // 10ns clk -> 320 ns

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                                 // 100 MHz

    // -----------------------------------------------------------------
    // DUT connections
    // -----------------------------------------------------------------
    reg  [DATA_WIDTH-1:0] s_axis_tdata  = 0;
    reg                   s_axis_tvalid = 1'b0;
    wire                  s_axis_tready;

    wire [DATA_WIDTH-1:0] m_axis_tdata;
    wire                  m_axis_tvalid;
    reg                   m_axis_tready = 1'b0;

    wire                  txd;
    wire                  tx_busy;
    wire                  rx_busy;
    wire                  rx_overrun_error;
    wire                  rx_frame_error;

    reg  [15:0]           prescale = PRESCALE;

    // ---- rxd source select ------------------------------------------
    //   0 = loopback from the DUT's own txd
    //   1 = BFM line (independent bit rate, or deliberately malformed)
    reg  use_bfm  = 1'b0;
    reg  bfm_line = 1'b1;
    wire rxd = use_bfm ? bfm_line : txd;

    uart #(
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .clk              (clk),
        .rst              (rst),
        .s_axis_tdata     (s_axis_tdata),
        .s_axis_tvalid    (s_axis_tvalid),
        .s_axis_tready    (s_axis_tready),
        .m_axis_tdata     (m_axis_tdata),
        .m_axis_tvalid    (m_axis_tvalid),
        .m_axis_tready    (m_axis_tready),
        .rxd              (rxd),
        .txd              (txd),
        .tx_busy          (tx_busy),
        .rx_busy          (rx_busy),
        .rx_overrun_error (rx_overrun_error),
        .rx_frame_error   (rx_frame_error),
        .prescale         (prescale)
    );

    integer errors = 0;

    // -----------------------------------------------------------------
    // Monitor
    //
    // frame_error and overrun_error are single-cycle pulses. Latch them
    // here or they are gone before any test can look. clear_flags() resets
    // the latches between tests.
    // -----------------------------------------------------------------
    reg [DATA_WIDTH-1:0] rx_bytes [0:63];
    integer              n_rx        = 0;
    reg                  seen_ferr   = 1'b0;
    reg                  seen_overrun= 1'b0;

    always @(posedge clk) begin
        if (!rst) begin
            if (m_axis_tvalid && m_axis_tready) begin
                rx_bytes[n_rx] = m_axis_tdata;
                n_rx           = n_rx + 1;
            end
            if (rx_frame_error)   seen_ferr    = 1'b1;
            if (rx_overrun_error) seen_overrun = 1'b1;
        end
    end

    task clear_flags;
        begin
            n_rx         = 0;
            seen_ferr    = 1'b0;
            seen_overrun = 1'b0;
        end
    endtask

    // -----------------------------------------------------------------
    // AXI-Stream master: push one byte into the TX
    //
    // Proper handshake -- assert tvalid, hold until tready, then drop.
    // Do NOT wait for tready before asserting tvalid: that is a handshake
    // deadlock, and it is the single most common way to "prove" a working
    // AXI slave is broken.
    // -----------------------------------------------------------------
    task axis_send(input [DATA_WIDTH-1:0] b);
        begin
            @(posedge clk);
            s_axis_tdata  <= b;
            s_axis_tvalid <= 1'b1;
            @(posedge clk);
            while (!s_axis_tready) @(posedge clk);
            s_axis_tvalid <= 1'b0;
        end
    endtask

    // -----------------------------------------------------------------
    // BFM transmitter -- drives bfm_line at an arbitrary bit period,
    // independent of the DUT's clock. Loopback CANNOT test baud tolerance
    // because both ends share one crystal; a loopback-only suite will pass
    // a receiver that fails against every real UART.
    //
    // bit_x1000 is in picoseconds (milli-nanoseconds) so ppm offsets are
    // expressible without floating point in the caller.
    // -----------------------------------------------------------------
    task bfm_send(input [DATA_WIDTH-1:0] b, input integer bit_x1000);
        integer i;
        begin
            bfm_line = 1'b0;                       // start
            #(bit_x1000 / 1000.0);
            for (i = 0; i < DATA_WIDTH; i = i + 1) begin
                bfm_line = b[i];                   // LSB first
                #(bit_x1000 / 1000.0);
            end
            bfm_line = 1'b1;                       // stop
            #(bit_x1000 / 1000.0);
        end
    endtask

    // BFM frame with a DELIBERATELY BROKEN stop bit -> must set frame_error
    task bfm_send_badstop(input [DATA_WIDTH-1:0] b, input integer bit_x1000);
        integer i;
        begin
            bfm_line = 1'b0;
            #(bit_x1000 / 1000.0);
            for (i = 0; i < DATA_WIDTH; i = i + 1) begin
                bfm_line = b[i];
                #(bit_x1000 / 1000.0);
            end
            bfm_line = 1'b0;                       // stop bit held SPACE
            #(bit_x1000 / 1000.0);
            bfm_line = 1'b1;                       // release
            #(bit_x1000 / 1000.0);
        end
    endtask

    // -----------------------------------------------------------------
    // Checks
    // -----------------------------------------------------------------
    task check_byte(input [DATA_WIDTH-1:0] expect_b, input [255:0] label);
        begin
            if (n_rx != 1) begin
                $display("  FAIL: %0s -- expected 1 byte, got %0d", label, n_rx);
                errors = errors + 1;
            end else if (rx_bytes[0] !== expect_b) begin
                $display("  FAIL: %0s -- expected 0x%02h, got 0x%02h",
                         label, expect_b, rx_bytes[0]);
                errors = errors + 1;
            end else if (seen_ferr) begin
                $display("  FAIL: %0s -- frame_error on a good frame", label);
                errors = errors + 1;
            end else begin
                $display("  ok  : %0s  0x%02h", label, expect_b);
            end
        end
    endtask

    // -----------------------------------------------------------------
    // Loopback: send a byte out txd, receive it back on rxd
    // -----------------------------------------------------------------
    task loopback_check(input [DATA_WIDTH-1:0] b);
        begin
            clear_flags;
            m_axis_tready = 1'b1;
            use_bfm       = 1'b0;
            axis_send(b);
            wait (tx_busy == 1'b1);
            wait (tx_busy == 1'b0);
            repeat (CLKS_PER_BIT * 3) @(posedge clk);
            check_byte(b, "loopback");
        end
    endtask

    // -----------------------------------------------------------------
    // BFM check at a given baud error
    // -----------------------------------------------------------------
    task bfm_check(input [DATA_WIDTH-1:0] b,
                   input integer          ppm_err,
                   input                  expect_ok);
        integer nominal_x1000;
        integer bit_x1000;
        begin
            nominal_x1000 = BIT_NS * 1000;
            // MULTIPLY BEFORE DIVIDE. (nominal/1000000)*ppm truncates to
            // zero for any nominal under 1e6 and silently turns the whole
            // sweep into N repeats of the 0% case -- passing, meaningless.
            bit_x1000 = nominal_x1000 + ((nominal_x1000 * ppm_err) / 1000000);

            use_bfm       = 1'b1;
            bfm_line      = 1'b1;
            m_axis_tready = 1'b1;
            // Idle and drain any byte still in flight from a previous call
            // BEFORE zeroing the counters, or each test scores the previous
            // test's byte.
            repeat (CLKS_PER_BIT * 3) @(posedge clk);
            clear_flags;

            bfm_send(b, bit_x1000);
            repeat (CLKS_PER_BIT * 3) @(posedge clk);
            use_bfm = 1'b0;

            if (expect_ok) begin
                if (n_rx == 1 && rx_bytes[0] === b && !seen_ferr) begin
                    $display("  ok  : %+0.2f%%  0x%02h clean",
                             ppm_err/10000.0, b);
                end else begin
                    $display("  FAIL: %+0.2f%%  expected 0x%02h clean; n_rx=%0d got=0x%02h ferr=%b",
                             ppm_err/10000.0, b, n_rx,
                             (n_rx > 0) ? rx_bytes[0] : 8'hxx, seen_ferr);
                    errors = errors + 1;
                end
            end else begin
                // Beyond spec: no pass/fail. We do not require a particular
                // failure mode, only visibility into what happens.
                if (n_rx == 1 && rx_bytes[0] === b && !seen_ferr)
                    $display("  note: %+0.2f%%  still worked (margin beats spec)",
                             ppm_err/10000.0);
                else
                    $display("  note: %+0.2f%%  broke (n_rx=%0d got=0x%02h ferr=%b)",
                             ppm_err/10000.0, n_rx,
                             (n_rx > 0) ? rx_bytes[0] : 8'hxx, seen_ferr);
            end
        end
    endtask

    // =================================================================
    // Test sequence
    // =================================================================
    initial begin
        $dumpfile("tb_uart_axis.vcd");
        $dumpvars(0, tb_uart_axis);

        m_axis_tready = 1'b0;
        repeat (8) @(posedge clk);
        rst = 1'b0;
        repeat (8) @(posedge clk);

        // -------------------------------------------------------------
        $display("--- 1. loopback, corner patterns ---");
        // 0x00: every data bit is a space -- looks like start bits
        // 0xFF: every data bit is a mark  -- looks like idle
        // 0x01/0x80: single bits at each end -- catches bit-order errors
        loopback_check(8'h00);
        loopback_check(8'hFF);
        loopback_check(8'hA5);
        loopback_check(8'h5A);
        loopback_check(8'h01);
        loopback_check(8'h80);
        loopback_check(8'h55);
        loopback_check(8'hAA);

        // -------------------------------------------------------------
        // Back-to-back with tvalid held high across frames. Exercises the
        // s_axis_tready_reg <= !s_axis_tready_reg toggle: tready is 1 in
        // that branch, so the toggle is a clear. If it were a genuine
        // toggle from an arbitrary state, this test would double-accept.
        // -------------------------------------------------------------
        $display("--- 2. AXI-Stream back-to-back (tready toggle-as-clear) ---");
        clear_flags;
        m_axis_tready = 1'b1;
        use_bfm       = 1'b0;
        begin : b2b
            integer i;
            reg [7:0] pat [0:3];
            pat[0] = 8'h11; pat[1] = 8'h22; pat[2] = 8'h33; pat[3] = 8'h44;
            for (i = 0; i < 4; i = i + 1) begin
                axis_send(pat[i]);
                wait (tx_busy == 1'b1);
                wait (tx_busy == 1'b0);
                repeat (CLKS_PER_BIT * 2) @(posedge clk);
            end
            repeat (CLKS_PER_BIT * 3) @(posedge clk);
            if (n_rx != 4) begin
                $display("  FAIL: expected 4 bytes, got %0d", n_rx);
                errors = errors + 1;
            end else begin
                for (i = 0; i < 4; i = i + 1) begin
                    if (rx_bytes[i] !== pat[i]) begin
                        $display("  FAIL: byte %0d expected 0x%02h got 0x%02h",
                                 i, pat[i], rx_bytes[i]);
                        errors = errors + 1;
                    end
                end
                if (errors == 0)
                    $display("  ok  : 4 bytes in order, no duplicates");
            end
        end

        // -------------------------------------------------------------
        // tready backpressure. Hold m_axis_tready low; tvalid must stay
        // asserted and the byte must survive until collected. This is the
        // AXI contract: once tvalid is high it cannot drop before a
        // handshake.
        // -------------------------------------------------------------
        $display("--- 3. m_axis_tready backpressure ---");
        clear_flags;
        m_axis_tready = 1'b0;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_send(8'h7E, BIT_NS * 1000);
        repeat (CLKS_PER_BIT * 2) @(posedge clk);

        if (!m_axis_tvalid) begin
            $display("  FAIL: tvalid low after a complete frame");
            errors = errors + 1;
        end else if (m_axis_tdata !== 8'h7E) begin
            $display("  FAIL: tdata=0x%02h, expected 0x7E", m_axis_tdata);
            errors = errors + 1;
        end else begin
            // Hold it a while: tvalid must not self-clear.
            repeat (CLKS_PER_BIT * 4) @(posedge clk);
            if (!m_axis_tvalid || m_axis_tdata !== 8'h7E) begin
                $display("  FAIL: tvalid/tdata did not hold under backpressure");
                errors = errors + 1;
            end else begin
                $display("  ok  : tvalid held, tdata stable = 0x7E");
            end
        end
        // Now collect it.
        @(posedge clk);
        m_axis_tready = 1'b1;
        @(posedge clk);
        @(posedge clk);
        m_axis_tready = 1'b0;
        @(posedge clk);
        if (m_axis_tvalid) begin
            $display("  FAIL: tvalid still high after handshake");
            errors = errors + 1;
        end else begin
            $display("  ok  : tvalid cleared on handshake");
        end
        use_bfm = 1'b0;

        // -------------------------------------------------------------
        // Overrun: two frames with tready held low. The second completes
        // while tvalid is still high -> overrun_error pulses for 1 clock.
        //
        // NOTE the DUT's semantics: overrun_error_reg <= m_axis_tvalid_reg,
        // i.e. it reports "the new byte landed on top of an uncollected
        // one". The NEW byte wins; the old one is lost. That is a design
        // choice, not a bug, but a consumer must know which byte survived.
        // -------------------------------------------------------------
        $display("--- 4. overrun (tready held low across two frames) ---");
        clear_flags;
        m_axis_tready = 1'b0;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_send(8'hC3, BIT_NS * 1000);
        bfm_send(8'h3C, BIT_NS * 1000);
        repeat (CLKS_PER_BIT * 3) @(posedge clk);
        if (!seen_overrun) begin
            $display("  FAIL: two uncollected frames did not raise overrun_error");
            errors = errors + 1;
        end else begin
            $display("  ok  : overrun_error pulsed");
        end
        if (m_axis_tdata !== 8'h3C) begin
            $display("  note: surviving byte = 0x%02h (expected the NEWER, 0x3C)",
                     m_axis_tdata);
        end else begin
            $display("  ok  : newer byte survived (0x3C), older one dropped");
        end
        // Drain
        @(posedge clk);
        m_axis_tready = 1'b1;
        @(posedge clk);
        @(posedge clk);
        use_bfm = 1'b0;

        // -------------------------------------------------------------
        // Frame error: stop bit driven SPACE instead of MARK.
        // Must pulse frame_error and must NOT emit a byte.
        // -------------------------------------------------------------
        $display("--- 5. frame error (stop bit = space) ---");
        clear_flags;
        m_axis_tready = 1'b1;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT * 2) @(posedge clk);
        bfm_send_badstop(8'h96, BIT_NS * 1000);
        repeat (CLKS_PER_BIT * 4) @(posedge clk);
        if (!seen_ferr) begin
            $display("  FAIL: bad stop bit did not raise frame_error");
            errors = errors + 1;
        end else if (n_rx != 0) begin
            $display("  FAIL: frame_error raised but %0d byte(s) still emitted", n_rx);
            errors = errors + 1;
        end else begin
            $display("  ok  : frame_error pulsed, no byte emitted");
        end
        use_bfm = 1'b0;
        repeat (CLKS_PER_BIT * 4) @(posedge clk);

        // -------------------------------------------------------------
        // Short glitch on an idle line. The RX re-checks at the start-bit
        // centre ((prescale<<2)-2 later); a glitch shorter than half a bit
        // must be rejected without emitting anything.
        // -------------------------------------------------------------
        $display("--- 6. short glitch on idle line ---");
        clear_flags;
        m_axis_tready = 1'b1;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_line = 1'b0;
        repeat (CLKS_PER_BIT / 4) @(posedge clk);
        bfm_line = 1'b1;
        repeat (CLKS_PER_BIT * 12) @(posedge clk);
        if (n_rx != 0) begin
            $display("  FAIL: glitch produced %0d phantom byte(s)", n_rx);
            errors = errors + 1;
        end else begin
            $display("  ok  : rejected");
        end
        use_bfm = 1'b0;

        // -------------------------------------------------------------
        // Long glitch: low for one full bit period.
        //
        // EXPECTED TO BE ACCEPTED as a start bit. This RX takes ONE sample
        // per bit, so a disturbance that is low at the sample point is
        // indistinguishable from a real start bit -- the signal is not
        // malformed at the only place the receiver may legitimately look.
        // Informational, not a pass/fail: it documents the design's noise
        // floor. Rejecting it needs majority voting (multiple samples per
        // bit) or an idle-time qualifier, neither of which this core has.
        // -------------------------------------------------------------
        $display("--- 7. long glitch (informational -- documents the noise floor) ---");
        clear_flags;
        m_axis_tready = 1'b1;
        use_bfm       = 1'b1;
        bfm_line      = 1'b1;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_line = 1'b0;
        repeat (CLKS_PER_BIT) @(posedge clk);
        bfm_line = 1'b1;
        repeat (CLKS_PER_BIT * 14) @(posedge clk);
        $display("  note: n_rx=%0d ferr=%b  (single-sample RX; expected)",
                 n_rx, seen_ferr);
        use_bfm = 1'b0;

        // -------------------------------------------------------------
        // Baud tolerance sweep, independent time base.
        // -------------------------------------------------------------
        $display("--- 8. baud tolerance sweep (independent BFM) ---");
        bfm_check(8'hA5,      0, 1);
        bfm_check(8'hA5,  10000, 1);   // +1.0%
        bfm_check(8'hA5, -10000, 1);   // -1.0%
        bfm_check(8'h5A,  20000, 1);   // +2.0%
        bfm_check(8'h5A, -20000, 1);   // -2.0%
        $display("  (beyond spec -- informational)");
        bfm_check(8'hFF,  40000, 0);   // +4.0%
        bfm_check(8'h00, -40000, 0);   // -4.0%

        // -------------------------------------------------------------
        // Runtime prescale change. prescale is a live input, not a
        // parameter -- confirm the core actually tracks it. This is the
        // main advantage of this core over a compile-time-divisor design.
        // -------------------------------------------------------------
        $display("--- 9. runtime prescale change ---");
        clear_flags;
        use_bfm       = 1'b0;
        m_axis_tready = 1'b1;
        prescale      = PRESCALE * 2;          // half the baud rate
        repeat (CLKS_PER_BIT * 4) @(posedge clk);
        axis_send(8'hD7);
        wait (tx_busy == 1'b1);
        wait (tx_busy == 1'b0);
        repeat (CLKS_PER_BIT * 8) @(posedge clk);
        check_byte(8'hD7, "loopback @ 2x prescale");
        prescale = PRESCALE;
        repeat (CLKS_PER_BIT * 4) @(posedge clk);

        // -------------------------------------------------------------
        // Reset mid-frame. Assert rst while a frame is on the wire; the
        // core must return to idle (txd high) and not emit a partial byte.
        // -------------------------------------------------------------
        $display("--- 10. reset mid-frame ---");
        clear_flags;
        use_bfm       = 1'b0;
        m_axis_tready = 1'b1;
        axis_send(8'h6B);
        wait (tx_busy == 1'b1);
        repeat (CLKS_PER_BIT * 3) @(posedge clk);   // mid-frame
        rst = 1'b1;
        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (CLKS_PER_BIT * 12) @(posedge clk);
        if (txd !== 1'b1) begin
            $display("  FAIL: txd=%b after reset -- must idle MARK", txd);
            errors = errors + 1;
        end else if (tx_busy !== 1'b0) begin
            $display("  FAIL: tx_busy still high after reset");
            errors = errors + 1;
        end else begin
            $display("  ok  : txd idles high, tx_busy clear");
        end

        // -------------------------------------------------------------
        $display("");
        if (errors == 0) $display("=== ALL TESTS PASSED ===");
        else             $display("=== %0d FAILURE(S) ===", errors);

        $display("");
        $display("NOTE (not a test failure): rxd is synchronised by ONE flop");
        $display("      (rxd_reg). On a physical pin that is a metastability");
        $display("      exposure. This TB drives rxd synchronously to clk, so");
        $display("      it cannot and does not test that. Add a second flop");
        $display("      with (* ASYNC_REG=\"TRUE\" *) before hardware bring-up.");
        $finish;
    end

    initial begin
        #20000000;
        $display("TIMEOUT -- check that prescale is clk/(baud*8), not clk/baud");
        $finish;
    end

endmodule
