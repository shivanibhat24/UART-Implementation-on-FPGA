--------------------------------------------------------------------------------
-- UART_TB — self-checking testbench for the Simple UART (jakubcabal/uart-for-fpga)
--------------------------------------------------------------------------------
-- WHAT IT TESTS
--   T1  TX->RX loopback of a single byte (0x55) — proves serializer,
--       deserializer, baud timing and the DIN_VLD/DIN_RDY handshake.
--   T2  Four back-to-back loopback bytes (0xA5, 0x00, 0xFF, 0x3C) — proves
--       the transmitter back-pressures correctly between frames.
--   T3  Byte (0x96) driven directly onto UART_RXD by the testbench at nominal
--       115200 baud — proves the receiver against an independent bit clock,
--       not just against its own transmitter.
--   T4  Frame with the stop bit forced low — proves FRAME_ERROR fires and
--       DOUT_VLD does NOT (per uart_rx.vhd, a bad frame never asserts VLD).
--   End-of-sim invariants: exactly one FRAME_ERROR total, zero PARITY_ERROR.
--
-- STYLE / PORTABILITY
--   Pure VHDL-93 constructs: runs unmodified in Vivado xsim (default VHDL
--   mode) and in GHDL. Every wait has a timeout, so a broken DUT produces
--   FAIL messages instead of a hung simulation. The clock stops when the
--   tests finish, so the simulator exits by itself — no forced $finish-style
--   failure assertion needed.
--   Expected: six PASS lines, then "ALL TESTS PASSED".
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;   -- for unsigned conversion in the report messages

entity UART_TB is
    -- A testbench has no ports: it is the top of the simulated universe.
end entity UART_TB;

architecture SIM of UART_TB is

    -- ==== TEST PARAMETERS ====================================================
    constant CLK_FREQ   : integer := 50e6;    -- must match the generic given to the DUT
    constant BAUD_RATE  : integer := 115200;
    constant CLK_PERIOD : time    := 20 ns;   -- 1 / 50 MHz
    -- One bit period at 115200 baud = 8680.55 ns. The 0.55 ns truncation is
    -- 0.006% — irrelevant next to the DUT's own +0.47% divider rounding.
    constant BIT_PERIOD : time    := 8680 ns;

    -- ==== DUT CONNECTIONS ====================================================
    signal clk          : std_logic := '0';
    signal rst          : std_logic := '1';                     -- start in reset
    signal uart_txd     : std_logic;
    signal uart_rxd     : std_logic;
    signal din          : std_logic_vector(7 downto 0) := (others => '0');
    signal din_vld      : std_logic := '0';
    signal din_rdy      : std_logic;
    signal dout         : std_logic_vector(7 downto 0);
    signal dout_vld     : std_logic;
    signal frame_error  : std_logic;
    signal parity_error : std_logic;

    -- ==== TESTBENCH PLUMBING =================================================
    -- loopback = '1'  -> UART_RXD is fed from the DUT's own UART_TXD (T1/T2)
    -- loopback = '0'  -> UART_RXD is driven by rxd_drive, i.e. the testbench
    --                    plays the role of the far-end transmitter (T3/T4)
    signal loopback   : std_logic := '1';
    signal rxd_drive  : std_logic := '1';   -- idle-high, like a real line
    signal sim_done   : boolean   := false; -- stops the clock => ends the sim

    -- ==== MONITOR STATE ======================================================
    -- DOUT_VLD / FRAME_ERROR are single-clock pulses. The stimulus process
    -- spends whole bit-periods inside procedures, so it WOULD miss them; a
    -- concurrent monitor therefore latches every event, and the stimulus
    -- checks the latched state. This is the standard cure for the
    -- "one-cycle strobe vs. blocking stimulus" race in testbenches.
    signal rx_count   : natural := 0;                     -- good frames seen
    signal ferr_count : natural := 0;                     -- frame errors seen
    signal perr_count : natural := 0;                     -- parity errors seen
    signal last_byte  : std_logic_vector(7 downto 0) := (others => '0');

begin

    -- ==== RXD SOURCE MUX =====================================================
    uart_rxd <= uart_txd when loopback = '1' else rxd_drive;

    -- ==== CLOCK GENERATOR ====================================================
    -- Runs until sim_done; when it stops, no events remain and the simulator
    -- exits cleanly on its own.
    clk_gen_p : process
    begin
        while not sim_done loop
            clk <= '0'; wait for CLK_PERIOD / 2;
            clk <= '1'; wait for CLK_PERIOD / 2;
        end loop;
        wait;
    end process;

    -- ==== DEVICE UNDER TEST ==================================================
    dut : entity work.UART
    generic map (
        CLK_FREQ      => CLK_FREQ,
        BAUD_RATE     => BAUD_RATE,
        PARITY_BIT    => "none",
        USE_DEBOUNCER => True
    )
    port map (
        CLK          => clk,
        RST          => rst,
        UART_TXD     => uart_txd,
        UART_RXD     => uart_rxd,
        DIN          => din,
        DIN_VLD      => din_vld,
        DIN_RDY      => din_rdy,
        DOUT         => dout,
        DOUT_VLD     => dout_vld,
        FRAME_ERROR  => frame_error,
        PARITY_ERROR => parity_error
    );

    -- ==== MONITOR ============================================================
    -- Latches every one-cycle event from the DUT so the stimulus process can
    -- inspect them at its leisure (see MONITOR STATE comment above).
    monitor_p : process (clk)
    begin
        if rising_edge(clk) then
            if dout_vld = '1' then
                last_byte <= dout;              -- capture the byte with its strobe
                rx_count  <= rx_count + 1;
            end if;
            if frame_error = '1' then
                ferr_count <= ferr_count + 1;
            end if;
            if parity_error = '1' then
                perr_count <= perr_count + 1;
            end if;
        end if;
    end process;

    -- ==== STIMULUS AND CHECKING ==============================================
    stim_p : process
        variable errors : natural := 0;   -- overall FAIL counter

        -- Small helper: readable byte value for report strings (VHDL-93 has
        -- no to_hstring, so decimal via integer'image is the portable choice).
        function img (v : std_logic_vector(7 downto 0)) return string is
        begin
            return integer'image(to_integer(unsigned(v)));
        end function;

        -- ---- host_send ------------------------------------------------------
        -- Drives one byte into the parallel TX interface with a proper
        -- valid/ready handshake: wait for DIN_RDY, then assert DIN_VLD for
        -- exactly one clock. Mirrors what a bus wrapper or CPU would do.
        procedure host_send (b : in std_logic_vector(7 downto 0)) is
        begin
            if din_rdy /= '1' then                       -- guard: only wait if not already ready
                wait until din_rdy = '1' for 15 * BIT_PERIOD;
                if din_rdy /= '1' then
                    report "FAIL: DIN_RDY never asserted - transmitter stuck" severity error;
                    errors := errors + 1;
                    return;
                end if;
            end if;
            wait until rising_edge(clk);                 -- align to the clock like real logic
            din     <= b;
            din_vld <= '1';
            wait until rising_edge(clk);                 -- byte is accepted on this edge (VLD and RDY both high)
            din_vld <= '0';
        end procedure;

        -- ---- expect_rx ------------------------------------------------------
        -- Waits (with timeout) until the monitor has counted one more good
        -- frame than 'prev', then compares the captured byte. The 'if' guard
        -- matters: "wait until" blocks for an EVENT even if the condition is
        -- already true, so a frame that landed before we got here must not
        -- make us wait forever.
        procedure expect_rx (prev     : in natural;
                             expected : in std_logic_vector(7 downto 0);
                             tag      : in string) is
        begin
            if rx_count /= prev + 1 then
                wait until rx_count = prev + 1 for 15 * BIT_PERIOD;
            end if;
            if rx_count /= prev + 1 then
                report "FAIL [" & tag & "]: timeout - no DOUT_VLD" severity error;
                errors := errors + 1;
            elsif last_byte /= expected then
                report "FAIL [" & tag & "]: got " & img(last_byte)
                     & " expected " & img(expected) severity error;
                errors := errors + 1;
            else
                report "PASS [" & tag & "]: byte " & img(expected) & " received correctly";
            end if;
        end procedure;

        -- ---- line_send ------------------------------------------------------
        -- The testbench acting as the far-end transmitter: bit-bangs one
        -- 8N1 frame onto rxd_drive with its own independent timing.
        -- good_stop = false forces the stop bit low to provoke FRAME_ERROR.
        procedure line_send (b : in std_logic_vector(7 downto 0);
                             good_stop : in boolean) is
        begin
            rxd_drive <= '0';                 -- start bit
            wait for BIT_PERIOD;
            for i in 0 to 7 loop              -- data bits, LSB first (UART convention)
                rxd_drive <= b(i);
                wait for BIT_PERIOD;
            end loop;
            if good_stop then                 -- stop bit: '1' is legal, '0' is a framing error
                rxd_drive <= '1';
            else
                rxd_drive <= '0';
            end if;
            wait for BIT_PERIOD;
            rxd_drive <= '1';                 -- return to idle
            wait for 3 * BIT_PERIOD;          -- recovery gap so the receiver resettles
        end procedure;

        variable prev_rx : natural;           -- rx_count snapshots taken BEFORE each frame
        variable prev_fe : natural;
    begin
        report "=== UART_TB start: CLK 50 MHz, 115200 8N1, debouncer on ===";

        -- ---- Reset and settling ---------------------------------------------
        rst <= '1';
        wait for 10 * CLK_PERIOD;             -- 200 ns of reset (>= 2 cycles required)
        rst <= '0';
        -- Let the CDC flops and 4-cycle debouncer flush their power-up values
        -- before stimulating; generous 20 us also covers the divider phase.
        wait for 20 us;

        -- ---- T1: loopback, single byte --------------------------------------
        loopback <= '1';
        prev_rx := rx_count;
        host_send(x"55");                     -- 0x55 = alternating bit pattern, worst case for timing drift
        expect_rx(prev_rx, x"55", "T1 loopback 0x55");

        -- ---- T2: loopback, four back-to-back bytes --------------------------
        prev_rx := rx_count; host_send(x"A5"); expect_rx(prev_rx, x"A5", "T2a loopback 0xA5");
        prev_rx := rx_count; host_send(x"00"); expect_rx(prev_rx, x"00", "T2b loopback 0x00");
        prev_rx := rx_count; host_send(x"FF"); expect_rx(prev_rx, x"FF", "T2c loopback 0xFF");
        prev_rx := rx_count; host_send(x"3C"); expect_rx(prev_rx, x"3C", "T2d loopback 0x3C");

        -- ---- T3: direct line drive, good frame ------------------------------
        loopback  <= '0';                     -- testbench now owns the line
        rxd_drive <= '1';
        wait for 3 * BIT_PERIOD;              -- guaranteed idle before the start bit
        prev_rx := rx_count;
        line_send(x"96", true);               -- note: DOUT_VLD fires DURING this call; the monitor catches it
        expect_rx(prev_rx, x"96", "T3 direct RX 0x96");

        -- ---- T4: direct line drive, broken stop bit -------------------------
        prev_rx := rx_count;
        prev_fe := ferr_count;
        line_send(x"77", false);              -- stop bit low => framing error
        wait for 2 * BIT_PERIOD;              -- allow the strobe to be counted
        if ferr_count /= prev_fe + 1 then
            report "FAIL [T4]: FRAME_ERROR did not assert on bad stop bit" severity error;
            errors := errors + 1;
        elsif rx_count /= prev_rx then
            report "FAIL [T4]: DOUT_VLD asserted for a broken frame" severity error;
            errors := errors + 1;
        else
            report "PASS [T4]: bad stop bit -> FRAME_ERROR, no DOUT_VLD";
        end if;

        -- ---- Final invariants ------------------------------------------------
        if ferr_count /= 1 then
            report "FAIL [inv]: unexpected FRAME_ERROR count = "
                 & integer'image(ferr_count) severity error;
            errors := errors + 1;
        end if;
        if perr_count /= 0 then
            report "FAIL [inv]: PARITY_ERROR asserted with parity disabled" severity error;
            errors := errors + 1;
        end if;

        -- ---- Verdict ---------------------------------------------------------
        if errors = 0 then
            report "=== ALL TESTS PASSED (6 checks + 2 invariants) ===";
        else
            report "=== " & integer'image(errors) & " TEST(S) FAILED ===" severity error;
        end if;

        sim_done <= true;                     -- stop the clock; simulator exits
        wait;
    end process;

end architecture SIM;
