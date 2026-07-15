library ieee;

-- Library module for implementing the standard logic
use ieee.std_logic_1164.all;

-- Library module for implementation of arithmetic logic
use ieee.numeric_std.all;


-- Defining the testbench entity
-- Testbench does not have input/output ports because it only stimulates the DUT
entity uart_tx_tb is
end uart_tx_tb;


architecture sim of uart_tx_tb is


    -- Defining the clock frequency for the testbench
    -- Using a small clock frequency so simulation is faster
    constant CLK_FREQ : integer := 50000000;

    -- Defining the baud rate used by the UART transmitter
    constant BAUD_RATE : integer := 115200;


    -- Creating the clock signal for driving the UART module
    signal clk : std_logic := '0';


    -- Reset signal to return the transmitter to the initial state
    signal rst : std_logic := '0';


    -- Signal used to request a UART transmission
    signal uart_start : std_logic := '0';


    -- Data that will be transmitted through UART
    signal uart_data : std_logic_vector(7 downto 0) := (others => '0');


    -- Serial output signal from the UART transmitter
    signal tx_pin : std_logic;


    -- Signal indicating whether transmission is active
    signal tx_busy : std_logic;



    -- Clock period calculation
    -- 50 MHz clock means 20 ns period
    constant CLK_PERIOD : time := 20 ns;


begin


    -- Creating the clock generation process
    -- This process continuously toggles the clock signal
    clk_process : process
    begin

        clk <= '0';
        wait for CLK_PERIOD/2;

        clk <= '1';
        wait for CLK_PERIOD/2;

    end process;



    -- Instantiating the UART transmitter module
    -- This is the actual hardware block we are testing
    uart_tx_inst : entity work.uart_tx

        generic map (

            CLK_FREQ  => CLK_FREQ,
            BAUD_RATE => BAUD_RATE

        )

        port map (

            clk        => clk,
            rst        => rst,

            uart_start => uart_start,
            uart_data  => uart_data,

            tx_pin     => tx_pin,
            tx_busy    => tx_busy

        );



    -- Main test process
    -- This process applies different input conditions and checks the output behaviour
    stimulus_process : process
    begin


        -- Applying reset to place the UART transmitter into the idle state
        rst <= '1';

        wait for 100 ns;


        -- Removing reset and allowing normal operation
        rst <= '0';

        wait for 100 ns;



        -- Sending the first byte of data
        -- ASCII 'A' = 01000001
        uart_data <= "01000011";


        -- Giving the start pulse to begin transmission
        uart_start <= '1';

        wait for CLK_PERIOD;


        -- Returning start signal back to zero
        -- UART should continue transmitting because data is already loaded
        uart_start <= '0';



        -- Waiting until the complete UART frame is transmitted
        -- Frame size = 10 bits
        -- Each bit takes approximately 1/115200 seconds
        wait until tx_busy = '0';



        -- Adding some delay before ending simulation
        wait for 1 us;



        -- End simulation
        assert false
        report "UART transmission test completed"
        severity failure;


    end process;


end sim;
