library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- UART Transmitter Module
entity uart_tx is
    generic (
        CLK_FREQ  : integer := 50000000;  -- Clock frequency in Hz
        BAUD_RATE : integer := 115200
    );

    port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        uart_start : in  std_logic;
        uart_data  : in  std_logic_vector(7 downto 0);

        tx_pin     : out std_logic;
        tx_busy    : out std_logic
    );

end uart_tx;


architecture rtl of uart_tx is

    -- Number of clock cycles per UART bit
    constant BAUD_DIV : integer := CLK_FREQ / BAUD_RATE;

    -- Baud rate counter
    signal baud_cnt : integer range 0 to BAUD_DIV-1 := 0;

    -- Bit counter for 10-bit UART frame
    signal bit_cnt : integer range 0 to 9 := 0;

    -- UART shift register:
    -- bit 0 = start bit
    -- bits 1-8 = data
    -- bit 9 = stop bit
    signal shift_reg : std_logic_vector(9 downto 0) := (others => '1');

    -- Internal busy flag
    signal busy : std_logic := '0';

begin

    -- Output assignments
    tx_busy <= busy;
    tx_pin  <= shift_reg(0);


    -- UART transmitter process
    process(clk)
    begin

        if rising_edge(clk) then

            -- Reset transmitter
            if rst = '1' then

                busy      <= '0';
                shift_reg <= (others => '1');
                baud_cnt  <= 0;
                bit_cnt   <= 0;


            else

                -- Start a new transmission
                if busy = '0' then

                    if uart_start = '1' then

                        busy <= '1';

                        -- UART frame:
                        -- Start bit (0)
                        -- 8 data bits
                        -- Stop bit (1)
                        shift_reg <= '1' & uart_data & '0';

                        baud_cnt <= 0;
                        bit_cnt  <= 0;

                    end if;


                else

                    -- Count clock cycles for each bit period
                    if baud_cnt = BAUD_DIV-1 then

                        baud_cnt <= 0;

                        -- Shift next bit onto TX pin
                        shift_reg <= '1' & shift_reg(9 downto 1);


                        -- Check if all bits have been transmitted
                        if bit_cnt = 9 then

                            busy    <= '0';
                            bit_cnt <= 0;

                        else

                            bit_cnt <= bit_cnt + 1;

                        end if;


                    else

                        baud_cnt <= baud_cnt + 1;

                    end if;

                end if;

            end if;

        end if;

    end process;

end rtl;
