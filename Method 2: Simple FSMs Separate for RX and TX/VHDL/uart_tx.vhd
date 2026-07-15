library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;


entity uart_tx is

    generic (
        CLK_FREQ  : integer := 50000000;
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



architecture Behavioral of uart_tx is


    constant BAUD_DIV : integer := CLK_FREQ / BAUD_RATE;


    type state_type is (IDLE, START, DATA, STOP);

    signal state : state_type := IDLE;


    signal baud_count : integer range 0 to BAUD_DIV-1 := 0;

    signal bit_count : integer range 0 to 7 := 0;


    signal tx_reg : std_logic := '1';

    signal busy_reg : std_logic := '0';


    signal data_reg : std_logic_vector(7 downto 0) := (others => '0');



begin


    tx_pin  <= tx_reg;

    tx_busy <= busy_reg;



    process(clk)

    begin

        if rising_edge(clk) then


            if rst = '1' then


                state <= IDLE;

                baud_count <= 0;

                bit_count <= 0;

                tx_reg <= '1';

                busy_reg <= '0';

                data_reg <= (others => '0');



            else


                case state is



                    ------------------------------------------------
                    -- IDLE: UART line HIGH
                    ------------------------------------------------

                    when IDLE =>


                        tx_reg <= '1';

                        busy_reg <= '0';

                        baud_count <= 0;


                        if uart_start = '1' then


                            data_reg <= uart_data;

                            busy_reg <= '1';


                            state <= START;


                        end if;



                    ------------------------------------------------
                    -- START BIT
                    ------------------------------------------------

                    when START =>


                        tx_reg <= '0';


                        if baud_count = BAUD_DIV-1 then


                            baud_count <= 0;

                            bit_count <= 0;

                            state <= DATA;


                        else

                            baud_count <= baud_count + 1;

                        end if;



                    ------------------------------------------------
                    -- DATA BITS (LSB FIRST)
                    ------------------------------------------------

                    when DATA =>


                        tx_reg <= data_reg(bit_count);


                        if baud_count = BAUD_DIV-1 then


                            baud_count <= 0;


                            if bit_count = 7 then

                                state <= STOP;


                            else

                                bit_count <= bit_count + 1;

                            end if;


                        else

                            baud_count <= baud_count + 1;

                        end if;



                    ------------------------------------------------
                    -- STOP BIT
                    ------------------------------------------------

                    when STOP =>


                        tx_reg <= '1';


                        if baud_count = BAUD_DIV-1 then


                            baud_count <= 0;


                            state <= IDLE;

                            busy_reg <= '0';


                        else

                            baud_count <= baud_count + 1;

                        end if;



                end case;


            end if;

        end if;


    end process;


end Behavioral;
