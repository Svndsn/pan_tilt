----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 05/16/2024 12:14:33 PM
-- Design Name: 
-- Module Name: spi_master_vhdl - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;
use IEEE.MATH_REAL.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity spi_master_vhdl is
    Generic (
            WORD_SIZE   : natural := 8;    -- size of transfer word in bits, must be power of two
            SLAVE_COUNT : natural := 1     -- count of SPI slaves
        );
        
    Port (
            CLK      : in  std_logic; -- system clock
            RST      : in  std_logic; -- high active synchronous reset
            -- SPI MASTER INTERFACE
            SCLK     : out std_logic; -- SPI clock
            SS     : out std_logic; -- SPI chip select, active in low
            MOSI     : out std_logic; -- SPI serial data from master to slave
            MISO     : in  std_logic; -- SPI serial data from slave to master
            -- INPUT USER INTERFACE
            DIN      : in  std_logic_vector(WORD_SIZE-1 downto 0); -- data for transmission to SPI slave
            DIN_VLD  : in  std_logic; -- when DIN_VLD = 1, data for transmission are valid
            START_TRANSMIT : in std_logic; -- when START_TRANSMIT is pulsed master will send and recieve
            -- OUTPUT USER INTERFACE
            DOUT     : out std_logic_vector(WORD_SIZE-1 downto 0); -- received data from SPI slave
            DOUT_VLD : out std_logic  -- when DOUT_VLD = 1, received data are valid
            );
end spi_master_vhdl;

architecture Behavioral of spi_master_vhdl is

    constant BIT_CNT_WIDTH : natural := natural(ceil(log2(real(WORD_SIZE))));

    signal bit_cnt            : unsigned(BIT_CNT_WIDTH-1 downto 0);
    signal bit_cnt_max        : std_logic;
    signal last_bit_en        : std_logic;
    signal SS_reg             : std_logic;
    signal load_data_en       : std_logic;
    signal rx_data_vld        : std_logic;
    signal clk_edge          : std_logic; -- 1 when rising, 0 when falling
    signal data_shreg_send    : std_logic_vector(WORD_SIZE-1 downto 0);
    signal data_shreg_recieve : std_logic_vector(WORD_SIZE-1 downto 0);

begin
    
    -- -------------------------------------------------------------------------
    --  CLK Synchronization
    -- -------------------------------------------------------------------------
    
    CLK_Synchronization : process (CLK)
    begin
        SS <= SS_reg;
    end process;
    
    -- -------------------------------------------------------------------------
    --  SS driver
    -- -------------------------------------------------------------------------
    
    slave_select : process (CLK)
    begin
        if(START_TRANSMIT = '1') then
            SS_reg <= '0';
        end if;
        if(SS_reg = '0' and bit_cnt_max = '1') then
            SS_reg <= '1';
        end if;
    end process;
    
    -- -------------------------------------------------------------------------
    --  SPI CLOCK REGISTER
    -- -------------------------------------------------------------------------

    -- The SPI clock register is necessary for clock edge detection.
    spi_clk_reg_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
                clk_edge <= '1';
            elsif (falling_edge(CLK)) then
                clk_edge <= '0';
        end if;
    end process;
    
    -- -------------------------------------------------------------------------
    --  RECEIVED BITS COUNTER
    -- -------------------------------------------------------------------------

    -- The counter counts received bits from the master. Counter is enabled when
    -- falling edge of SPI clock is detected and not asserted SS_reg.
    bit_cnt_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                bit_cnt <= (others => '0');
            elsif (falling_edge(CLK) and SS_reg = '0') then
                if (bit_cnt_max = '1') then
                    bit_cnt <= (others => '0');
                else
                    bit_cnt <= bit_cnt + 1;
                end if;
            end if;
        end if;
    end process;

    -- The flag of maximal value of the bit counter.
    bit_cnt_max <= '1' when (bit_cnt = WORD_SIZE-1) else '0';

    -- -------------------------------------------------------------------------
    --  LAST BIT FLAG REGISTER
    -- -------------------------------------------------------------------------

    -- The flag of last bit of received byte is only registered the flag of
    -- maximal value of the bit counter.
    last_bit_en_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (RST = '1') then
                last_bit_en <= '0';
            else
                last_bit_en <= bit_cnt_max;
            end if;
        end if;
    end process;
    
    -- -------------------------------------------------------------------------
    --  RECEIVED DATA VALID FLAG
    -- -------------------------------------------------------------------------

    -- Received data from slave are valid when falling edge of clock is
    -- detected and the last bit of received byte is detected.
    rx_data_vld <= not clk_edge and last_bit_en;
    
    -- The new input data is loaded into the shift register when the SPI
    -- is ready and input data are valid.
    load_data_en <= SS_reg and DIN_VLD;

    -- -------------------------------------------------------------------------
    --  DATA SHIFT REGISTER
    -- -------------------------------------------------------------------------

    -- The send shift register is used for sending data to the slave.
    -- The data are left shifted when rising edge of SPI clock is detected 
    -- and SS_reg is not assert.
    data_shreg_send_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (load_data_en = '1') then
                data_shreg_send <= DIN;
            elsif (SS_reg = '0') then
                data_shreg_send <= data_shreg_send(WORD_SIZE-2 downto 0) & '0';
            end if;
        end if;
    end process;
    -- The receive shift register is used for receiving data from the slave.
    -- The data are left shifted when rising edge of SPI clock is detected and SS_reg is not assert.
    data_shreg_recieve_p : process (CLK)
    begin
        if (rising_edge(CLK)) then
            if (SS_reg = '0') then
                data_shreg_recieve <= data_shreg_recieve(WORD_SIZE-2 downto 0) & MISO;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    --  MOSI REGISTER
    -- -------------------------------------------------------------------------

    -- The output MOSI register ensures that the bits are transmit to the slave
    -- when is not assert SS_reg and falling edge of SPI clock is detected.
    miso_p : process (CLK)
    begin
        if (falling_edge(CLK)) then
            if (load_data_en = '1') then
                MOSI <= DIN(WORD_SIZE-1);
            elsif (falling_edge(CLK) and SS_reg = '0') then
                MOSI <= data_shreg_send(WORD_SIZE-1);
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    --  ASSIGNING OUTPUT SIGNALS
    -- -------------------------------------------------------------------------
    
    DOUT     <= data_shreg_recieve;
    DOUT_VLD <= rx_data_vld;
    
end Behavioral;
