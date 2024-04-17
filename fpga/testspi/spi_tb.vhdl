library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.finish;

entity spi_tb is
end spi_tb;

architecture behave of spi_tb is
  constant         WORD_SIZE : natural := 8; -- size of transfer word in bits, must be power of two
  signal i_CLK      :  std_logic := '0'; -- system clock
  signal i_RST      :  std_logic := '0'; -- high active synchronous reset
  signal i_SCLK     :  std_logic := '0'; -- SPI clock
  signal i_CS_N     :  std_logic := '1'; -- SPI chip select, active in low
  signal i_MOSI     :  std_logic; -- SPI serial data from master to slave
  signal o_MISO     :  std_logic; -- SPI serial data from slave to master

  signal i_DIN      : std_logic_vector(WORD_SIZE-1 downto 0); -- data for transmission to SPI master
  signal i_DIN_VLD  : std_logic; -- when DIN_VLD = 1, data for transmission are valid
  signal o_DIN_RDY  : std_logic; -- when DIN_RDY = 1, SPI slave is ready to accept valid data for transmission
  signal o_DOUT     : std_logic_vector(WORD_SIZE-1 downto 0); -- received data from SPI master
  signal o_DOUT_VLD : std_logic;  -- when DOUT_VLD = 1, received data are valid

begin
spi : entity work.SPI_SLAVE
    Generic map(
        WORD_SIZE => WORD_SIZE -- size of transfer word in bits, must be power of two
    )
    Port map(
        CLK => i_CLK,
        RST => i_RST,
        -- SPI SLAVE INTERFACE
        SCLK => i_SCLK,
        CS_N => i_CS_N,
        MOSI => i_MOSI,
        MISO => o_MISO,
        -- USER INTERFACE
        DIN => i_DIN,
        DIN_VLD => i_DIN_VLD,
        DIN_RDY => o_DIN_RDY,
        DOUT => o_DOUT,
        DOUT_VLD => o_DOUT_VLD
    );
    i_CLK <= not i_CLK after 1 ns;
    i_SCLK <= not i_SCLK after 5 ns;

  process is
  begin
    wait for 3 ns;
    i_CS_N <= '0';
    i_MOSI <= '1';
    wait for 10 ns;
    i_MOSI <= '0';
    wait for 10 ns;
    i_MOSI <= '1';
    wait for 10 ns;
    i_MOSI <= '0';
    wait for 10 ns;
    i_MOSI <= '1';
    wait for 10 ns;
    i_MOSI <= '0';
    wait for 10 ns;
    i_MOSI <= '1';
    wait for 10 ns;
    i_MOSI <= '0';
    wait for 10 ns;
    i_CS_N <= '1';
    finish;
  end process;
end behave;
