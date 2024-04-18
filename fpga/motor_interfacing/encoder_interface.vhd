----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 03/22/2024 11:47:17 AM
-- Design Name: 
-- Module Name: encoder_interface - Behavioral
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

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

-- Entity declaration for encoder interface
entity encoder_interface is
  port (
    rst : in std_logic;
    -- Encoder signals
    a : in  std_logic;
    b : in  std_logic;
    -- Count output
    cnt  : out std_logic_vector(9 downto 0)  -- Encoder resolution 1024
  );
end entity encoder_interface;

architecture rtl of encoder_interface is

  -- Internal signals
  signal temp_cnt : unsigned(9 downto 0) := "1000000000"; --Start at 512

begin

  -- FSM for direction and count logic
  process(a, rst)
  begin
    if rst = '1' then
        temp_cnt <= "1000000000"; -- Reset to 512
    elsif(rising_edge(a)) then
        if(b='1') then
            temp_cnt <= temp_cnt -1;
        else
            temp_cnt <= temp_cnt +1;
        end if;
    end if; 
  cnt <= std_logic_vector(temp_cnt);
  end process;

end architecture rtl;

