# SPI communication
## Mode
- CLK low when idle
- MOSI and MISO low when idle
- SS high when idle 
- Data should be read on rising edge of the clock
- Data should be sent on falling edge of the clock
## SPI on tiva
- PD0 - SSI1Clk
- PD1 - SSI1Fss
- PD2 - SSI1Rx
- PD3 - SSI1Tx
## SPI on fpga
- a0 - SSI1Clk
- a1 - SSI1Fss
- a2 - SSI1Rx
- a3 - SSI1Tx
