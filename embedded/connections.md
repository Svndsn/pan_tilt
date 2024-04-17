## H-bridge (outputs)
- PE1 -> ENA (Tilt)
- PE2 -> IN1A (Tilt)
- PE3 -> IN2A (Tilt)
- PA5 -> ENB (Pan)
- PA6 -> IN1B (Pan)
- PA7 -> IN2B (Pan)
## Protection board (inputs)
- PA2 <- Index0 (Pan) (interrupt)
- PA3 <- Index1 (Tilt) (interrupt)
- PC4 <- Sensor1A (Pan) (interrupt)
- PC5 <- Sensor1B (Pan)
- PC6 <- Sensor2A (Tilt) (interrupt)
- PC7 <- Sensor2B (Tilt)
## SPI 
- PD0 -> SSI1Clk
- PD1 -> SSI1Fss
- PD2 <- SSI1Rx
- PD3 -> SSI1Tx
