## ============================================================
## Basys3 SPI Slave - Pin Constraints (xc7a35tcpg236-1)
## TÜBİTAK 2209A - RF Protokol Sınıflandırma Projesi
## ============================================================

## ====================================================
## 100 MHz System Clock
## ====================================================
set_property PACKAGE_PIN W5      [get_ports CLK100MHZ]
set_property IOSTANDARD LVCMOS33 [get_ports CLK100MHZ]
create_clock -period 10.000 -name sys_clk [get_ports CLK100MHZ]

## ====================================================
## PMOD JA - SPI Data Lines
## ====================================================

## JA1 - MOSI (STM32 PA7)
set_property PACKAGE_PIN J1      [get_ports JA1]
set_property IOSTANDARD LVCMOS33 [get_ports JA1]
set_property PULLDOWN TRUE       [get_ports JA1]

## JA2 - MISO (STM32 PA6)
set_property PACKAGE_PIN L2      [get_ports JA2]
set_property IOSTANDARD LVCMOS33 [get_ports JA2]

## JA7 - SS (STM32 PA4)
set_property PACKAGE_PIN H1      [get_ports JA7]
set_property IOSTANDARD LVCMOS33 [get_ports JA7]
set_property PULLUP TRUE         [get_ports JA7]

## ====================================================
## PMOD JB - SPI Clock + INT
## ====================================================

## JB2 - SCLK (STM32 PA5) - MRCC pin (Bank 16, A16)
set_property PACKAGE_PIN A16     [get_ports JB2]
set_property IOSTANDARD LVCMOS33 [get_ports JB2]
set_property PULLDOWN TRUE       [get_ports JB2]

## JB3 - INT → STM32 (SRCC pin, Bank 16, B15)
set_property PACKAGE_PIN B15     [get_ports JB3_INT]
set_property IOSTANDARD LVCMOS33 [get_ports JB3_INT]

## ====================================================
## Pmod JC Port - Pmod AD1 (ADC121S101) Bağlantısı
## ====================================================
## JC1 = ~CS  (Pmod AD1 pin 1)
## JC2 = D0   (Pmod AD1 pin 2 - Channel A MISO)
## JC4 = SCLK (Pmod AD1 pin 4)
## JC3, JC7-10 = kullanılmıyor
## ====================================================
set_property -dict { PACKAGE_PIN K17  IOSTANDARD LVCMOS33 } [get_ports JC1]
set_property -dict { PACKAGE_PIN M18  IOSTANDARD LVCMOS33 } [get_ports JC2]
set_property -dict { PACKAGE_PIN M19  IOSTANDARD LVCMOS33 } [get_ports JC4]

## ADC SPI clock (12.5 MHz, USE_ADC=1 iken aktif)
## Setup/hold için input delay set edilebilir, ancak frame uzun olduğu için
## kritik değil. Sentez optimize eder.

## ====================================================
## LED Outputs
## ====================================================
set_property PACKAGE_PIN U16     [get_ports {LED[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED[0]}]

set_property PACKAGE_PIN E19     [get_ports {LED[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED[1]}]

set_property PACKAGE_PIN U19     [get_ports {LED[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED[2]}]

set_property PACKAGE_PIN V19     [get_ports {LED[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED[3]}]

set_property PACKAGE_PIN W18     [get_ports {LED[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED[4]}]

set_property PACKAGE_PIN U15     [get_ports {LED[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED[5]}]

set_property PACKAGE_PIN U14     [get_ports {LED[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED[6]}]

set_property PACKAGE_PIN V14     [get_ports {LED[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED[7]}]

## ====================================================
## SPI Clock Timing
## JB2 (A16) MRCC pine atandi - BUFG'ye dogrudan baglanabilir
## ====================================================
create_clock -period 100.000 -name spi_sclk [get_ports JB2]

set_clock_groups -asynchronous \
    -group [get_clocks sys_clk] \
    -group [get_clocks spi_sclk]

## Input/Output Delays
set_input_delay  -clock spi_sclk -max 20.000 [get_ports JA1]
set_input_delay  -clock spi_sclk -min  0.000 [get_ports JA1]
set_input_delay  -clock spi_sclk -max 20.000 [get_ports JA7]
set_input_delay  -clock spi_sclk -min  0.000 [get_ports JA7]
set_output_delay -clock spi_sclk -max 20.000 [get_ports JA2]
set_output_delay -clock spi_sclk -min  0.000 [get_ports JA2]

## ====================================================
## SS (JA7) Asenkron Reset - False Path
## spi_slave modülünde bit_count ve byte_count registerlari
## JA7'yi asenkron CLR olarak kullaniyor.
## SS, SCLK ile zamanlanmamis bir kontrol sinyali oldugu icin
## hold/removal analizi uygulanamaz.
## ====================================================
set_false_path -from [get_ports JA7]
