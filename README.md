# RP2040-HAT-FREERTOS-W5500-COREMARK-C  
This project utilizes the W5500-EVB-Pico board to compare the performance when transmitting data received through SPI communication in Dual Board mode to an OpenSSL server via W5500. Additionally, it compares the performance when transmitting the same amount of data to an OpenSSL server via W5500 in Single Board mode.  
  
![w5500-evb-pico_360](https://github.com/aimee0000/RP2040-HAT-FREERTOS-W5500-COREMARK-C/assets/150221423/e7e0b93c-0a1e-400a-be69-32c175e9e60f)
  
#### Dual Board coremark Test  
W5500-EVB-Pico(SPI Master) ---> W5500-EVB-Pico(SPI Slave&SSL client) ---> PC(OpenSSL server)  
* SPI Master Project : https://github.com/aimee0000/RP2040-HAT-FREERTOS-W5500-COREMARK-C/tree/main/examples/CoreMark_spi_master  
* SPI Slave&SSL cleint Project : https://github.com/aimee0000/RP2040-HAT-FREERTOS-W5500-COREMARK-C/tree/main/examples/spi_slave_w5500_over_ssl  
#### Single Board coremark Test  
W5500-EVB-Pico(SSL client) ---> PC(OpenSSL server)  
* Project : https://github.com/aimee0000/RP2040-HAT-FREERTOS-W5500-COREMARK-C/tree/main/examples/CoreMark_w5500_over_ssl  
  
  
## Hardware  
* Device : W5500-EVB-Pico  
* CPU clock : 133MHz  
* SPI : use DMA/ baudrate 11.0833Mbps  
  
## Software  
* Visual Studio Code : https://code.visualstudio.com/  
* FreeRTOS Example for RP2040 : https://github.com/Wiznet/RP2040-HAT-FREERTOS-C
