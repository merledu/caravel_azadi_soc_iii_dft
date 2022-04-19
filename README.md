# caravel_azadi_soc_iii_dft
This project is the extended version of Azadi-SoC, which includes all of the peripherals which were in Azadi-II and few more this time, which were not stable at the time of Azadi-II. The Azadi-III includes the following peripherals. PWM 2-Channel, OpenRAM 1KB x 4 for ICCM 1KB x 4 for DCCM Ibex core(named as brq_core) FPU (single-precision) TileLink (UL) UART QSPI SPI GPIOs Design Goals: Azadi-III is aimed to extend the base ibex core(RV32IMC) with a fully functional single precision floating point unit and RISCV compliant debug module for on chip debugging and some standard peripherals for communicating with other devices. all these modules will be interlinked using standard Tilelink Bus protocol. The project aims at adding DFT support to Caravel chip to enable post fabrication testing using Automatic Testing Equipment (ATE). Scan chain is applied to making a design testable , observable and controllable after it has been manufactured.
