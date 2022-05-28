// SPDX-FileCopyrightText: 2020 Efabless Corporation
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//      http://www.apache.org/licenses/LICENSE-2.0 
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// 
// SPDX-License-Identifier: Apache-2.0
   
// Designed by a Team at Micro Electronics Research Lab, Usman Institute of Technology.
// https://www.merledupk.org
`default_nettype wire
`define MPRJ_IO_PADS 38
module azadi_soc_top_caravel (
  `ifdef USE_POWER_PINS
      inout vdda1,	// User area 1 3.3V supply
      inout vdda2,	// User area 2 3.3V supply
      inout vssa1,	// User area 1 analog ground
      inout vssa2,	// User area 2 analog ground
      inout vccd1,	// User area 1 1.8V supply
      inout vccd2,	// User area 2 1.8v supply
      inout vssd1,	// User area 1 digital ground
      inout vssd2,	// User area 2 digital ground
  `endif

    // Wishbone Slave ports (WB MI A)
    input         wb_clk_i,
    input         wb_rst_i,
    input         wbs_stb_i,
    input         wbs_cyc_i,
    input         wbs_we_i,
    input [3:0]   wbs_sel_i,
    input [31:0]  wbs_dat_i,
    input [31:0]  wbs_adr_i,
    output        wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs, MPRJ_IO_PADS = 38
    input  [`MPRJ_IO_PADS-1:0] io_in,  
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs do not have analog_io).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);

  wire clk_i;  
  wire rst_ni; 
  wire prog;
  wire led;
  
  // Clocks per bit
  wire [15:0] clks_per_bit;  

  // gpios interface
  wire [13:0] gpio_i;
  wire [13:0] gpio_o;
  wire [13:0] gpio_oe;


  // uart-periph interface
  wire uart_tx;
  wire uart_rx;

  // PWM interface  
  wire pwm_o_1;
  wire pwm_o_2;
  wire pwm1_oe;
  wire pwm2_oe;

  // SPI interface
  wire [1:0] ss_o;        
  wire       sclk_o;      
  wire       sd_o;
  wire       sd_oe;       
  wire       sd_i;

  wire       qsclk_o;
  wire       qcs_o;
  wire [3:0] qsd_i;
  wire [3:0] qsd_o;
  wire [3:0] qsd_oe;

// SPI pin mux

  assign io_out[0] = sclk_o;
  assign io_oeb[0] = 1'b0;
  assign io_out[1] = sd_o;
  assign io_oeb[1] = ~sd_oe;

  assign sd_i      = io_in[2];
  assign io_oeb[2] = 1'b1;
  assign io_out[2] = 1'b0;

  assign io_out[3] = ss_o[0];
  assign io_oeb[3] = 1'b0;
  assign io_out[4] = ss_o[1];
  assign io_oeb[4] = 1'b0;

// UART pin mux

  assign uart_rx   = io_in[5];
  assign io_oeb[5] = 1'b1;
  assign io_out[5] = 1'b0;

  assign io_out[6] = uart_tx;
  assign io_oeb[6] = 1'b0;

// PROG button 

  assign prog      = io_in[7];
  assign io_oeb[7] = 1'b1;
  assign io_out[7] = 1'b0;

// PWM pin mux

  assign io_out[8] = pwm_o_1;
  assign io_oeb[8] = ~pwm1_oe;
  assign io_out[9] = pwm_o_2;
  assign io_oeb[9] = ~pwm2_oe;

// scan pins 

  assign io_oeb[13:10] = 4'b1111;
  assign io_out[13:10] = 4'b0;

  assign io_oeb[15:14] = 2'b0;

// GPIO pin mux 

  assign io_out[29:16] = gpio_o;
  assign gpio_i        = io_in[29:16];
  assign io_oeb[29:16] = ~gpio_oe;

// BOOT LED pin

  assign io_out[30] = led;
  assign io_oeb[30] = 1'b0;
// QSPI pin mux

  assign io_out[31] = qcs_o;
  assign io_oeb[31] = 1'b0;

  assign io_out[32] = qsclk_o;
  assign io_oeb[32] = 1'b0;

  assign io_out[36:33] = qsd_o;
  assign qsd_i         = io_in[36:33];
  assign io_oeb[36:33] = ~qsd_oe;

  // Logic Analyzer ports
  assign clks_per_bit  = la_data_in[15:0];

  azadi_soc_top soc_top(
  `ifdef USE_POWER_PINS
    .vccd1(vccd1),
    .vssd1(vssd1),
  `endif
    .clk_i    (wb_clk_i),
    .rst_ni   (~wb_rst_i),
    .prog     (prog),
    .boot_led (led),

    // Clocks per bits
    .clks_per_bit(clks_per_bit), 

    // gpios interface
    .gpio_i  (gpio_i),
    .gpio_o  (gpio_o),
    .gpio_oe (gpio_oe),

    // uart-periph interface
    .uart_tx (uart_tx), // output
    .uart_rx (uart_rx), // input

    // PWM interface  
    .pwm_o   (pwm_o_1),
    .pwm_o_2 (pwm_o_2),
    .pwm1_oe (pwm1_oe),
    .pwm2_oe (pwm2_oe),

    // SPI interface
    .ss_o    (ss_o),        // [3:0] 
    .sclk_o  (sclk_o),      
    .sd_o    (sd_o),
    .sd_oe   (sd_oe),       
    .sd_i    (sd_i),

// qspi interface

    .qsclk_o (qsclk_o),
    .qcs_o   (qcs_o),
    .qsd_i   (qsd_i),
    .qsd_o   (qsd_o),
    .qsd_oe  (qsd_oe),
    .SE	     (io_in[10]), 
    .TM      (io_in[11]), 
    .SI0     (io_in[12]), 
    .SO0     (io_out[14]), 
    .SI1     (io_in[13]), 
    .SO1     (io_out[15])
  );

endmodule
