//////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2013, Andrew "bunnie" Huang
//
// See the NOTICE file distributed with this work for additional 
// information regarding copyright ownership.  The copyright holder 
// licenses this file to you under the Apache License, Version 2.0 
// (the "License"); you may not use this file except in compliance
// with the License.  You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing,
// code distributed under the License is distributed on an
// "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
// KIND, either express or implied.  See the License for the
// specific language governing permissions and limitations
// under the License.
//////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module adc_rx(
	      input wire [7:0] data_i_p,
	      input wire [7:0] data_i_n,

	      input wire [7:0] data_q_p,
	      input wire [7:0] data_q_n,
	      
	      input wire clk_p,
	      input wire clk_n,

	      output reg [63:0] adc_i,
	      output reg [63:0] adc_q,
	      output wire oclk,
	      output wire oclkx2,
	      
	      input wire reset
	      );

   wire 	      adc_gclk;
   wire 	      adc_bitslip;
   wire 	      adc_pll_locked;
   wire 	      adc_bufpll_locked;
   wire 	      adc_clk;

   wire [63:0] 	      adc_i_fast;
   wire [63:0] 	      adc_q_fast;

   always @(posedge oclk) begin
      // i-channel:   0100_X000  // swaps due to P/N pair swapping for routability
      // q-channel:   1010_1000
      adc_i <= adc_i_fast ^ 64'h4040_4040_4040_4040; 
      adc_q <= adc_q_fast ^ 64'hA8A8_A8A8_A8A8_A8A8;
   end

   serdes_1_to_n_clk_pll_s8_diff input_adc_clk (
						.clkin_p(clk_p), 
						.clkin_n(clk_n), 
						.rxioclk(adc_clk), 
						.pattern1(2'b10), 
						.pattern2(2'b01), 
						.rx_serdesstrobe(adc_serdesstrobe), 
						.reset(reset), 
						.rx_bufg_pll_x1(adc_gclk), 
						.rx_pll_lckd(adc_pll_locked),
						.rx_pllout_div8(oclk),
						.rx_pllout_div4(oclkx2),
//						.rx_pllout_xs(), 
						.bitslip(adc_bitslip), 
						.rx_bufpll_lckd(adc_bufpll_locked) 
//						.datain()
						) ;
   
   serdes_1_to_n_data_s8_diff input_adc_i (
					     .use_phase_detector(1'b1), 
					     .datain_p(data_i_p[7:0]), 
					     .datain_n(data_i_n[7:0]), 
					     .rxioclk(adc_clk), 
					     .rxserdesstrobe(adc_serdesstrobe), 
					     .reset(reset), 
					     .gclk(oclk), 
					     .bitslip(adc_bitslip),
//					     .debug_in(), 
//					     .debug(),
					     .data_out(adc_i_fast) 
					     ) ;

   serdes_1_to_n_data_s8_diff input_adc_q (
					     .use_phase_detector(1'b1), 
					     .datain_p(data_q_p[7:0]), 
					     .datain_n(data_q_n[7:0]), 
					     .rxioclk(adc_clk), 
					     .rxserdesstrobe(adc_serdesstrobe), 
					     .reset(reset), 
					     .gclk(oclk), 
					     .bitslip(adc_bitslip),  // not used for now
//					     .debug_in(), 
//					     .debug(),
					     .data_out(adc_q_fast) 
					     ) ;
   

endmodule // adc_rx
