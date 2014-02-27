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

module adc_rx_tb;

   reg clk_p;
   wire clk_n;
   
   reg [7:0] data_i_p;
   wire [7:0] data_i_n;

   reg [7:0]  data_q_p;
   wire [7:0] data_q_n;

   wire [63:0] adc_i;
   wire [63:0] adc_q;
   wire        oclk;

   reg 	       reset;
   
   adc_rx adc_rx(.data_i_p(data_i_p[7:0]),
		 .data_i_n(data_i_n[7:0]),
		 .data_q_p(data_q_p[7:0]),
		 .data_q_n(data_q_n[7:0]),

		 .clk_p(clk_p),
		 .clk_n(clk_n),

		 .adc_i(adc_i[63:0]),
		 .adc_q(adc_q[63:0]),
		 .oclk(oclk),

		 .reset(reset)
		 );
   
   parameter PERIOD = 16'd2;   // 500 MHz
   always begin
      clk_p = 1'b0;
      #(PERIOD/2) clk_p = 1'b1;
      #(PERIOD/2);
   end
   
   assign clk_n = !clk_p;
   assign data_i_n = ~data_i_p;
   assign data_q_n = ~data_q_p;

   initial begin
      reset = 1'b1;
      data_i_p[7:0] = 8'b0;
      data_q_p[7:0] = 8'b0;

      $stop;
      #(PERIOD*128);
      reset = 1'b0;
      #(PERIOD*128);

      // do stuff
      repeat(1000) begin
	 data_i_p[7:0] = ({$random} % 256);
	 data_q_p[7:0] = ({$random} % 256);
	 #(PERIOD/2);
      end

      $stop;
   end // initial begin
   
endmodule // adc_rx_tb
