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

module adc08d1020_serial_tb;
   reg clk;
   reg [15:0] data;

   reg [3:0]  addr;
   reg 	      commit;
   wire       busy;
   
   parameter PERIOD = 16'd80;   // 12.5 MHz
   always begin
      clk = 1'b0;
      #(PERIOD/2) clk = 1'b1;
      #(PERIOD/2);
   end

   wire ADC_SCLK, ADC_SDATA, ADC_SCS;
   
   adc08d1020_serial adcserial (
			       .sclk(ADC_SCLK),
			       .sdata(ADC_SDATA),
			       .scs(ADC_SCS),
			       .w_data(data),
			       .w_addr(addr),
			       .commit(commit),
			       .busy(busy),
			       .clk12p5(clk)
			   );

   initial begin
      data = 16'b0;
      addr = 4'b0;
      commit = 1'b0;
            
      $stop;

      // reset at gate level
      #(PERIOD*16);

      data = 16'hA11F;
      #(PERIOD*16);
      
      addr = 4'h3;
      commit = 1'b1;

      #(PERIOD*100);
      
      addr = 4'h0;
      commit = 1'b0;
      
      #(PERIOD*100);
      $stop;
   end // initial begin
endmodule // tadc08d1020_serial_tb


