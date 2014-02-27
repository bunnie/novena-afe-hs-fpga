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

module ddr3_eim_burst(
		      input wire bclk,
		      
		      // sync to bclk
		      input wire [15:0] bus_ad, // raw mux data
		      input wire [2:0] bus_a, // high address bits
		      input wire adv, // active high, so connect to !EIM_LBA
		      input wire rw,  // low = write, high = read, so connect to EIM_RW
		      input wire cs,  // active high, so connect to !EIM_CS[1]

		      output reg [15:0] rbk_d, // readback tri-state interface

		      // sync to bclk
		      output wire [2:0] ddr3_rd_cmd,
		      output wire [5:0] ddr3_rd_bl,
		      output reg [29:0] ddr3_rd_adr,
		      output reg ddr3_rd_cmd_en,
		      input wire ddr3_rd_cmd_empty,
		      input wire ddr3_rd_cmd_full,

		      // sync to bclk
		      input wire [31:0] ddr3_rd_data,
		      input wire [6:0] ddr3_rd_count,
		      input wire ddr3_rd_empty,
		      input wire ddr3_rd_full,
		      output wire ddr3_rd_en,

		      // async
		      input wire clear_error,
		      output reg full_error,
		      output reg empty_error,

		      input wire enable, // when high, enable page filling
		      input wire init, // when asserted, module resets and goes to zero-full page state
		      input wire [17:0] startpage, // address where to start reading from (4k page-aligned)

		      input wire [15:0] burstlen, // length of a burst
		      
		      input wire bclk_reset
		      );

   reg 				 activated;
   reg [15:0] 			 bus_ad_r;
   reg 				 cs_r;
   reg [2:0] 			 bus_a_r;
   reg 				 rw_r;
   reg 				 adv_r;
   
   wire [15:0] 	 rdata;
   reg [1:0] 	 reset_s;
   reg [2:0] 	 enable_s;
   reg [2:0] 	 init_s;
   reg [17:0] 	 startpage_s;
   reg [1:0] 	 clear_error_s;
   wire 	 fifo_prog_empty;
   reg [15:0] 	 burstlen_s;

   // issue read commands to the memory interface
   // read address is directly a register
   assign ddr3_rd_cmd = 3'b001;
   assign ddr3_rd_bl = 6'b111111; // 64-element burst read (x32 bits wide = 256 bytes)

   reg [1:0] 	 setup_ok; // assert when conditions are right to issue a read
   always @(posedge bclk) begin
      setup_ok[1:0] <= {setup_ok[0],
			((ddr3_rd_count[6:0] == 7'b0) || ddr3_rd_empty) && fifo_prog_empty && enable_s[2]};
      
      if( reset_s[1] ) begin
	 ddr3_rd_cmd_en <= 1'b0;
      end else begin
	 if(!setup_ok[1] && setup_ok[0]) begin  // issue only on rising edge of setup_ok
	    ddr3_rd_cmd_en <= 1'b1;
	 end else begin
	    ddr3_rd_cmd_en <= 1'b0;
	 end
      end

      if( reset_s[1] ) begin
	 ddr3_rd_adr[29:0] <= {startpage_s[17:0],12'b0}; // page-aligned address
      end else begin
	 if( ddr3_rd_cmd_en ) begin // this is active for exactly one cycle
	    ddr3_rd_adr[29:0] <= ddr3_rd_adr[29:0] + 30'h100; // inc by one burst stride
	    // and we totally allow wrap-around, if you're not careful
	 end else begin
	    ddr3_rd_adr[29:0] <= ddr3_rd_adr[29:0];
	 end
      end
      
   end // always @ (posedge bclk)

   wire [15:0] fifo_deb_dout;
   assign ddr3_rd_en = 1'b1; // always read when data's present
   wire        full, empty;
   fifo_ddr3_eim_burst fifo_deb (
				 .rst(reset_s[1]), // input rst
				 .wr_clk(bclk), // input wr_clk
				 .rd_clk(bclk), // input rd_clk
				 .din(ddr3_rd_data), // input [31 : 0] din
				 .wr_en(!ddr3_rd_empty), // input wr_en
				 .rd_en(activated && (burstlen_s[15:0] > burstcnt[15:0])), // input rd_en
				 .dout(fifo_deb_dout), // output [15 : 0] dout
				 .full(full), // output full
//				 .almost_full(almost_full), // output almost_full
				 .empty(empty), // output empty
//				 .almost_empty(almost_empty), // output almost_empty
				 .prog_empty(fifo_prog_empty) // output prog_empty
				 );

   reg 	       empty_d;
   always @(posedge bclk) begin
      empty_d <= empty; // prep rising edge det of empty
      
      if(clear_error_s[1] || reset_s[1]) begin
	 full_error <= 1'b0;
      end else begin
	 if( full ) begin 
	    full_error <= 1'b1;
	 end else begin
	    full_error <= full_error;
	 end
      end

      if(clear_error_s[1] || reset_s[1]) begin
	 empty_error <= 1'b0;
      end else begin
	 if( !empty_d && empty ) begin // empty should never rise once it falls
	    empty_error <= 1'b1;
	 end else begin
	    empty_error <= empty_error;
	 end
      end
   end // always @ (posedge bclk)

   // incoming synchronizers
   always @(posedge bclk) begin
      init_s[2:0] <= {init_s[1:0], init};
      reset_s[1:0] <= {reset_s[0], (!init_s[2] && init_s[1]) || bclk_reset};
      enable_s[2:0] <= {enable_s[1:0], enable};
      startpage_s <= startpage; // just one level of sync because this should be static at point of use
      clear_error_s[1:0] <= {clear_error_s[0], clear_error};
   end

   reg [15:0] burstcnt;
   ////// address decode path
   always @(posedge bclk) begin
      bus_ad_r <= bus_ad;
      bus_a_r <= bus_a;
      cs_r <= cs;
      rw_r <= rw;
      adv_r <= adv;
      burstlen_s <= burstlen;
      
      if( cs_r && adv_r && ({bus_a_r, bus_ad_r[15:12]} == 7'h4_F) ) begin // 0xc04_fxxx page for bursting
	 activated <= 1'b1;
      end else if( !cs_r ) begin
	 activated <= 1'b0;
      end else begin
	 activated <= activated;
      end // else: !if( !cs )

      if( !activated ) begin
	 burstcnt <= 16'h0;
      end else begin
	 burstcnt <= burstcnt + 16'h1; 
      end
   end // always @ (posedge clk)

   always @(*) begin
      if( activated && rw_r ) begin
	 rbk_d = fifo_deb_dout;
      end else begin
	 rbk_d = 16'hZZZZ;
      end
   end
   
endmodule // ddr3_eim_burst

