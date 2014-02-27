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

module eim_debug(
		 input wire bclk_i,
		 input wire bclk_dll,
		 input wire adv_in,
		 input wire rw_in,
		 input wire cs1_in,
		 input wire [2:0] a_in,
		 input wire [15:0] din_in,
		 input wire oe_in,

		 input wire eim_trig_ena,
		 input wire eim_mon_reset,
		 input wire eim_mon_adv,

		 output wire mon_empty,
		 output wire mon_full,

		 output wire [23:0] mon_data,
		 
		 input wire bclk_reset
		 );

   wire pretrig_full;
   wire pretrig_almost_full;
   wire [23:0] pretrig_d;
   
   eim_pretrigger eim_pre (
			   .wr_clk(bclk_i), // input wr_clk
			   .rd_clk(bclk_dll), // input rd_clk
			   .din({1'b0,oe_in,adv_in,rw_in,cs1_in,a_in[2:0],din_in[15:0]}), // input [22 : 0] din
			   .wr_en(1'b1), // input wr_en
			   .rd_en(pretrig_full | pretrig_almost_full), // input rd_en
			   .dout(pretrig_d[23:0]), // output [23 : 0] dout
			   .full(pretrig_full), // output full
			   .almost_full(pretrig_almost_full),
//			   .empty(empty), // output empty
//			   .underflow(underflow), // output underflow
//			   .rd_data_count(rd_data_count) // output [7 : 0] rd_data_count
			   .rst(bclk_reset) // input rst
			   );

   wire        mon_trig;
   reg         mon_wren;
   reg         mon_oneshot;
   
   assign mon_trig = eim_trig_ena & !cs1_in; // trigger on CS1 going active

   // trigger state machine: wait for trigger; then fill the FIFO; then hold until reset
   always @(posedge bclk_dll) begin
      if(eim_mon_reset) begin
	 mon_oneshot <= 1'b0;
	 mon_wren <= 1'b0;
      end else begin
	 if(mon_trig && !mon_wren && !mon_oneshot) begin // init then catch trigger
	    mon_wren <= 1'b1;
	    mon_oneshot <= 1'b1; // ignore future triggers
	 end else if( mon_oneshot && mon_wren ) begin // next state after trigger, write until full
	    mon_oneshot <= 1'b1;
	    if(mon_full) begin
	       mon_wren <= 1'b0; // once full, disable mon_wren
	    end else begin
	       mon_wren <= 1'b1;
	    end
	 end else if( mon_oneshot && !mon_wren ) begin // this is the terminal state from logging
	    mon_oneshot <= 1'b1;
	    mon_wren <= 1'b0;
	 end else begin // we reach this case if oneshot hasn't fired yet, e.g. waiting for tigger event
	    mon_oneshot <= 1'b0;
	    mon_wren <= 1'b0;
	 end
      end // else: !if(eim_mon_reset)
   end // always @ (posedge bclk_dll)

   reg eim_mon_pulse;
   reg [1:0] eim_mon_adv_d;

   always @(posedge bclk_dll) begin
      eim_mon_adv_d[0] <= eim_mon_adv;
      eim_mon_adv_d[1] <= eim_mon_adv_d[0];

      eim_mon_pulse <= !eim_mon_adv_d[1] & eim_mon_adv_d[0];
   end

   //// TODO: add almost_full to FIFO and map mon_full to almost_full to avoid overflow-by-one
   eim_monitor eim_mon (
				   .wr_clk(bclk_dll), // input wr_clk
				   .rd_clk(bclk_dll), // input rd_clk
				   .din(pretrig_d[23:0]), // input [21 : 0] din
				   .wr_en(mon_wren), // input wr_en
				   .rd_en(eim_mon_pulse), // input rd_en
				   .dout(mon_data[23:0]), // output [21 : 0] dout
				   .almost_full(mon_full), // output full
//				   .overflow(overflow), // output overflow
				   .empty(mon_empty), // output empty
//				   .rd_data_count(rd_data_count) // output [9 : 0] rd_data_count
				   .rst(bclk_reset | eim_mon_reset) // input rst
				   );

endmodule // eim_debug

