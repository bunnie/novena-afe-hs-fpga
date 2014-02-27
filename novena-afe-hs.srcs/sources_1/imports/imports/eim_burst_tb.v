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

module eim_burst_tb;

   reg 	       reset;
   reg 	       adc_oclk;

   reg [2:0]   EIM_A;
   reg 	       EIM_LBA;
   reg 	       EIM_RW;
   reg [1:0]   EIM_CS;
   reg [15:0]  eim_din;
   wire [15:0] ro_d_b;

   reg 	       rbk_clear_error;
   wire        rbk_full_error;
   wire        rbk_empty_error;

   reg 	       rbk_enable;
   reg 	       rbk_init;
   reg [17:0]  rbk_startpage;
   
   wire 		      ddr3_p3_cmd_empty; // from mcb
   wire		      ddr3_p3_cmd_full;
   
   wire [31:0] 		      ddr3_p3_rd_data; // from mcb
   wire 		      ddr3_p3_rd_full;
   wire 		      ddr3_p3_rd_empty;
   wire [6:0] 		      ddr3_p3_rd_count;
   wire 		      ddr3_p3_rd_overflow;
   reg 		      ddr3_p3_rd_error;

   wire p3_cmd_en;
   wire p3_burst_cmd_en;
   wire [2:0] p3_cmd_instr;
   wire [2:0] p3_burst_cmd_instr;
   wire [5:0] p3_cmd_bl;
   wire [5:0] p3_burst_cmd_bl;
   wire [29:0] p3_cmd_byte_addr;
   wire [29:0] p3_burst_addr;
   wire        p3_rd_en;
   wire        p3_burst_rd_en;

   assign p3_cmd_en = p3_burst_cmd_en;
   assign p3_cmd_instr = p3_burst_cmd_instr;
   assign p3_cmd_bl = p3_burst_cmd_bl;
   assign p3_cmd_byte_addr = p3_burst_addr;
   assign p3_rd_en = p3_burst_rd_en;

   ddr3_eim_burst eim_burst(.bclk(adc_oclk),
			    .bus_ad(eim_din),
			    .bus_a(EIM_A[2:0]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
			    .rbk_d(ro_d_b),
			    
			    .ddr3_rd_cmd(p3_burst_cmd_instr),
			    .ddr3_rd_bl(p3_burst_cmd_bl),
			    .ddr3_rd_adr(p3_burst_addr),
			    .ddr3_rd_cmd_en(p3_burst_cmd_en),
			    .ddr3_rd_cmd_empty(ddr3_p3_cmd_empty),
			    .ddr3_rd_cmd_full(ddr3_p3_cmd_full),

			    .ddr3_rd_data(ddr3_p3_rd_data[31:0]),
			    .ddr3_rd_count(ddr3_p3_rd_count),
			    .ddr3_rd_empty(ddr3_p3_rd_empty),
			    .ddr3_rd_full(ddr3_p3_rd_full),
			    .ddr3_rd_en(p3_burst_rd_en),

			    .clear_error(rbk_clear_error),
			    .full_error(rbk_full_error),
			    .empty_error(rbk_empty_error),

			    .enable(rbk_enable),
			    .init(rbk_init),
			    .startpage(rbk_startpage[17:0]),
			    
			    .bclk_reset(reset)
			    );
   
   
   parameter PERIOD_OCLK = 16'd8; // 125 MHz
   always begin
      adc_oclk = 1'b0;
      #(PERIOD_OCLK/2) adc_oclk = 1'b1;
      #(PERIOD_OCLK/2);
   end

   task eim_rd_cycle;
      begin
	 EIM_LBA = 0;
	 EIM_CS = 2'b01;
	 EIM_A = 3'b100;
	 EIM_RW = 1'b1;
	 eim_din = 16'h3042;
	 #(PERIOD_OCLK);
	 eim_din = 16'hF000;
	 #(PERIOD_OCLK);
	 EIM_LBA = 1;
	 
	 eim_din = 16'hzzzz;
	 #(PERIOD_OCLK*5);

	 #(PERIOD_OCLK*16); // readback data here
	 EIM_CS = 2'b11;
      end
   endtask; // eim_rd_cycle
   
/*
    reg [7:0] ddrcycle;

   always @(posedge adc_oclk) begin
      if(p3_burst_cmd_en) begin
	 ddr3_p3_cmd_empty <= 1'b0;
	 ddrcycle <= ddrcycle + 8'b1;
      end else if(ddrcycle >= 8'h10) begin
	 ddr3_p3_cmd_empty <= 1'b0;
      end

      if( ddrcycle != 8'b0 ) begin
	 if( ddrcycle == 8'h28 ) begin
	    ddrcycle <= 8'b0;
	    ddr3_p3_rd_empty <= 1'b1;
	 end else begin
	    ddrcycle <= ddrcycle + 8'b1;
	 end
      end

      if( ddrcycle > 8'h26 ) begin
	 ddr3_p3_cmd_empty <= 1'b1;
      end

      if( ddrcycle > 8'h7 ) begin
	 ddr3_p3_rd_empty <= 1'b0;
	 ddr3_p3_rd_count <= ddrcycle;
	 ddr3_p3_rd_data <= {$random} % 33'h1_0000_0000;
      end
   end
*/
   
   // emulate a very simple DDR3-MIG moderated memory interface here
   reg [1:0]   cmd_count;
   reg [7:0]   data_count;
   reg [7:0]   wdata_count;
   reg [31:0]  latency_timer;
   reg [31:0]  wlatency_timer;
   reg 	       add_data;
   reg 	       ddr3_go;
   reg 	       ddr3_wgo;
   reg [1:0]   wcmd_count;

   parameter DDR3_TURNAROUND_TIME = 32'h20;
   wire        ddr3_rd_cmd_en;
   wire        ddr3_wr_cmd_en;
   reg         ddr3_rd_cmd_full;
   reg 	       ddr3_wr_cmd_full;
   reg 	       ddr3_wr_full;

   reg 	       ddr3_rd_dat_overflow;
   reg 	       ddr3_rd_dat_full;
   reg 	       ddr3_rd_dat_empty;
   reg [31:0]  ddr3_rd_dat;

   wire        clk;

   wire        ddr3_wr_dat_en;
   reg 	       ddr3_wr_empty;
   wire        ddr3_rd_dat_en;
   assign ddr3_wr_dat_en = 1'b0;
   assign ddr3_rd_dat_en = p3_burst_rd_en;
   
   assign clk = adc_oclk;
   assign ddr3_rd_cmd_en = p3_burst_cmd_en;
   assign ddr3_wr_cmd_en = 1'b0;
   assign ddr3_p3_cmd_full = ddr3_rd_cmd_full;
   assign ddr3_p3_rd_overflow = ddr3_rd_dat_overflow;
   assign ddr3_p3_rd_full = ddr3_rd_dat_full;
   assign ddr3_p3_rd_count[6:0] = data_count[6:0];
   assign ddr3_p3_rd_empty = ddr3_rd_dat_empty;
   assign ddr3_p3_rd_data = ddr3_rd_dat;
      
   assign ddr3_wr_cmd_empty = (wcmd_count == 2'b00);
   assign ddr3_p3_cmd_empty = (cmd_count == 2'b00);
   
   // we're going to cheat and use blocking assignments.
   always @(posedge clk) begin
      if( ddr3_rd_cmd_en ) begin
	 if( cmd_count == 2'b11 ) begin
	    cmd_count = cmd_count;
	 end else begin
	    cmd_count = cmd_count + 2'b1;
	 end
      end else begin
	 cmd_count = cmd_count;
      end

      if( ddr3_wr_cmd_en ) begin
	 if( wcmd_count == 2'b11 ) begin
	    wcmd_count = wcmd_count;
	 end else begin
	    wcmd_count = wcmd_count + 2'b1;
	 end
      end else begin
	 wcmd_count = wcmd_count;
      end
      
      if( cmd_count == 2'b11 ) begin
	 ddr3_rd_cmd_full = 1'b1;
      end else begin
	 ddr3_rd_cmd_full = 1'b0;
      end

      if( wcmd_count == 2'b11  || (wcmd_count == 2'b10 && ddr3_wr_cmd_en)) begin
	 ddr3_wr_cmd_full = 1'b1;
      end else begin
	 ddr3_wr_cmd_full = 1'b0;
      end
      
      if( cmd_count > 2'b00 ) begin
	 ddr3_go = 1'b1;
      end else begin
	 ddr3_go = 1'b0;
      end

      if( wcmd_count > 2'b00 ) begin
	 ddr3_wgo = 1'b1;
      end else begin
	 ddr3_wgo = 1'b0;
      end

      if( ddr3_wr_dat_en ) begin
	 wdata_count <= wdata_count + 8'b1;
      end

      if( wdata_count > 8'b0 ) begin
	 ddr3_wr_empty <= 1'b0;
      end else begin
	 ddr3_wr_empty <= 1'b1;
      end

      if( ddr3_wgo ) begin
	 wlatency_timer = wlatency_timer + 32'b1;
	 if( wdata_count != 8'b0 ) begin
	    wdata_count <= wdata_count - 8'b1;
	 end
      end

      if( wlatency_timer >= 32'd16 ) begin
	 wlatency_timer <= 32'd0;
	 wcmd_count <= wcmd_count - 2'b1;
      end

      if( wdata_count >= 8'd64 ) begin
	 ddr3_wr_full <= 1'b1;
      end else begin
	 ddr3_wr_full <= 1'b0;
      end
      
      if( ddr3_go && !add_data ) begin
	 latency_timer = latency_timer + 32'b1;
      end else if (add_data || !ddr3_go) begin
	 latency_timer = 32'b0;
      end else begin
	 latency_timer = latency_timer;
      end
	 
      if( latency_timer > DDR3_TURNAROUND_TIME ) begin
	 add_data = 1;
	 cmd_count = cmd_count - 2'b1;
	 latency_timer = 32'b0;
      end else begin
	 add_data = 1'b0;
      end

      if( add_data ) begin
	 if( data_count < 8'd64 ) begin
	    data_count = data_count + 8'd16;  // we get 16 words at a time
	    ddr3_rd_dat_full = 1'b0;
	 end else if (data_count == 8'd64) begin
	    ddr3_rd_dat_overflow = 1'b0;
	    ddr3_rd_dat_full = 1'b1;
	    data_count = data_count;
	 end else begin
	    ddr3_rd_dat_overflow = 1'b1;
	    ddr3_rd_dat_full = 1'b1;
	    data_count = data_count;
	 end
      end else begin
	 // need a delete data entry here too
	 data_count = data_count;
      end // else: !if( add_data )


      if( ddr3_rd_dat_en ) begin
	 if( data_count > 8'd0 ) begin
	    ddr3_rd_dat = ddr3_rd_dat + 32'h01010101;
	    data_count = data_count - 8'b1;
	 end else begin
	    data_count = 8'b0;
	 end
      end

      if( data_count[6:0] > 7'd0 ) begin
	 ddr3_rd_dat_empty = 1'b0;
      end else begin
	 ddr3_rd_dat_empty = 1'b1;
      end
      
   end

   initial begin
      // emu params
      cmd_count = 2'b0;
      data_count = 8'b0;
      latency_timer = 32'b0;
      add_data = 1'b0;
      ddr3_go = 1'b0;
      
      wcmd_count = 2'b0;
      wdata_count = 8'b0;
      wlatency_timer = 32'b0;
      ddr3_wgo = 1'b0;
      ddr3_rd_dat = 32'b0;
      ddr3_rd_dat_overflow = 1'b0;
            
      // tb params
      // reset = 1'b1;
      
      EIM_A = 3'b0;
      EIM_LBA = 1'b1;
      EIM_RW = 1'b1;
      EIM_CS = 2'b11;
      eim_din = 16'h0;

      rbk_clear_error = 1'b0;
      rbk_enable = 1'b0;
      rbk_init = 1'b0;
      rbk_startpage = 18'h42E;

//      ddr3_p3_cmd_empty = 1'b1;
//      ddr3_p3_cmd_full = 1'b0;
//      ddr3_p3_rd_data = 32'h0;
//      ddr3_p3_rd_full = 1'b0;
//      ddr3_p3_rd_empty = 1'b1;
//      ddr3_p3_rd_count = 7'b0;
//      ddr3_p3_rd_overflow = 1'b0;
      ddr3_p3_rd_error = 1'b0;

//      ddrcycle = 8'b0;

      #(PERIOD_OCLK*16);
      
      $stop;
      #(PERIOD_OCLK*128);
      reset = 1'b0;
      rbk_clear_error = 1'b1;
      rbk_init = 1'b1;
      #(PERIOD_OCLK*128);

      rbk_clear_error = 1'b0;
      rbk_init = 1'b0;
      rbk_enable = 1'b1;
      
      #(PERIOD_OCLK*128);

      // do stuff
      eim_rd_cycle();
      #(PERIOD_OCLK*30);
      eim_rd_cycle();
      #(PERIOD_OCLK*30);
      eim_rd_cycle();
      #(PERIOD_OCLK*2048);
      eim_rd_cycle();
      #(PERIOD_OCLK*2048);

      repeat( 32 ) begin
	 eim_rd_cycle();
	 #(PERIOD_OCLK*30);
      end

      #(PERIOD_OCLK*512);
      
      $stop;
   end // initial begin
   
endmodule // adc_ddr3_tb

