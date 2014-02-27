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

module adc_ddr3_tb;

   reg clk_p;
   wire clk_n;
   
   reg [7:0] data_i_p;
   wire [7:0] data_i_n;

   reg [7:0]  data_q_p;
   wire [7:0] data_q_n;

   wire [63:0] adc_i;
   wire [63:0] adc_q;
   reg 	       clk;

   reg [29:0] 	      sample_length;
   reg [17:0] 	      rbk_startpage;
   
   reg 	       reset;

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
   
   wire 	      adc_reset;
   sync_reset adc_res_sync( .glbl_reset(reset), .clk(clk), .reset(adc_reset) );
   reg 		      adc_oclk;

   reg 		      adc_sample;
   reg [2:0] 	      adc_sample_s;
   wire 	      adc_sample_oclk;

   reg [29:0] 	      sample_count;
   
   always @(posedge adc_oclk) begin // bring adc_sample into oclk domain
      adc_sample_s[2:0] <= {adc_sample_s[1:0], adc_sample};
   end

   always @(posedge adc_oclk) begin
      if( !adc_sample_s[2] & adc_sample_s[1] ) begin
	 sample_count <= 30'b0;
      end else if( adc_sample_s[2] && (sample_count < sample_length) ) begin
	 sample_count <= sample_count + 30'b1;
      end else begin
	 sample_count <= sample_count;
      end
   end
   assign adc_sample_oclk = (sample_count < sample_length) && adc_sample_s[2];
   
   adc_rx adc_rx(.data_i_p(data_i_p[7:0]),
		 .data_i_n(data_i_n[7:0]),
		 .data_q_p(data_q_p[7:0]),
		 .data_q_n(data_q_n[7:0]),

		 .clk_p(clk_p),
		 .clk_n(clk_n),

		 .adc_i(adc_i),
		 .adc_q(adc_q),
		 .oclk(adc_oclk),

		 .reset(adc_reset)
		 );

   //// ADC to DDR3 interface
   // ddr3 port wires
   wire        adc_ddr3_cmd_en;  // ok
   wire [3:0]  adc_ddr3_cmd_instr;  // ok
   wire [5:0]  adc_ddr3_cmd_bl;  // ok
   wire [29:0] adc_ddr3_cmd_byte_addr;  // ok
   reg         adc_ddr3_cmd_empty; 
   reg 	       adc_ddr3_cmd_full; // ok
   wire        adc_ddr3_wr_en;  // ok
   wire [7:0]  adc_ddr3_wr_mask; // ok
   wire [63:0] adc_ddr3_wr_data; // ok
   reg         adc_ddr3_wr_full; // ok
   reg         adc_ddr3_wr_empty;  // ok
   reg [6:0]   adc_ddr3_wr_count;
   reg         adc_ddr3_wr_underrun; // ok
   reg         adc_ddr3_wr_error; // ok

   // local state
   reg [5:0] 	      ddr3_burstcnt;
   reg [29:0] 	      ddr3_adc_adr;
   
   assign adc_ddr3_cmd_instr = 4'b0000;
   assign adc_ddr3_cmd_byte_addr = ddr3_adc_adr[29:0];
   assign adc_ddr3_cmd_bl = 6'b01_1111; // always write 32 fifo entries, 0-indexed
   assign adc_ddr3_cmd_en = (ddr3_burstcnt == 6'b11_1111);

   // i-data on even addresses, aligned on 64-bit boundaries
   assign adc_ddr3_wr_data[63:0] = ddr3_burstcnt[0] ? adc_q[63:0] : adc_i[63:0];
   // write whenever sampling is commanded, always write groups of 32
   assign adc_ddr3_wr_en = (adc_sample_oclk || (ddr3_burstcnt != 6'b0));
   assign adc_ddr3_wr_mask = 8'b0; // don't mask any writes
   
   always @(posedge adc_oclk) begin
      if( adc_sample_oclk || (ddr3_burstcnt != 6'b0) ) begin
	 ddr3_burstcnt <= ddr3_burstcnt + 6'b1;
      end else begin
	 ddr3_burstcnt <= 6'b0;
      end

      // this line below means every time sample is toggled, write pointer goes to 0 automatically
      if(!adc_ddr3_wr_en) begin // when we're not writing, reset pointer to 0
	 ddr3_adc_adr <= 30'b0; // reset buffer to zero when sampling is turned off, and buffer committed
	 // change the above constant if we want to do multiple offsets in DDR3
      end else begin
	 if(adc_ddr3_cmd_en) begin
	    ddr3_adc_adr <= ddr3_adc_adr + 30'b1000;
	 end else begin
	    ddr3_adc_adr <= ddr3_adc_adr;
	 end
      end
   end // always @ (posedge adc_oclk)


/*   
   wire c1_clk0, c1_rst0;
   wire 		      ddr3_dll_locked;
   wire 		      ddr3clk;
   wire 		      ddr3_calib_done; // to mcb
   wire 		      ddr3_p3_cmd_en; // to mcb
   wire [2:0] 		      ddr3_p3_cmd_instr;
   wire [5:0] 		      ddr3_p3_cmd_bl;
   wire [29:0] 		      ddr3_p3_cmd_byte_addr;
   wire 		      ddr3_p3_cmd_empty; // from mcb
   wire 		      ddr3_p3_cmd_full;

   wire 		      ddr3_p3_rd_en; // to mcb
   wire [31:0] 		      ddr3_p3_rd_data; // from mcb
   wire 		      ddr3_p3_rd_full;
   wire 		      ddr3_p3_rd_empty;
   wire [6:0] 		      ddr3_p3_rd_count;
   wire 		      ddr3_p3_rd_overflow;
   wire 		      ddr3_p3_rd_error;

   wire 		      dll_reset;

   ddr3_clkgen ddr3_clkgen (
			    .clk50in(clk),
			    .clk400(ddr3clk),
			    .RESET(dll_reset),
			    .LOCKED(ddr3_dll_locked)
			    );
   
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

   wire ddr3_reset_local;
   ddr3_eim_burst eim_burst(.bclk(bclk_i),
			    .bus_ad(eim_din),
			    .bus_a(EIM_A[18:16]), .adv(!EIM_LBA), .rw(EIM_RW), .cs(!EIM_CS[1]),
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
			    
			    .bclk_reset(bclk_reset)
			    );
   
   sync_reset ddr3_res_sync( .glbl_reset(log_reset), .clk(c1_clk0), .reset(ddr3_reset_local) );

   parameter C1_MEM_ADDR_WIDTH     = 14;
   parameter C1_MEM_BANKADDR_WIDTH = 3;
   parameter C1_NUM_DQ_PINS        = 16;
   
   wire [C1_MEM_ADDR_WIDTH-1:0]    mcb1_dram_a;
   wire [C1_MEM_BANKADDR_WIDTH-1:0]  mcb1_dram_ba;  
   wire                             mcb1_dram_ck;  
   wire                             mcb1_dram_ck_n;
   wire [C1_NUM_DQ_PINS-1:0]        mcb1_dram_dq;   
   wire                             mcb1_dram_dqs;  
   wire                             mcb1_dram_dqs_n;
   wire                             mcb1_dram_dm; 
   wire                             mcb1_dram_ras_n; 
   wire                             mcb1_dram_cas_n; 
   wire                             mcb1_dram_we_n;  
   wire                             mcb1_dram_cke; 
   wire                             mcb1_dram_odt;
   wire                             mcb1_dram_reset_n; 

   wire                             mcb1_dram_udqs;    // for X16 parts
   wire                             mcb1_dram_udqs_n;  // for X16 parts
   wire                             mcb1_dram_udm;     // for X16 parts
   
   ddr3_if_4port # (
    .C1_P0_MASK_SIZE(8),
    .C1_P0_DATA_PORT_SIZE(64),
    .C1_P1_MASK_SIZE(4),
    .C1_P1_DATA_PORT_SIZE(32),
    .DEBUG_EN(0),
    .C1_MEMCLK_PERIOD(2500),
    .C1_CALIB_SOFT_IP("TRUE"),
    .C1_SIMULATION("FALSE"),
    .C1_RST_ACT_LOW(0),
    .C1_INPUT_CLK_TYPE("SINGLE_ENDED"),
    .C1_MEM_ADDR_ORDER("ROW_BANK_COLUMN"),
    .C1_NUM_DQ_PINS(16),
    .C1_MEM_ADDR_WIDTH(14),
    .C1_MEM_BANKADDR_WIDTH(3)
	      )
   u_ddr3_if (

	      .c1_sys_clk             (ddr3clk),
	      .c1_sys_rst_i           (reset),
	      
	      .mcb1_dram_dq           (mcb1_dram_dq),  
	      .mcb1_dram_a            (mcb1_dram_a),  
	      .mcb1_dram_ba           (mcb1_dram_ba),
	      .mcb1_dram_ras_n        (mcb1_dram_ras_n),                        
	      .mcb1_dram_cas_n        (mcb1_dram_cas_n),                        
	      .mcb1_dram_we_n         (mcb1_dram_we_n),                          
	      .mcb1_dram_odt          (mcb1_dram_odt),
	      .mcb1_dram_cke          (mcb1_dram_cke),                          
	      .mcb1_dram_ck           (mcb1_dram_ck),                          
	      .mcb1_dram_ck_n         (mcb1_dram_ck_n),       
	      .mcb1_dram_dqs          (mcb1_dram_dqs),                          
	      .mcb1_dram_dqs_n        (mcb1_dram_dqs_n),
	      .mcb1_dram_udqs         (mcb1_dram_udqs),    // for X16 parts   
	      .mcb1_dram_udqs_n       (mcb1_dram_udqs_n),  // for X16 parts
	      .mcb1_dram_udm          (mcb1_dram_udm),     // for X16 parts
	      .mcb1_dram_dm           (mcb1_dram_dm),
	      .mcb1_dram_reset_n      (mcb1_dram_reset_n),
	      .c1_clk0		        (c1_clk0),
	      .c1_rst0		        (c1_rst0),
	
	      .c1_calib_done    (ddr3_calib_done),
	      .mcb1_rzq               (F_DDR3_RZQ),  
	      .mcb1_zio               (F_DDR3_ZIO),

	      .c1_p0_cmd_clk                          (adc_oclk),
	      .c1_p0_cmd_en                           (adc_ddr3_cmd_en),
	      .c1_p0_cmd_instr                        (adc_ddr3_cmd_instr),
	      .c1_p0_cmd_bl                           (adc_ddr3_cmd_bl),
	      .c1_p0_cmd_byte_addr                    (adc_ddr3_cmd_byte_addr),
	      .c1_p0_cmd_empty                        (adc_ddr3_cmd_empty),
	      .c1_p0_cmd_full                         (adc_ddr3_cmd_full),
	      .c1_p0_wr_clk                           (adc_oclk),
	      .c1_p0_wr_en                            (adc_ddr3_wr_en),
	      .c1_p0_wr_mask                          (adc_ddr3_wr_mask),
	      .c1_p0_wr_data                          (adc_ddr3_wr_data),
	      .c1_p0_wr_full                          (adc_ddr3_wr_full),
	      .c1_p0_wr_empty                         (adc_ddr3_wr_empty),
	      .c1_p0_wr_count                         (adc_ddr3_wr_count),
	      .c1_p0_wr_underrun                      (adc_ddr3_wr_underrun),
	      .c1_p0_wr_error                         (adc_ddr3_wr_error),
	      .c1_p0_rd_clk                           (adc_oclk),
	      .c1_p0_rd_en                            (1'b0),
//	      .c1_p0_rd_data                          (adc_ddr3_rd_data),   // read function is not used
//	      .c1_p0_rd_full                          (adc_ddr3_rd_full),
//	      .c1_p0_rd_empty                         (adc_ddr3_rd_empty),
//	      .c1_p0_rd_count                         (adc_ddr3_rd_count),
//	      .c1_p0_rd_overflow                      (adc_ddr3_rd_overflow),
//	      .c1_p0_rd_error                         (adc_ddr3_rd_error),
	      
	      // port 2 dedicated to CPU interface
	      .c1_p2_cmd_clk                          (bclk_dll),
	      .c1_p2_cmd_en                           (p2_cmd_en),
	      .c1_p2_cmd_instr                        (p2_cmd_instr),
	      .c1_p2_cmd_bl                           (p2_cmd_bl),
	      .c1_p2_cmd_byte_addr                    (p2_cmd_byte_addr),
	      .c1_p2_cmd_empty                        (ddr3_p2_cmd_empty),
	      .c1_p2_cmd_full                         (ddr3_p2_cmd_full),
	      .c1_p2_wr_clk                           (bclk_dll),
	      .c1_p2_wr_en                            (p2_wr_en),
	      .c1_p2_wr_mask                          (p2_wr_mask),
	      .c1_p2_wr_data                          (p2_wr_data),
	      .c1_p2_wr_full                          (ddr3_p2_wr_full),
	      .c1_p2_wr_empty                         (ddr3_p2_wr_empty),
	      .c1_p2_wr_count                         (ddr3_p2_wr_count),
	      .c1_p2_wr_underrun                      (ddr3_p2_wr_underrun),
	      .c1_p2_wr_error                         (ddr3_p2_wr_error)
	      );

// ========================================================================== //
// Memory model instances                                                     // 
// ========================================================================== //

   generate
   if(C1_NUM_DQ_PINS == 16) begin : MEM_INST1
     ddr3_model_c1 u_mem_c1(
      .ck         (mcb1_dram_ck),
      .ck_n       (mcb1_dram_ck_n),
      .cke        (mcb1_dram_cke),
      .cs_n       (1'b0),
      .ras_n      (mcb1_dram_ras_n),
      .cas_n      (mcb1_dram_cas_n),
      .we_n       (mcb1_dram_we_n),
      .dm_tdqs    ({mcb1_dram_udm,mcb1_dram_dm}),
      .ba         (mcb1_dram_ba),
      .addr       (mcb1_dram_a),
      .dq         (mcb1_dram_dq),
      .dqs        ({mcb1_dram_udqs,mcb1_dram_dqs}),
      .dqs_n      ({mcb1_dram_udqs_n,mcb1_dram_dqs_n}),
      .tdqs_n     (),
      .odt        (mcb1_dram_odt),
      .rst_n      (mcb1_dram_reset_n)
      );
   end else begin
     ddr3_model_c1 u_mem_c1(
      .ck         (mcb1_dram_ck),
      .ck_n       (mcb1_dram_ck_n),
      .cke        (mcb1_dram_cke),
      .cs_n       (1'b0),
      .ras_n      (mcb1_dram_ras_n),
      .cas_n      (mcb1_dram_cas_n),
      .we_n       (mcb1_dram_we_n),
      .dm_tdqs    (mcb1_dram_dm),
      .ba         (mcb1_dram_ba),
      .addr       (mcb1_dram_a),
      .dq         (mcb1_dram_dq),
      .dqs        (mcb1_dram_dqs),
      .dqs_n      (mcb1_dram_dqs_n),
      .tdqs_n     (),
      .odt        (mcb1_dram_odt),
      .rst_n      (mcb1_dram_reset_n)
     );
  end
endgenerate

*/   
   
   parameter PERIOD = 16'd4;   // 250 MHz
   always begin
      clk_p = 1'b0;
      #(PERIOD/2) clk_p = 1'b1;
      #(PERIOD/2);
   end

   parameter PERIOD50 = 16'd20; // 50 MHz
   always begin
      clk = 1'b0;
      #(PERIOD50/2) clk = 1'b1;
      #(PERIOD50/2);
   end

   parameter PERIOD_OCLK = 16'd8; // 125 MHz
   always begin
      adc_oclk = 1'b0;
      #(PERIOD_OCLK/2) adc_oclk = 1'b1;
      #(PERIOD_OCLK/2);
   end
   
   assign clk_n = !clk_p;
   assign data_i_n = ~data_i_p;
   assign data_q_n = ~data_q_p;

   initial begin
      reset = 1'b1;
      data_i_p[7:0] = 8'b0;
      data_q_p[7:0] = 8'b0;

      adc_ddr3_cmd_empty = 1'b0;
      adc_ddr3_cmd_full = 1'b0;
      adc_ddr3_wr_full = 1'b0;
      adc_ddr3_wr_empty = 1'b0;
      adc_ddr3_wr_count = 6'b0;
      adc_ddr3_wr_error = 1'b0;
      adc_ddr3_wr_underrun = 1'b0;
      
      sample_length = 30'h1000;
      adc_sample = 1'b0;
      rbk_startpage = 18'h0;
      rbk_clear_error = 1'b0;
      rbk_enable = 1'b0;
      rbk_init = 1'b0;
      
      EIM_A = 3'b0;
      EIM_LBA = 0;
      EIM_RW = 0;
      EIM_CS = 2'b11;
      eim_din = 16'h0;
      
      $stop;
      #(PERIOD*128);
      reset = 1'b0;
      #(PERIOD*128);

      // do stuff
      repeat(100) begin
	 data_i_p[7:0] = ({$random} % 256);
	 data_q_p[7:0] = ({$random} % 256);
	 #(PERIOD/2);
      end
      adc_sample = 1'b1;
      repeat(1000) begin
	 data_i_p[7:0] = ({$random} % 256);
	 data_q_p[7:0] = ({$random} % 256);
	 #(PERIOD/2);
      end

      $stop;
   end // initial begin
   
endmodule // adc_ddr3_tb

