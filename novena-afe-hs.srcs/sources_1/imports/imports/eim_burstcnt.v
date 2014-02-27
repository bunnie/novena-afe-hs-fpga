module eim_burstcnt(
		    input wire bclk,

		    // sync to bclk
		    input wire [15:0] bus_ad, // raw mux data
		    input wire [2:0] bus_a, // high address bits
		    input wire adv, // active high, so connect to !EIM_LBA
		    input wire rw,  // low = write, high = read, so connect to EIM_RW
		    input wire cs,  // active high, so connect to !EIM_CS[1]

		    output reg [15:0] measured_burst
		    );

   reg 				 activated;
   reg [15:0] 			 bus_ad_r;
   reg 				 cs_r;
   reg [2:0] 			 bus_a_r;
   reg 				 rw_r;
   reg 				 adv_r;

   reg [15:0] 			 burstcnt;
   reg [15:0] 			 finalcnt;

   reg 				 activated_d;
   
   always @(posedge bclk) begin
      bus_ad_r <= bus_ad;
      bus_a_r <= bus_a;
      cs_r <= cs;
      rw_r <= rw;
      adv_r <= adv;
      
      if( cs_r && adv_r && ({bus_a_r, bus_ad_r[15:12]} == 7'h4_F) ) begin // 0xc04_fxxx page for bursting
	 activated <= 1'b1;
      end else if( !cs_r ) begin
	 activated <= 1'b0;
      end else begin
	 activated <= activated;
      end // else: !if( !cs )

      if( !activated ) begin
	 finalcnt <= burstcnt;
	 burstcnt <= 16'h0;
      end else begin
	 burstcnt <= burstcnt + 16'h1;
	 finalcnt <= finalcnt;
      end

      activated_d <= activated;
      if( activated_d & !activated ) begin // on falling edge of activated...
	 measured_burst <= finalcnt + 1'b1; // account for 0-offset indexing
      end else begin
	 measured_burst <= measured_burst;
      end
   end // always @ (posedge clk)

   

endmodule // eim_burstcnt
