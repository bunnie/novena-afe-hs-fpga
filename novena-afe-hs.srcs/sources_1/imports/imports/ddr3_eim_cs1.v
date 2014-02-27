// adapter from DDR3 interface to 64-bit CS1 burst interface

module ddr3_eim_cs1(
		    input wire clk,
		    
		    input wire [63:0] ctl, 
		    input wire ctl_stb,  // strobes when control register updated
		    output wire [63:0] burst_rd,
		    input wire rd_stb,   // strobes when burst read register has been accessed
		    output wire [63:0] status,

		    output wire [2:0] ddr3_rd_cmd,
		    output wire [5:0] ddr3_rd_bl,
		    output wire [29:0] ddr3_rd_adr,
		    output wire ddr3_rd_cmd_en,
		    input wire ddr3_rd_cmd_empty,
		    input wire ddr3_rd_cmd_full,

		    input wire [31:0] ddr3_rd_data,
		    input wire [6:0] ddr3_rd_count,
		    input wire ddr3_rd_empty,
		    input wire ddr3_rd_full,
		    output reg ddr3_rd_en,
		    
		    input wire reset
		    );

   reg [29:0] 		 cmd_adr;
   reg [4:0] 		 num_pkts; // 256 byte queue max, 8-byte packets = 32 max packet request
   reg [4:0] 		 outstanding;

   reg [63:0] 		 rd_cache;

   reg 			 cmd_go;

   reg 			 reset_errors;
   reg 			 cmd_err;

   reg [7:0] 		 readcount;
   
   assign burst_rd[63:0] = rd_cache[63:0];
   
   
   assign status = {readcount,
		    3'b0,cmd_err, 
		    ddr3_rd_cmd_empty, ddr3_rd_cmd_full, ddr3_rd_empty, ddr3_rd_full, 
		    1'b0, ddr3_rd_count[6:0]};

   // readcount helps sanity-check i.MX6 errors. Sometimes the compiler will kick out
   // integer (32-bit) reads and double the read count. Read count should equal packet request count.
   always @(posedge clk) begin
      if( ctl_stb ) begin
	 readcount <= 8'b0;
      end else if( rd_stb ) begin
	 readcount <= readcount + 8'b1;
      end else begin
	 readcount <= readcount;
      end
   end

   always @(posedge clk) begin
      if( ctl_stb ) begin
	 cmd_adr <= ctl[29:0];
	 num_pkts <= ctl[36:32]; // specified in 64-bit increments, up to 32 packets; 0 = no transaction
      end else begin
	 cmd_adr <= cmd_adr;
	 num_pkts <= num_pkts;
      end
      
      cmd_go <= ctl_stb && (ctl[36:32] != 5'b0);
   end

   assign ddr3_rd_cmd = 3'b001; // always read
   assign ddr3_rd_adr = {cmd_adr[29:2],2'b00}; // passed through from host, snapped to 32-bit boundary
   assign ddr3_rd_bl[5:0] = {num_pkts[4:0],1'b0} - 6'b1;
   assign ddr3_rd_cmd_en = cmd_go; // always assume we have space in cmd fifo

   parameter READ_IDLE        = 6'b1 << 0;
   parameter READ_PENDING     = 6'b1 << 1;
   parameter READ_FETCH       = 6'b1 << 2;
   parameter READ_UPDATE_LSB  = 6'b1 << 3;
   parameter READ_UPDATE_MSB  = 6'b1 << 4;
   parameter READ_WAIT        = 6'b1 << 5;

   parameter READ_nSTATES = 6;
   reg [(READ_nSTATES - 1):0] 		 cstate;
   reg [(READ_nSTATES - 1):0] 		 nstate;

   always @(posedge clk) begin
      cstate <= nstate;
   end

   // state evolution
   always @(*) begin
      case(cstate)
	// starting assumptions:
	//  - rd_cache is invalid
	//  - no burst read is pending
	READ_IDLE: begin
	   if( cmd_go ) begin
	      nstate <= READ_PENDING;
	   end else begin
	      nstate <= READ_IDLE;
	   end
	end
	READ_PENDING: begin
	   if( outstanding != 5'b0 ) begin
	      if( ddr3_rd_count[6:0] < 7'b10 ) begin
		 nstate <= READ_PENDING;
	      end else begin
		 nstate <= READ_FETCH;
	      end
	   end else begin
	      nstate <= READ_IDLE;
	   end
	end // case: READ_PENDING
	READ_FETCH: begin
	   nstate <= READ_UPDATE_LSB;
	end
	READ_UPDATE_LSB: begin
	   nstate <= READ_UPDATE_MSB;
	end
	READ_UPDATE_MSB: begin
	   nstate <= READ_WAIT;
	end
	READ_WAIT: begin
	   if( rd_stb ) begin  // min 30 bclk cycles between read strobes, so plenty of time to cycle back
	      nstate <= READ_PENDING;
	   end else begin
	      nstate <= READ_WAIT;
	   end
	end
	default: begin
	   nstate <= READ_IDLE;
	end
      endcase // case (cstate)
   end // always @ (*)

   // state output
   always @(posedge clk) begin
      case(cstate)
	READ_IDLE: begin
	   outstanding[4:0] <= num_pkts[4:0];
	   rd_cache <= rd_cache;
	   if( ddr3_rd_count[6:0] > 7'b0 ) begin
	      ddr3_rd_en <= 1'b1; // discard any mismatch/overruns
	   end else begin
	      ddr3_rd_en <= 1'b0;
	   end
	end
	READ_PENDING: begin
	   outstanding <= outstanding;
	   rd_cache <= rd_cache;
	   ddr3_rd_en <= 1'b0;
	end
	READ_FETCH: begin
	   outstanding <= outstanding;
	   rd_cache <= rd_cache;
	   ddr3_rd_en <= 1'b1;
	end
	READ_UPDATE_LSB: begin
	   outstanding <= outstanding;
	   rd_cache[63:0] <= {rd_cache[63:32],ddr3_rd_data[31:0]};
	   ddr3_rd_en <= 1'b1;
	end
	READ_UPDATE_MSB: begin
	   outstanding <= outstanding - 5'b1;
	   rd_cache[63:0] <= {ddr3_rd_data[31:0],rd_cache[31:0]};
//	   rd_cache[63:0] <= {ddr3_rd_data[31:0] + 32'hbabe1234,rd_cache[31:0]};
	   ddr3_rd_en <= 1'b0;
	end
	READ_WAIT: begin
	   outstanding <= outstanding;
	   rd_cache <= rd_cache;
	   ddr3_rd_en <= 1'b0;
	end
	default: begin
	   outstanding <= outstanding;
	   rd_cache <= rd_cache;
	   ddr3_rd_en <= 1'b0;
	end
      endcase // case (cstate)
   end
   
   
   // catch and track errors -- we assume that cmd fifo always has space for us in this implementation
   // make sure we catch when this fails
   always @(posedge clk) begin
      reset_errors <= ctl[63];
      if( reset_errors ) begin
	 cmd_err <= 1'b0;
      end else begin
	 if( cmd_go && ddr3_rd_cmd_full ) begin
	    cmd_err <= 1'b1;
	 end else begin
	    cmd_err <= cmd_err;
	 end
      end
   end // always @ (posedge clk)
   
endmodule // ddr3_eim_cs1
