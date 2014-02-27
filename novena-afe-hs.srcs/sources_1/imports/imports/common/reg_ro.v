module reg_ro(
	      input wire clk,
	      input wire [18:0] bus_a,
	      input wire [18:0] my_a,
	      input wire [15:0] reg_d,
	      input wire re,
	      output reg [15:0] bus_d
	      );

   reg [15:0] 			 state;

   always @(posedge clk) begin
      state <= reg_d;
   end

   always @(bus_a or my_a or re or state) begin
      if( (bus_a[18:1] == my_a[18:1]) && re ) begin
	 bus_d = state;
      end else begin
	 bus_d = 16'hZZZZ;
      end
   end

endmodule // reg_ro
