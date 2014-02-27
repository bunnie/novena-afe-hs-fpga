module reg_wo(
	      input wire clk,
	      input wire [18:0] bus_a,
	      input wire [18:0] my_a,
	      input wire [15:0] bus_d,
	      input wire we,
	      input wire re,
	      output wire [15:0] reg_d,
	      output reg [15:0] rbk_d
	      );

   reg [15:0] 			 state;

   always @(posedge clk) begin
      if( (bus_a[18:1] == my_a[18:1]) && we ) begin
	 state <= bus_d;
      end else begin
	 state <= state;
      end
   end

   assign reg_d = state;

   always @(bus_a or my_a or re or state) begin
      if( (bus_a[18:1] == my_a[18:1]) && re ) begin
	 rbk_d = state;
      end else begin
	 rbk_d = 16'hZZZZ;
      end
   end

endmodule // reg_wo
