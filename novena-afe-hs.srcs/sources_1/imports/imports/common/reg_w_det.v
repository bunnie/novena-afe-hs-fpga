module reg_w_det(
	      input wire clk,
	      input wire [18:0] bus_a,
	      input wire [18:0] my_a,
	      input wire [15:0] bus_d,
	      input wire ena,
	      input wire [7:0] trigger, // bitmask set that triggers the pulse, exact match required
	      output wire pulse

	      );

   reg 			  gotwrite, gotwrite_d;
   reg [7:0] 		  state;
   
   always @(posedge clk) begin
      if( (bus_a[18:1] == my_a[18:1]) && ena ) begin
	 state <= bus_d;
	 gotwrite <= 1'b0;
      end else begin
	 state <= state;
	 gotwrite <= 1'b1;
      end
   end

   always @(posedge clk) begin
      gotwrite_d <= gotwrite;
   end

   assign pulse = (state == bitmask) && (!gotwrite_d && gotwrite);

endmodule // reg_w_det



