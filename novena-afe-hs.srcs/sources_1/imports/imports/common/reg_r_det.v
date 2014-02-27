module reg_r_det(
	      input wire clk,
	      input wire [18:0] bus_a,
	      input wire [18:0] my_a,
	      input wire ena,
	      output reg pulse

	      );

   reg 			  gotread, gotread_d;
   reg [7:0] 		  state;
   
   always @(posedge clk) begin
      if( (bus_a[18:1] == my_a[18:1]) && ena ) begin
	 gotread <= 1'b0;
      end else begin
	 gotread <= 1'b1;
      end
   end

   always @(posedge clk) begin
      gotread_d <= gotread;
      pulse <= !gotread_d && gotread;
   end

endmodule // reg_r_det




