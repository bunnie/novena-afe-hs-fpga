module rising_edge(
		   input wire clk,
		   input wire level,
		   output wire pulse
		   );

   reg 			       state;
   
   always @(posedge clk) begin
      state <= level;
   end
   assign pulse = !state && level;

endmodule // rising_edge
