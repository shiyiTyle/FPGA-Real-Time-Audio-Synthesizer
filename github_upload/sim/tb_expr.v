`timescale 1ns / 1ps
module tb_expr;
	reg signed [15:0] y0, y1;
	reg [7:0] frac;
	wire signed [16:0] diff = {y1[15], y1} - {y0[15], y0};
	wire signed [24:0] prod = diff * $signed({1'b0, frac});
	wire signed [17:0] A = {y0[15], y0} + (prod >>> 8);
	wire signed [17:0] B = $signed({y0[15], y0}) + (prod >>> 8);
	wire signed [17:0] C = $signed({{2{y0[15]}}, y0}) + (prod >>> 8);
	wire signed [17:0] D = $signed(y0) + (prod >>> 8);
	wire signed [24:0] E = $signed({{8{y0[15]}}, y0}) + $signed(prod >>> 8);
	initial begin
		y0 = 16'sd804; y1 = 16'sd0; frac = 8'd67; #1;
		$display("y0=%0d ps=%0d expect=%0d | A=%0d B=%0d C=%0d D=%0d E=%0d",
			y0, prod >>> 8, y0 + (prod >>> 8), A, B, C, D, E);
		y0 = -16'sd30000; y1 = -16'sd29000; frac = 8'd200; #1;
		$display("y0=%0d ps=%0d expect=%0d | A=%0d B=%0d C=%0d D=%0d E=%0d",
			y0, prod >>> 8, y0 + (prod >>> 8), A, B, C, D, E);
		$finish;
	end
endmodule
