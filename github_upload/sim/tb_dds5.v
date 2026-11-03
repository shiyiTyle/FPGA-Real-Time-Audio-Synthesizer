`timescale 1ns / 1ps
// 诊断用：单声部 dds_voice，TIMBRE=0 正弦 / 5 长笛，打印内部节点
module tb_dds5 #(parameter integer TIMBRE = 5);
	reg clk = 1'b0;
	reg rst_n = 1'b0;
	reg gate = 1'b0;
	reg [5:0] tick_cnt = 6'd0;
	wire tick = (tick_cnt == 6'd63);
	always #10 clk = ~clk;
	always @(posedge clk) tick_cnt <= tick_cnt + 6'd1;

	wire signed [15:0] sample_out;
	reg [17:0] n = 18'd0;

	dds_voice #(
		.ATT_SAMPLES(98), .DEC_SAMPLES(16000), .REL_SAMPLES(12207), .SUS_LEVEL(6554),
		.ATT_SAMPLES2(3906), .DEC_SAMPLES2(9766), .REL_SAMPLES2(19531), .SUS_LEVEL2(22938)
	) u (
		.clk(clk), .rst_n(rst_n), .tick(tick), .gate(gate),
		.pinc(32'd68960566), .timbre(TIMBRE[2:0]), .vib_man(8'd0),
		.env_sel(1'b0), .sample_out(sample_out)
	);

	always @(posedge clk) begin
		if (rst_n && tick_cnt == 6'd62) n <= n + 18'd1;
		if (rst_n && tick_cnt == 6'd62 && n >= 18'd150 && n < 18'd3000 && n[2:0] == 3'd0)
			$display("N=%0d out=%0d wave=%0d sin=%0d y0=%0d y1=%0d frac=%0d isum=%0d env=%0d st=%0d",
				n, sample_out, u.wave_i2, u.wave_sin, u.y0, u.y1, u.frac, u.interp_sum,
				u.env, u.env_state);
	end

	initial begin
		rst_n = 1'b0;
		repeat (200) @(posedge tick);
		rst_n = 1'b1;
		repeat (100) @(posedge tick);
		gate = 1'b1;
		repeat (4000) @(posedge tick);
		$display("TB_DDS5_DONE");
		$finish;
	end
endmodule
