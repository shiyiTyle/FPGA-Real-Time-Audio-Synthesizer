`timescale 1ns / 1ps
//=============================================================================
// tb_flute —— dds_voice 长笛路径快速验证
// 按住 80000 tick，dump vib_depth（应 0.4s 后渐入到 255）与颤音相位偏移
//=============================================================================
module tb_flute;
	reg clk = 1'b0;
	reg rst_n = 1'b0;
	reg tick = 1'b0;

	always #10 clk = ~clk;
	reg [5:0] tcnt;
	always @(posedge clk)
		tcnt <= tcnt + 6'd1;
	always @(posedge clk)
		tick <= (tcnt == 6'd63);

	wire signed [15:0] so;
	dds_voice #(
		.ATT_SAMPLES(98), .DEC_SAMPLES(16000), .REL_SAMPLES(12207), .SUS_LEVEL(6554),
		.ATT_SAMPLES2(3906), .DEC_SAMPLES2(9766), .REL_SAMPLES2(19531), .SUS_LEVEL2(22938)
	) u_v (
		.clk(clk), .rst_n(rst_n), .tick(tick),
		.gate(1'b1), .pinc(32'd68960566), .vib_man(8'd0),
		.timbre(3'd5), .env_sel(1'b0), .sample_out(so)
	);

	reg [16:0] tick_n;
	always @(posedge clk)
		if (rst_n && tcnt == 6'd62) tick_n <= tick_n + 17'd1;

	integer fd;
	always @(posedge clk) begin
		if (rst_n && tcnt == 6'd62 && tick_n % 2000 == 0)
			$fdisplay(fd, "%0d depth=%0d env_st=%0d env=%0d vib_off=%0d",
				tick_n, u_v.vib_depth, u_v.env_state, u_v.env, u_v.vib_off);
	end

	initial begin
		fd = $fopen("probe_flute.txt", "w");
		tcnt = 6'd0;
		tick_n = 17'd0;
		@(posedge clk);
		rst_n = 1'b1;
		repeat (80000 * 64) @(posedge clk);
		$fclose(fd);
		$display("TB_FLUTE_DONE");
		$finish;
	end
endmodule
