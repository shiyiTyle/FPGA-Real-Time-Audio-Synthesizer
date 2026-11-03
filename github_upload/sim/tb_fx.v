`timescale 1ns / 1ps
//=============================================================================
// tb_fx —— audio_fx 专用快速仿真
// 时间线（tick）：0~2000 静音；2000~2640 拨弦脉冲（G5 半周期方波）；
// 40000 打开混响再测。探针输出 probe_fx.txt
//=============================================================================
module tb_fx;
	reg clk = 1'b0;
	reg rst_n = 1'b0;
	reg fx_rev = 1'b0, fx_dly = 1'b0;
	reg signed [15:0] sample_l = 16'sd0, sample_r = 16'sd0;

	always #10 clk = ~clk;

	reg [5:0] tick_cnt = 6'd0;
	wire      tick     = (tick_cnt == 6'd63);
	always @(posedge clk)
		tick_cnt <= tick_cnt + 6'd1;

	wire signed [15:0] out_l, out_r;

	audio_fx u_fx (
		.clk(clk), .rst_n(rst_n), .tick(tick),
		.fx_rev(fx_rev), .fx_dly(fx_dly),
		.sample_l(sample_l), .sample_r(sample_r),
		.out_l(out_l), .out_r(out_r)
	);

	// 计 tick 数
	reg [17:0] tcnt = 18'd0;
	always @(posedge clk)
		if (rst_n && tick_cnt == 6'd62)
			tcnt <= tcnt + 18'd1;

	integer fd;
	always @(posedge clk) begin
		if (rst_n && tick_cnt == 6'd62 && tcnt[2:0] == 3'd0)
			$fdisplay(fd, "%0d in=%0d dly_old=%0d dly_wet=%0d rev_wet=%0d wet_r=%0d out=%0d",
				tcnt, sample_l, u_fx.dly_old, u_fx.dly_wet, u_fx.rev_wet, u_fx.wet_r, out_l);
	end

	initial begin
		fd = $fopen("probe_fx.txt", "w");
		// 拨弦脉冲：从 tick 2000 起 640 个 tick 的 ±10000 方波（约 6.1Hz 包络内 G5）
		fx_dly = 1'b1;
		@(posedge tick);
		rst_n = 1'b1;
		repeat (1990) @(posedge tick);
		repeat (640) begin
			sample_l = (tcnt[4:0] < 5'd16) ? 16'sd10000 : -16'sd10000;
			sample_r = sample_l;
			@(posedge tick);
		end
		sample_l = 16'sd0;
		sample_r = 16'sd0;
		repeat (20000) @(posedge tick);		// 看第一个回声（+16383 附近）
		// 再测混响：tick ~22650 打开混响并再拨一次
		fx_rev = 1'b1;
		repeat (640) begin
			sample_l = (tcnt[4:0] < 5'd16) ? 16'sd10000 : -16'sd10000;
			sample_r = sample_l;
			@(posedge tick);
		end
		sample_l = 16'sd0;
		sample_r = 16'sd0;
		repeat (30000) @(posedge tick);		// 看混响尾
		$fclose(fd);
		$display("TB_FX_DONE tcnt=%0d", tcnt);
		$finish;
	end
endmodule
