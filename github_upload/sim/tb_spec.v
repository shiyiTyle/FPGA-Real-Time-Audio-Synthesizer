`timescale 1ns / 1ps
//=============================================================================
// tb_spec —— viz_spec (Goertzel 16 频点) 正确性验证
//
// 喂 f Hz 正弦，跑满 3 个窗口后检查：
//   f=1000Hz  -> 最高柱应落在 bin6(966Hz) 或 bin7(1272Hz)
//   f=2000Hz  -> 最高柱应落在 bin9(2136Hz) 或 bin8(1628Hz)
//   f=300Hz   -> 最高柱应落在 bin1(254Hz) 或 bin2(356Hz)
//=============================================================================
module tb_spec;
	reg clk = 1'b0;
	reg rst_n = 1'b0;
	always #10 clk = ~clk;

	reg signed [15:0] audio = 16'sd0;
	reg [5:0] tdiv = 6'd0;
	wire audio_tick = (tdiv == 6'd63);		// fs = 50MHz/64 = 781.25kHz？否
	// 说明：真实系统 fs = 50MHz/64/16 = 48828.125Hz（i2s_tx 每 64 个 BCLK 一个帧，
	// BCLK 又是 clk/16）。这里为了仿真速度用 clk/16 作为样本节拍并在频率上等效缩放：
	// 实际用 fs_sim = 50MHz/16 = 3.125MHz 会让系数失配，故本 TB 按真实比例
	// 每 64*16=1024 个 clk 出一个样本（fs=48828Hz）。
	reg [9:0] sdiv = 10'd0;
	wire      tick_real = (sdiv == 10'd1023);
	always @(posedge clk) begin
		tdiv <= tdiv + 6'd1;
		sdiv <= sdiv + 10'd1;
	end

	real      fs = 48828.125;
	real      f_tone = 1000.0;
	integer   nsamp = 0;
	real      tt;

	wire [111:0] bar_flat;
	wire         upd;

	always @(posedge clk) begin
		if (tick_real) begin
			tt = nsamp / fs;
			audio <= $rtoi(20000.0 * $sin(6.283185307 * f_tone * tt));
			nsamp = nsamp + 1;
		end
	end

	viz_spec u_spec (
		.clk(clk), .rst_n(rst_n),
		.audio(audio), .audio_tick(tick_real),
		.bar_flat(bar_flat), .upd(upd)
	);

	integer b, best, bestv, v, upd_cnt, zeros;
	task show_bars;
		begin
			best = 0; bestv = -1; upd_cnt = 0;
			for (b = 0; b < 16; b = b + 1) begin
				v = bar_flat[b*7 +: 7];
				if (v > bestv) begin bestv = v; best = b; end
			end
			$display("  bars = %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
				bar_flat[0*7+:7], bar_flat[1*7+:7], bar_flat[2*7+:7], bar_flat[3*7+:7],
				bar_flat[4*7+:7], bar_flat[5*7+:7], bar_flat[6*7+:7], bar_flat[7*7+:7],
				bar_flat[8*7+:7], bar_flat[9*7+:7], bar_flat[10*7+:7], bar_flat[11*7+:7],
				bar_flat[12*7+:7], bar_flat[13*7+:7], bar_flat[14*7+:7], bar_flat[15*7+:7]);
			$display("  peak bin = %0d (height %0d)", best, bestv);
		end
	endtask

	initial begin
		repeat (20) @(posedge clk);
		rst_n = 1'b1;

		// 1000 Hz
		f_tone = 1000.0;
		repeat (3*1024*960 + 5000) @(posedge clk);
		$display("f=1000Hz -> expect peak at bin6(966) or bin7(1272)");
		show_bars;
		if (best == 6 || best == 7) $display("PASS: 1000Hz peak bin correct");
		else $display("FAIL: 1000Hz peak bin wrong");

		// 2000 Hz
		f_tone = 2000.0;
		repeat (3*1024*960 + 5000) @(posedge clk);
		$display("f=2000Hz -> expect peak at bin8(1628) or bin9(2136)");
		show_bars;
		if (best == 8 || best == 9) $display("PASS: 2000Hz peak bin correct");
		else $display("FAIL: 2000Hz peak bin wrong");

		// 300 Hz
		f_tone = 300.0;
		repeat (3*1024*960 + 5000) @(posedge clk);
		$display("f=300Hz -> expect peak at bin1(254) or bin2(356)");
		show_bars;
		if (best == 1 || best == 2) $display("PASS: 300Hz peak bin correct");
		else $display("FAIL: 300Hz peak bin wrong");

		$display("TB_SPEC_DONE");
		$finish;
	end
endmodule
