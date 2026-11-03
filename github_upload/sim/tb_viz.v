`timescale 1ns / 1ps
//=============================================================================
// tb_viz —— viz_scope 屏幕正确性验证（虚拟 LCD 帧缓冲）
//
// 用 240x112 的"是否绿色"位图模拟屏幕，按 viz 下发的每个矩形任务刷新：
//   1) 静音段：绿色像素 = 240 列 x 3 行 = 720 左右
//   2) 大音量段：绿色像素显著增多
//   3) 回到静音后：必须回到 720 左右（验证擦除，回归"越积越绿"的 bug）
//=============================================================================
module tb_viz;
	reg clk = 1'b0;
	reg rst_n = 1'b0;
	always #10 clk = ~clk;

	reg  signed [15:0] audio = 16'sd0;
	reg  [5:0]  tdiv = 6'd0;
	wire        audio_tick = (tdiv == 6'd15);
	always @(posedge clk) tdiv <= tdiv + 6'd1;

	reg [15:0] key_state = 16'h0000;
	reg [2:0]  timbre = 3'd0;
	reg        fx_rev = 1'b0, fx_dly = 1'b0;
	reg [15:0] adc0 = 16'd0, adc1 = 16'd0, adc2 = 16'd0;
	reg        dbg_adc = 1'b0;

	wire        go;
	wire [7:0]  x0, y0, x1, y1;
	wire        solid;
	wire [15:0] col;
	reg         model_busy = 1'b0;
	reg  [5:0]  bcnt = 6'd0;
	wire [15:0] frames;
	wire [111:0] bar_flat;
	wire         bar_upd;

	wire drv_ready = !model_busy;
	wire drv_busy  = model_busy;

	viz_spec u_spec (
		.clk(clk), .rst_n(rst_n),
		.audio(audio), .audio_tick(audio_tick),
		.bar_flat(bar_flat), .upd(bar_upd)
	);

	viz_scope u_viz (
		.clk(clk), .rst_n(rst_n),
		.audio(audio), .audio_tick(audio_tick),
		.key_state(key_state), .timbre(timbre), .fx_rev(fx_rev), .fx_dly(fx_dly),
		.bar_flat(bar_flat), .bar_upd(bar_upd),
		.adc0(adc0), .adc1(adc1), .adc2(adc2), .dbg_adc(dbg_adc),
		.go(go), .x0(x0), .y0(y0), .x1(x1), .y1(y1),
		.solid(solid), .col(col),
		.drv_ready(drv_ready), .drv_busy(drv_busy), .frames(frames)
	);

	// ---- virtual screen: 1 = green (waveform area only) ----
	reg green [0:111][0:239];
	integer px, py, cnt, k;

	always @(posedge clk) begin
		if (go && !model_busy) begin
			for (py = y0; py <= y1; py = py + 1)
				for (px = x0; px <= x1; px = px + 1)
					if (py <= 111 && px <= 239) begin
						if (solid && col == 16'h07E0) green[py][px] <= 1'b1;
						else if (solid && col == 16'h0000) green[py][px] <= 1'b0;
					end
			model_busy <= 1'b1;
			bcnt <= 6'd0;
		end else if (model_busy) begin
			if (bcnt == 6'd8) model_busy <= 1'b0;
			else bcnt <= bcnt + 6'd1;
		end
	end

	task count_green(output integer n, output integer far_n);
		begin
			n = 0;
			far_n = 0;
			for (py = 0; py <= 111; py = py + 1)
				for (px = 0; px <= 239; px = px + 1)
					if (green[py][px]) begin
						n = n + 1;
						if (py < 40 || py > 72) far_n = far_n + 1;
					end
		end
	endtask

	integer ph_i;
	reg [4:0] sq;
	always @(posedge clk) begin
		if (audio_tick) begin
			sq <= sq + 5'd1;
			case (ph_i)
			0: audio <= 16'sd0;
			1: audio <= (sq[4]) ? 16'sd18000 : -16'sd18000;
			default: audio <= 16'sd0;
			endcase
		end
	end

	integer g1, g2, g3, f1, f2, f3;
	initial begin
		sq = 5'd0; ph_i = 0;
		for (py = 0; py <= 111; py = py + 1)
			for (px = 0; px <= 239; px = px + 1) green[py][px] = 1'b0;
		repeat (20) @(posedge clk);
		rst_n = 1'b1;

		wait (frames >= 3);
		count_green(g1, f1);
		$display("PHASE0 silence : green=%0d far=%0d frames=%0d (expect far=0)", g1, f1, frames);

		ph_i = 1;
		wait (frames >= 7);
		count_green(g2, f2);
		$display("PHASE1 loud    : green=%0d far=%0d frames=%0d (expect far>1000)", g2, f2, frames);

		ph_i = 2;
		wait (frames >= 11);
		count_green(g3, f3);
		$display("PHASE2 silence : green=%0d far=%0d frames=%0d (expect far=0 again)", g3, f3, frames);

		if (f1 > 50)        $display("FAIL: silence baseline not flat");
		else if (f2 < 400)  $display("FAIL: loud trace did not spread");
		else if (f3 > 50)   $display("FAIL: not erased after returning to silence");
		else                $display("PASS: trace spreads and fully erases");

		$display("TB_VIZ_DONE");
		$finish;
	end

	initial begin
		#80_000_000;
		$display("FAIL: timeout frames=%0d", frames);
		$finish;
	end
endmodule
