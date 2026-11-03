`timescale 1ns / 1ps
//=============================================================================
// tb_audio —— 直接解码 video_top 的 I2S 输出，判断"FPGA 到底有没有送出音频"
//
// 用 PLAY_MODE=1（自动演示音序，不需要键盘），跑 120ms 后统计：
//   峰值 |sample|、非零样本比例、i2s_data 翻转次数
// 若峰值接近 0 → FPGA 侧没出声，问题在 RTL；
// 若峰值明显非零 → FPGA 输出正常，问题在功放/接线/供电。
//=============================================================================
module tb_audio;
	reg clk = 1'b0;
	always #10 clk = ~clk;

	reg rst_n = 1'b0;
	reg [3:0] kbd_col = 4'hF;
	reg key_s1 = 1'b0, key_s2 = 1'b0, tp_sda = 1'b1;
	wire [3:0] kbd_row;
	wire ads_sda, ads_scl, tp_scl;
	wire i2s_bclk, i2s_lrclk, i2s_data, i2s_mclk, dbg_lat;
	wire lcd_scl, lcd_sda, lcd_res, lcd_dc, lcd_cs;

	pullup (ads_sda);

	video_top #(
		.PLAY_MODE (1),		// 演示音序：不需要按键就会发声
		.POT_EN    (0)
	) u_dut (
		.sys_clk(clk), .kbd_col(kbd_col), .kbd_row(kbd_row),
		.key_s1(key_s1), .key_s2(key_s2),
		.ads_sda(ads_sda), .ads_scl(ads_scl), .tp_scl(tp_scl), .tp_sda(tp_sda),
		.i2s_bclk(i2s_bclk), .i2s_lrclk(i2s_lrclk), .i2s_data(i2s_data),
		.i2s_mclk(i2s_mclk),
		.lcd_scl(lcd_scl), .lcd_sda(lcd_sda), .lcd_res(lcd_res),
		.lcd_dc(lcd_dc), .lcd_cs(lcd_cs),
		.dbg_lat(dbg_lat)
	);

	// ---- I2S 从机解码（左声道，每帧 32 位槽）----
	reg  [5:0]  bcnt = 6'd0;
	reg  [15:0] sh   = 16'd0;
	integer     nz   = 0;		// 非零样本数
	integer     ns   = 0;		// 样本总数
	integer     peak = 0;		// 峰值绝对值
	integer     togg = 0;		// i2s_data 翻转次数
	integer     mag, i;

	reg i2s_data_d = 1'b0;
	always @(posedge clk) begin
		i2s_data_d <= i2s_data;
		if (i2s_data != i2s_data_d) togg = togg + 1;
	end

	always @(posedge i2s_bclk) begin
		bcnt <= bcnt + 6'd1;
		if (i2s_lrclk == 1'b0) begin
			sh <= {sh[14:0], i2s_data};
			// 左声道槽 bit1..bit16 是有效数据
			if (bcnt >= 6'd1 && bcnt <= 6'd16) begin
				// 刚移入的是最低位；这里用移位近似幅度估计
			end
		end
		if (bcnt == 6'd63) begin
			bcnt <= 6'd0;
			// 若左声道有数据，sh 里是最后 16 位
			mag = sh[15] ? (65536 - sh) : sh;
			if (mag > 32767) mag = 32767;
			ns = ns + 1;
			if (mag > 8) nz = nz + 1;
			if (mag > peak) peak = mag;
		end
	end

	initial begin
		repeat (50) @(posedge clk);
		rst_n = 1'b1;
		#120_000_000;			// 120ms
		$display("I2S: samples=%0d nonzero=%0d peak=%0d toggles=%0d",
			ns, nz, peak, togg);
		if (peak > 500 && nz > ns/4)
			$display("RESULT: FPGA 正在输出音频（峰值 %0d）-> 问题在功放/接线", peak);
		else if (togg < 10000)
			$display("RESULT: i2s_data 几乎不翻转（%0d 次）-> FPGA 没送数据", togg);
		else
			$display("RESULT: 有翻转但幅度很小（峰值 %0d）-> 信号被衰减", peak);
		$finish;
	end
endmodule
