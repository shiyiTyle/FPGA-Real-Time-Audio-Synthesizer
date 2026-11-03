`timescale 1ns / 1ps
//=============================================================================
// tb_top —— video_top 整链冒烟仿真：验证例化/接线无误、60ms 内无 X 传播
// （ADS 未接模型：总线 pullup 读全 1，bend 按无压力处理；无按键 -> 静音）
//=============================================================================
module tb_top;
	reg clk = 1'b0;
	reg key_s1 = 1'b0, key_s2 = 1'b0;
	reg [3:0] kbd_col = 4'hF;			// 无键按下（列上拉，低=按下）
	reg tp_sda = 1'b1;

	wire [3:0] kbd_row;
	wire ads_sda, ads_scl, tp_scl;
	wire i2s_bclk, i2s_lrclk, i2s_data, i2s_mclk, dbg_lat;
	wire lcd_scl, lcd_sda, lcd_res, lcd_dc, lcd_cs;

	pullup(ads_sda);					// 模拟 I2C 总线上拉

	video_top u_dut (
		.sys_clk  (clk),
		.kbd_col  (kbd_col),
		.kbd_row  (kbd_row),
		.key_s1   (key_s1),
		.key_s2   (key_s2),
		.ads_sda  (ads_sda),
		.ads_scl  (ads_scl),
		.tp_scl   (tp_scl),
		.tp_sda   (tp_sda),
		.i2s_bclk (i2s_bclk),
		.i2s_lrclk(i2s_lrclk),
		.i2s_data (i2s_data),
		.i2s_mclk (i2s_mclk),
		.lcd_scl  (lcd_scl),
		.lcd_sda  (lcd_sda),
		.lcd_res  (lcd_res),
		.lcd_dc   (lcd_dc),
		.lcd_cs   (lcd_cs),
		.dbg_lat  (dbg_lat)
	);

	// LCD SPI 从机：统计事务，验证屏确实被驱动
	reg [7:0] lcd_sh;
	reg [2:0] lcd_bit = 3'd0;
	integer   lcd_bytes = 0;
	integer   lcd_cmds  = 0;
	always @(posedge lcd_scl) begin
		if (!lcd_cs) begin
			lcd_sh  <= {lcd_sh[6:0], lcd_sda};
			lcd_bit <= lcd_bit + 3'd1;
			if (lcd_bit == 3'd7) begin
				lcd_bit <= 3'd0;
				lcd_bytes = lcd_bytes + 1;
				if (!lcd_dc) lcd_cmds = lcd_cmds + 1;
			end
		end
	end
	// 每次任务启动时打印窗口（找 y0>y1 的非法任务）
	integer njobs = 0;
	reg go_d = 1'b0;
	always @(posedge clk) begin
		go_d <= u_dut.lcd_go;
		if (u_dut.lcd_go && !go_d) begin
			njobs = njobs + 1;
			if (njobs <= 6 || u_dut.lcd_y1 < u_dut.lcd_y0)
				$display("JOB %0d: x %0d..%0d  y %0d..%0d  y0gt1=%b",
					njobs, u_dut.lcd_x0, u_dut.lcd_x1, u_dut.lcd_y0, u_dut.lcd_y1,
					(u_dut.lcd_y1 < u_dut.lcd_y0));
		end
	end
	// 状态跟踪
	reg [2:0] ph_d = 3'd0;
	integer capprobe = 0;
	always @(posedge clk) begin
		if (u_dut.u_viz.ph != ph_d) begin
			ph_d <= u_dut.u_viz.ph;
			$display("PH %0d -> %0d @%0t  cap_wr=%0d peak=%0d",
				ph_d, u_dut.u_viz.ph, $time, u_dut.u_viz.cap_wr, u_dut.u_viz.peak);
		end
		if (u_dut.u_viz.ph == 3'd2 && u_dut.sample_tick && capprobe < 3) begin
			capprobe = capprobe + 1;
			$display("CAPPROBE: audio=%h gain=%0d ynow=%h cap_wr=%0d",
				u_dut.u_viz.audio, u_dut.u_viz.gain, u_dut.u_viz.ynow, u_dut.u_viz.cap_wr);
		end
	end

	always #10 clk = ~clk;

	initial begin
		#60_000_000;					// 60ms：POR + ADS 初始化 + LCD 初始化中
		$display("DBG lcd: res=%b cs=%b scl=%b bytes=%0d cmds=%0d viz_frames=%0d",
			lcd_res, lcd_cs, lcd_scl, lcd_bytes, lcd_cmds, u_dut.viz_frames);
		if ((^u_dut.mix_l_fx === 1'bx) || (^u_dut.mix_l === 1'bx)
		    || (^u_dut.i2s_data === 1'bx) || (^u_dut.pinc_bent === 1'bx))
			$display("SMOKE_FAIL: X detected on audio chain");
		else if (lcd_bytes < 40)
			$display("SMOKE_FAIL: LCD 未被驱动（字节数 %0d）", lcd_bytes);
		else
			$display("SMOKE_PASS: 音频链无 X，LCD 正在被驱动（%0d 字节 / %0d 命令）",
				lcd_bytes, lcd_cmds);

		// LCD 初始化需 RES 100ms + 100ms + sleep-out 120ms，再清屏 73.7ms，
		// 首帧最早 ~470ms 才完成，故检查点放到 900ms
		#840_000_000;
		$display("DBG t=900ms: viz_frames=%0d lcd_bytes=%0d lcd_cmds=%0d",
			u_dut.viz_frames, lcd_bytes, lcd_cmds);
		$display("DBG viz ph=%0d ds=%0d col_i=%0d cell_i=%0d gain=%0d peak=%0d",
			u_dut.u_viz.ph, u_dut.u_viz.ds, u_dut.u_viz.col_i, u_dut.u_viz.cell_i,
			u_dut.u_viz.gain, u_dut.u_viz.peak);
		if (u_dut.viz_frames > 0)
			$display("SMOKE_PASS: 可视化已在出帧（frames=%0d）", u_dut.viz_frames);
		else
			$display("SMOKE_FAIL: 可视化未出帧");
		$finish;
	end
endmodule
