`timescale 1ns / 1ps
//=============================================================================
// tb_lat —— 控制→发声全链延迟测量（只加 TB，不改任何 RTL）
//
// 实例化与 video_top（PLAY_MODE=0, INPUT_SRC=0, POT_EN=0）完全一致的链路：
//   keypad_scan(5000/8) -> note_alloc(10000) -> bend_ctrl(静息态,因子=1.0)
//   -> synth_core(钢琴包络, CH_MODE=0) -> i2s_tx(16 分频)
// 滑音 porta_ctrl 在 POT_EN=0 时被顶层硬旁路，故不实例化（零影响）。
// I2S 延时启动 168ms 是 video_top 的上电一次性门控，不属于每次演奏延迟。
//
// 激励：S1（row0 x col0）带真实机械抖动的按下（同 tb_bounce 模型）。
// 测量并 $display：
//   t0    = 首次接触（down|bounce 首次为 1）
//   t_col = 扫描首次采到列线为低
//   t_ks  = 消抖后 key_state[0] 上升
//   t_gate= 声部 gate 上升（=dbg_lat 拉高时刻）
//   t_snd = 合成器首个非零音频样本
//   t_i2s = i2s_data 在 gate 后首次跳变（音频位流开始携带声音）
// 同时导出 VCD 供绘图。
//=============================================================================
module tb_lat;
	reg clk = 1'b0;
	always #10 clk = ~clk;					// 50MHz
	reg rst_n = 1'b0;

	//---------------------------------------------------------------------
	// 按键模型（带机械抖动，同 tb_bounce）
	//---------------------------------------------------------------------
	reg bounce = 1'b0;
	reg down   = 1'b0;

	wire [3:0] row_n;
	wire [3:0] col_raw;
	assign col_raw[0] = (down || bounce) && (row_n[0] === 1'b0) ? 1'b0 : 1'bz;
	wire [3:0] col_n = col_raw === 4'hxx ? 4'hF : (col_raw | 4'hE);
	pullup (col_raw[0]); pullup (col_raw[1]);
	pullup (col_raw[2]); pullup (col_raw[3]);

	//---------------------------------------------------------------------
	// 被测链路
	//---------------------------------------------------------------------
	wire [15:0]  key_state;
	wire [3:0]   gate_v;
	wire [127:0] pinc_v;
	wire [63:0]  vel_v;
	wire [2:0]   timbre;
	wire [3:0]   env_sel_v;
	wire         fx_rev, fx_dly;

	keypad_scan #(.ROW_SETTLE(5000), .DBNC(8)) u_kbd (
		.clk(clk), .rst_n(rst_n), .row_n(row_n),
		.col_n(col_n), .key_state(key_state));

	note_alloc #(.BASE_VEL(10000)) u_alloc (
		.clk(clk), .rst_n(rst_n), .key_state(key_state),
		.gate_v(gate_v), .pinc_v(pinc_v), .vel_v(vel_v),
		.timbre_o(timbre), .env_sel_v(env_sel_v),
		.fx_rev(fx_rev), .fx_dly(fx_dly));

	// 弯音：FSR 静息态（码值 26000 = 松开基准），因子恒 1.0，零延迟贡献
	wire        sample_tick;
	wire signed [15:0] sample_l, sample_r;
	wire [127:0] pinc_bent;
	bend_ctrl #(.REL_CODE(16'd26000), .BEND_SEMI(2), .SLEW_STEP(16'd150)) u_bend (
		.clk(clk), .rst_n(rst_n), .tick(sample_tick),
		.upd(1'b0), .press_val(16'd26000),
		.pinc_in(pinc_v), .pinc_out(pinc_bent));

	synth_core #(
		.N_VOICE(4),
		.ATT_SAMPLES(98), .DEC_SAMPLES(16000),
		.REL_SAMPLES(12207), .SUS_LEVEL(6554),
		.ATT_SAMPLES2(3906), .DEC_SAMPLES2(9766),
		.REL_SAMPLES2(19531), .SUS_LEVEL2(22938),
		.CH_MODE(0)
	) u_synth (
		.clk(clk), .rst_n(rst_n), .tick(sample_tick),
		.gate_v(gate_v), .pinc_v(pinc_bent), .vel_v(vel_v),
		.env_sel_v(env_sel_v), .timbre_i(timbre), .vib_man(8'd0),
		.sample_l(sample_l), .sample_r(sample_r));

	wire i2s_bclk, i2s_lrclk, i2s_data, i2s_mclk;
	i2s_tx #(.CLK_HZ(50000000), .CLK_DIV(16)) u_i2s (
		.clk(clk), .rst_n(rst_n),
		.sample_l(sample_l), .sample_r(sample_r),
		.i2s_bclk(i2s_bclk), .i2s_lrclk(i2s_lrclk),
		.i2s_data(i2s_data), .i2s_mclk(i2s_mclk),
		.sample_tick(sample_tick));

	//---------------------------------------------------------------------
	// 时刻测量
	//---------------------------------------------------------------------
	realtime t0 = 0, t_col = 0, t_ks = 0, t_gate = 0, t_snd = 0, t_i2s = 0;
	reg got_t0 = 0, got_col = 0, got_ks = 0, got_gate = 0, got_snd = 0, got_i2s = 0;

	always @(down, bounce)
		if (!got_t0 && (down || bounce)) begin
			t0 = $realtime; got_t0 = 1;
		end

	always @(negedge col_n[0])
		if (!got_col) begin
			t_col = $realtime; got_col = 1;
		end

	always @(posedge key_state[0])
		if (!got_ks) begin
			t_ks = $realtime; got_ks = 1;
		end

	always @(posedge gate_v[0])
		if (!got_gate) begin
			t_gate = $realtime; got_gate = 1;
		end

	always @(sample_l)
		if (!got_snd && got_gate && sample_l !== 16'sd0) begin
			t_snd = $realtime; got_snd = 1;
		end

	reg i2s_d = 0;
	always @(posedge clk) i2s_d <= i2s_data;
	always @(posedge i2s_d)
		if (!got_i2s && got_gate) begin
			t_i2s = $realtime; got_i2s = 1;
		end

	//---------------------------------------------------------------------
	// 激励与报告
	//---------------------------------------------------------------------
	initial begin
		$dumpfile("D:/zcode_work/sim_tb/lat.vcd");
		$dumpvars(1, tb_lat);
		repeat (10) @(posedge clk);
		rst_n = 1'b1;
		#2_000_000;					// t=2ms 开始按下（带抖动）
		bounce = 1'b1; down = 1'b1;
		#300_000;  bounce = 1'b0;
		#400_000;  bounce = 1'b1;
		#700_000;  bounce = 1'b0;
		#500_000;  bounce = 1'b1;
		#1200_000; bounce = 1'b0;	// t≈5.1ms 稳定接触
		down = 1'b1;
		#15_000_000;				// 按住到 t≈20ms
		down = 1'b0; bounce = 1'b1;	// 松开沿抖动
		#500_000;  bounce = 1'b0;
		#800_000;  bounce = 1'b1;
		#600_000;  bounce = 1'b0;
		#8_000_000;					// 看一段释放段

		$display("RESULT t0_ms=%0d",           t0 / 1.0e6);
		$display("RESULT t_col_ms=%0d",        t_col / 1.0e6);
		$display("RESULT t_ks_ms=%0d",         t_ks / 1.0e6);
		$display("RESULT t_gate_ms=%0d",       t_gate / 1.0e6);
		$display("RESULT t_snd_ms=%0d",        t_snd / 1.0e6);
		$display("RESULT t_i2s_ms=%0d",        t_i2s / 1.0e6);
		$display("RESULT d_scan_us=%0d",   (t_col - t0) / 1.0e3);
		$display("RESULT d_dbnc_ms=%0d",   (t_gate - t_col) / 1.0e6);
		$display("RESULT d_out_us=%0d",    (t_i2s - t_gate) / 1.0e3);
		$display("RESULT d_total_ms=%0d",  (t_i2s - t0) / 1.0e6);
		$display("TB_LAT_DONE");
		$finish;
	end
endmodule
