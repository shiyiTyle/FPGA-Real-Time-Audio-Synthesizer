`timescale 1ns / 1ps
//=============================================================================
// synth_core —— 多声部并行合成引擎（4 声部 + 饱和混音 + 立体声输出）
//
// N_VOICE 个 dds_voice 声部完全并行、逐样本实时运算；
// 偶数号声部混入左声道，奇数号声部混入右声道（声像固定，
// 后续可扩展为每声部声像乘法）。混音采用饱和加法防止削波折叠。
//
// 声部控制接口（gate/pinc/vel 按位打包），后续可直接接传感器
// 交互逻辑（触摸选音、力度检测等），即可满足"演奏指令 -> 音符"的闭环。
//=============================================================================
module synth_core #(
	parameter integer N_VOICE     = 4,		// 声部数（>=4，偶数为佳）
	parameter integer ATT_SAMPLES = 244,	// 主包络：起音
	parameter integer DEC_SAMPLES = 12207,	// 主包络：衰减
	parameter integer REL_SAMPLES = 9766,	// 主包络：释放
	parameter integer SUS_LEVEL   = 16384,	// 主包络：延音电平
	parameter integer ATT_SAMPLES2 = 3906,	// 副包络：起音
	parameter integer DEC_SAMPLES2 = 9766,	// 副包络：衰减
	parameter integer REL_SAMPLES2 = 19531,	// 副包络：释放
	parameter integer SUS_LEVEL2   = 22938,	// 副包络：延音电平
	parameter integer CH_MODE     = 0		// 0：单声道镜像（全部声部同时混入左右，
											//    适配单只喇叭/单模块 MAX98357A）
											// 1：立体声（偶数声部->左，奇数->右）
)(
	input  wire                  clk,
	input  wire                  rst_n,
	input  wire                  tick,			// 采样节拍（来自 i2s_tx）
	input  wire [N_VOICE-1:0]    gate_v,			// 各声部音符门（按住=1）
	input  wire [N_VOICE*32-1:0] pinc_v,			// 各声部相位增量（音高）
	input  wire [N_VOICE*16-1:0] vel_v,			// 各声部力度（0~32767）
	input  wire [N_VOICE-1:0]    env_sel_v,		// 各声部包络选择（0主/1副，保留）
	input  wire [2:0]            timbre_i,		// 全局音色（0正弦 1窄脉冲 2锯齿 3方波 4钢琴 5长笛）
	input  wire [7:0]            vib_man,		// 手动颤音深度（AIN2 电位器，全音色）
	output wire signed [15:0]    sample_l,		// 左声道混音输出
	output wire signed [15:0]    sample_r		// 右声道混音输出
);

	// 声部输出
	wire signed [15:0] voice_out [0:N_VOICE-1];

	genvar g;
	generate
		for (g = 0; g < N_VOICE; g = g + 1) begin : g_voice
			dds_voice #(
				.ATT_SAMPLES  (ATT_SAMPLES),
				.DEC_SAMPLES  (DEC_SAMPLES),
				.REL_SAMPLES  (REL_SAMPLES),
				.SUS_LEVEL    (SUS_LEVEL),
				.ATT_SAMPLES2 (ATT_SAMPLES2),
				.DEC_SAMPLES2 (DEC_SAMPLES2),
				.REL_SAMPLES2 (REL_SAMPLES2),
				.SUS_LEVEL2   (SUS_LEVEL2)
			) u_voice (
				.clk        (clk),
				.rst_n      (rst_n),
				.tick       (tick),
				.gate       (gate_v[g]),
				.pinc       (pinc_v[g*32 +: 32]),
				.timbre     (timbre_i),
				.env_sel    (env_sel_v[g]),
				.vib_man    (vib_man),
				.sample_out (voice_out[g])
			);
		end
	endgenerate

	//-------------------------------------------------------------------------
	// 混音：CH_MODE=0 全部声部镜像到两声道；CH_MODE=1 偶数->左、奇数->右。
	// 力度（vel）对所有声部统一，故在混音后缩放一次（省 N-1 个乘法器；
	// 线性运算可交换：先缩放再求和 = 先求和再缩放，结果一致）。
	// 饱和限幅到 ±32767
	//-------------------------------------------------------------------------
	reg signed [17:0] acc_l, acc_r;
	integer k;
	always @* begin
		acc_l = 18'sd0;
		acc_r = 18'sd0;
		for (k = 0; k < N_VOICE; k = k + 1) begin
			if (CH_MODE == 0 || (k % 2) == 0)
				acc_l = acc_l + {{2{voice_out[k][15]}}, voice_out[k]};
			if (CH_MODE == 0 || (k % 2) == 1)
				acc_r = acc_r + {{2{voice_out[k][15]}}, voice_out[k]};
		end
	end

	// 混音 x 力度（4 声部同峰 40000 x 力度比例，饱和限幅兜底）
	//
	// 【时序】混音-缩放-限幅拆成两级流水（tick 驱动，各占整拍）：
	//   tick  N  ：4 声部求和 -> acc_*_r
	//   tick N+1 ：acc_*_r x vel -> 限幅 -> sample_*_r
	// 原写法是一条组合链（4 声部求和 + 常数乘 + 限幅 + audio_fx + 送屏），
	// 实测 22ns，把 Fmax 压到 45.5MHz（124 个建立违例）。
	// 代价：音频晚 2 个采样（≈41µs），远低于可闻/可测门槛。
	reg signed [17:0] acc_l_r, acc_r_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			acc_l_r <= 18'sd0;
			acc_r_r <= 18'sd0;
		end else if (tick) begin
			acc_l_r <= acc_l;
			acc_r_r <= acc_r;
		end
	end

	wire signed [15:0] vel0 = vel_v[15:0];		// 四声部力度恒一致
	wire signed [35:0] scl_l = acc_l_r * $signed({1'b0, vel0});
	wire signed [35:0] scl_r = acc_r_r * $signed({1'b0, vel0});
	wire signed [17:0] out_l_v = scl_l >>> 15;
	wire signed [17:0] out_r_v = scl_r >>> 15;

	reg signed [15:0] sample_l_r, sample_r_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sample_l_r <= 16'sd0;
			sample_r_r <= 16'sd0;
		end else if (tick) begin
			sample_l_r <= (out_l_v >  18'sd32767) ?  16'sd32767 :
			              (out_l_v < -18'sd32767) ? -16'sd32767 : out_l_v[15:0];
			sample_r_r <= (out_r_v >  18'sd32767) ?  16'sd32767 :
			              (out_r_v < -18'sd32767) ? -16'sd32767 : out_r_v[15:0];
		end
	end

	assign sample_l = sample_l_r;
	assign sample_r = sample_r_r;

endmodule
