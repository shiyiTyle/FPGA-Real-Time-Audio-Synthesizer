`timescale 1ns / 1ps
//=============================================================================
// bend_ctrl —— FSR 压力 -> 音高弯音（演奏表情维度：连续音高控制）
//
// 输入 ADS1115 压力码值 press_val。实测接线方向（LM393 模块 AO -> AIN0）：
//   松开 FSR：AO≈3.25V -> press_val≈26000
//   按压 FSR：AO 降到 0V  -> press_val 变小（压死≈0）
// 故压力量 eff = REL_CODE - press_val（松开=0，压死=REL_CODE）。
//
// eff 经斜率限制（防拉链噪声、平滑手指抖动）后线性映射为弯音指数 idx：
//   idx 0..255 对应 0..12 半音，BEND_SEMI 决定实际上限（满压力升高几个半音）
// 查 32 段线性插值表得频率比例因子 F（Q16 格式，65536=1.0 ~ 131072=2.0），
// 四声部相位增量统一缩放：pinc_out = pinc_in x F >> 16。
// 松开时 F=65536 精确等于 1.0，本音音分零误差；和弦各声部同乘 F，相对音程不变。
//
// 看门狗：ADS1115 若 WD_MS 毫秒无更新脉冲（掉线/未初始化），eff 目标强制 0，
// 全琴自动回到本音——杜绝"断线后全体飙音"的失效模式。
//
// 【DSP 节省】四个声部的 32x18 乘法共用 1 个乘法器：tick 后分 4 拍依次算
//   （采样周期 1024 clk，占用 4 拍），结果存 pinc_q 供声部在下一 tick 使用
//   ——弯音整体晚一个采样生效（ms 级斜坡，完全不可闻）。数学结果与并行版
//   逐位一致。
//=============================================================================
module bend_ctrl #(
	parameter [15:0] REL_CODE   = 16'd26000,	// 松开 FSR 时的 ADS 读数（实测 3.25V）
	parameter integer BEND_SEMI = 2,			// 满压力升高半音数 0~12（0=关闭弯音）
	parameter [15:0] SLEW_STEP  = 16'd150,		// 每采样节拍 eff 最大变化（全程≈3.5ms）
	parameter integer WD_MS     = 20			// 看门狗超时 ms（无 upd 视为掉线）
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        tick,		// 采样节拍（48.8kHz，来自 i2s_tx）
	input  wire        upd,		// ADS1115 新值脉冲
	input  wire [15:0] press_val,
	input  wire [127:0] pinc_in,	// 4 声部原始相位增量（低位=声部0）
	output wire [127:0] pinc_out	// 弯音后相位增量
);

	//-------------------------------------------------------------------------
	// 压力量化：eff = REL_CODE - press_val（按压=读数变小，方向按实测）
	//-------------------------------------------------------------------------
	wire [15:0] drop = (press_val < REL_CODE) ? (REL_CODE - press_val) : 16'd0;

	// 看门狗：upd 停更 -> 目标归零（复位值=超时，首个 upd 到来前保持本音）
	// WD_MS 有效范围 1~80（22 位计数上限 4.19M 拍 = 83ms）
	localparam [21:0] WD_LIMIT_W = WD_MS * 50000;	// 50MHz clk 每 ms 5 万拍
	reg [21:0] wd_cnt;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			wd_cnt <= WD_LIMIT_W;
		else if (upd)
			wd_cnt <= 22'd0;
		else if (wd_cnt < WD_LIMIT_W)
			wd_cnt <= wd_cnt + 1'b1;
	end
	wire        wd_dead = (wd_cnt >= WD_LIMIT_W);
	wire [15:0] eff_tgt = wd_dead ? 16'd0 : drop;

	//-------------------------------------------------------------------------
	// 斜率限制：eff 每采样节拍最多走 SLEW_STEP（≈3.5ms 走满全程）
	//-------------------------------------------------------------------------
	reg [15:0] eff_q;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			eff_q <= 16'd0;
		else if (tick) begin
			if (eff_q < eff_tgt)
				eff_q <= ((eff_tgt - eff_q) > SLEW_STEP) ? (eff_q + SLEW_STEP) : eff_tgt;
			else if (eff_q > eff_tgt)
				eff_q <= ((eff_q - eff_tgt) > SLEW_STEP) ? (eff_q - SLEW_STEP) : eff_tgt;
		end
	end

	//-------------------------------------------------------------------------
	// eff -> 弯音指数 idx：0..255 线性对应 0..12 半音
	//   idx = eff x BEND_SEMI/12 x 256 / REL_CODE，增益编译期算好
	//-------------------------------------------------------------------------
	localparam integer IDX_GAIN = (BEND_SEMI * 32'd16777216) / (12 * REL_CODE);
	wire [25:0] prod_i = eff_q * IDX_GAIN;			// 数学上限 2^24，26 位够
	wire [9:0]  idx10  = prod_i[25:16];
	wire [7:0]  idx    = (idx10 > 10'd255) ? 8'd255 : idx10[7:0];

	//-------------------------------------------------------------------------
	// F 查表（32 段线性插值）：F(k/32 八度) = round(65536 x 2^(k/32))，k=0..32
	//-------------------------------------------------------------------------
	function [17:0] lut_f;
		input [5:0] k;
		case (k)
			6'd0 : lut_f = 18'd65536;
			6'd1 : lut_f = 18'd66971;
			6'd2 : lut_f = 18'd68438;
			6'd3 : lut_f = 18'd69936;
			6'd4 : lut_f = 18'd71468;
			6'd5 : lut_f = 18'd73032;
			6'd6 : lut_f = 18'd74632;
			6'd7 : lut_f = 18'd76266;
			6'd8 : lut_f = 18'd77936;
			6'd9 : lut_f = 18'd79642;
			6'd10: lut_f = 18'd81386;
			6'd11: lut_f = 18'd83169;
			6'd12: lut_f = 18'd84990;
			6'd13: lut_f = 18'd86851;
			6'd14: lut_f = 18'd88752;
			6'd15: lut_f = 18'd90696;
			6'd16: lut_f = 18'd92682;
			6'd17: lut_f = 18'd94711;
			6'd18: lut_f = 18'd96785;
			6'd19: lut_f = 18'd98905;
			6'd20: lut_f = 18'd101070;
			6'd21: lut_f = 18'd103283;
			6'd22: lut_f = 18'd105545;
			6'd23: lut_f = 18'd107856;
			6'd24: lut_f = 18'd110218;
			6'd25: lut_f = 18'd112631;
			6'd26: lut_f = 18'd115098;
			6'd27: lut_f = 18'd117618;
			6'd28: lut_f = 18'd120194;
			6'd29: lut_f = 18'd122825;
			6'd30: lut_f = 18'd125515;
			6'd31: lut_f = 18'd128263;
			6'd32: lut_f = 18'd131072;
			default: lut_f = 18'd65536;
		endcase
	endfunction

	wire [5:0]  k_hi  = {1'b0, idx[7:3]};			// 粗档 0..32（+1 后不溢出）
	wire [17:0] f_lo  = lut_f(k_hi);
	wire [17:0] f_hi  = lut_f(k_hi + 6'd1);
	// 【时序】查表结果先寄存一拍再插值：原"32 选 1 查表 + 乘法 + 加"是一条
	// 18.8ns 的组合链（slack 1.16ns，限制 Fmax 到 53MHz）。弯音量是慢变
	// 信号，多一拍（20ns）无影响。
	reg  [17:0] f_lo_r, f_hi_r;
	reg  [2:0]  ffrac;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			f_lo_r <= 18'd65536;
			f_hi_r <= 18'd65536;
			ffrac  <= 3'd0;
		end else begin
			f_lo_r <= f_lo;
			f_hi_r <= f_hi;
			ffrac  <= idx[2:0];
		end
	end
	wire [17:0] f_delta = f_hi_r - f_lo_r;
	wire [20:0] f_mul   = f_delta * {18'd0, ffrac};
	reg  [17:0] f_reg;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_reg <= 18'd65536;		// 复位到 1.0（无弯音），否则 X 污染 pinc 链
		else
			f_reg <= f_lo_r + (f_mul >> 3);
	end

	//-------------------------------------------------------------------------
	// 四声部 pinc x F >> 16 —— 单乘法器时分复用（tick 后 4 拍算完）
	//-------------------------------------------------------------------------
	reg  [1:0]  bvi;			// 声部索引
	reg  [1:0]  brun;			// 0=空闲，1..4=正在算第 brun 拍，3=等 f_reg 稳定一拍
	reg  [31:0] pinq [0:3];
	wire [127:0] pin_sh = pinc_in >> {bvi, 5'b00000};	// bvi*32 位右移
	wire [31:0]  pin_sel = pin_sh[31:0];

	wire [49:0] bprod = pin_sel * f_reg;		// 32x18，唯一乘法器
	wire [31:0] bsat  = (|bprod[49:48]) ? 32'hFFFF_FFFF : bprod[47:16];

	integer bi;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			bvi <= 2'd0; brun <= 2'd0;
			for (bi = 0; bi < 4; bi = bi + 1) pinq[bi] <= 32'd0;
		end else begin
			if (tick) begin
				bvi <= 2'd0; brun <= 2'd3;		// 先等一拍让 f_reg 稳定
			end else if (brun == 2'd3) begin
				brun <= 2'd1;
			end else if (brun != 2'd0) begin
				pinq[bvi] <= bsat;
				bvi  <= bvi + 2'd1;
				brun <= (bvi == 2'd3) ? 2'd0 : brun + 2'd1;
			end
		end
	end

	genvar gj;
	generate
		for (gj = 0; gj < 4; gj = gj + 1) begin : g_out
			assign pinc_out[gj*32 +: 32] = pinq[gj];
		end
	endgenerate

endmodule
