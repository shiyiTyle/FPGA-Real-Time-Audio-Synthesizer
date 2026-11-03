`timescale 1ns / 1ps
//=============================================================================
// dds_voice —— 单声部 DDS 波形发生器（正弦插值表 + ADSR + 乐器音色合成）
//
// 基本原理（DDS 直接数字频率合成）：
//   32 位相位累加器每个采样节拍加一次相位增量 PINC，
//   相位高位查 1/4 周期正弦表并做线性插值，得到高纯度正弦波；
//   再逐采样乘包络与力度，形成完整的"音符"。
//
// 音色（timbre 输入全局选择 0~5）：
//   0 正弦（f，插值查表，纯净柔和）  1 窄脉冲（2f，25% 占空，尖亮短笛）
//   2 锯齿（f，明亮铜管）            3 方波（f/2，厚重贝斯）
//   4 钢琴：加法合成 f + 2f + 3f 三个分量，各自独立衰减——起音满幅
//      （亮攻击，基波 x5/8、2f/3f 相对占比 ≈0.73/0.44，2f ≈0.3s、
//      3f ≈0.13s 先后消失），基频 ~1.3s 自然衰减到无声（真钢琴无延音、
//      按住也不持续），松键阻尼快速收尾
//   5 长笛：正弦 f + ~15% 二次谐波 + 5.5Hz 颤音（进入延音后 ~0.4s 渐入，
//      模拟演奏习惯）+ 起音气声（LFSR 噪声 ~25ms 短促衰减，吐气 chiff），
//      70ms 柔起音、满延音、~135ms 释放
//
// 【DSP 节省】每声部乘法 7 -> 4 个：
//   1) 力度乘法移除——力度对所有声部恒定，改在 synth_core 混音后统一
//      缩放一次（线性运算可交换，结果逐位一致）；
//   2) 长笛二次谐波系数 4915 是常数 -> 移位加实现（精确无误差）；
//   3) 颤音乘法 pinc 截 27 位（pinc>>>5 后乘，>>>10 补回，数学等价），
//      27x16 只需 1 个 18x27 DSP；该乘法器两拍轮转、兼做电位器振幅颤音
//      （wave_e x trem_factor），仍零新增 DSP；
//   4) 钢琴 2f/3f 与长笛气声三个乘法共用 1 个（tick 后 2~4 拍轮转，
//      分量晚一个采样生效——20µs 相位差不可闻）。
//
// 相位-频率换算（fs = 50MHz/16/64 = 48828.125 Hz）：
//   PINC = 2^32 x f / fs
//
// 逐样本流水线（tick 后 ~5 个 clk 输出稳定，保持到下一个 tick）：
//   T+0  相位累加（长笛颤音偏移在此叠加）+ 包络/乐器状态机
//   T+1  查表地址寄存（基频 + 2f/3f 分量相位）
//   T+2  波形查表（基频折叠 y0/y1 + 分量直读 sin2/sin3 + LFSR 噪声）
//   T+3  波形选择 / 乐器分量叠加 -> wave_i
//   T+4  x 包络 -> wave_e
//   T+5  限幅 -> sample_out
//=============================================================================
module dds_voice #(
	parameter integer ATT_SAMPLES  = 244,	// 主包络·起音（样本数）
	parameter integer DEC_SAMPLES  = 12207,	// 主包络·衰减
	parameter integer REL_SAMPLES  = 9766,	// 主包络·释放
	parameter integer SUS_LEVEL    = 16384,	// 主包络·延音电平 0~32767
	parameter integer ATT_SAMPLES2 = 3906,	// 副包络·起音（保留，当前未选）
	parameter integer DEC_SAMPLES2 = 9766,
	parameter integer REL_SAMPLES2 = 19531,
	parameter integer SUS_LEVEL2   = 22938
)(
	input  wire               clk,
	input  wire               rst_n,
	input  wire               tick,		// 采样节拍，1 个 clk 宽脉冲
	input  wire               gate,		// 1 = 音符按住，0 = 松开（进入释放）
	input  wire [31:0]        pinc,		// 相位增量（决定音高）
	input  wire [2:0]         timbre,	// 0正弦 1窄脉冲 2锯齿 3方波 4钢琴 5长笛
	input  wire [7:0]         vib_man,	// 手动颤音深度 0~255（AIN2 电位器，全音色生效，0=关）
	input  wire               env_sel,	// 保留：原始波形双套包络选择
	output reg  signed [15:0] sample_out	// 声部输出样本
);

	//-------------------------------------------------------------------------
	// 主/副包络步进量（原始波形用；步进向上取整，时间为"不超过"标称时长）
	//-------------------------------------------------------------------------
	localparam integer SUS_LVL   = (SUS_LEVEL  > 32767) ? 32767 : SUS_LEVEL;
	localparam integer A_STEP    = (32767 + ATT_SAMPLES  - 1) / ATT_SAMPLES;
	localparam integer D_DELTA   = (SUS_LEVEL  < 32767) ? (32767 - SUS_LEVEL)  : 1;
	localparam integer D_STEP    = (D_DELTA    + DEC_SAMPLES  - 1) / DEC_SAMPLES;
	localparam integer R_STEP    = (32767 + REL_SAMPLES  - 1) / REL_SAMPLES;

	localparam integer SUS_LVL2  = (SUS_LEVEL2 > 32767) ? 32767 : SUS_LEVEL2;
	localparam integer A_STEP2   = (32767 + ATT_SAMPLES2 - 1) / ATT_SAMPLES2;
	localparam integer D_DELTA2  = (SUS_LEVEL2 < 32767) ? (32767 - SUS_LEVEL2) : 1;
	localparam integer D_STEP2   = (D_DELTA2   + DEC_SAMPLES2 - 1) / DEC_SAMPLES2;
	localparam integer R_STEP2   = (32767 + REL_SAMPLES2 - 1) / REL_SAMPLES2;

	//-------------------------------------------------------------------------
	// 钢琴音色参数（timbre=4）。指数衰减实现：env -= env>>K（|1 兜底），
	// τ = 2^K 个样本；-40dB ≈ 4.6τ，-60dB ≈ 6.9τ。调音改这里的 K 与初值。
	//-------------------------------------------------------------------------
	localparam integer PIANO_A_STEP  = (32767 + 98 - 1) / 98;	// 起音 ≈2ms（线性）
	localparam integer PIANO_DEC_K   = 13;		// 基频衰减：-60dB ≈1.3s
	localparam integer PIANO_REL_K   = 10;		// 松键阻尼：-60dB ≈0.29s
	localparam integer PIANO_DEAD    = 48;		// 包络低于此视为已衰减完
	localparam [15:0]  PIANO_E2_INIT = 16'd15000;	// 2f 分量初始幅度（≈0.46 满度，占基波 ≈73%）
	localparam [15:0]  PIANO_E3_INIT = 16'd9000;	// 3f 分量初始幅度（≈0.27 满度，占基波 ≈44%）
	localparam integer PIANO_E2_K    = 13;		// 2f 衰减（15000<2^14 → ≈1/采样线性，≈0.31s 到零）
	localparam integer PIANO_E3_K    = 12;		// 3f 衰减（先 >>2 指数后线性兜底，≈0.13s 到零）

	//-------------------------------------------------------------------------
	// 长笛音色参数（timbre=5）。调音改这里。
	//-------------------------------------------------------------------------
	localparam integer FLUTE_A_STEP  = (32767 + 3418 - 1) / 3418;	// 起音 ≈67ms（线性）
	localparam integer FLUTE_R_STEP  = (32767 + 7324 - 1) / 7324;	// 释放 ≈134ms
	localparam [31:0]  VIB_PINC      = 32'd481038;	// 颤音频率 5.5Hz
	localparam integer VIB_RAMP_TICKS = 77;		// 深度每 +1 的 tick 数（255 步 ≈0.4s）
	localparam integer VIB_SHIFT     = 15;		// （保留）满深度偏移 = pinc x 255/32768 ≈ ±13.5 音分
	localparam [15:0]  FLUTE_H2      = 16'd4915;	// 二次谐波幅度（≈15%，过高会在基频负半周推过零）
	localparam [15:0]  CHIFF_INIT    = 16'd2400;	// 起音气声初始幅度
	localparam [15:0]  CHIFF_STEP    = 16'd2;		// 气声每 tick 衰减（≈25ms 消失）

	wire use_piano = (timbre == 3'd4);
	wire use_flute = (timbre == 3'd5);

	// 有效包络参数：乐器音色自带包络，原始波形用主/副两套（env_sel 选择）
	wire [31:0] a_step_w = use_piano ? PIANO_A_STEP :
	                       use_flute ? FLUTE_A_STEP :
	                       env_sel   ? A_STEP2       : A_STEP;
	wire [31:0] r_step_w = use_flute ? FLUTE_R_STEP :
	                       env_sel   ? R_STEP2       : R_STEP;
	wire [31:0] sus_tgt_w = use_flute ? 32767 :
	                        env_sel   ? SUS_LVL2     : SUS_LVL;
	wire [31:0] d_step_w = use_flute ? 1 :
	                       env_sel   ? D_STEP2       : D_STEP;

	//-------------------------------------------------------------------------
	// 包络状态机（原始波形=线性 ADSR；钢琴=快攻+指数衰减到无声+阻尼释放；
	// 长笛=柔攻+满延音+中速释放，共用一个状态机）
	//-------------------------------------------------------------------------
	localparam [2:0] ENV_IDLE = 3'd0, ENV_ATT = 3'd1, ENV_DEC = 3'd2,
	                 ENV_SUS  = 3'd3, ENV_REL = 3'd4;

	reg [2:0]  env_state;
	reg [15:0] env;

	wire [15:0] piano_dec = (env >> PIANO_DEC_K) | 16'd1;
	wire [15:0] piano_rel = (env >> PIANO_REL_K) | 16'd1;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			env_state <= ENV_IDLE;
			env       <= 16'd0;
		end else if (tick) begin
			case (env_state)
				ENV_IDLE:
					if (gate) begin
						env_state <= ENV_ATT;
						env       <= env + a_step_w[15:0];
					end
				ENV_ATT: begin
					if (!gate)
						env_state <= ENV_REL;
					else if (env >= 32767 - a_step_w) begin
						env       <= 16'd32767;
						env_state <= ENV_DEC;
					end else
						env <= env + a_step_w[15:0];
				end
				ENV_DEC: begin
					if (!gate)
						env_state <= ENV_REL;
					else if (use_piano) begin
						// 钢琴：指数衰减到无声（无延音段）
						if (env <= piano_dec + PIANO_DEAD) begin
							env       <= 16'd0;
							env_state <= ENV_SUS;
						end else
							env <= env - piano_dec;
					end else if (env <= sus_tgt_w + d_step_w) begin
						env       <= sus_tgt_w[15:0];
						env_state <= ENV_SUS;
					end else
						env <= env - d_step_w[15:0];
				end
				ENV_SUS:
					if (!gate)
						env_state <= ENV_REL;
					else if (use_piano && env != 16'd0) begin
						// 音色切到钢琴时可能停在旧延音电平：继续指数衰减收尾
						if (env <= piano_dec)
							env <= 16'd0;
						else
							env <= env - piano_dec;
					end
				ENV_REL: begin
					if (gate)
						env_state <= ENV_ATT;			// 重触发
					else if (use_piano) begin
						if (env <= piano_rel) begin
							env       <= 16'd0;
							env_state <= ENV_IDLE;
						end else
							env <= env - piano_rel;
					end else if (env <= r_step_w) begin
						env       <= 16'd0;
						env_state <= ENV_IDLE;
					end else
						env <= env - r_step_w[15:0];
				end
				default: begin
					env_state <= ENV_IDLE;
					env       <= 16'd0;
				end
			endcase
		end
	end

	//-------------------------------------------------------------------------
	// 1/4 周期正弦表（0°~90°，64 点，满量程 32767）
	//-------------------------------------------------------------------------
	function signed [15:0] sin_lut_q;
		input [5:0] a;
		begin
			case (a)
				6'd0 : sin_lut_q = 16'sd0;
				6'd1 : sin_lut_q = 16'sd804;
				6'd2 : sin_lut_q = 16'sd1608;
				6'd3 : sin_lut_q = 16'sd2410;
				6'd4 : sin_lut_q = 16'sd3212;
				6'd5 : sin_lut_q = 16'sd4011;
				6'd6 : sin_lut_q = 16'sd4808;
				6'd7 : sin_lut_q = 16'sd5602;
				6'd8 : sin_lut_q = 16'sd6393;
				6'd9 : sin_lut_q = 16'sd7179;
				6'd10: sin_lut_q = 16'sd7962;
				6'd11: sin_lut_q = 16'sd8739;
				6'd12: sin_lut_q = 16'sd9512;
				6'd13: sin_lut_q = 16'sd10278;
				6'd14: sin_lut_q = 16'sd11039;
				6'd15: sin_lut_q = 16'sd11793;
				6'd16: sin_lut_q = 16'sd12539;
				6'd17: sin_lut_q = 16'sd13279;
				6'd18: sin_lut_q = 16'sd14010;
				6'd19: sin_lut_q = 16'sd14732;
				6'd20: sin_lut_q = 16'sd15446;
				6'd21: sin_lut_q = 16'sd16151;
				6'd22: sin_lut_q = 16'sd16846;
				6'd23: sin_lut_q = 16'sd17530;
				6'd24: sin_lut_q = 16'sd18204;
				6'd25: sin_lut_q = 16'sd18868;
				6'd26: sin_lut_q = 16'sd19519;
				6'd27: sin_lut_q = 16'sd20159;
				6'd28: sin_lut_q = 16'sd20787;
				6'd29: sin_lut_q = 16'sd21403;
				6'd30: sin_lut_q = 16'sd22005;
				6'd31: sin_lut_q = 16'sd22594;
				6'd32: sin_lut_q = 16'sd23170;
				6'd33: sin_lut_q = 16'sd23731;
				6'd34: sin_lut_q = 16'sd24279;
				6'd35: sin_lut_q = 16'sd24811;
				6'd36: sin_lut_q = 16'sd25329;
				6'd37: sin_lut_q = 16'sd25832;
				6'd38: sin_lut_q = 16'sd26319;
				6'd39: sin_lut_q = 16'sd26790;
				6'd40: sin_lut_q = 16'sd27245;
				6'd41: sin_lut_q = 16'sd27683;
				6'd42: sin_lut_q = 16'sd28105;
				6'd43: sin_lut_q = 16'sd28510;
				6'd44: sin_lut_q = 16'sd28898;
				6'd45: sin_lut_q = 16'sd29268;
				6'd46: sin_lut_q = 16'sd29621;
				6'd47: sin_lut_q = 16'sd29956;
				6'd48: sin_lut_q = 16'sd30273;
				6'd49: sin_lut_q = 16'sd30571;
				6'd50: sin_lut_q = 16'sd30852;
				6'd51: sin_lut_q = 16'sd31113;
				6'd52: sin_lut_q = 16'sd31356;
				6'd53: sin_lut_q = 16'sd31580;
				6'd54: sin_lut_q = 16'sd31785;
				6'd55: sin_lut_q = 16'sd31971;
				6'd56: sin_lut_q = 16'sd32137;
				6'd57: sin_lut_q = 16'sd32285;
				6'd58: sin_lut_q = 16'sd32412;
				6'd59: sin_lut_q = 16'sd32521;
				6'd60: sin_lut_q = 16'sd32609;
				6'd61: sin_lut_q = 16'sd32678;
				6'd62: sin_lut_q = 16'sd32728;
				6'd63: sin_lut_q = 16'sd32757;
				default: sin_lut_q = 16'sd0;
			endcase
		end
	endfunction

	// 整周期折叠查表：cyc[7:6]=象限，奇象限地址镜像，高半周期取反
	function signed [15:0] sin_fold;
		input [7:0] cyc;
		reg [1:0] quad;
		reg [5:0] aq;
		reg [5:0] eff;
		begin
			quad = cyc[7:6];
			aq   = cyc[5:0];
			eff  = quad[0] ? (6'd63 - aq) : aq;
			sin_fold = quad[1] ? -sin_lut_q(eff) : sin_lut_q(eff);
		end
	endfunction

	//-------------------------------------------------------------------------
	// 相位累加（长笛颤音偏移在此叠加；静音期冻结相位，降低翻转率）
	//-------------------------------------------------------------------------
	reg  [31:0] phase_acc;
	reg         sub_ff;		// 副八度极性（低八度方波用，相位高字节回绕时翻转）
	reg  [31:0] vib_phase;	// 颤音 LFO 相位（自由运行）
	reg  [7:0]  vib_depth;	// 颤音深度 0~255（进延音后渐入）

		// 三角波 LFO（相位满周期往返，听感与正弦 LFO 无差异，省一次查表）
		wire [7:0]  tri_u = vib_phase[31] ? ~vib_phase[30:23] : vib_phase[30:23];
		wire signed [8:0] tri9 = $signed({1'b0, tri_u}) - 9'sd128;		// -128..127
		wire signed [17:0] sd_prod = $signed(tri9) * $signed({1'b0, vib_depth});
		wire signed [10:0] sdepth  = sd_prod >>> 7;						// ±254（长笛内置 ≈±13.3 音分）

		//-------------------------------------------------------------------------
		// 【2026-10-31 末改版】电位器（A2/vib_man）由"音高偏移"改为**振幅颤音**
		// （tremolo）：音高不变，音量随 5.5Hz LFO 像揉弦一样"呼吸"起伏，听感
		// 比音高颤音明显得多。长笛内置音高颤音保留不变（仅长笛、仍走 LFO）。
		// 共享乘法器（原 vo_prod，27x18 单 DSP）改为两拍轮转，零新增 DSP：
		//   拍A 音高颤音（仅长笛）：(pinc>>>5) x sdepth → vib_off（与原来一致）
		//   拍B 振幅颤音（全音色）：wave_e x trem_factor → trem_r
		// trem_factor = 16384 - vib_man x tri9 >> 3（零均值，音量 ±25% 摆动、
		//   旋钮到底也不断音；vib_man x tri9 是 9x9 小乘法，LUT 实现不占 DSP）
		//-------------------------------------------------------------------------
		wire signed [17:0] trem        = $signed({1'b0, vib_man}) * tri9;	// ±255x128
		wire signed [15:0] trem_factor = 16'sd16384 - (trem >>> 3);	// [12336, 20432]

		reg         m_ph;			// 0=乘数当前是音高组，1=乘数当前是振幅组
		reg  signed [27:0] m_a;
		reg  signed [15:0] m_b;
		wire signed [43:0] m_prod = m_a * m_b;
		reg  signed [43:0] m_res;
		reg  signed [26:0] vib_off;
		reg  signed [16:0] trem_r;

		always @(posedge clk or negedge rst_n) begin
			if (!rst_n) begin
				m_ph <= 1'b0; m_a <= 28'sd0; m_b <= 16'sd0; m_res <= 44'sd0;
				vib_off <= 27'sd0; trem_r <= 17'sd0;
			end else begin
				m_res <= m_prod;
				if (m_ph) begin
					// 乘数当前是音高组：m_res 正在收音高乘积（下一拍才可读），
					// 现在可读的旧 m_res 是上一拍的振幅乘积 -> 收进 trem_r；
					// 本拍发振幅组乘数
					trem_r <= m_res >>> 14;
					m_a <= $signed({{11{wave_e[16]}}, wave_e});
					m_b <= trem_factor;
				end else begin
					// 乘数当前是振幅组：旧 m_res 是上一拍的音高乘积 -> 收进
					// vib_off；本拍发音高组乘数（仅长笛非零）
					vib_off <= m_res >>> 10;
					m_a <= $signed({1'b0, pinc[31:5]});
					m_b <= use_flute ? $signed({{5{sdepth[10]}}, sdepth}) : 16'sd0;
				end
				m_ph <= ~m_ph;
			end
		end
	wire signed [32:0] psum = {1'b0, phase_acc} + {1'b0, pinc}
	                        + {{6{vib_off[26]}}, vib_off};
	wire [31:0] phase_new = psum[31:0];
	wire [31:0] p2_phase  = phase_new << 1;					// 2f 分量相位
	wire [31:0] p3_phase  = (phase_new << 1) + phase_new;	// 3f 分量相位

	// 流水线第 1 级寄存器：基频/分量查表地址 + 插值小数
	reg [7:0] cyc0;
	reg [7:0] cyc1;
	reg [7:0] cyc_p2;
	reg [7:0] cyc_p3;
	reg [7:0] frac;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			phase_acc <= 32'd0;
			sub_ff    <= 1'b0;
			vib_phase <= 32'd0;
			cyc0      <= 8'd0;
			cyc1      <= 8'd1;
			cyc_p2    <= 8'd0;
			cyc_p3    <= 8'd0;
			frac      <= 8'd0;
		end else begin
			// 【修复】VIB_PINC 是按采样节拍（48.8kHz）算的 5.5Hz；原来放在
			// tick 外每 clk 累加，实际颤音频率 = 5.5Hz x 1024 ≈ 5.6kHz，
			// 听感是"滋滋"的粗糙颤音、屏上波形发毛。必须每采样推进一次。
			if (tick) begin
				vib_phase <= vib_phase + VIB_PINC;
				if (env_state != ENV_IDLE || gate) begin
					phase_acc <= psum[31:0];
					// 相位高字节回绕 = 走完一个基频周期 -> 翻转 f/2 极性
					if (phase_new[31:24] < cyc0)
						sub_ff <= ~sub_ff;
				end
				cyc0   <= phase_new[31:24];
				cyc1   <= phase_new[31:24] + 8'd1;	// 8 位自然回绕，跨象限连续
				cyc_p2 <= p2_phase[31:24];
				cyc_p3 <= p3_phase[31:24];
				frac   <= phase_new[23:16];
			end
		end
	end

	//-------------------------------------------------------------------------
	// 钢琴分量包络 e2/e3 + 长笛气声 chiff + 颤音深度渐入 + LFSR 噪声
	// （指数衰减：e -= e>>K，|1 兜底保证衰减到 0）
	//-------------------------------------------------------------------------
	reg [15:0] e2;
	reg [15:0] e3;
	reg [15:0] chiff;
	reg [15:0] lfsr;
	reg        gate_d;
	reg [6:0]  vib_pace;

	wire [15:0] e2_dec = (e2 >> PIANO_E2_K) | 16'd1;
	wire [15:0] e3_dec = (e3 >> PIANO_E3_K) | 16'd1;
	wire        gate_rise = gate & ~gate_d;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			e2       <= 16'd0;
			e3       <= 16'd0;
			chiff    <= 16'd0;
			lfsr     <= 16'hACE1;
			gate_d   <= 1'b0;
			vib_pace <= 7'd0;
			vib_depth<= 8'd0;		// 颤音深度唯一驱动块在此（避免多驱动 EX2000）
		end else begin
			gate_d <= gate;
			if (tick) begin
				lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
				// 【修复 2026-10-31】e2/e3/chiff 衰减原来写在 tick 外，每个
				// 50MHz 时钟都减一次，比设计意图（按采样节拍）快 1024 倍：
				// 谐波 <0.4ms 就死光，听上去是纯正弦 + 一声咔哒。必须按采样
				// 节拍推进。
				if (e2 != 16'd0)
					e2 <= (e2 > e2_dec) ? (e2 - e2_dec) : 16'd0;
				if (e3 != 16'd0)
					e3 <= (e3 > e3_dec) ? (e3 - e3_dec) : 16'd0;
				if (chiff > CHIFF_STEP)
					chiff <= chiff - CHIFF_STEP;
				else
					chiff <= 16'd0;
				// 颤音深度渐入：长笛进入延音段后每 VIB_RAMP_TICKS 拍 +1
				if (use_flute && gate && env_state == ENV_SUS) begin
					if (vib_pace == VIB_RAMP_TICKS - 1) begin
						vib_pace <= 7'd0;
						if (vib_depth != 8'd255)
							vib_depth <= vib_depth + 8'd1;
					end else
						vib_pace <= vib_pace + 7'd1;
				end
			end
			// 【修复 2026-10-31】乐器分量包络的初始化原来也写在 tick 内：
			// gate_rise 只有 1 个 clk 宽，与 tick（每 1024 拍 1 个）重合的
			// 概率 ≈1/1024，实际演奏中谐波/气声几乎从不初始化——这才是
			// "钢琴听上去和正弦没区别"的真正根因（比衰减快 1024 倍更致命）。
			// 初始化只发生在按键瞬间、与采样节拍无关，必须无条件执行。
			if (gate_rise) begin
				e2       <= PIANO_E2_INIT;
				e3       <= PIANO_E3_INIT;
				chiff    <= CHIFF_INIT;
				vib_depth<= 8'd0;
				vib_pace <= 7'd0;
			end
		end
	end

	//-------------------------------------------------------------------------
	// 流水线第 2 级：折叠查表 y0/y1（基频插值用）+ 分量直读 sin2/sin3
	// 各级输入寄存器在非 tick 拍保持不变，重复计算结果相同，无需 valid 标记
	//-------------------------------------------------------------------------
	reg  signed [15:0] y0, y1;
	reg  signed [15:0] sin2, sin3;
	reg  signed [15:0] noise;		// LFSR 噪声（有符号）

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			y0    <= 16'sd0;
			y1    <= 16'sd0;
			sin2  <= 16'sd0;
			sin3  <= 16'sd0;
			noise <= 16'sd0;
		end else begin
			y0    <= sin_fold(cyc0);
			y1    <= sin_fold(cyc1);
			sin2  <= sin_fold(cyc_p2);
			sin3  <= sin_fold(cyc_p3);
			noise <= $signed(lfsr);
		end
	end

	//-------------------------------------------------------------------------
	// 乐器分量共享乘法器（钢琴 2f/3f + 长笛气声三合一，tick 后轮转）
	//   钢琴：拍 1 发 sin2 x e2 -> 拍 2 收 p2_r、发 sin3 x e3 -> 拍 3 收 p3_r
	//   长笛：拍 1 发 noise x chiff -> 拍 2 收 ch_r
	//   分量比基频晚一个采样（20µs，谐波相位差 <0.1rad，不可闻）
	//-------------------------------------------------------------------------
	reg  signed [15:0] ia;
	reg  signed [16:0] ib;
	wire signed [32:0] iprod = ia * ib;
	reg  signed [32:0] ires;
	reg  [1:0]  ist;			// 0 空闲；1/2 钢琴两拍；3/4 长笛两拍
	reg  signed [15:0] p2_r, p3_r, ch_r;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ia <= 16'sd0; ib <= 17'sd0; ires <= 33'sd0; ist <= 2'd0;
			p2_r <= 16'sd0; p3_r <= 16'sd0; ch_r <= 16'sd0;
		end else begin
			ires <= iprod;
			case (ist)
				2'd0: begin						// 发起：本 tick 的分量乘法
					if (use_piano) begin
						ia  <= sin2;			// 上一采样的 sin2（晚 1 拍，不可闻）
						ib  <= $signed({1'b0, e2});
						ist <= 2'd1;
					end else if (use_flute) begin
						ia  <= noise;
						ib  <= $signed({1'b0, chiff});
						ist <= 2'd1;
					end
				end
				2'd1: begin						// ires 已是第 1 拍乘积；发起第 2 拍
					if (use_piano) begin
						ia  <= sin3;
						ib  <= $signed({1'b0, e3});
					end
					ist <= 2'd2;
				end
				2'd2: begin						// 收第 1 拍结果
					if (use_piano) begin
						p2_r <= ires >>> 15;
						ist  <= 2'd3;
					end else begin
						ch_r <= ires >>> 15;
						ist  <= 2'd0;
					end
				end
				default: begin					// 2'd3：收第 2 拍结果（钢琴 p3）
					p3_r <= ires >>> 15;
					ist  <= 2'd0;
				end
			endcase
		end
	end

	//-------------------------------------------------------------------------
	// 流水线第 3 级：波形选择 / 乐器分量叠加
	//-------------------------------------------------------------------------
	// 波形源 1：正弦（插值查表路径，限幅仅波峰附近可能越界一个点）
	// 【定点】插值式必须 $signed 包住拼接：`{y0[15],y0}` 是 unsigned，
	// 与负的 (prod>>>8) 相加会按无符号规则扩展，实测 50.8% 的样本算成
	// ±13 万的错值、被限幅到 ±32767——一半采样打到电源轨，
	// 听感是"正弦≈方波"、长笛发毛，屏上是锯齿状残留。
	wire signed [16:0] diff = {y1[15], y1} - {y0[15], y0};
	wire signed [24:0] prod = diff * $signed({1'b0, frac});
	wire signed [17:0] interp_sum = $signed({y0[15], y0}) + (prod >>> 8);
	reg  signed [17:0] interp_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) interp_r <= 18'sd0;
		else        interp_r <= interp_sum;
	end
	wire signed [15:0] wave_sin = (interp_r > 18'sd32767) ? 16'sd32767 :
	                              (interp_r < -18'sd32767) ? -16'sd32767 :
	                              interp_r[15:0];

	// 波形源（除正弦外均由相位位直接生成，无乘法）
	// T1 高八度窄脉冲：相位[30:23] 构成 2f 的 8 位周期，25% 占空比
	wire [7:0]         cyc2f    = {cyc0[6:0], frac[7]};
	wire signed [15:0] wave_pls = (cyc2f < 8'd64) ? 16'sd32767 : -16'sd32767;
	// T2 锯齿（基频 f）
	wire signed [15:0] wave_saw = {cyc0, 8'b0};
	// T3 低八度方波：每两个基频周期翻转一次极性（f/2）
	wire signed [15:0] wave_sub = sub_ff ? 16'sd32767 : -16'sd32767;

		// 【电平】长笛基波压到 3/4（sin_3q）：正弦本身满幅（±32757），叠加
		// 15% 二次谐波会超过满量程被限幅（实测持续段 16% 采样削波、听感
		// "电音滋滋"）。x3/4 后峰值 ≈0.86 满度，无削波。
		wire signed [15:0] sin_3q = $signed(wave_sin) - ($signed(wave_sin) >>> 2);

		// 【2026-10-31 末听测"钢琴与正弦区别再明显一点"（正弦/长笛均不动）】
		// 钢琴基波单独压到 x5/8（sin_5q，钢琴专用）：谐波相对占比显著提高
		// ——2f/基波 ≈0.73、3f/基波 ≈0.44（x3/4 时为 0.51/0.30），起音更亮
		// 更"金属"；三者峰值 ≈1.36 满度，仅在起音相位对齐时经限幅触顶、
		// 随 e2/e3 衰减消失，听感是带饱和的琴槌起音而不是持续削波。
		wire signed [15:0] sin_5q = ($signed(wave_sin) >>> 1) + ($signed(wave_sin) >>> 3);

		// T4 钢琴：基频（x5/8 插值正弦）+ 2f x e2 + 3f x e3（各自独立指数衰减，
		// p2_r/p3_r 由共享乘法器提供，晚一拍）
		// 【定点】分量必须按 18 位有符号扩展（原写法 $signed({1'b0,p2_r})
		// 会把负半周变成 +65536 的大正数 → 谐波被整流、输出带直流偏置）
		wire signed [17:0] piano_sum = $signed({{2{sin_5q[15]}}, sin_5q})
		                             + $signed({{2{p2_r[15]}}, p2_r})
		                             + $signed({{2{p3_r[15]}}, p3_r});
	wire signed [15:0] wave_pno = (piano_sum > 18'sd32767) ? 16'sd32767 :
	                              (piano_sum < -18'sd32767) ? -16'sd32767 :
	                              piano_sum[15:0];

	// T5 长笛：基频（插值正弦）+ 二次谐波（常数 4915 移位加，精确）
	//   4915 = 4096+512+256+32+16+2+1
	wire signed [28:0] s2x = {{13{sin2[15]}}, sin2};
	wire signed [28:0] fh2_sum = (s2x <<< 12) + (s2x <<< 9) + (s2x <<< 8)
	                           + (s2x <<< 5) + (s2x <<< 4) + (s2x <<< 1) + s2x;
	wire signed [15:0] fh2_v = fh2_sum >>> 15;
	// 【时序】二次谐波先寄存一拍：sin2 -> 移位加树 -> 音色 mux 是 18.4ns
	// 的组合链（限制 Fmax）。寄存后与插值正弦的流水级也对齐。
	reg  signed [15:0] fh2_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) fh2_r <= 16'sd0;
		else        fh2_r <= fh2_v;
	end
		// 【定点】ch_r 是有符号噪声包络，负值必须符号扩展
		wire signed [17:0] flute_sum = $signed({{2{sin_3q[15]}}, sin_3q})
		                             + $signed({{2{fh2_r[15]}}, fh2_r})
		                             + $signed({{2{ch_r[15]}}, ch_r});
	wire signed [15:0] wave_flt = (flute_sum > 18'sd32767) ? 16'sd32767 :
	                              (flute_sum < -18'sd32767) ? -16'sd32767 :
	                              flute_sum[15:0];

	// 音色选择
	reg signed [15:0] wave_sel;
	always @* begin
		case (timbre)
			3'd1:    wave_sel = wave_pls;
			3'd2:    wave_sel = wave_saw;
			3'd3:    wave_sel = wave_sub;
			3'd4:    wave_sel = wave_pno;
			3'd5:    wave_sel = wave_flt;
			default: wave_sel = wave_sin;
		endcase
	end

	reg  signed [15:0] wave_i;		// 第 3 级：当前波形值（±32767 限幅）
	reg  signed [15:0] wave_i2;		// 第 3 级缓冲：防止综合器把 wave_i 吸收进
									// env 乘法器的 DSP 输入寄存器（那样音色 mux
									// 会并进长组合链，实测 19.3ns / Fmax 50.3MHz）
	reg  signed [16:0] wave_e;		// 第 4 级：x 包络

	wire signed [31:0] env_prod = wave_i2 * $signed({1'b0, env});

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wave_i     <= 16'sd0;
			wave_i2    <= 16'sd0;
			wave_e     <= 17'sd0;
			sample_out <= 16'sd0;
		end else begin
			wave_i  <= wave_sel;
			wave_i2 <= wave_i;

				// 第 4 级：x 包络（包络峰值 32767，结果不超过满度）
				wave_e <= env_prod >>> 15;

				// 第 5 级：振幅颤音结果限幅输出（vib_man=0 时 trem_r==wave_e，
				// 与直通完全一致；力度缩放已在 synth_core 混音级统一完成）
				if (trem_r > 17'sd32767)
					sample_out <= 16'sd32767;
				else if (trem_r < -17'sd32767)
					sample_out <= -16'sd32767;
				else
					sample_out <= trem_r[15:0];
		end
	end

endmodule
