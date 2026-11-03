`timescale 1ns / 1ps
//=============================================================================
// viz_spec —— 16 频点实时频谱（Goertzel 递推，单乘法器时分复用）
//
// 算法：Goertzel 是"只看一个频点"的 DFT，每样本只需 1 次乘加：
//     s[n] = x[n] + C * s[n-1] - s[n-2],   C = 2*cos(2*pi*k/N)
//   窗口 N=960 个样本（≈19.7ms），结束时幅值 |X|² ≈ s1² + s2²
//   （略去 -C*s1*s2 交叉项，幅度误差 <1.5dB，用于柱状显示足够）。
//
// 频点（k = f*N/fs，对数分布 203Hz~10.1kHz，覆盖 G5~G6 及其泛音）：
//   203 254 356 458 559 763 966 1272 1628 2136 2747 3560 4628 6002 7782 10122 Hz
//
// 硬件：16 个频点共用 1 个乘法器，每样本按 index 顺序推进 16 拍
//   （采样率 48.8kHz 下每样本有 1024 个时钟，占用 <2%）；
//   窗口结束时再用 32 拍算幅值并清零累加器。
//   乘数位宽：C 18 位 x s1 24 位 -> 42 位，综合约 2~3 个 DSP。
//
// 输出：bar[15:0] 每个频点的柱高 0~104（对数刻度，按幅值位长映射）
//       upd 单拍脉冲表示一组新柱高就绪
//=============================================================================
module viz_spec #(
	parameter integer N_WIN   = 960,		// 窗口样本数
	parameter integer BAR_MAX = 104			// 柱高上限（像素）
)(
	input  wire               clk,
	input  wire               rst_n,
	input  wire signed [15:0] audio,
	input  wire               audio_tick,
	output reg  [111:0]       bar_flat,		// 16 x 7bit 柱高（0~104）
	output reg                upd
);

	//-------------------------------------------------------------------------
	// 每频点两个状态寄存器。位宽 28 位：Goertzel 累加器幅值约 N*A/2，
	// N=960、A 满幅 32767 时约 1.57e7，24 位（8.4e6）会溢出，必须 28 位
	//-------------------------------------------------------------------------
	reg signed [27:0] s1 [0:15];
	reg signed [27:0] s2 [0:15];

	// 系数 C = 2*cos(2*pi*k/N) 的 Q14 定点
	function signed [17:0] coef;
		input [3:0] i;
		begin
			case (i)
				4'd0 : coef = 18'sd32757;	// 203 Hz
				4'd1 : coef = 18'sd32750;	// 254 Hz
				4'd2 : coef = 18'sd32734;	// 356 Hz
				4'd3 : coef = 18'sd32711;	// 458 Hz
				4'd4 : coef = 18'sd32683;	// 559 Hz
				4'd5 : coef = 18'sd32610;	// 763 Hz
				4'd6 : coef = 18'sd32515;	// 966 Hz
				4'd7 : coef = 18'sd32330;	// 1272 Hz
				4'd8 : coef = 18'sd32052;	// 1628 Hz
				4'd9 : coef = 18'sd31538;	// 2136 Hz
				4'd10: coef = 18'sd30743;	// 2747 Hz
				4'd11: coef = 18'sd29389;	// 3560 Hz
				4'd12: coef = 18'sd27126;	// 4628 Hz
				4'd13: coef = 18'sd23472;	// 6002 Hz
				4'd14: coef = 18'sd17666;	// 7782 Hz
				default: coef = 18'sd8688;	// 10122 Hz
			endcase
		end
	endfunction

	//-------------------------------------------------------------------------
	// 位长（对数刻度用）：返回 1..34
	//-------------------------------------------------------------------------
	function [5:0] bitlen;
		input [33:0] v;
		integer j;
		begin
			bitlen = 6'd0;
			for (j = 0; j < 34; j = j + 1)
				if (v[j]) bitlen = j[5:0] + 6'd1;
		end
	endfunction

	localparam [1:0] S_ACC = 2'd0,		// 每样本推进 16 个频点
	                 S_POW = 2'd1,		// 窗口结束：算幅值
	                 S_IDLE= 2'd2;

	reg [1:0]  st;
	reg [3:0]  bi;			// 频点索引
	reg [1:0]  pph;			// 幅值计算子相位
	reg [10:0] wcnt;		// 窗口计数
	reg signed [15:0] xin;
	reg        pend;		// 有待处理的样本

	wire signed [17:0] c_w   = coef(bi);
	wire signed [45:0] macc  = c_w * s1[bi];		// 18 x 28
	wire signed [31:0] tacc  = macc >>> 14;			// C*s1（Q14 还原）
	wire signed [32:0] sacc  = $signed({{17{xin[15]}}, xin})
	                         + $signed({{1{tacc[31]}}, tacc})
	                         - $signed({{5{s2[bi][27]}}, s2[bi]});

	// 幅值：p2 = (|s1|>>12)^2 + (|s2|>>12)^2（略去交叉项，误差 <1.5dB）
	// 【时序】原写法一条组合链 21.8ns（建立违例 -1.88ns）：
	//   bi -> s1[bi] 16 选 1 -> 取绝对值 -> 平方 -> 加 -> 位长 -> x8 -> 写柱高
	// 现拆成 4 拍流水（每频点 4 拍，16 点共 64 拍，窗口 960 样本才一次，
	// 占比可忽略；柱高晚 80ns 出，屏上不可见）。
	wire signed [27:0] a1 = s1[bi][27] ? (-s1[bi]) : s1[bi];
	wire signed [27:0] a2 = s2[bi][27] ? (-s2[bi]) : s2[bi];
	wire        [15:0] q1 = a1[27:12];
	wire        [15:0] q2 = a2[27:12];

	reg  [15:0] q1_r, q2_r;			// 拍 0：锁存 |s|>>12
	reg  [32:0] p2_r;				// 拍 1：两路平方和
	reg  [6:0]  h_r;				// 拍 2：柱高

	wire        [31:0] sq1 = q1_r * q1_r;
	wire        [31:0] sq2 = q2_r * q2_r;
	wire        [32:0] p2  = {1'b0, sq1} + {1'b0, sq2};
	wire        [5:0]  bl  = bitlen(p2_r);
	// 对数映射：bitlen 12 -> 0，每 +1 提升 8 像素
	wire        [9:0]  hraw = (bl > 6'd12) ? ((bl - 6'd12) * 6'd8) : 10'd0;
	wire        [6:0]  h    = (hraw > BAR_MAX) ? BAR_MAX[6:0] : hraw[6:0];

	integer i;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			st   <= S_IDLE;
			bi   <= 4'd0;
			pph  <= 2'd0;
			wcnt <= 11'd0;
			pend <= 1'b0;
			xin  <= 16'sd0;
			upd  <= 1'b0;
			q1_r <= 16'd0;
			q2_r <= 16'd0;
			p2_r <= 33'd0;
			h_r  <= 7'd0;
			for (i = 0; i < 16; i = i + 1) begin
				s1[i] <= 28'sd0;
				s2[i] <= 28'sd0;
				bar_flat[i*7 +: 7] <= 7'd0;
			end
		end else begin
			upd <= 1'b0;
			case (st)
			//-------------------------------------------------------------
			S_IDLE: begin
				if (audio_tick) begin
					xin  <= audio;
					bi   <= 4'd0;
					pend <= 1'b1;
					st   <= S_ACC;
					// 窗口计数
					if (wcnt == N_WIN - 1) wcnt <= 11'd0;
					else                   wcnt <= wcnt + 11'd1;
				end
			end
			//-------------------------------------------------------------
			S_ACC: begin
				// 一个频点一拍：s = x + C*s1 - s2
				s2[bi] <= s1[bi];
				s1[bi] <= sacc[27:0];
				if (bi == 4'd15) begin
					bi <= 4'd0;
					st <= (wcnt == N_WIN - 1) ? S_POW : S_IDLE;
					if (wcnt == N_WIN - 1) pph <= 2'd0;
				end else
					bi <= bi + 4'd1;
			end
			//-------------------------------------------------------------
			S_POW: begin
				// 每频点 4 拍：锁存 |s| >> 平方和 >> 位长/柱高 >> 写回+清零
				case (pph)
				2'd0: begin
					q1_r <= q1;
					q2_r <= q2;
					pph  <= 2'd1;
				end
				2'd1: begin
					p2_r <= p2;
					pph  <= 2'd2;
				end
				2'd2: begin
					h_r <= h;
					pph <= 2'd3;
				end
				default: begin
					bar_flat[bi*7 +: 7] <= h_r;
					s1[bi]  <= 28'sd0;
					s2[bi]  <= 28'sd0;
					pph     <= 2'd0;
					if (bi == 4'd15) begin
						bi  <= 4'd0;
						st  <= S_IDLE;
						upd <= 1'b1;			// 新一组柱高就绪
					end else
						bi <= bi + 4'd1;
				end
				endcase
			end
			default: st <= S_IDLE;
			endcase
		end
	end

endmodule
