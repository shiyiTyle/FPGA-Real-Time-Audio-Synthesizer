`timescale 1ns / 1ps
//=============================================================================
// porta_ctrl —— 滑音控制（电位器控制，第二表情维度：滑音）
//
// 功能：输入 note_alloc 的 4 声部相位增量，输出按"指数逼近"平滑过渡后的值：
//     cur += (tgt - cur) >> sh
//   时间常数 τ = 2^sh 个样本；滑音量电位器 glide 0~255 映射 sh = 6~13：
//     0 = 近瞬时（τ≈1.3ms），255 = 最慢（τ≈168ms，95% 到位 ≈0.77s）
//   无活跃声部（gate=0）时 cur 直接跟随目标（无声期不滑，上电即就位）；
//   新键按下瞬间 tgt 跳变，cur 从旧音平滑滑向新音 —— 经典 portamento 听感。
//   全部用移位/加法实现，无除法无乘法。
//
// 位置：note_alloc 之后、bend_ctrl 之前（先滑音后弯音，互不干扰：
//   滑音是增量过渡、弯音是乘法缩放）。
//=============================================================================
module porta_ctrl #(
	parameter integer SHIFT_BASE = 6,		// glide=0 时的时间常数移位（近瞬时）
	parameter integer SHIFT_STEP = 0		// 预留：调音时整体偏移
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        tick,		// 采样节拍
	input  wire [7:0]  glide,		// 滑音量 0~255（0=瞬时，255=最慢）
	input  wire [3:0]  gate_v,		// 各声部音符门（与 pinc 同步）
	input  wire [127:0] pinc_in,	// 4 声部目标相位增量（低位=声部0）
	output wire [127:0] pinc_out	// 滑音过渡后的相位增量
);

	// glide 高 3 位 -> sh = 6~13（τ = 64~8192 样本 = 1.3ms~168ms）
	wire [3:0] sh = SHIFT_BASE + SHIFT_STEP + glide[7:5];

	genvar g;
	generate
		for (g = 0; g < 4; g = g + 1) begin : g_porta
			reg  [31:0] cur;		// 当前相位增量
			reg         gate_q;		// 上一拍 gate

			wire [31:0] tgt = pinc_in[g*32 +: 32];
			wire signed [32:0] delta = $signed({1'b0, tgt}) - $signed({1'b0, cur});
			wire signed [32:0] step  = delta >>> sh;
			wire [31:0] cur_next = cur + step[31:0];

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					cur    <= 32'd0;
					gate_q <= 1'b0;
				end else if (tick) begin
					gate_q <= gate_v[g];
					// 无活跃声部：直跟目标（无声期不滑，上电即就位）
					if (!gate_v[g] && !gate_q)
						cur <= tgt;
					else
						cur <= cur_next;
				end
			end
			assign pinc_out[g*32 +: 32] = cur;
		end
	endgenerate

endmodule
