`timescale 1ns / 1ps
//=============================================================================
// keypad_scan —— 4x4 矩阵键盘扫描 + 消抖
//
// 接法（HW-834 等）：行 R1~R4 由 FPGA 驱动，列 C1~C4 输入（内部上拉）。
// 按键跨接在行/列交叉点：按下 = 行接到列。
//
// 扫描方式：同一时刻仅一行驱动低电平，其余行置高阻（防多键共列时的
// 电流对冲/鬼影）；每行保持 ROW_SETTLE 个 clk 后采样 4 列。
//   ROW_SETTLE = 2500（0.05ms/行）→ 整轮 0.2ms，每键采样率 5kHz
//
// 消抖：每键 5 位饱和计数器（采样=按下 +1，否则 -1），
//   计数 >= DBNC 判按下。**DBNC 必须让窗口覆盖触点抖动**：
//   DBNC=16 @ 5kHz = 3.2ms（与原版 DBNC=4 @ 1.25kHz 的 3.2ms 积分窗口
//   完全一致，手感/抗抖能力相同）；曾试过 DBNC=8（1.6ms）→ 抖动漏滤，
//   按键重复触发，声音明显变怪，已回退。要更快响应就调小 DBNC，
//   但 1.6ms 以下会开始误触发。
//
// 输出 key_state[15:0]：bit0=S1 ... bit15=S16，1=按下。
// 按键到扫描位映射：S(n) 位于第 r 行第 c 列，n = r*4 + c + 1（r,c 从 0 起），
//   即行 R1 上为 S1~S4（列 C1~C4），R2 上为 S5~S8，依此类推。
//=============================================================================
module keypad_scan #(
	parameter integer ROW_SETTLE = 2500,	// 每行驱动保持时长（clk 数）
	parameter integer DBNC       = 16		// 消抖确认样本数（16@5kHz = 3.2ms）
)(
	input  wire        clk,
	input  wire        rst_n,
	output reg  [3:0]  row_n,		// 行扫描输出：低=选中，Z=空闲（row_n[0]=R1）
	input  wire [3:0]  col_n,		// 列输入：低=该列有键按下（col_n[0]=C1）
	output reg  [15:0] key_state	// 消抖后键态：1=按下
);

	reg [13:0] settle_cnt;
	reg [1:0]  row_idx;
	reg [4:0]  db [0:15];		// 每键饱和计数器（0~31）
	integer i;

	wire sample_now = (settle_cnt == ROW_SETTLE - 1);

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			settle_cnt <= 14'd0;
			row_idx    <= 2'd0;
			row_n      <= {1'bz, 1'bz, 1'bz, 1'b0};	// 从 R1 开始
			key_state  <= 16'd0;
			for (i = 0; i < 16; i = i + 1)
				db[i] <= 5'd0;
		end else begin
			if (sample_now) begin
				settle_cnt <= 14'd0;

				// 本行稳定后的采样：低=按下，更新该行 4 键的计数器
				for (i = 0; i < 4; i = i + 1) begin
					if (!col_n[i]) begin
						if (db[row_idx*4 + i] != 5'd31)
							db[row_idx*4 + i] <= db[row_idx*4 + i] + 1'b1;
					end else begin
						if (db[row_idx*4 + i] != 5'd0)
							db[row_idx*4 + i] <= db[row_idx*4 + i] - 1'b1;
					end
				end

				// 切到下一行（当前行已采样完毕），注意 one-hot 与行号对齐
				row_idx <= row_idx + 1'b1;
				case (row_idx)
					2'd0:    row_n <= {1'bz, 1'b0, 1'bz, 1'bz};	// -> R2
					2'd1:    row_n <= {1'bz, 1'bz, 1'b0, 1'bz};	// -> R3
					2'd2:    row_n <= {1'b0, 1'bz, 1'bz, 1'bz};	// -> R4
					default: row_n <= {1'bz, 1'bz, 1'bz, 1'b0};	// -> R1
				endcase
			end else begin
				settle_cnt <= settle_cnt + 1'b1;
			end

			// 阈值判决：连续 DBNC 次采样为按下才确认（消抖）
			for (i = 0; i < 16; i = i + 1)
				key_state[i] <= (db[i] >= DBNC);
		end
	end

endmodule
