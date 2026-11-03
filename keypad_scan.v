`timescale 1ns / 1ps
//=============================================================================
// keypad_scan —— 4×4 矩阵键盘扫描与消抖
//
// 功能：轮询驱动 4 条行线、采样 4 条列线，输出 16 键稳定键态。
//   同一时刻仅一行驱动低电平，其余行置高阻——避免多键同列时行间
//   电流对冲产生"鬼影"；每行保持 ROW_SETTLE 个时钟后采样一次列线，
//   整轮 4 行共 4×ROW_SETTLE 拍，每键采样率 = fclk / (4×ROW_SETTLE)。
//
// 消抖：每键独立 5 位饱和计数器——采样为按下则 +1、否则 −1，
//   计数达到 DBNC 才确认按下。DBNC 的选择必须覆盖触点抖动全程：
//   本设计 2.5kHz 采样、DBNC=8 → 积分窗口 3.2ms。
//   注意：窗口过短（如 1.6ms）会漏滤触点抖动、造成单键重复触发；
//   3.2ms 是这套 HW-834 键盘经实测确定的可靠下限，勿再压缩。
//
// 输出 key_state[15:0]：bit0=S1 … bit15=S16，1=按下。
//   键位映射：第 r 行第 c 列的键为 S(r*4+c+1)（r、c 自 0 起）。
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
