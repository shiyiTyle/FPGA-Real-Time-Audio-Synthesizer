`timescale 1ns / 1ps
//=============================================================================
// ttp229_drv —— TTP229 16 键电容触摸板驱动（串行 SCL/SDA 读出 -> key_state）
//
// 硬件：TTP229-BSF 触摸板模块（16 键），4 线接口 VCC/GND/SCL/SDA：
//       模块 VCC -> 3V3，GND -> GND，SCL 由 FPGA 驱动，SDA 是模块输出。
//       （若模块带 DV 引脚：保持悬空，本驱动直接轮询移位读出）
//
// 协议（16 键模式，按模块标准串行读出约定）：
//   每个 SCL 高电平期间 SDA 呈现一个键位，共 16 位，键按下 = 1；
//   主机：SCL 0->1 -> 等半周期采样 SDA -> SCL 1->0 -> 等半周期，重复 16 次。
//   移位方向固定为"最先收到的位落在 bit0"，两次连续读出都一致才更新
//   键态（按键确认约 2.4ms 消抖，松键立即释放）。
//
// 时序（CLK_HALF=250 @50MHz）：半周期 5us，一次 16 位读出 160us，
//   两次读出间隔 READ_GAP=1ms，轮询周期约 1.2ms。
//
// 键位映射：FIRST_KEY=0 表示读出流的第一个键位是 TP0（丝印 S1 键），
//   直接对位 key_state[15:0]（与矩阵键盘 keypad_scan 输出格式完全一致，
//   可无缝替换对接 note_alloc）；若实测键位整体镜像，把 FIRST_KEY 改 15。
//=============================================================================
module ttp229_drv #(
	parameter integer CLK_HALF  = 250,		// SCL 半周期 clk 数（5us）
	parameter integer READ_GAP  = 50000,	// 两次读出间隔 1ms
	parameter integer FIRST_KEY = 0			// 流出首个键位对应的 key_state 下标（0 或 15）
)(
	input  wire        clk,
	input  wire        rst_n,
	output reg         scl,			// 时钟输出 -> 模块 SCL
	input  wire        sda,			// 数据输入 <- 模块 SDA
	output reg  [15:0] key_state	// 消抖后键态：bit0=S1 ... bit15=S16
);

	localparam [3:0] ST_GAP   = 4'd0,	// 读出间隙等待
	                 ST_RISE  = 4'd1,	// SCL 0->1，等待数据稳定
	                 ST_SAMPLE= 4'd2,	// 采样 SDA
	                 ST_FALL  = 4'd3;	// SCL 1->0

	// 位序反转（FIRST_KEY=15 时用）：s[0]=d[15] ... s[15]=d[0]
	function [15:0] rev16;
		input [15:0] d;
		integer i;
		begin
			for (i = 0; i < 16; i = i + 1)
				rev16[i] = d[15 - i];
		end
	endfunction

	reg [3:0]  state;
	reg [15:0] tick;
	reg [15:0] sh;				// 本次读出的移位结果（bit0=最先收到的位）
	reg [4:0]  bitn;			// 剩余位数
	reg [15:0] gap_cnt;
	reg [15:0] prev_read;		// 上一次读出

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state     <= ST_GAP;
			tick      <= 16'd0;
			sh        <= 16'd0;
			bitn      <= 5'd15;
			gap_cnt   <= 16'd0;
			prev_read <= 16'd0;
			key_state <= 16'd0;
			scl       <= 1'b0;
		end else begin
			case (state)
			ST_GAP: begin
				scl <= 1'b0;
				if (gap_cnt == READ_GAP - 1) begin
					gap_cnt <= 16'd0;
					bitn    <= 5'd15;
					sh      <= 16'd0;
					state   <= ST_RISE;
				end else
					gap_cnt <= gap_cnt + 1'b1;
			end
			ST_RISE: begin
				if (tick == CLK_HALF - 1) begin
					tick  <= 16'd0;
					scl   <= 1'b1;			// SCL 上升沿，模块推出下一位
					state <= ST_SAMPLE;
				end else
					tick <= tick + 1'b1;
			end
			ST_SAMPLE: begin
				if (tick == CLK_HALF - 1) begin
					tick  <= 16'd0;
					// 右移装入：最先收到的位始终落在 bit0
					sh    <= {sda, sh[15:1]};
					scl   <= 1'b0;			// SCL 下降
					state <= ST_FALL;
				end else
					tick <= tick + 1'b1;
			end
			ST_FALL: begin
				if (tick == CLK_HALF - 1) begin
					tick <= 16'd0;
					if (bitn == 5'd0) begin
						// 一轮 16 位读完：连续两次一致才更新（消抖）
						prev_read <= sh;
						if (FIRST_KEY == 0)
							key_state <= sh & prev_read;
						else
							key_state <= rev16(sh & prev_read);
						state <= ST_GAP;
					end else begin
						bitn  <= bitn - 1'b1;
						state <= ST_RISE;
					end
				end else
					tick <= tick + 1'b1;
			end
			default: state <= ST_GAP;
			endcase
		end
	end

endmodule
