`timescale 1ns / 1ps
//=============================================================================
// i2s_tx —— I2S 主机发送器（FPGA 为主机，MAX98357A 等 I2S 音频 DAC 为从机）
//
// 时钟关系（sys_clk = 50MHz，CLK_DIV = 16）：
//   BCLK  = 50MHz / 16        = 3.125 MHz  = 64 x fs
//   LRCLK = BCLK / 64         = 48.828 kHz = fs
//   MCLK  = 50MHz / 8         = 6.25 MHz   = 128 x fs
//
// 标准 I2S（飞利浦）时序：
//   - 数据在 BCLK 下降沿变化，DAC 在 BCLK 上升沿采样；
//   - LRCLK 在 BCLK 下降沿翻转；
//   - MSB 相对 LRCLK 翻转沿恰好延迟 1 个 BCLK（装载时高位补 1 个哑元位）。
//
// 每帧（64 个 BCLK）开始时产生一个 clk 宽的 sample_tick 脉冲，
// 用于通知合成引擎计算下一帧样本；本帧装载的是上一次计算完成的样本。
// 左右声道样本取自同一计算时刻，无声道间时延。
//=============================================================================
module i2s_tx #(
	parameter integer CLK_HZ  = 50000000,	// 系统时钟频率（Hz）
	parameter integer CLK_DIV = 16			// sys_clk / CLK_DIV = BCLK（须为 64 x fs）
)(
	input  wire               clk,
	input  wire               rst_n,
	input  wire signed [15:0] sample_l,		// 左声道样本（16 位有符号）
	input  wire signed [15:0] sample_r,		// 右声道样本（16 位有符号）

	output reg                i2s_bclk,		// 位时钟 → DAC BCK
	output reg                i2s_lrclk,		// 声道时钟 → DAC LRCK（0=左，1=右）
	output wire               i2s_data,		// 串行数据 → DAC DIN
	output reg                i2s_mclk,		// 主时钟  → DAC SCK（128 x fs，可不接）
	output reg                sample_tick	// 帧起始脉冲，1 个 clk 宽
);

	localparam integer HALF_DIV = CLK_DIV / 2;

	reg [15:0] div_cnt;		// BCLK 半周期分频计数
	reg [5:0]  bit_cnt;		// 帧内位槽计数（0~63，在 BCLK 下降沿步进）
	reg [31:0] shift_reg;	// 串行移位寄存器
	reg [2:0]  mclk_cnt;	// MCLK 分频计数

	// 半周期到界：BCLK 翻转一次
	wire half_end  = (div_cnt == HALF_DIV - 1);
	wire bclk_fall = half_end &&  i2s_bclk;	// BCLK 1->0（数据在此变化）

	assign i2s_data = shift_reg[31];

	always @(posedge clk) begin
		if (!rst_n) begin
			div_cnt    <= 16'd0;
			bit_cnt    <= 6'd0;
			shift_reg  <= 32'd0;
			i2s_bclk   <= 1'b0;
			i2s_lrclk  <= 1'b0;
			i2s_mclk   <= 1'b0;
			mclk_cnt   <= 3'd0;
			sample_tick<= 1'b0;
		end else begin
			sample_tick <= 1'b0;

			// MCLK = sys_clk / 8 = 128 x fs
			if (mclk_cnt == 3'd3) begin
				mclk_cnt <= 3'd0;
				i2s_mclk <= ~i2s_mclk;
			end else begin
				mclk_cnt <= mclk_cnt + 1'b1;
			end

			if (half_end) begin
				div_cnt  <= 16'd0;
				i2s_bclk <= ~i2s_bclk;

				// 所有数据动作都在 BCLK 下降沿进行（DAC 在上升沿采样）
				if (bclk_fall) begin
					if (bit_cnt == 6'd63) begin
						// 左声道槽开始：WS->0，最高位前补哑元 0（实现 I2S 的 1 BCLK MSB 延迟）
						bit_cnt     <= 6'd0;
						i2s_lrclk   <= 1'b0;
						shift_reg   <= {1'b0, sample_l, 15'b0};
						sample_tick <= 1'b1;
					end else if (bit_cnt == 6'd31) begin
						// 右声道槽开始：WS->1
						bit_cnt   <= bit_cnt + 1'b1;
						i2s_lrclk <= 1'b1;
						shift_reg <= {1'b0, sample_r, 15'b0};
					end else begin
						bit_cnt   <= bit_cnt + 1'b1;
						shift_reg <= {shift_reg[30:0], 1'b0};
					end
				end
			end else begin
				div_cnt <= div_cnt + 1'b1;
			end
		end
	end

endmodule
