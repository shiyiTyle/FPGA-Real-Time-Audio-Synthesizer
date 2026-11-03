`timescale 1ns / 1ps
//=============================================================================
// demo_seq —— 自动演奏演示音序器（无人值守演示与链路自检）
//
// 功能：上电后循环播放，验证"双声道 + 多声部 + ADSR"链路：
//   声部 0 -> 左声道：C5 (523.25Hz) 拨弦，每 600ms 一次
//   声部 1 -> 右声道：E5 (659.26Hz) 拨弦，错开 300ms
//   声部 2/3：静默（留给交互扩展）
//
// DEMO_MODE = 0：交替拨弦（听感：左右耳交替拨弦，包络起伏明显）
// DEMO_MODE = 1：连续双长音（左右耳各一个稳态音，便于示波器验证）
//
// 按键 S2（按下为高）复位音序，从头重播。
//=============================================================================
module demo_seq #(
	parameter integer DEMO_MODE = 0
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        demo_rst,	// 音序复位（高有效）
	output wire [3:0]  gate_v,
	output wire [127:0] pinc_v,
	output wire [63:0] vel_v
);

	// 相位增量常数：fs = 50MHz/16/64 = 48828.125Hz，PINC = 2^32 x f / fs
	localparam [31:0] PINC_C5   = 32'd46025656;	// 523.25 Hz
	localparam [31:0] PINC_E5   = 32'd57988693;	// 659.26 Hz
	localparam [15:0] VEL_DEMO  = 16'd14745;	// 力度 ≈45% 满度
	// 注：力度对四声部统一（synth_core 在混音级缩放一次）；声部 2/3 靠
	// gate=0 静音，力度值无关

	// 演奏节拍（50MHz 时钟周期数）
	localparam integer PERIOD   = 30000000;	// 600ms 循环
	localparam integer GATE_LEN = 17500000;	// 按住 350ms（起音+衰减+延音）
	localparam integer OFFSET1  = 15000000;	// 声部 1 相对声部 0 延迟 300ms

	reg [24:0] cnt;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			cnt <= 25'd0;
		else if (demo_rst)
			cnt <= 25'd0;
		else if (cnt >= PERIOD - 1)
			cnt <= 25'd0;
		else
			cnt <= cnt + 1'b1;
	end

	// 声部 1 的相位在周期内整体延后 OFFSET1（26 位避免相加溢出）
	wire [25:0] p1_sum  = {1'b0, cnt} + 26'd15000000;
	wire [25:0] phase1  = (p1_sum >= 26'd30000000) ? (p1_sum - 26'd30000000) : p1_sum;

	generate
		if (DEMO_MODE == 0) begin : g_pluck
			assign gate_v = {2'b00, (phase1 < GATE_LEN), (cnt < GATE_LEN)};
		end else begin : g_steady
			assign gate_v = 4'b0011;
		end
	endgenerate

	assign pinc_v = {32'd0, 32'd0, PINC_E5, PINC_C5};
	assign vel_v  = {4{VEL_DEMO}};

endmodule
