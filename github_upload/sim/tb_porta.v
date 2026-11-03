`timescale 1ns / 1ps
//=============================================================================
// tb_porta —— porta_ctrl 单元仿真（POT_EN=1 后该模块首次进入 bitstream）
//
// 验证四条行为：
//   1) 无声段（gate=0 且上一拍也 0）：cur 直跟目标，上电即就位
//   2) 起音滑音：一步 = delta >> sh，sh = 6 + glide[7:5]
//   3) 滑音到位：3τ（192 拍）后残差 < 10%（理论 (63/64)^192 ≈ 5%）
//   4) 旋钮最大（glide=255 -> sh=13）：单步远小于目标差，且方向正确（下行）
//=============================================================================
module tb_porta;

	reg clk = 1'b0;
	reg rst_n = 1'b0;
	reg tick = 1'b0;
	reg [7:0]   gl   = 8'd0;
	reg [3:0]   gate = 4'd0;
	reg [127:0] pinc = 128'd0;
	wire [127:0] pout;

	integer errs = 0;
	integer d, stp, base;
	reg [31:0] A, B, C;

	porta_ctrl #(.SHIFT_BASE(6)) dut (
		.clk      (clk),
		.rst_n    (rst_n),
		.tick     (tick),
		.glide    (gl),
		.gate_v   (gate),
		.pinc_in  (pinc),
		.pinc_out (pout)
	);

	always #10 clk = ~clk;

	// tick：每 8 个 clk 一个采样节拍
	reg [2:0] tc = 3'd0;
	always @(posedge clk) begin
		tc   <= tc + 3'd1;
		tick <= (tc == 3'd7);
	end

	task check;
		input [255:0] name;
		input [31:0]  got;
		input [31:0]  exp;
		input integer tol;
		begin
			d = got - exp;
			if (d < 0) d = -d;
			if (d > tol) begin
				errs = errs + 1;
				$display("FAIL %0s: got %0d exp %0d (|d|=%0d > %0d)", name, got, exp, d, tol);
			end else
				$display("PASS %0s: %0d (exp %0d)", name, got, exp);
		end
	endtask

	// 走一拍并等到 cur 真正更新后再读：tick 拉高只是装载使能，
	// cur 的更新发生在下一个 clk 上升沿（该沿 tick 仍为 1）
	task tick_step;
		begin
			@(posedge tick);
			@(posedge clk);
			#1;
		end
	endtask

	initial begin
		A = 32'd68960566;		// G5
		B = 32'd103324104;		// D6
		C = 32'd77405619;		// A5

		rst_n = 1'b0;
		repeat (5) @(posedge clk);
		rst_n = 1'b1;

		// 1) 无声段直跟目标
		pinc[31:0] = A; gate = 4'd0; gl = 8'd0;
		tick_step;
		check("silent follow A", pout[31:0], A, 0);

		// 2) 起音滑音 gl=0 -> sh=6：一步 = (B-A)>>6
		pinc[31:0] = B; gate = 4'd1;
		tick_step;
		check("first glide step", pout[31:0], A + ((B - A) >>> 6), 0);

		// 3) 3τ（192 拍）内应基本到位
		repeat (191) tick_step;
		d = pout[31:0] - B; if (d < 0) d = -d;
		if (d > (B / 10)) begin
			errs = errs + 1;
			$display("FAIL settled gl=0: off by %0d", d);
		end else
			$display("PASS settled gl=0: off by %0d", d);

		// 4) 旋钮最大 glide=255 -> sh=13：下行一步 = (C-cur)>>13
		gl = 8'd255;
		base = pout[31:0];
		pinc[31:0] = C;
		tick_step;
		// 负差值必须走有符号整数再做算术右移，直接对无符号表达式移位会变成逻辑右移
		d   = C - base;
		stp = d >>> 13;
		check("max glide step", pout[31:0], base + stp, 0);

		// 4b) 连续 200 拍仍在 cur 与 C 之间（不越界、方向正确）
		repeat (200) tick_step;
		if (pout[31:0] > base || pout[31:0] < C) begin
			errs = errs + 1;
			$display("FAIL glide overshoot: %0d not in [%0d,%0d]", pout[31:0], C, base);
		end else
			$display("PASS glide monotonic: %0d in [%0d,%0d]", pout[31:0], C, base);

		// 5) 松开两拍后（gate_q 已清）：改目标立即到位，无声期不滑
		gate = 4'd0;
		repeat (2) tick_step;
		pinc[31:0] = B;
		tick_step;
		check("silent snap B", pout[31:0], B, 0);

		if (errs == 0) $display("PORTA_PASS");
		else           $display("PORTA_FAIL errs=%0d", errs);
		$finish;
	end

endmodule
