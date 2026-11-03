`timescale 1ns / 1ps
//=============================================================================
// tb_piano —— 钢琴谐波包络衰减节拍回归（2026-10-31 修复验证）
//
// 背景：e2/e3（2f/3f 谐波包络）与 chiff 的衰减原来写在 tick 外，每个
// 50MHz 时钟减一次，比设计意图（按采样节拍）快 1024 倍——谐波 <0.4ms
// 死光，钢琴听上去就是纯正弦（用户实测"钢琴和正弦没有区别"的根因）。
//
// 验证：
//   1) 1ms 时 e2/e3 仍然大（旧 bug 下早已衰减到 0）
//   2) 100ms 时 e2 还在衰减中、e3 接近归零（3f 先消失的钢琴亮度结构）
//   3) 两个时刻输出波形峰值都足够大（基频持续发声）
//=============================================================================
module tb_piano;
	reg clk = 1'b0;
	always #10 clk = ~clk;

	reg rst_n = 1'b0;
	reg [5:0] td = 6'd0;
	wire      tick = (td == 6'd15);
	always @(posedge clk) td <= td + 6'd1;

	reg        gate = 1'b0;
	reg [31:0] pinc = 32'd103324104;	// D6
	wire signed [15:0] s;

	dds_voice #(
		.ATT_SAMPLES (98), .DEC_SAMPLES (16000), .REL_SAMPLES (12207),
		.SUS_LEVEL (6554)
	) dut (
		.clk(clk), .rst_n(rst_n), .tick(tick), .gate(gate),
		.pinc(pinc), .timbre(3'd4), .vib_man(8'd0), .env_sel(1'b0),
		.sample_out(s)
	);

	integer errs = 0;
	integer i;
	reg signed [15:0] pk;

	// 64 个采样节拍内的输出峰值
	task measure_peak;
		begin
			pk = 16'sd0;
			for (i = 0; i < 64; i = i + 1) begin
				@(posedge tick);
				if (s[15]) pk = (-s > pk) ? -s : pk;
				else       pk = ( s > pk) ?  s : pk;
			end
		end
	endtask

	initial begin
		rst_n = 1'b0;
		repeat (20) @(posedge clk);
		rst_n = 1'b1;
		gate = 1'b1;

		// ---- 1ms（≈49 节拍）：谐波应仍在（旧 bug 此处 e2/e3 已为 0）----
		repeat (49) @(posedge tick);
		if (dut.e2 > 16'd8000)
			$display("PASS e2 @1ms = %0d", dut.e2);
		else begin errs = errs + 1; $display("FAIL e2 @1ms = %0d（旧 bug 特征）", dut.e2); end
		if (dut.e3 > 16'd3000)
			$display("PASS e3 @1ms = %0d", dut.e3);
		else begin errs = errs + 1; $display("FAIL e3 @1ms = %0d", dut.e3); end
		measure_peak;
		if (pk > 16'sd8000)
			$display("PASS peak @1ms = %0d（起音有力度）", pk);
		else begin errs = errs + 1; $display("FAIL peak @1ms = %0d", pk); end

		// ---- 100ms（累计 ≈4883 节拍）：e2 衰减中、e3 亮度消退中 ----
		// （E2=15000 线性 1/拍 → @4883 ≈10117；E3_K=12 速率 >>12|1：
		//   9000→8192 为 2/拍、以下 1/拍 → @4883 ≈3500，零点 ≈176ms）
		repeat (4834) @(posedge tick);
		if (dut.e2 < 16'd12000 && dut.e2 > 16'd6000)
			$display("PASS e2 @100ms = %0d（持续衰减中）", dut.e2);
		else begin errs = errs + 1; $display("FAIL e2 @100ms = %0d", dut.e2); end
		if (dut.e3 < 16'd4500 && dut.e3 > 16'd2500)
			$display("PASS e3 @100ms = %0d（3f 亮度消退中）", dut.e3);
		else begin errs = errs + 1; $display("FAIL e3 @100ms = %0d", dut.e3); end
		measure_peak;
		if (pk > 16'sd3000)
			$display("PASS peak @100ms = %0d（基频仍在发声）", pk);
		else begin errs = errs + 1; $display("FAIL peak @100ms = %0d", pk); end

		// ---- 150ms（≈7300 节拍）：e3 已接近熄灭（实测零点 ≈176ms）----
		repeat (2400) @(posedge tick);
		if (dut.e3 < 16'd2000)
			$display("PASS e3 @150ms = %0d（接近熄灭）", dut.e3);
		else begin errs = errs + 1; $display("FAIL e3 @150ms = %0d", dut.e3); end

		if (errs == 0) $display("PIANO_PASS");
		else           $display("PIANO_FAIL errs=%0d", errs);
		$finish;
	end

endmodule
