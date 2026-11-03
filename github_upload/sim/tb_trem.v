`timescale 1ns / 1ps
//=============================================================================
// tb_trem —— 电位器振幅颤音（2026-10-31 末改版）验证：
//   1) vib_man=255：trem_r 峰值随 5.5Hz LFO 呼吸——正峰窗口 ≈0.77 倍、
//      负峰窗口 ≈1.23 倍，比值 ≈1.6x（音高不变、音量摆动）
//   2) vib_man=0：正负峰峰值基本相等（与直通一致）
// 测量要点：DEC_SAMPLES 拉长使包络在测量期间恒定；测 trem_r（限幅前）
//   避免 1.23 倍上摆被输出限幅器削顶干扰比值。
//=============================================================================
module tb_trem;
	reg clk = 1'b0;
	always #10 clk = ~clk;

	reg rst_n = 1'b0;
	reg [5:0] td = 6'd0;
	wire      tick = (td == 6'd15);
	always @(posedge clk) td <= td + 6'd1;

	reg        gate = 1'b0;
	reg [7:0]  vib_man = 8'd0;
	reg [31:0] pinc = 32'd103324104;	// D6
	wire signed [15:0] s;

	dds_voice #(
		.ATT_SAMPLES (98), .DEC_SAMPLES (1), .REL_SAMPLES (12207),
		.SUS_LEVEL (32767)
	) dut (
		.clk(clk), .rst_n(rst_n), .tick(tick), .gate(gate),
		.pinc(pinc), .timbre(3'd0), .vib_man(vib_man), .env_sel(1'b0),
		.sample_out(s)
	);

	integer errs = 0;
	integer i, d;
	reg signed [16:0] pk_pos, pk_neg;

	// 64 个采样节拍内 trem_r 的峰值（64 样本 ≈ 1.5 个 D6 周期，能采到波峰）
	task peak64(output integer p);
		begin
			p = 0;
			for (i = 0; i < 64; i = i + 1) begin
				@(posedge tick);
				if (dut.trem_r[16]) p = (-dut.trem_r > p) ? -dut.trem_r : p;
				else                p = ( dut.trem_r > p) ?  dut.trem_r : p;
			end
		end
	endtask

	initial begin
		rst_n = 1'b0;
		repeat (20) @(posedge clk);
		rst_n = 1'b1;
		gate = 1'b1;
		repeat (300) @(posedge tick);		// 等起音完成、包络到顶

		// ---- 旋钮拧满：LFO 正峰窗口 vs 负峰窗口 ----
		vib_man = 8'd255;
		wait (dut.tri9 >= 9'sd120);
		peak64(pk_pos);
		wait (dut.tri9 <= -9'sd120);
		peak64(pk_neg);
		$display("D=255: pk_pos=%0d pk_neg=%0d (期望比值≈1.6x)", pk_pos, pk_neg);
		if (pk_neg > pk_pos * 7 / 5 && pk_neg < pk_pos * 37 / 20)
			$display("PASS tremolo swing");
		else begin errs = errs + 1; $display("FAIL tremolo swing"); end

		// ---- 旋钮最小：正负峰应基本相等（直通） ----
		vib_man = 8'd0;
		wait (dut.tri9 >= 9'sd120);
		peak64(pk_pos);
		wait (dut.tri9 <= -9'sd120);
		peak64(pk_neg);
		$display("D=0: pk_pos=%0d pk_neg=%0d", pk_pos, pk_neg);
		d = pk_pos - pk_neg;
		if (d < 0) d = -d;
		if (d <= pk_pos / 10)
			$display("PASS unity at D=0");
		else begin errs = errs + 1; $display("FAIL unity at D=0"); end

		if (errs == 0) $display("TREM_PASS");
		else           $display("TREM_FAIL errs=%0d", errs);
		$finish;
	end

endmodule
