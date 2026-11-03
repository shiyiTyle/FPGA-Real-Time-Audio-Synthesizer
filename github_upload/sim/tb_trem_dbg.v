`timescale 1ns / 1ps
// 调试探针：每 200 个采样节拍打印颤音通路内部信号
module tb_trem_dbg;
	reg clk = 1'b0;
	always #10 clk = ~clk;

	reg rst_n = 1'b0;
	reg [5:0] td = 6'd0;
	wire      tick = (td == 6'd15);
	always @(posedge clk) td <= td + 6'd1;

	reg        gate = 1'b0;
	reg [7:0]  vib_man = 8'd255;
	reg [31:0] pinc = 32'd103324104;
	wire signed [15:0] s;

	dds_voice #(
		.ATT_SAMPLES (98), .DEC_SAMPLES (16000), .REL_SAMPLES (12207),
		.SUS_LEVEL (6554)
	) dut (
		.clk(clk), .rst_n(rst_n), .tick(tick), .gate(gate),
		.pinc(pinc), .timbre(3'd0), .vib_man(vib_man), .env_sel(1'b0),
		.sample_out(s)
	);

	integer tn = 0;
	always @(posedge clk) begin
		if (rst_n && tick && tn < 12) begin
			tn = tn + 1;
			$display("t%0d: wave_e=%0d trem_factor=%0d m_ph=%0d m_res=%0d trem_r=%0d out=%0d tri9=%0d",
				tn, dut.wave_e, dut.trem_factor, dut.m_ph, dut.m_res, dut.trem_r, s, dut.tri9);
		end
	end

	initial begin
		rst_n = 1'b0;
		repeat (20) @(posedge clk);
		rst_n = 1'b1;
		gate = 1'b1;
		repeat (3000) @(posedge tick);
		$finish;
	end
endmodule
