`timescale 1ns / 1ps
//=============================================================================
// tb_bounce —— 消抖决定性实验（不改 RTL，只加 TB）
//
// 模拟一次带真实机械抖动的按键（按下沿抖 5 次，间隔 0.3~1.5ms），
// 同时实例化两个版本的扫描+分配器：
//   A) 当前版 keypad_scan(5000/8) + note_alloc
//   B) backup_stable_20261025 版 keypad_scan(10000/4) + note_alloc
// 统计各自 press_edge 次数和 gate 上升次数。
// 若 A 出现 >1 次边沿而 B 只有 1 次 → 复现"重音"，定位在消抖参数；
// 若 A/B 都只有 1 次 → 消抖无罪，"重音"来自时序违例（SDC 未加）。
//=============================================================================
module tb_bounce;
	reg clk = 1'b0;
	always #10 clk = ~clk;
	reg rst_n = 1'b0;

	//---------------------------------------------------------------------
	// 键盘模型（抖动激励）
	//---------------------------------------------------------------------
	reg bounce = 1'b0;			// 1=正在抖动
	reg down  = 1'b0;			// 1=键稳定按下

	wire [3:0] rowA, rowB;
	wire [3:0] colA, colB;

	// 键 S1 = row0 x col0：行被扫到低且键接触时把列拉低
	assign colA[0] = (down || bounce) && (rowA[0] === 1'b0) ? 1'b0 : 1'bz;
	assign colB[0] = (down || bounce) && (rowB[0] === 1'b0) ? 1'b0 : 1'bz;
	wire [3:0] colA_pu = colA === 8'hxx ? 4'hF : (colA | 4'hE);
	wire [3:0] colB_pu = colB === 8'hxx ? 4'hF : (colB | 4'hE);
	pullup (colA[0]); pullup (colA[1]); pullup (colA[2]); pullup (colA[3]);
	pullup (colB[0]); pullup (colB[1]); pullup (colB[2]); pullup (colB[3]);

	//---------------------------------------------------------------------
	// A：当前版
	//---------------------------------------------------------------------
	wire [15:0] ksA, ksB;
	wire [3:0]  gA, gB;
	wire [127:0] pA, pB;
	wire [63:0]  vA, vB;
	wire [2:0]  tA, tB;
	wire [3:0]  eA, eB;

	keypad_scan #(.ROW_SETTLE(5000), .DBNC(8)) uA (
		.clk(clk), .rst_n(rst_n), .row_n(rowA),
		.col_n(colA === 4'hxx ? 4'hF : (colA | 4'hE)), .key_state(ksA));
	note_alloc uAn (.clk(clk), .rst_n(rst_n), .key_state(ksA),
		.gate_v(gA), .pinc_v(pA), .vel_v(vA), .timbre_o(tA),
		.env_sel_v(eA), .fx_rev(), .fx_dly());

	// B：早期稳定版（backup_stable_20261025）
	keypad_scan_bk #(.ROW_SETTLE(10000)) uB (
		.clk(clk), .rst_n(rst_n), .row_n(rowB),
		.col_n(colB === 4'hxx ? 4'hF : (colB | 4'hE)), .key_state(ksB));
	note_alloc_bk uBn (.clk(clk), .rst_n(rst_n), .key_state(ksB),
		.gate_v(gB), .pinc_v(pB), .vel_v(vB), .timbre_o(tB), .env_sel_v(eB));

	//---------------------------------------------------------------------
	// 边沿统计
	//---------------------------------------------------------------------
	integer peA = 0, peB = 0, grA = 0, grB = 0;
	reg [15:0] ksA_d = 0, ksB_d = 0;
	reg [3:0]  gA_d = 0, gB_d = 0;
	integer k;
	always @(posedge clk) begin
		if (rst_n) begin
			for (k = 0; k < 16; k = k + 1) begin
				if (ksA[k] & ~ksA_d[k]) peA = peA + 1;
				if (ksB[k] & ~ksB_d[k]) peB = peB + 1;
			end
			ksA_d <= ksA; ksB_d <= ksB;
			if (gA[0] & ~gA_d[0]) grA = grA + 1;
			if (gB[0] & ~gB_d[0]) grB = grB + 1;
			gA_d <= gA; gB_d <= gB;
		end
	end

	//---------------------------------------------------------------------
	// 抖动时序：t=1ms 按下，抖 5 次（间隔 0.3~1.5ms），t=4ms 稳定，
	// t=30ms 松开（松键也抖 3 次）
	//---------------------------------------------------------------------
	reg [7:0] bseq = 8'd0;
	initial begin
		repeat (10) @(posedge clk);
		rst_n = 1'b1;
		#1_000_000;					// 1ms
		// 按下沿抖动：接触-断开交替
		bounce = 1'b1; down = 1'b1;
		#300_000;  bounce = 1'b0;	// 0.3ms 接触
		#400_000;  bounce = 1'b1;	// 0.4ms 断开
		#700_000;  bounce = 1'b0;	// 0.7ms 接触
		#500_000;  bounce = 1'b1;	// 0.5ms 断开
		#1200_000; bounce = 1'b0;	// 1.2ms 接触
		#600_000;  bounce = 1'b1;	// 0.6ms 断开
		#800_000;  bounce = 1'b0;	// 0.8ms 接触 -> 稳定
		down = 1'b1;

		#26_000_000;				// 稳定按住 26ms
		// 松开沿抖动
		down = 1'b0; bounce = 1'b1;
		#500_000;  bounce = 1'b0;
		#800_000;  bounce = 1'b1;
		#600_000;  bounce = 1'b0;

		#10_000_000;
		$display("RESULT press_edge: A(now)=%0d  B(old)=%0d", peA, peB);
		$display("RESULT gate_rise : A(now)=%0d  B(old)=%0d", grA, grB);
		if (peA > 1 || grA > 1)
			$display("VERDICT: 当前消抖复现重复触发 -> '重音'来自按键消抖");
		else if (peB > 1)
			$display("VERDICT: 两版都重复（激励过强）");
		else
			$display("VERDICT: 当前消抖与旧版同样干净 -> '重音'更可能来自时序违例(SDC)");
		$display("TB_BOUNCE_DONE");
		$finish;
	end
endmodule
