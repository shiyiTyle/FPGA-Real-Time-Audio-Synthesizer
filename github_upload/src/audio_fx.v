`timescale 1ns / 1ps
//=============================================================================
// audio_fx —— 数字音频效果器（混响 + 延迟回声，纯硬件，干湿并联）
//
// 位置：synth_core 之后、i2s_tx 之前。干声直通、湿声只做加法——
//   干声通路不引入任何采样级延迟，演奏延迟指标（拓展3 ≤5ms）不受影响。
// 湿声处理单声道（(L+R)/2；CH_MODE=0 时 L=R 恒等、无幅度损失），
//   左右声道各自叠加同一湿声。
// 效果关闭时效果缓冲写 0（约一秒内自然清空），重新打开无爆音。
//
// 延迟回声：16384 样本（≈0.335s）环形缓冲 + 0.45 反馈，湿声系数 0.35
// 混响：Schroeder 结构——4 路并联梳状（g=0.805，延迟 1236/1316/1414/1501
//   样本，输入衰减 /5）求和 x1/4 -> 2 路串联全通（g=0.7，延迟 249/378 样本），
//   湿声 x0.5（2026-10-31 两轮听测：原 x0.25/输入 /8 听不出 → 0.375 与 /6 →
//   "再明显一点" 0.5 与 /5，累计较初版 ≈+10dB）
//
// 【DSP 节省】本模块全部 10 个乘法缩减为 1 个共享乘法器 + 14 拍线性序列器：
//   tick 后依次执行 回声fb/回声wet/梳状x4/全通1gx/全通1gy·全通2gx（共用
//   同一个乘积 ap1_y x g）/全通2gy，14 拍 << 采样周期 1024 拍。
//   乘法输入/输出全部寄存——原版"全通 y 反馈→乘法→缓冲写"的 24ns 组合
//   链被切断（那正是布局布线 -4.1ns 最差路径）。
//   湿声反馈环路晚 1 个样本闭合（回声/混响尾长变化 20µs，不可闻）；
//   干声通路与开关行为完全不变。
//
// BSRAM 用量：延迟线 15 块 + 混响 10 块 ≈ 25 块（GW5A-25 共 56 块）
//=============================================================================
module audio_fx #(
	parameter integer DLY_LEN = 16384		// 延迟线长度（样本，16384 ≈0.335s）
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        tick,				// 采样节拍（来自 i2s_tx）
	input  wire        fx_rev,				// 混响开关（1=开）
	input  wire        fx_dly,				// 延迟回声开关（1=开）
	input  wire signed [15:0] sample_l,		// 干声左
	input  wire signed [15:0] sample_r,		// 干声右
	output wire signed [15:0] out_l,		// 干声+湿声 -> 左
	output wire signed [15:0] out_r			// 干声+湿声 -> 右
);

	// 单声道化：CH_MODE=0 时 L=R，(L+R)>>1 恒等于 L
	wire signed [16:0] mono_sum = sample_l + sample_r;
	wire signed [15:0] mono     = mono_sum[16:1];

	//-------------------------------------------------------------------------
	// 饱和限幅到 ±32767
	//-------------------------------------------------------------------------
	function signed [15:0] sat16;
		input signed [23:0] v;
		begin
			if (v > 24'sd32767)       sat16 = 16'sd32767;
			else if (v < -24'sd32767) sat16 = -16'sd32767;
			else                      sat16 = v[15:0];
		end
	endfunction

	//-------------------------------------------------------------------------
	// 预热门控：复位后缓冲未写满一圈前读出口钳 0（防上电 X 进入反馈环）
	//-------------------------------------------------------------------------
	reg [14:0] warm;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) warm <= 15'd0;
		else if (tick && warm != 15'h7FFF) warm <= warm + 15'd1;
	end
	wire warm_dly = (warm >= DLY_LEN[14:0]);
	wire warm_cmb = (warm >= 15'd1236);
	wire warm_ap1 = (warm >= 15'd249);
	wire warm_ap2 = (warm >= 15'd378);

	localparam signed [15:0] DLY_FB  = 16'sd14746;	// 反馈 0.45
	localparam signed [15:0] DLY_WET = 16'sd11469;	// 湿声 0.35
	localparam signed [15:0] CMB_G   = 16'sd26377;	// 梳状反馈 0.805
	localparam signed [15:0] AP_G    = 16'sd22938;	// 全通系数 0.7

	//=========================================================================
	// 共享乘法器（18x16 有符号，结果寄存一拍）
	//=========================================================================
	reg  signed [17:0] m_a;
	reg  signed [15:0] m_b;
	wire signed [33:0] m_prod = m_a * m_b;
	reg  signed [33:0] m_res;
	always @(posedge clk or negedge rst_n)
		if (!rst_n) m_res <= 34'sd0;
		else        m_res <= m_prod;

	//=========================================================================
	// 序列器：tick 后 st=1..14 依次执行（st=0 空闲）
	//   拍 N 驱动乘数 -> 拍 N+1 m_res 有效 -> 拍 N+1 末写结果
	//=========================================================================
	reg [3:0] st;

	//=========================================================================
	// 回声（环形缓冲：读 wr+1 -> 延迟 DLY_LEN-1 样本，读写不同址）
	//=========================================================================
	reg  [15:0] mem_dly [0:DLY_LEN-1];
	reg  [13:0] dly_wr;
	reg  signed [15:0] dly_old;
	reg  signed [15:0] dly_wet;

	//=========================================================================
	// 梳状 x4
	//=========================================================================
	wire signed [17:0] comb_out [0:3];

	genvar ci;
	generate
		for (ci = 0; ci < 4; ci = ci + 1) begin : g_comb
			localparam integer CLEN = (ci == 0) ? 1236 :
			                          (ci == 1) ? 1316 :
			                          (ci == 2) ? 1414 : 1501;
			reg [15:0] mem [0:CLEN-1];
			reg [10:0] ptr;
			reg [10:0] wptr;		// 写地址（tick 时锁存自增前的 ptr）
			reg signed [15:0] cout;
			always @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					ptr  <= 11'd0;
					wptr <= 11'd0;
					cout <= 16'sd0;
				end else if (tick) begin
					wptr <= ptr;		// 锁存写地址（必须先于 ptr 自增）
					cout <= warm_cmb ? mem[ptr] : 16'sd0;
					ptr  <= (ptr == CLEN - 1) ? 11'd0 : ptr + 11'd1;
				end else if (st == 4'd4 + ci) begin
					// 拍 4+ci：m_res 才是 comb_out[ci] x CMB_G（乘数在 3+ci 拍
					// 发出、下一拍才出结果）。原写法在 3+ci 拍写回，拿到的是上
					// 一个状态的乘积（回声湿声/前一路梳状），反馈接错 → 尾音
					// 越涨越大、发嗡。
				// 输入衰减 /5（=1/4-1/16+1/64 移位加）：2026-10-31 二次听测
				// "再明显一点"，由 /6 提至 /5。梳状共振增益 1/(1-g)≈5.1，
				// 满幅输入下梳状内部峰值 ≈1.02 满度——仅极端峰值经 sat16
				// 限幅，实际演奏电平不饱和、反馈仍稳定
				mem[wptr] <= fx_rev ? sat16(($signed({mono[15], mono}) >>> 2)
				              - ($signed({mono[15], mono}) >>> 4)
				              + ($signed({mono[15], mono}) >>> 6)
				              + $signed(m_res[33:15])) : 16'sd0;
				end
			end
			assign comb_out[ci] = {{2{cout[15]}}, cout};
		end
	endgenerate

	reg signed [17:0] comb_mix_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			comb_mix_r <= 18'sd0;
		else if (tick)
			comb_mix_r <= (comb_out[0] + comb_out[1] + comb_out[2] + comb_out[3]) >>> 2;
	end

	//=========================================================================
	// 全通 1（延迟 249）：y = d - gx，缓冲写 x + g·y
	// 全通 2（延迟 378）：gx 输入 = 全通 1 的 y（两处共用同一乘积）
	//=========================================================================
	reg [15:0] mem_ap1 [0:248];
	reg  [7:0] ap1_ptr;
	reg  [7:0] ap1_wptr;	// 写地址（tick 时锁存自增前的 ptr，防回绕下溢 OOB）
	reg  signed [15:0] ap1_d;
	reg  signed [16:0] ap1_y_r;
	reg  signed [15:0] ap1_gx;

	reg [15:0] mem_ap2 [0:377];
	reg  [8:0] ap2_ptr;
	reg  [8:0] ap2_wptr;
	reg  signed [15:0] ap2_d;
	reg  signed [16:0] ap2_y_r;
	reg  signed [15:0] ap2_gx;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ap1_ptr <= 8'd0;  ap1_wptr <= 8'd0;  ap1_d <= 16'sd0;  ap1_y_r <= 17'sd0;  ap1_gx <= 16'sd0;
		end else begin
			if (tick) begin
				ap1_wptr <= ap1_ptr;	// 锁存写地址（必须先于 ptr 自增）
				ap1_d  <= warm_ap1 ? mem_ap1[ap1_ptr] : 16'sd0;
				ap1_ptr<= (ap1_ptr == 8'd248) ? 8'd0 : ap1_ptr + 8'd1;
			end
			if (st == 4'd8)
				ap1_gx <= m_res[33:15];					// gx = comb_mix x g >> 15
			if (st == 4'd9)
					ap1_y_r <= $signed({ap1_d[15], ap1_d}) - ap1_gx;
				if (st == 4'd11) begin
					mem_ap1[ap1_wptr] <= fx_rev ? sat16($signed(comb_mix_r)
					                     + $signed(m_res[33:15])) : 16'sd0;
				end
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ap2_ptr <= 9'd0;  ap2_wptr <= 9'd0;  ap2_d <= 16'sd0;  ap2_y_r <= 17'sd0;  ap2_gx <= 16'sd0;
		end else begin
			if (tick) begin
				ap2_wptr <= ap2_ptr;	// 锁存写地址
				ap2_d  <= warm_ap2 ? mem_ap2[ap2_ptr] : 16'sd0;
				ap2_ptr<= (ap2_ptr == 9'd377) ? 9'd0 : ap2_ptr + 9'd1;
			end
			if (st == 4'd11)
				ap2_gx <= m_res[33:15];		// ap2_gx 与 ap1_gy 是同一乘积，此处捕获
			if (st == 4'd13)
				ap2_y_r <= $signed({ap2_d[15], ap2_d}) - ap2_gx;
				if (st == 4'd14)
					mem_ap2[ap2_wptr] <= fx_rev ? sat16($signed({ap1_y_r[16], ap1_y_r})
					                     + $signed(m_res[33:15])) : 16'sd0;
		end
	end

	//=========================================================================
	// 回声缓冲写（拍 2 末）与延迟读
	//=========================================================================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			dly_wr  <= 14'd0;
			dly_old <= 16'sd0;
			dly_wet <= 16'sd0;
		end else begin
			if (tick) begin
				dly_old <= warm_dly ? mem_dly[dly_wr + 14'd1] : 16'sd0;
				dly_wr  <= dly_wr + 14'd1;
			end
				if (st == 4'd2)
					mem_dly[dly_wr - 14'd1] <= fx_dly ? sat16($signed({mono[15], mono})
					                            + $signed(m_res[33:15])) : 16'sd0;
			if (st == 4'd3)
				dly_wet <= m_res[33:15];
		end
	end

	//-------------------------------------------------------------------------
	// 乘数选择（按拍组合；引用的所有寄存器均已在前文声明）
	//-------------------------------------------------------------------------
	// 【定点】负数必须符号扩展：原写法 {2'b00,dly_old}/{1'b0,ap1_y_r} 是
	// 零扩展，负半周会变成 +65536 级的大正数 → 反馈被整流成单极性，
	// 尾巴就是"滋滋"电音而不是余音（实测回声重复段 100% 正值）。
	always @* begin
		case (st)
			4'd1:  begin m_a = $signed({{2{dly_old[15]}}, dly_old}); m_b = DLY_FB;  end
			4'd2:  begin m_a = $signed({{2{dly_old[15]}}, dly_old}); m_b = DLY_WET; end
			4'd3:  begin m_a = comb_out[0];                m_b = CMB_G;   end
			4'd4:  begin m_a = comb_out[1];                m_b = CMB_G;   end
			4'd5:  begin m_a = comb_out[2];                m_b = CMB_G;   end
			4'd6:  begin m_a = comb_out[3];                m_b = CMB_G;   end
			4'd7:  begin m_a = $signed(comb_mix_r);        m_b = AP_G;    end
			4'd10: begin m_a = $signed({ap1_y_r[16], ap1_y_r}); m_b = AP_G; end
			4'd13: begin m_a = $signed({ap2_y_r[16], ap2_y_r}); m_b = AP_G; end
			default: begin m_a = 18'sd0; m_b = 16'sd0; end
		endcase
	end

	//=========================================================================
	// 湿声汇总（开关门控）+ 干湿相加输出
	//=========================================================================
	// 混响湿声 x0.5（2026-10-31 二次听测"再明显一点"，由 0.375 提至 0.5）
	wire signed [17:0] rev_wet = ap2_y_r >>> 1;
	wire signed [17:0] wet_sum = (fx_dly ? $signed({dly_wet[15], dly_wet}) : 18'sd0)
	                           + (fx_rev ? $signed(rev_wet) : 18'sd0);

	reg signed [17:0] wet_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			wet_r <= 18'sd0;
		else if (st == 4'd14)
			wet_r <= wet_sum;
	end

	wire signed [18:0] out_l_sum = $signed({sample_l[15], sample_l}) + wet_r;
	wire signed [18:0] out_r_sum = $signed({sample_r[15], sample_r}) + wet_r;

	assign out_l = sat16(out_l_sum);
	assign out_r = sat16(out_r_sum);

	//=========================================================================
	// 序列器计数器
	//=========================================================================
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			st <= 4'd0;
		else if (tick)
			st <= 4'd1;							// 每 tick 重启序列
		else if (st != 4'd0 && st != 4'd14)
			st <= st + 4'd1;
		else
			st <= 4'd0;
	end

endmodule
