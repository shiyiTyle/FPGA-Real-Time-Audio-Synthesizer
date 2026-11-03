`timescale 1ns / 1ps
//=============================================================================
// tb_sim —— 拓展2 快速音频仿真
//
// 直驱 note_alloc.key_state（跳过键盘扫描），tick = 64 clk（逻辑内所有
// 时长仍以"每 tick 一个样本"计，因此相位/包络/颤音/回声长度与真机一致，
// 仅仿真墙钟时间加速 16 倍）。
//
// 输出 sim_dry.txt / sim_wet.txt：每行一个 16 位有符号样本（tick 序），
// 由 node 脚本转 WAV 直接试听 + 数值检查。
//
// 时间线（tick）：
//   1200~15848    S1 正弦基准（默认音色，按住 0.30s）
//   20848~80848   S13 钢琴 + S1 按住 1.23s（自然衰减到无声）
//   101848~181848 S14 长笛 + S1 按住 1.64s（柔攻 + 颤音 0.4s 渐入）
//   192848~195288 S16 延迟开 + 50ms 拨弦（听 ~0.335s 间隔回声）
//   230848~233288 S15 混响也开 + 50ms 拨弦（混响+回声尾）
//=============================================================================
module tb_sim;
	reg clk = 1'b0;
	reg rst_n = 1'b0;
	reg [15:0] key_state = 16'h0000;

	always #10 clk = ~clk;					// 50MHz

	reg [5:0] tick_cnt = 6'd0;
	wire      tick     = (tick_cnt == 6'd63);
	always @(posedge clk)
		tick_cnt <= tick_cnt + 6'd1;

	wire [3:0]   gate_v;
	wire [127:0] pinc_v;
	wire [63:0]  vel_v;
	wire [2:0]   timbre;
	wire [3:0]   env_sel_v;
	wire         fx_rev, fx_dly;
	reg  [7:0]   glide = 8'd0;		// 滑音电位器量
	reg  [7:0]   vib_man = 8'd0;	// 颤音电位器量
	wire signed [15:0] mix_l, mix_r, out_l, out_r;

	note_alloc u_alloc (
		.clk(clk), .rst_n(rst_n), .key_state(key_state),
		.gate_v(gate_v), .pinc_v(pinc_v), .vel_v(vel_v),
		.timbre_o(timbre), .env_sel_v(env_sel_v),
		.fx_rev(fx_rev), .fx_dly(fx_dly)
	);

	wire [127:0] pinc_g;

	porta_ctrl u_porta (
		.clk(clk), .rst_n(rst_n), .tick(tick),
		.glide(glide), .gate_v(gate_v),
		.pinc_in(pinc_v), .pinc_out(pinc_g)
	);

	synth_core #(
		.ATT_SAMPLES (98),     .DEC_SAMPLES (16000),
		.REL_SAMPLES (12207),  .SUS_LEVEL   (6554),
		.ATT_SAMPLES2(3906),   .DEC_SAMPLES2(9766),
		.REL_SAMPLES2(19531),  .SUS_LEVEL2  (22938),
		.CH_MODE(0)
	) u_synth (
		.clk(clk), .rst_n(rst_n), .tick(tick),
		.gate_v(gate_v), .pinc_v(pinc_g), .vel_v(vel_v),
		.env_sel_v(env_sel_v), .timbre_i(timbre), .vib_man(vib_man),
		.sample_l(mix_l), .sample_r(mix_r)
	);

	audio_fx u_fx (
		.clk(clk), .rst_n(rst_n), .tick(tick),
		.fx_rev(fx_rev), .fx_dly(fx_dly),
		.sample_l(mix_l), .sample_r(mix_r),
		.out_l(out_l), .out_r(out_r)
	);

	// 样本采集：流水线在 tick 后 8 clk 内稳定，取 tick_cnt==62（下一 tick 前）
	integer fd_d, fd_w, fd_p;
	reg [17:0] probe_cnt;
	always @(posedge clk) begin
		if (rst_n && tick_cnt == 6'd62) begin
			$fdisplay(fd_d, "%0d", mix_l);
			$fdisplay(fd_w, "%0d", out_l);
			probe_cnt <= probe_cnt + 18'd1;
		end
	end
	// 内部探针：滑音过渡段（S1->S5 切换后）看 porta cur 指数滑动曲线
	always @(posedge clk) begin
		if (rst_n && tick_cnt == 6'd62
		    && probe_cnt >= 310720 && probe_cnt <= 316000
		    && probe_cnt[3:0] == 4'd0)
			$fdisplay(fd_p, "%0d cur=%0d tgt=%0d gate=%b vib_off=%0d",
				probe_cnt, u_porta.g_porta[0].cur, u_porta.g_porta[0].tgt,
				gate_v, u_synth.g_voice[0].u_voice.vib_off);
	end
	// X 定位：dly_wet 首次出现 X 时打印效果器内部状态
	reg xfound = 1'b0;
	always @(posedge clk) begin
		if (rst_n && !xfound && (^u_fx.dly_wet === 1'bx)) begin
			xfound <= 1'b1;
			$display("XDET @%0d ticks: dly_wr=%0d dly_old=%h st=%0d m_a=%h m_b=%h m_res=%h warm=%0d fx_dly=%b fx_rev=%b",
				probe_cnt, u_fx.dly_wr, u_fx.dly_old, u_fx.st,
				u_fx.m_a, u_fx.m_b, u_fx.m_res, u_fx.warm, u_fx.fx_dly, u_fx.fx_rev);
			$display("XDET ap1_y_r=%h ap2_y_r=%h ap1_gx=%h ap2_gx=%h ap1_d=%h ap2_d=%h comb_mix=%h cout0=%h",
				u_fx.ap1_y_r, u_fx.ap2_y_r, u_fx.ap1_gx, u_fx.ap2_gx,
				u_fx.ap1_d, u_fx.ap2_d, u_fx.comb_mix_r, u_fx.g_comb[0].cout);
		end
	end

	task t_wait(input integer n);		// 等 n 个采样 tick（64 clk 一个）
		integer j;
		begin
			for (j = 0; j < n; j = j + 1)
				@(posedge tick);
		end
	endtask

	initial begin
		fd_d = $fopen("sim_dry.txt", "w");
		fd_w = $fopen("sim_wet.txt", "w");
		fd_p = $fopen("probe.txt", "w");
		probe_cnt = 18'd0;
		t_wait(200);
		rst_n = 1'b1;

		// 1. 正弦基准
		t_wait(1000);
		key_state = 16'h0001;				// S1 = G5
		t_wait(14648);
		key_state = 16'h0000;
		t_wait(4000);

		// 2. 钢琴
		key_state = 16'h1000;				// S13
		t_wait(1000);
		key_state = 16'h0001;
		t_wait(60000);
		key_state = 16'h0000;				// 松开时已近无声
		t_wait(20000);

		// 3. 长笛（后段把颤音旋钮拧到最大：手动颤音与自动颤音叠加）
		key_state = 16'h2000;				// S14
		t_wait(1000);
		key_state = 16'h0001;
		t_wait(58000);
		vib_man = 8'd255;					// 旋钮满档：+手动颤音
		t_wait(22000);
		vib_man = 8'd0;
		key_state = 16'h0000;
		t_wait(10000);

		// 4. 延迟回声（S16 开）
		key_state = 16'h8000;
		t_wait(1000);
		key_state = 16'h0001;
		t_wait(2440);						// 50ms 拨弦
		key_state = 16'h0000;
		t_wait(38000);

		// 5. 混响（S15 也开）
		key_state = 16'h4000;
		t_wait(1000);
		key_state = 16'h0001;
		t_wait(2440);
		key_state = 16'h0000;
		t_wait(40000);

		// 6. 滑音（portamento）：glide 拧大，按住 S1 后切 S5——从 G5 滑到 D6
		glide = 8'd224;						// sh=6+7=13，τ≈168ms
		t_wait(3000);
		key_state = 16'h0001;				// S1 = G5
		t_wait(30000);
		key_state = 16'h0010;				// 切 S5 = D6（声部保持，tgt 跳变 -> 滑音）
		t_wait(30000);
		key_state = 16'h0000;
		glide = 8'd0;
		t_wait(10000);

		$display("TB_SIM_DONE");
		$fclose(fd_d);
		$fclose(fd_w);
		$fclose(fd_p);
		$finish;
	end
endmodule
