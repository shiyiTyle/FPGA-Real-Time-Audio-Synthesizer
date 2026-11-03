`timescale 1ns / 1ps
//=============================================================================
// viz_scope —— 实时可视化（拓展3：视听反馈）· 资源精简版
//
// 屏幕布局（240x240 竖屏）：
//   y   0..111   波形：240 列，每列画一条 3 像素高的短竖线（点串成波形）
//   y 112..215   16 根频谱柱（由 viz_spec 提供柱高）
//   y 216..239   16 格演奏指引 / 状态条
//
// 【为什么是"每列一个点"而不是 min/max 包络】
//   本项目音频引擎（4 声部 + 钢琴/长笛多分量 + 混响延迟）在 GW5A-25 上
//   已占 71% 逻辑，可视化必须极省。min/max 包络 + 逐列擦旧段的写法
//   实测要 2,786 逻辑（占 12%），把总量推到 94% 直接布线失败（PR0004）。
//   改为每列 1 个采样点 + 局部擦除后，逻辑降到几百，总量回到可布线区。
//   240 列对 960 样本是 1/4 抽取，G6(1568Hz) 在 960 样本里有 ~30 个周期、
//   每周期 ~8 个点，波形形状仍然清楚。
//
// 擦除策略：每列记住上一帧的点位 prev_y[col]，只擦旧点、画新点，
//   点位没变就跳过 —— 不会像早期版本那样绿色越积越多。
//=============================================================================
module viz_scope #(
	parameter integer SCOPE_Y0 = 0,		// 波形区上沿
	parameter integer SCOPE_H  = 112,	// 波形区高度（偶数）
	parameter integer KEY_Y0   = 216,	// 状态条上沿
	parameter integer KEY_H    = 24,	// 状态条高度
	parameter integer SPEC_Y0  = 112,	// 频谱区上沿
	parameter integer SPEC_Y1  = 215,	// 频谱区下沿（柱底）
	parameter integer TRIG_TMO = 1200	// 触发超时（样本数，≈25ms）
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire signed [15:0] audio,
	input  wire        audio_tick,
	input  wire [15:0] key_state,
	input  wire [2:0]  timbre,
	input  wire        fx_rev,
	input  wire        fx_dly,
	input  wire [111:0] bar_flat,		// 16 x 7bit 频谱柱高
	input  wire        bar_upd,
	input  wire [15:0] adc0,			// ADS 三通道原始读数（调试电平条）
	input  wire [15:0] adc1,
	input  wire [15:0] adc2,
	input  wire        dbg_adc,		// 1=按住核心板 S1，状态条改显 ADC 电平
	output reg         go,
	output reg  [7:0]  x0,
	output reg  [7:0]  y0,
	output reg  [7:0]  x1,
	output reg  [7:0]  y1,
	output reg         solid,
	output reg  [15:0] col,
	input  wire        drv_ready,
	input  wire        drv_busy,
	output reg  [15:0] frames
);

	localparam [15:0] C_BG      = 16'h0000;
	localparam [15:0] C_TRACE   = 16'h07E0;		// 波形亮绿
	localparam [15:0] C_KEY_ON  = 16'h07E0;
	localparam [15:0] C_KEY_OFF = 16'h1082;
	localparam [15:0] C_TMB_ON  = 16'h07FF;
	localparam [15:0] C_FX_ON   = 16'hFFE0;
	localparam [15:0] C_IDLE    = 16'h1082;
	localparam [15:0] C_SPEC    = 16'h2C9F;

	localparam integer MID = SCOPE_H / 2;
	localparam signed [24:0] MID25 = MID;
	localparam [7:0] SH1 = SCOPE_H - 1;
	localparam [7:0] KY0 = KEY_Y0;
	localparam [7:0] KY1 = KEY_Y0 + KEY_H - 1;

	//-------------------------------------------------------------------------
	// 每帧点位（8 位行号）与上一帧点位
	//-------------------------------------------------------------------------
	reg  [7:0] col_y  [0:239];
	reg  [7:0] prev_y [0:239];
	reg  [7:0] pv_q;
	reg  [7:0] ylast, pvlast;		// 上一列的新/旧点位（用于连成折线）
	reg  [8:0] cap_wr;				// 480 样本（240 列 x 2）
	reg  [15:0] peak;
	reg  [3:0]  gain;

	reg signed [15:0] prev_s;
	// 【时序】输入先过一级寄存：音频链路（混音+效果）到这里的组合路径
	// 原本 21.6ns（建立违例 -1.95ns）。屏显示晚 20µs 完全不可见。
	reg signed [15:0] audio_r;
	reg               tick_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			audio_r <= 16'sd0;
			tick_r  <= 1'b0;
		end else begin
			audio_r <= audio;
			tick_r  <= audio_tick;
		end
	end
	wire        trig = (prev_s < 16'sd0) && (audio_r >= 16'sd0);
	reg  [11:0] tmo;

	wire signed [15:0] abs_s = audio_r[15] ? -audio_r : audio_r;

	// 采样点 -> 屏幕行（用上一帧的 gain）
	wire signed [24:0] asc   = $signed(audio_r) <<< gain;
	wire signed [24:0] yc    = MID25 - (asc >>> 9);
	wire [7:0]         ynow  = (yc < 0) ? 8'd0 : (yc > SH1) ? SH1 : yc[7:0];

	//-------------------------------------------------------------------------
	// 流程
	//-------------------------------------------------------------------------
	localparam [2:0] P_CLEAR=3'd0, P_ARM=3'd1, P_CAP=3'd2,
	                 P_GAIN=3'd3, P_DRAW=3'd4, P_SPEC=3'd5, P_KEYS=3'd6;

	reg [2:0]  ph;
	reg [7:0]  col_i;
	reg [3:0]  cell_i;
	reg [2:0]  ds;
	reg [2:0]  cds;
	reg [3:0]  spec_i;
	reg [2:0]  sds;
	reg [7:0]  ny;				// 本列新点位
	reg [15:0] cell_prev [0:15];
	reg [6:0]  prev_bar [0:15];
	reg [6:0]  nh, oh;
	reg [7:0]  nt_top, ot_top;
	reg        nhz;
	integer i;

	wire [11:0] cx0 = {cell_i, 4'd0} - {4'd0, cell_i};		// cell*15
	wire [11:0] cx1 = {cell_i, 4'd0} - {4'd0, cell_i} + 12'd14;
	wire [11:0] bx0 = {spec_i, 4'd0} - {4'd0, spec_i};
	wire [11:0] bx1 = {spec_i, 4'd0} - {4'd0, spec_i} + 12'd14;

	wire tmb_sel = ({1'b0, timbre} + 4'd8) == {1'b0, cell_i};
	reg [15:0] cell_col;

	// ADC 调试电平条（按住核心板 S1 时状态条变为三通道 5 级电平表）：
	//   格 0~4 = A0（FSR，绿）  格 5 间隔（黑）  格 6~10 = A1 滑音（青）
	//   格 11~15 = A2 颤音（黄）。级距 41（3.3V 满量程读数 ≈206）。
	// 排查电位器接线：转哪个旋钮、哪组条就该动；悬空时该组会随机闪烁。
	wire [7:0] adc_lvl0 = adc0[15:7];
	wire [7:0] adc_lvl1 = adc1[15:7];
	wire [7:0] adc_lvl2 = adc2[15:7];
	reg [15:0] cell_col_dbg;
	always @* begin
		case (cell_i)
			4'd0:  cell_col_dbg = (adc_lvl0 >= 8'd41)  ? C_KEY_ON : C_IDLE;
			4'd1:  cell_col_dbg = (adc_lvl0 >= 8'd82)  ? C_KEY_ON : C_IDLE;
			4'd2:  cell_col_dbg = (adc_lvl0 >= 8'd123) ? C_KEY_ON : C_IDLE;
			4'd3:  cell_col_dbg = (adc_lvl0 >= 8'd164) ? C_KEY_ON : C_IDLE;
			4'd4:  cell_col_dbg = (adc_lvl0 >= 8'd205) ? C_KEY_ON : C_IDLE;
			4'd5:  cell_col_dbg = C_BG;
			4'd6:  cell_col_dbg = (adc_lvl1 >= 8'd41)  ? C_TMB_ON : C_IDLE;
			4'd7:  cell_col_dbg = (adc_lvl1 >= 8'd82)  ? C_TMB_ON : C_IDLE;
			4'd8:  cell_col_dbg = (adc_lvl1 >= 8'd123) ? C_TMB_ON : C_IDLE;
			4'd9:  cell_col_dbg = (adc_lvl1 >= 8'd164) ? C_TMB_ON : C_IDLE;
			4'd10: cell_col_dbg = (adc_lvl1 >= 8'd205) ? C_TMB_ON : C_IDLE;
			4'd11: cell_col_dbg = (adc_lvl2 >= 8'd41)  ? C_FX_ON : C_IDLE;
			4'd12: cell_col_dbg = (adc_lvl2 >= 8'd82)  ? C_FX_ON : C_IDLE;
			4'd13: cell_col_dbg = (adc_lvl2 >= 8'd123) ? C_FX_ON : C_IDLE;
			4'd14: cell_col_dbg = (adc_lvl2 >= 8'd164) ? C_FX_ON : C_IDLE;
			default: cell_col_dbg = (adc_lvl2 >= 8'd205) ? C_FX_ON : C_IDLE;
		endcase
	end

	always @* begin
		if (dbg_adc)
			cell_col = cell_col_dbg;
		else if (cell_i < 4'd8)
			cell_col = key_state[cell_i] ? C_KEY_ON : C_KEY_OFF;
		else if (cell_i < 4'd14)
			cell_col = tmb_sel ? C_TMB_ON : C_IDLE;
		else if (cell_i == 4'd14)
			cell_col = fx_rev ? C_FX_ON : C_IDLE;
		else
			cell_col = fx_dly ? C_FX_ON : C_IDLE;
	end

	// 折线端点：本列与上一列的点位取 min/max，连成连续波形。
	// 第 0 列没有"上一列"，必须用本列自己的点；否则每帧都会从 y=0
	// 画一条贯穿半个屏幕的竖线（就是屏左侧那条竖线）。
	wire [7:0] ylast_e  = (col_i == 8'd0) ? ny   : ylast;
	wire [7:0] pvlast_e = (col_i == 8'd0) ? pv_q : pvlast;
	wire [7:0] new_lo = (ylast_e < ny) ? ylast_e : ny;
	wire [7:0] new_hi = (ylast_e > ny) ? ylast_e : ny;
	wire [7:0] old_lo = (pvlast_e < pv_q) ? pvlast_e : pv_q;
	wire [7:0] old_hi = (pvlast_e > pv_q) ? pvlast_e : pv_q;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ph <= P_CLEAR; ds <= 3'd0; cds <= 3'd0; go <= 1'b0;
			x0 <= 8'd0; y0 <= 8'd0; x1 <= 8'd239; y1 <= 8'd239;
			solid <= 1'b1; col <= C_BG;
			col_i <= 8'd0; cell_i <= 4'd0; spec_i <= 4'd0; sds <= 3'd0;
			cap_wr <= 10'd0; prev_s <= 16'sd0; tmo <= 12'd0;
			peak <= 16'd0; gain <= 4'd0;
			pv_q <= 8'd0; ny <= 8'd0; ylast <= 8'd0; pvlast <= 8'd0;
			nh <= 7'd0; oh <= 7'd0; nt_top <= 8'd0; ot_top <= 8'd0; nhz <= 1'b0;
			frames <= 16'd0;
			for (i = 0; i < 16; i = i + 1) begin
				cell_prev[i] <= 16'hFFFF;
				prev_bar[i]  <= 7'd0;
			end
			// 注意：col_y / prev_y 不能写复位循环，否则综合器无法推断成
			// 分布式 RAM，会退化成 480 个寄存器 + 选择器（实测把逻辑推到 95%）。
			// 这两个数组都是"先写后读"，无需初始化。
		end else begin
			if (tick_r) begin
				prev_s <= audio_r;
				if (ph == P_CAP && abs_s > peak) peak <= abs_s;
			end

			case (ph)
			//-------------------------------------------------------------
			P_CLEAR: begin
				if (!go && drv_ready) begin
					go <= 1'b1;
					x0 <= 8'd0; y0 <= 8'd0; x1 <= 8'd239; y1 <= 8'd239;
					solid <= 1'b1; col <= C_BG;
				end else if (go && drv_busy) begin
					go <= 1'b0;
					// 首帧擦除：把 prev_y 设成无效值靠"点位不同"自然重画，
					// 这里直接把 prev 全设为 0xFF 不行（会画到 SH1），
					// 用 prv 有效标志代替：首帧跳过一次擦除即可
					ph <= P_ARM;
				end
			end
			//-------------------------------------------------------------
			P_ARM: begin
				if (tick_r) begin
					if (trig || tmo >= TRIG_TMO - 1) begin
						tmo <= 12'd0; cap_wr <= 10'd0; peak <= 16'd0; ph <= P_CAP;
					end else
						tmo <= tmo + 12'd1;
				end
			end
			//-------------------------------------------------------------
			P_CAP: begin
				if (tick_r) begin
					if (cap_wr[0] == 1'b0)			// 每 2 个样本取 1 列
						col_y[cap_wr[8:1]] <= ynow;
					cap_wr <= cap_wr + 9'd1;
					if (cap_wr == 9'd479) begin
						col_i  <= 8'd0;
						ylast  <= 8'd0;
						pvlast <= 8'd0;
						ph     <= P_GAIN;
					end
				end
			end
			//-------------------------------------------------------------
			P_GAIN: begin
				if (peak >= 16'd12800)      gain <= 4'd0;
				else if (peak >= 16'd6400)  gain <= 4'd1;
				else if (peak >= 16'd3200)  gain <= 4'd2;
				else if (peak >= 16'd1600)  gain <= 4'd3;
				else if (peak >= 16'd800)   gain <= 4'd4;
				else if (peak >= 16'd400)   gain <= 4'd5;
				else if (peak >= 16'd200)   gain <= 4'd6;
				else                        gain <= 4'd7;
				col_i  <= 8'd0;
				cell_i <= 4'd0;
				ds     <= 3'd0;
				cds    <= 3'd0;
				ylast  <= 8'd0;			// 必须复位：否则首帧 X 会让 y0/y1 变成 X，
				pvlast <= 8'd0;			// 驱动侧像素数下溢成 65535，屏卡死
				ph     <= P_DRAW;
			end
			//-------------------------------------------------------------
			// 每列：擦旧点（若位置变了）+ 画新点
			P_DRAW: begin
				if (col_i < 8'd240) begin
					case (cds)
					3'd0: begin
						pv_q <= prev_y[col_i];
						ny   <= col_y[col_i];
						cds  <= 3'd1;
					end
					3'd1: begin
						if (old_lo == new_lo && old_hi == new_hi && frames != 16'd0) begin
							ylast  <= ny;			// 段未变，跳过绘制
							pvlast <= pv_q;
							col_i  <= col_i + 8'd1;
							cds    <= 3'd0;
						end else
							cds <= 3'd2;
					end
					3'd2: begin
						// 先擦旧折线段
						if (frames == 16'd0) begin
							cds <= 3'd4;				// 首帧无需擦
						end else if (!go && drv_ready) begin
							go  <= 1'b1;
							x0  <= col_i; x1 <= col_i;
							y0  <= old_lo; y1 <= old_hi;
							solid <= 1'b1;
							col <= C_BG;
							cds <= 3'd3;
						end
					end
					3'd3: begin
						if (go && drv_busy) begin go <= 1'b0; cds <= 3'd5; end
					end
					3'd4: begin
						// 画新折线段
						if (!go && drv_ready) begin
							go  <= 1'b1;
							x0  <= col_i; x1 <= col_i;
							y0  <= new_lo; y1 <= new_hi;
							solid <= 1'b1;
							col <= C_TRACE;
							cds <= 3'd6;
						end
					end
					3'd5: begin
						if (!drv_busy && drv_ready) cds <= 3'd4;
					end
					default: begin
						if (go && drv_busy) begin
							go <= 1'b0;
							prev_y[col_i] <= ny;
							ylast  <= ny;
							pvlast <= pv_q;
							col_i  <= col_i + 8'd1;
							cds    <= 3'd0;
						end
					end
					endcase
				end else begin
					ph     <= P_SPEC;
					spec_i <= 4'd0;
					sds    <= 3'd0;
				end
			end
			//-------------------------------------------------------------
			P_SPEC: begin
				if (spec_i == 4'd16) begin
					ph <= P_KEYS; cell_i <= 4'd0; ds <= 3'd0;
				end else begin
					case (sds)
					3'd0: begin
						nh <= bar_flat[spec_i*7 +: 7];
						oh <= prev_bar[spec_i];
						sds <= 3'd1;
					end
					3'd1: begin
						nt_top <= SPEC_Y1[7:0] + 8'd1 - {1'b0, nh};
						ot_top <= SPEC_Y1[7:0] + 8'd1 - {1'b0, oh};
						nhz    <= (nh != 7'd0);
						sds    <= 3'd2;
					end
					3'd2: begin
						if (nh == oh) begin
							spec_i <= spec_i + 4'd1; sds <= 3'd0;
						end else if (!go && drv_ready) begin
							go  <= 1'b1;
							x0  <= bx0[7:0]; x1 <= bx1[7:0];
							y0  <= (nt_top < ot_top) ? nt_top : ot_top;
							y1  <= SPEC_Y1[7:0];
							solid <= 1'b1;
							col <= C_BG;
							sds <= 3'd3;
						end
					end
					3'd3: begin
						if (go && drv_busy) begin go <= 1'b0; sds <= 3'd4; end
					end
					3'd4: begin
						if (!drv_busy && drv_ready) begin
							if (!nhz) begin
								prev_bar[spec_i] <= 7'd0;
								spec_i <= spec_i + 4'd1;
								sds <= 3'd0;
							end else
								sds <= 3'd5;
						end
					end
					3'd5: begin
						if (!go && drv_ready) begin
							go  <= 1'b1;
							x0  <= bx0[7:0]; x1 <= bx1[7:0];
							y0  <= nt_top; y1 <= SPEC_Y1[7:0];
							solid <= 1'b1;
							col <= C_SPEC;
							sds <= 3'd6;
						end
					end
					3'd6: begin
						if (go && drv_busy) begin go <= 1'b0; sds <= 3'd7; end
					end
					default: begin
						if (!drv_busy && drv_ready) begin
							prev_bar[spec_i] <= nh;
							spec_i <= spec_i + 4'd1;
							sds <= 3'd0;
						end
					end
					endcase
				end
			end
			//-------------------------------------------------------------
			P_KEYS: begin
				if (cell_i < 4'd16) begin
					case (ds)
					3'd0: begin
						if (cell_col != cell_prev[cell_i]) ds <= 3'd1;
						else cell_i <= cell_i + 4'd1;
					end
					3'd1: begin
						if (!go && drv_ready) begin
							go  <= 1'b1;
							x0  <= cx0[7:0]; x1 <= cx1[7:0];
							y0  <= KY0; y1 <= KY1;
							solid <= 1'b1;
							col <= cell_col;
							ds  <= 3'd2;
						end
					end
					3'd2: begin
						if (go && drv_busy) begin go <= 1'b0; ds <= 3'd3; end
					end
					default: begin
						if (!drv_busy && drv_ready) begin
							cell_prev[cell_i] <= cell_col;
							cell_i <= cell_i + 4'd1;
							ds <= 3'd0;
						end
					end
					endcase
				end else begin
					frames <= frames + 16'd1;
					col_i  <= 8'd0;
					cell_i <= 4'd0;
					ds     <= 3'd0;
					cds    <= 3'd0;
					ph     <= P_ARM;
				end
			end
			default: ph <= P_ARM;
			endcase
		end
	end

endmodule
