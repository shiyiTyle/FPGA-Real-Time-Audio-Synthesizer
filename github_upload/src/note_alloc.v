`timescale 1ns / 1ps
//=============================================================================
// note_alloc —— 琴键到声部的分配器 + 全局音色选择 + 效果开关
//
// 音符键（按下占用空闲声部，最多 4 音复音，松开释放进入 ADSR 释放段）：
//   S1~S8   (k=0~7)  : G5 A5 B5 C6 D6 E6 F#6 G6   主音域（哆来咪发唆啦西多）
// 原始波形音色键（按一下全局切换，不占用声部；上电默认正弦）：
//   S9  = 正弦（f）   S10 = 窄脉冲（2f）
//   S11 = 锯齿（f）   S12 = 方波（f/2）
// 乐器音色键（拓展2：按一下切到乐器音色，同样全局生效、不占声部）：
//   S13 = 钢琴（加法合成 3 分量，各自独立指数衰减，亮攻击自然衰减）
//   S14 = 长笛（正弦+二次谐波+延迟渐入颤音+起音气声）
// 效果开关键（按一下开、再按一下关；效果器干湿并联，不影响演奏延迟）：
//   S15 = 混响开/关   S16 = 延迟回声开/关
//
// 音高表（G 大调，等律 A4=440，PINC = 2^32 x f / 48828.125）
//=============================================================================
module note_alloc #(
	parameter integer BASE_VEL = 10000	// 每声部力度（4 音同峰 40000，饱和限幅兜底）
)(
	input  wire         clk,
	input  wire         rst_n,
	input  wire [15:0]  key_state,		// 消抖键态（来自 keypad_scan）
	output wire [3:0]   gate_v,
	output wire [127:0] pinc_v,
	output wire [63:0]  vel_v,
	output wire [2:0]   timbre_o,		// 当前音色（0正弦 1窄脉冲 2锯齿 3方波 4钢琴 5长笛）
	output wire [3:0]   env_sel_v,		// 保留接口：S13~S16 已改乐器/效果键，恒 0（主包络）
	output wire         fx_rev,			// 混响开关（1=开）
	output wire         fx_dly			// 延迟回声开关（1=开）
);

	//-------------------------------------------------------------------------
	// 音高查表：音符键只有 S1~S8（k=0~7）
	//-------------------------------------------------------------------------
	function [31:0] pinc_lut;
		input [3:0] k;
		reg [31:0] base;
		begin
			case (k[2:0])
				3'd0: base = 32'd68960566;		// G5  = 783.99 Hz（哆）
				3'd1: base = 32'd77405619;		// A5  = 880.00 Hz（来）
				3'd2: base = 32'd86884869;		// B5  = 987.77 Hz（咪）
				3'd3: base = 32'd92051312;		// C6  = 1046.50 Hz（发）
				3'd4: base = 32'd103324104;		// D6  = 1174.66 Hz（唆）
				3'd5: base = 32'd115977386;		// E6  = 1318.51 Hz（啦）
				3'd6: base = 32'd130180216;		// F#6 = 1399.98 Hz（西）
				default: base = 32'd137921132;	// G6  = 1567.98 Hz（哆）
			endcase
			pinc_lut = base;
		end
	endfunction

	//-------------------------------------------------------------------------
	// 声部分配表
	//-------------------------------------------------------------------------
	reg [3:0]  v_note [0:3];	// 各声部当前音符编号（键序号 0~7）
	reg [3:0]  v_active;		// 各声部占用标志
	reg [15:0] key_prev;
	reg [1:0]  steal_ptr;		// 抢占轮转指针
	reg [2:0]  timbre_r;		// 当前音色
	reg        fx_rev_r;
	reg        fx_dly_r;

	wire [15:0] press_edge   = key_state & ~key_prev;
	wire [15:0] release_edge = ~key_state & key_prev;

	//-------------------------------------------------------------------------
	// 分配逻辑：一个 clk 内可能有多条边（同轮扫描多键），用阻塞临时变量
	// 顺序处理后再统一提交；S9~S16 是音色/效果键，不参与声部分配
	//-------------------------------------------------------------------------
	integer k;
	reg [3:0] na0, na1, na2, na3;
	reg [3:0] nact;
	reg [1:0] nsteal;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			v_note[0] <= 4'd0; v_note[1] <= 4'd0;
			v_note[2] <= 4'd0; v_note[3] <= 4'd0;
			v_active  <= 4'd0;
			key_prev  <= 16'd0;
			steal_ptr <= 2'd0;
			timbre_r  <= 3'd0;				// 上电默认正弦
			fx_rev_r  <= 1'b0;				// 上电效果全关（干声）
			fx_dly_r  <= 1'b0;
		end else begin
			na0 = v_note[0]; na1 = v_note[1];
			na2 = v_note[2]; na3 = v_note[3];
			nact = v_active; nsteal = steal_ptr;

			for (k = 0; k < 8; k = k + 1) begin
				if (press_edge[k]) begin
					if (!nact[0])      begin na0 = k[3:0]; nact[0] = 1'b1; end
					else if (!nact[1]) begin na1 = k[3:0]; nact[1] = 1'b1; end
					else if (!nact[2]) begin na2 = k[3:0]; nact[2] = 1'b1; end
					else if (!nact[3]) begin na3 = k[3:0]; nact[3] = 1'b1; end
					else begin
						// 4 声部全忙：轮转抢占
						case (nsteal)
							2'd0: begin na0 = k[3:0]; end
							2'd1: begin na1 = k[3:0]; end
							2'd2: begin na2 = k[3:0]; end
							2'd3: begin na3 = k[3:0]; end
						endcase
						nsteal = nsteal + 1'b1;
					end
				end
				if (release_edge[k]) begin
					if ( nact[0] && na0 == k[3:0]) nact[0] = 1'b0;
					if ( nact[1] && na1 == k[3:0]) nact[1] = 1'b0;
					if ( nact[2] && na2 == k[3:0]) nact[2] = 1'b0;
					if ( nact[3] && na3 == k[3:0]) nact[3] = 1'b0;
				end
			end

			// 原始波形音色键：后按的生效
			if (press_edge[8])  timbre_r <= 3'd0;	// S9  正弦（f）
			if (press_edge[9])  timbre_r <= 3'd1;	// S10 窄脉冲（2f）
			if (press_edge[10]) timbre_r <= 3'd2;	// S11 锯齿（f）
			if (press_edge[11]) timbre_r <= 3'd3;	// S12 方波（f/2）
			// 乐器音色键
			if (press_edge[12]) timbre_r <= 3'd4;	// S13 钢琴
			if (press_edge[13]) timbre_r <= 3'd5;	// S14 长笛
			// 效果开关：边沿翻转
			if (press_edge[14]) fx_rev_r <= ~fx_rev_r;	// S15 混响
			if (press_edge[15]) fx_dly_r <= ~fx_dly_r;	// S16 延迟

			v_note[0] <= na0; v_note[1] <= na1;
			v_note[2] <= na2; v_note[3] <= na3;
			v_active  <= nact;
			steal_ptr <= nsteal;
			key_prev  <= key_state;
		end
	end

	assign gate_v = v_active;
	assign pinc_v = {pinc_lut(v_note[3]), pinc_lut(v_note[2]),
	                 pinc_lut(v_note[1]), pinc_lut(v_note[0])};

	localparam [15:0] VEL = BASE_VEL;
	assign vel_v  = {4{VEL}};

	assign timbre_o  = timbre_r;
	assign env_sel_v = 4'b0;	// S13~S16 不再发声，主包络恒选
	assign fx_rev    = fx_rev_r;
	assign fx_dly    = fx_dly_r;

endmodule
