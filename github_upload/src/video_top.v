`timescale 1ns / 1ps
//=============================================================================
// video_top —— 电子乐器引擎顶层
//
// 板卡：Tang Primer 25K（GW5A-25）+ 60033 Dock
// DAC ：MAX98357A I2S 功放（PCM5102 亦兼容，见 README）
//
// 结构：
//   keypad_scan / ttp229_drv（选音+触发）-> note_alloc（4 声部分配）
//     或 demo_seq（自动演示）-> bend_ctrl（FSR 压力弯音，ADS1115 采集）
//     -> synth_core（4 声部合成混音） -> audio_fx（混响/延迟，干湿并联）
//     -> i2s_tx（I2S 发送）
//
// 演奏方式（PLAY_MODE = 0，默认；两个独立演奏维度）：
//   维度1 选音+触发：4x4 矩阵键盘（HW-834，后续可换 TTP229 触摸板）：
//     S1~S8   = G 大调哆来咪发唆啦西多（G5~G6，主音域）
//     S9~S12  = 音色切换：正弦/高八度脉冲/锯齿/低八度方波（默认正弦）
//     S13     = 钢琴音色（加法合成 3 分量，各自独立指数衰减）
//     S14     = 长笛音色（正弦+二次谐波+延迟渐入颤音+起音气声）
//     S15     = 混响开/关（按一下切换）
//     S16     = 延迟回声开/关（按一下切换）
//   维度2 表情：FSR402 -> LM393 模块 AO -> ZY-ADS1115（I2C，SCL=A11/SDA=E11）
//     压力越大音高越高（满压力升 BEND_SEMI 个半音），松开回琴键本音；
//     ADS 采样 860SPS，压力->音高更新 ≈2ms；断线看门狗自动回本音
//   钢琴包络：2ms 起音 / 330ms 衰减 / 20% 延音 / 250ms 释放
//   最多 4 音复音；按键触发->发声 ≈4.5ms、压力->音高更新 ≈2ms，
//   均满足赛题 <=10ms
// PLAY_MODE = 1：自动演示（左 C5 / 右 E5 交替拨弦，核心板 S2 复位音序）
//
// 引脚（见 fpga_project.cst）：
//   sys_clk  E2      板载 50MHz 晶振
//   kbd_row  D10/B10/G10/C10  键盘行 R1~R4（扫描低有效，空闲高阻）
//   kbd_col  G11/D11/B11/C11  键盘列 C1~C4（内部上拉，按下=低）
//   key_s2   H10     核心板按键 S2（演示模式下复位音序）
//   i2s_bclk G7  -> DAC BCLK / i2s_lrclk H5 -> LRC / i2s_data H8 -> DIN
//   i2s_mclk F5  -> DAC SCK（MAX98357A 无此脚，不接）
//   ads_scl  A11 -> ADS1115 SCL / ads_sda E11 <-> ADS1115 SDA（开漏）
//   tp_scl   L2 / tp_sda K1  TTP229 触摸板（INPUT_SRC=1 时启用）
//=============================================================================
module video_top #(
	parameter integer PLAY_MODE   = 0,	// 0：键盘演奏；1：自动演示
	parameter integer INPUT_SRC   = 0,	// 0：4x4 矩阵键盘；1：TTP229 电容触摸板
	parameter integer POT_EN      = 1	// 1：已接电位器——A1=滑音旋钮，A2 短接 GND（颤音恒 0）
										// 0：**拔掉电位器时必须改回 0**，此时滑音/颤音彻底
										//    旁路（音色与最初一致）。A1/A2 悬空会读到乱数，
										//    保持 1 而不接电位器，每个音都会滑音+颤音，
										//    听起来完全不像乐器（已实际踩过）
)(
	input  wire       sys_clk ,	// 50MHz（E2）
	input  wire [3:0] kbd_col ,	// 键盘列 C1~C4（按下=低）
	output wire [3:0] kbd_row ,	// 键盘行 R1~R4（扫描输出）
	input  wire       key_s1  ,	// 核心板按键 S1（键盘模式下未用，保留）
	input  wire       key_s2  ,	// 核心板按键 S2（演示模式复位）
	inout  wire       ads_sda ,	// ADS1115 I2C 数据（开漏，E11）
	output wire       ads_scl ,	// ADS1115 I2C 时钟（A11）
	output wire       tp_scl  ,	// TTP229 触摸板 SCL（L2）
	input  wire       tp_sda  ,	// TTP229 触摸板 SDA（K1，内部上拉）

	output wire  i2s_bclk ,
	output wire  i2s_lrclk ,
	output wire  i2s_data ,
	output wire  i2s_mclk ,

	// ST7789 240x240 IPS（拓展3 可视化；接顶部 40 针排针右端）
	output wire  lcd_scl  ,		// L4
	output wire  lcd_sda  ,		// G2
	output wire  lcd_res  ,		// J4
	output wire  lcd_dc   ,		// G1
	output wire  lcd_cs   ,		// K4

	output wire  dbg_lat	// 调试：任一音符声部触发=1（G5，测总延迟用，可不接）
);

		//-------------------------------------------------------------------------
		// 上电复位：计数器链上电默认为 0，数到高位为 1 后释放（约 0.66ms）。
		// 声明初始化保证仿真与 FPGA 上电值一致（reg 仿真默认 X，X+1 恒 X）
		//-------------------------------------------------------------------------
		reg [15:0] por_cnt = 16'd0;
	always @(posedge sys_clk)
		por_cnt <= (por_cnt == 16'hFFFF) ? 16'hFFFF : por_cnt + 1'b1;
	wire rst_n = por_cnt[15];

	//-------------------------------------------------------------------------
	// 核心板按键 S2 同步 + 消抖（演示模式的音序复位用）
	//-------------------------------------------------------------------------
	reg [1:0]  s2_sr;
	reg        s2_db;
	reg [18:0] s2_cnt;

	always @(posedge sys_clk or negedge rst_n) begin
		if (!rst_n) begin
			s2_sr  <= 2'b00;
			s2_db  <= 1'b0;
			s2_cnt <= 19'd0;
		end else begin
			s2_sr <= {s2_sr[0], key_s2};
			if (s2_sr[1] == s2_db)
				s2_cnt <= 19'd0;
			else if (s2_cnt == 19'd500000) begin
				s2_db  <= s2_sr[1];
				s2_cnt <= 19'd0;
			end else
				s2_cnt <= s2_cnt + 1'b1;
		end
	end

	//-------------------------------------------------------------------------
	// 演奏输入：4x4 矩阵键盘 / TTP229 电容触摸板（INPUT_SRC 二选一，
	// 输出同为 key_state[15:0]，未用分支被综合器裁剪）
	//-------------------------------------------------------------------------
	wire [15:0] key_state;
	wire [15:0] key_state_kpad;
	wire [15:0] key_state_tp;
	wire [3:0]   gate_k;
	wire [127:0] pinc_k;
	wire [63:0]  vel_k;
	wire [2:0]   timbre;		// 0正弦 1窄脉冲 2锯齿 3方波 4钢琴 5长笛
	wire [3:0]   env_sel_v;
	wire         fx_rev;		// 混响开关（S15）
	wire         fx_dly;		// 延迟回声开关（S16）

	keypad_scan #(
		.ROW_SETTLE (5000),		// 0.1ms/行（10 倍 RC 时间常数，长引线也稳），整轮 0.4ms
		.DBNC       (8)			// 8 次 @ 2.5kHz = 3.2ms（与原版完全相同的积分窗口）
	) u_kbd (
		.clk       (sys_clk),
		.rst_n     (rst_n),
		.row_n     (kbd_row),
		.col_n     (kbd_col),
		.key_state (key_state_kpad)
	);

	ttp229_drv #(
		.CLK_HALF  (250),		// SCL 半周期 5us
		.READ_GAP  (50000),		// 两次读出间隔 1ms
		.FIRST_KEY (0)			// 键位整体镜像时改 15
	) u_tp (
		.clk       (sys_clk),
		.rst_n     (rst_n),
		.scl       (tp_scl),
		.sda       (tp_sda),
		.key_state (key_state_tp)
	);

	generate
		if (INPUT_SRC == 1) begin : g_src_tp
			assign key_state = key_state_tp;
		end else begin : g_src_kpad
			assign key_state = key_state_kpad;
		end
	endgenerate

	note_alloc #(
		.BASE_VEL (10000)		// 单声部力度；4 音同峰 40000 由饱和限幅兜底
	) u_alloc (
		.clk       (sys_clk),
		.rst_n     (rst_n),
		.key_state (key_state),
		.gate_v    (gate_k),
		.pinc_v    (pinc_k),
		.vel_v     (vel_k),
		.timbre_o  (timbre),
		.env_sel_v (env_sel_v),
		.fx_rev    (fx_rev),
		.fx_dly    (fx_dly)
	);

		//-------------------------------------------------------------------------
		// FSR 压力 -> 音高弯音（第二演奏维度：连续音高表情）
		//   维度扩展（拓展2）：ADS1115 三通道轮询
		//   AIN0=FSR（弯音）  AIN1=滑音电位器  AIN2=颤音电位器
		//   实测方向：松开 AO≈3.25V（码值≈26000），按压越重 AO 越低（压死≈0V）
		//   驱动 860SPS 单次转换模式，FSR 更新 ≈5.6ms、电位器 ≈11.2ms；
		//   断线看门狗由 bend_ctrl 承担
		//-------------------------------------------------------------------------
		wire [15:0] press_val;
		wire        press_upd;
		wire [15:0] pot1_val;		// AIN1 滑音电位器码
		wire        pot1_upd;
		wire [15:0] pot2_val;		// AIN2 颤音电位器码
		wire        pot2_upd;

		ads1115_drv #(
			.I2C_HALF  (250),			// SCL 100kHz
			.INIT_WAIT (2500000),		// 上电 50ms
			.CFG_WAIT  (500000),		// 首次转换 10ms
			.RD_WAIT   (80000)			// 单次转换 1.16ms，等 1.6ms 留余量
		) u_ads (
			.clk  (sys_clk),
			.rst_n(rst_n),
			.sda  (ads_sda),
			.scl  (ads_scl),
			.val0 (press_val),
			.upd0 (press_upd),
			.val1 (pot1_val),
			.upd1 (pot1_upd),
			.val2 (pot2_val),
			.upd2 (pot2_upd)
		);

		// 电位器映射：ADC 码 0~26400 -> 0~255 控制量（>>7 基础映射）
		// 【修正 2026-10-31】滑音量 x5/4 饱和归一：3.3V 满量程读数 ≈206，
		// 原映射下 sh 在旋钮 1/3 处就封顶，上半圈无控制感。x5/4 后满旋钮
		// 才到 255（sh 6~13 全行程可用）。颤音深度保持 >>7 原映射
		// （满旋钮 ≈±21.6 音分，不改变既有听感）。
		// POT_EN=0（未接电位器）时必须强制为 0：ADS 悬空输入会读到乱数，
		// 否则每个音都会被滑音和颤音"污染"，完全不像乐器
		wire [8:0] glide9    = {1'b0, pot1_val[15:7]} + {2'b0, pot1_val[15:9]};
		wire [7:0] pot_glide = POT_EN ? (glide9[8] ? 8'hFF : glide9[7:0]) : 8'd0;	// 滑音量
		wire [7:0] pot_vib   = POT_EN ? pot2_val[15:7] : 8'd0;	// 颤音深度

	//-------------------------------------------------------------------------
	// 声部控制总线：键盘演奏 / 自动演示二选一（综合器自动裁剪未用分支）
	//-------------------------------------------------------------------------
	wire [3:0]   gate_v;
	wire [127:0] pinc_v;
	wire [63:0]  vel_v;

	wire [3:0]   gate_d;
	wire [127:0] pinc_d;
	wire [63:0]  vel_d;

	demo_seq #(
		.DEMO_MODE (0)
	) u_demo_seq (
		.clk      (sys_clk),
		.rst_n    (rst_n),
		.demo_rst (s2_db),
		.gate_v   (gate_d),
		.pinc_v   (pinc_d),
		.vel_v    (vel_d)
	);

	generate
		if (PLAY_MODE == 0) begin : g_play_key
			// 维度1 触发+选音：按键直接发声（键位功能与键位表完全不变）
			assign gate_v = gate_k;
			assign pinc_v = pinc_k;
			assign vel_v  = vel_k;
		end else begin : g_play_demo
			assign gate_v = gate_d;
			assign pinc_v = pinc_d;
			assign vel_v  = vel_d;
		end
	endgenerate

	// 延迟测量辅助：音符声部触发标志（上升沿=引擎开始渲染该音符）
	assign dbg_lat = |gate_v;

	//-------------------------------------------------------------------------
	// 合成引擎 -> I2S 发送
	// 链路：pinc 先经 porta_ctrl 滑音过渡（电位器量控），再经 bend_ctrl 弯音
	// 缩放，最后进 synth_core（手动颤音深度 vib_man 亦来自电位器）
	//-------------------------------------------------------------------------
	wire sample_tick;
	wire signed [15:0] mix_l, mix_r;

	wire [127:0] pinc_porta;
	wire [127:0] pinc_glide;
	wire [127:0] pinc_bent;

	porta_ctrl #(
		.SHIFT_BASE (6)				// glide 0~255 -> 时间常数移位 6~13（τ 1.3ms~168ms）
	) u_porta (
		.clk      (sys_clk),
		.rst_n    (rst_n),
		.tick     (sample_tick),
		.glide    (pot_glide),
		.gate_v   (gate_v),
		.pinc_in  (pinc_v),
		.pinc_out (pinc_porta)
	);

	// POT_EN=0 时彻底旁路滑音（不是"glide=0"而是直接连线）：
	// 保证未接电位器时相位增量一字不差地直通，音色与最初完全一致
	assign pinc_glide = POT_EN ? pinc_porta : pinc_v;

	bend_ctrl #(
		.REL_CODE   (16'd26000),	// 松开 FSR 时 ADS 读数（AO 3.25V 实测）
		.BEND_SEMI  (2),			// 满压力升高的半音数（0~12，0=关闭弯音）
		.SLEW_STEP  (16'd150)		// 弯音平滑：全程 ≈3.5ms
	) u_bend (
		.clk      (sys_clk),
		.rst_n    (rst_n),
		.tick     (sample_tick),
		.upd      (press_upd),
		.press_val(press_val),
		.pinc_in  (pinc_glide),
		.pinc_out (pinc_bent)
	);

	synth_core #(
		.N_VOICE     (4),
		// 主包络（原始波形音色 S9~S12 用）：钢琴式
		.ATT_SAMPLES (98),		// A 起音 ≈2ms
		.DEC_SAMPLES (16000),	// D 衰减 ≈328ms
		.REL_SAMPLES (12207),	// R 释放 ≈250ms
		.SUS_LEVEL   (6554),	// S 延音电平 20%
		// 副包络（保留接口，当前 env_sel 恒 0 不选）
		.ATT_SAMPLES2 (3906),	// A 起音 ≈80ms（缓入）
		.DEC_SAMPLES2 (9766),	// D 衰减 ≈200ms
		.REL_SAMPLES2 (19531),	// R 释放 ≈400ms（长尾）
		.SUS_LEVEL2   (22938),	// S 延音电平 70%
		.CH_MODE     (0)		// 0：单声道镜像（单只喇叭）；双模块真立体声改 1
	) u_synth (
		.clk       (sys_clk),
		.rst_n     (rst_n),
		.tick      (sample_tick),
		.gate_v    (gate_v),
		.pinc_v    (pinc_bent),
		.vel_v     (vel_v),
		.env_sel_v (env_sel_v),
		.timbre_i  (timbre),
		.vib_man   (pot_vib),
		.sample_l  (mix_l),
		.sample_r  (mix_r)
	);

	//-------------------------------------------------------------------------
	// 数字音频效果器（拓展2）：混响 S15 开关 / 延迟回声 S16 开关。
	// 干湿并联——干声直通相加湿声，演奏延迟不受影响
	//-------------------------------------------------------------------------
	wire signed [15:0] mix_l_fx, mix_r_fx;

	audio_fx u_fx (
		.clk      (sys_clk),
		.rst_n    (rst_n),
		.tick     (sample_tick),
		.fx_rev   (fx_rev),
		.fx_dly   (fx_dly),
		.sample_l (mix_l),
		.sample_r (mix_r),
		.out_l    (mix_l_fx),
		.out_r    (mix_r_fx)
	);

	//-------------------------------------------------------------------------
	// I2S 延时启动（提高上电可靠性）
	//   FPGA 配置期间 I2S 引脚是悬空的，MAX98357A 可能把随机的 BCLK 跳变
	//   当成时钟而锁在错误状态——典型现象就是"上电没声、重烧一次又好了"。
	//   这里让 BCLK/LRCK/DATA 在复位后保持低电平约 168ms，等功放供电稳定
	//   再开始出时钟，让它从干净的状态同步。
	//-------------------------------------------------------------------------
	reg [23:0] i2s_dly = 24'd0;
	always @(posedge sys_clk)
		if (!rst_n)            i2s_dly <= 24'd0;
		else if (!i2s_dly[23]) i2s_dly <= i2s_dly + 24'd1;
	wire rst_n_i2s = rst_n & i2s_dly[23];

	i2s_tx #(
		.CLK_HZ  (50000000),
		.CLK_DIV (16)
	) u_i2s_tx (
		.clk        (sys_clk),
		.rst_n      (rst_n_i2s),
		.sample_l   (mix_l_fx),
		.sample_r   (mix_r_fx),
		.i2s_bclk   (i2s_bclk),
		.i2s_lrclk  (i2s_lrclk),
		.i2s_data   (i2s_data),
		.i2s_mclk   (i2s_mclk),
		.sample_tick(sample_tick)
	);

	//-------------------------------------------------------------------------
	// 拓展3：实时视听反馈（ST7789 240x240 IPS，SPI 12.5MHz）
	//   viz_scope 把合成信号画成示波器波形 + 16 格演奏指引/状态条
	//   LCD 在 SPI 上属于慢外设，但完全独立于音频通路：
	//   屏不刷新也不影响发声，故不改变演奏延迟指标
	//-------------------------------------------------------------------------
	wire        lcd_go;
	wire [7:0]  lcd_x0, lcd_y0, lcd_x1, lcd_y1;
	wire        lcd_solid;
	wire [15:0] lcd_col;
	wire [15:0] lcd_px_idx;
	wire        lcd_busy, lcd_ready;
	wire [15:0] viz_frames;
	wire [111:0] bar_flat;		// 16 频点柱高（7bit x16）
	wire         bar_upd;

	st7789_drv #(
		.SPI_DIV   (4),			// SCL = 12.5MHz
		.Y_OFF     (0)			// 竖屏无偏移；若画面整体错位改 80
	) u_lcd (
		.clk       (sys_clk),
		.rst_n     (rst_n),
		.go        (lcd_go),
		.x0        (lcd_x0),
		.y0        (lcd_y0),
		.x1        (lcd_x1),
		.y1        (lcd_y1),
		.solid     (lcd_solid),
		.col_solid (lcd_col),
		.px_idx    (lcd_px_idx),
		.px_in     (16'h0000),
		.busy      (lcd_busy),
		.ready     (lcd_ready),
		.lcd_scl   (lcd_scl),
		.lcd_sda   (lcd_sda),
		.lcd_cs    (lcd_cs),
		.lcd_dc    (lcd_dc),
		.lcd_res   (lcd_res)
	);

	viz_spec u_spec (
		.clk        (sys_clk),
		.rst_n      (rst_n),
		.audio      (mix_l_fx),
		.audio_tick (sample_tick),
		.bar_flat   (bar_flat),
		.upd        (bar_upd)
	);

	viz_scope u_viz (
		.clk       (sys_clk),
		.rst_n     (rst_n),
		.audio     (mix_l_fx),	// 画最终输出波形（含混响/延迟效果）
		.audio_tick(sample_tick),
		.key_state (key_state),
		.timbre    (timbre),
		.fx_rev    (fx_rev),
		.fx_dly    (fx_dly),
		.bar_flat  (bar_flat),
		.bar_upd   (bar_upd),
		.adc0      (press_val),	// 按住核心板 S1：状态条变三通道 ADC 电平条
		.adc1      (pot1_val),	//   A0=FSR(绿) A1=滑音(青) A2=颤音(黄)
		.adc2      (pot2_val),	//   排查电位器接线用（转哪个旋钮看哪组动）
		.dbg_adc   (key_s1),
		.go        (lcd_go),
		.x0        (lcd_x0),
		.y0        (lcd_y0),
		.x1        (lcd_x1),
		.y1        (lcd_y1),
		.solid     (lcd_solid),
		.col       (lcd_col),
		.drv_ready (lcd_ready),
		.drv_busy  (lcd_busy),
		.frames    (viz_frames)
	);

endmodule
