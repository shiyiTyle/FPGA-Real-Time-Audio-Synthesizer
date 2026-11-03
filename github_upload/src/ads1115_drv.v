`timescale 1ns / 1ps
//=============================================================================
// ads1115_drv —— ADS1115 I2C ADC 驱动（三通道轮询：A0/A1/A2）
//
// 通道分配（拓展2 表情维度）：
//   AIN0（ch0）= FSR 压力 -> 音高弯音（bend_ctrl）
//   AIN1（ch1）= 滑音电位器 -> porta_ctrl 滑音时长
//   AIN2（ch2）= 颤音电位器 -> 全音色颤音深度
//
// 轮询序列 0->1->0->2 循环：FSR 每约 5.6ms 更新一次（主演奏维度优先），
// 两个电位器每约 11.2ms 更新一次。看门狗（bend_ctrl WD=20ms）不受影响。
//
// 【重要 2026-10-31】单次转换模式（MODE=1），每个通道写 OS=1 触发一次转换：
//   原来用连续转换模式时，写入新 MUX 后旧通道的转换仍占着结果寄存器，
//   新通道出数最坏要 ≈2.33ms，而 RD_WAIT 只有 1.2ms——A1/A2 约 3/4 的
//   读数是 A0(FSR) 的旧值（实测症状：滑音量恒最大、每音都滑音；颤音恒
//   满档、全音色抖；两个电位器怎么转都没用；按 FSR 反而带动"颤音"）。
//   单次模式每次写配置精确触发一次转换（1/860s≈1.16ms 完成），读到的
//   结果保证是新通道的；RD_WAIT 1.6ms 另留 10% 内部时钟容差余量。
// 配置：各通道 MUX 单端、PGA=±4.096V（覆盖 0~3.3V）、单次转换、860SPS
//   ch0: 0xC3E3  ch1: 0xD3E3  ch2: 0xE3E3（MUX=AIN0/AIN1/AIN2）
// 流程：上电等 50ms -> 写 ch0 配置 -> 等 10ms 首次转换 -> 循环：
//   写当前通道配置 -> 等 1.6ms 转换 -> 读 2 字节 -> 更新 val/upd -> 下一通道
//
// I2C 时序：SCL 100kHz（半周期 250 clk @50MHz），SDA 开漏（FPGA 拉低/高阻）
//=============================================================================
module ads1115_drv #(
	parameter integer I2C_HALF  = 250,		// SCL 半周期 clk 数（100kHz）
	parameter integer INIT_WAIT = 2500000,	// 上电等待 50ms
	parameter integer CFG_WAIT  = 500000,	// 首次写配置后等转换 10ms
	parameter integer RD_WAIT   = 80000		// 写配置后等 1.6ms（单次转换 1.16ms + 余量）
)(
	input  wire        clk,
	input  wire        rst_n,
	inout  wire        sda,
	output reg         scl,
	output reg  [15:0] val0,		// ch0 FSR（AIN0）
	output reg         upd0,
	output reg  [15:0] val1,		// ch1 滑音电位器（AIN1）
	output reg         upd1,
	output reg  [15:0] val2,		// ch2 颤音电位器（AIN2）
	output reg         upd2
);

	// ADS1115 地址：ADDR=GND -> 0x48（写 0x90 / 读 0x91）
	localparam [7:0] ADDR_W = 8'h90;
	localparam [7:0] ADDR_R = 8'h91;
		// 配置字：OS=1 | MUX=100/101/110 | PGA=001(±4.096V) | MODE=1(单次) | DR=111(860SPS)
		// （MODE=1 见文件头【重要】：连续模式下切通道后读数有竞态，读到旧通道值）
		localparam [7:0] CFG_HI0 = 8'hC3;	// AIN0
		localparam [7:0] CFG_HI1 = 8'hD3;	// AIN1
		localparam [7:0] CFG_HI2 = 8'hE3;	// AIN2
		localparam [7:0] CFG_LO  = 8'hE3;

	localparam [3:0] ST_INIT = 4'd0, ST_START = 4'd1, ST_BYTE = 4'd2,
	                 ST_ACKM = 4'd3, ST_RSTART = 4'd4, ST_RCV = 4'd5,
	                 ST_MACK = 4'd6, ST_STOP = 4'd7, ST_WAIT = 4'd8;

	// 轮询序列 0->1->0->2：ch 是 seq 的组合函数
	reg  [1:0]  seq;		// 0:ch0 1:ch1 2:ch0 3:ch2
	wire [1:0]  ch = (seq == 2'd1) ? 2'd1 : (seq == 2'd3) ? 2'd2 : 2'd0;
	wire [7:0]  cfg_hi = (ch == 2'd0) ? CFG_HI0 : (ch == 2'd1) ? CFG_HI1 : CFG_HI2;

	reg [3:0]  state;
	reg [1:0]  ph;			// 位内半相位
	reg [7:0]  tick;		// 半相位计时
	reg [7:0]  sh;			// 发送/接收移位寄存器
	reg [2:0]  bitn;
	reg        sda_oe;		// 1=拉低 SDA，0=释放
	reg        xfer;		// 0=写配置事务，1=读转换事务
	reg [2:0]  step;		// 事务内字节序号
	reg [7:0]  rcv_hi;
	reg        mack_lvl;	// 主应答：0=ACK，1=NACK
	reg [21:0] wait_cnt;
	reg [21:0] wait_target;
	reg        first_cfg;	// 首次写配置用长等待

	wire sda_in = sda;
	assign sda = sda_oe ? 1'b0 : 1'bz;

	// 事务字节表（按 事务类型/序号）
	function [7:0] byte_sel;
		input       xf;
		input [2:0] st;
		begin
			case ({xf, st})
				{1'b0, 3'd0}: byte_sel = ADDR_W;
				{1'b0, 3'd1}: byte_sel = 8'h01;	// 指针 -> 配置寄存器
				{1'b0, 3'd2}: byte_sel = cfg_hi;	// 当前通道 MUX 配置
				{1'b0, 3'd3}: byte_sel = CFG_LO;
				{1'b1, 3'd0}: byte_sel = ADDR_W;
				{1'b1, 3'd1}: byte_sel = 8'h00;	// 指针 -> 转换寄存器
				{1'b1, 3'd3}: byte_sel = ADDR_R;
				default:      byte_sel = 8'h00;
			endcase
		end
	endfunction

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= ST_INIT;
			ph <= 2'd0; tick <= 8'd0; sh <= 8'd0; bitn <= 3'd0;
			sda_oe <= 1'b0; scl <= 1'b1;
			xfer <= 1'b0; step <= 3'd0; seq <= 2'd0;
			rcv_hi <= 8'd0; mack_lvl <= 1'b0;
			wait_cnt <= 22'd0; wait_target <= 22'd0;
			first_cfg <= 1'b1;
			val0 <= 16'd0; val1 <= 16'd0; val2 <= 16'd0;
			upd0 <= 1'b0; upd1 <= 1'b0; upd2 <= 1'b0;
		end else begin
			upd0 <= 1'b0; upd1 <= 1'b0; upd2 <= 1'b0;
			case (state)
			//--------------------------------------------------------------
			ST_INIT: begin
				scl <= 1'b1; sda_oe <= 1'b0;
				if (wait_cnt == INIT_WAIT - 1) begin
					wait_cnt <= 22'd0; ph <= 2'd0; state <= ST_START;
				end else
					wait_cnt <= wait_cnt + 1'b1;
			end
			//--------------------------------------------------------------
			ST_START: begin
				if (tick == I2C_HALF - 1) begin
					tick <= 8'd0;
					case (ph)
						2'd0: ph <= 2'd1;					// 总线空闲（都高）
						2'd1: begin sda_oe <= 1'b1; ph <= 2'd2; end	// SDA 下降=START
						default: begin						// SCL 拉低，装载首字节
							scl <= 1'b0;
							sh <= byte_sel(xfer, step); bitn <= 3'd7; ph <= 2'd0;
							state <= ST_BYTE;
						end
					endcase
				end else
					tick <= tick + 1'b1;
			end
			//--------------------------------------------------------------
			ST_BYTE: begin
				if (tick == I2C_HALF - 1) begin
					tick <= 8'd0;
					if (ph == 2'd0) begin
						scl <= 1'b0;
						sda_oe <= !sh[bitn];				// SCL 低期间摆数据
						ph <= 2'd1;
					end else begin
						scl <= 1'b1;						// SCL 高，从机采样
						if (bitn == 3'd0) begin
							ph <= 2'd0; state <= ST_ACKM;
						end else begin
							bitn <= bitn - 1'b1; ph <= 2'd0;
						end
					end
				end else
					tick <= tick + 1'b1;
			end
			//--------------------------------------------------------------
			ST_ACKM: begin
				if (tick == I2C_HALF - 1) begin
					tick <= 8'd0;
					if (ph == 2'd0) begin
						scl <= 1'b0; sda_oe <= 1'b0;		// 释放 SDA 给从机应答
						ph <= 2'd1;
					end else begin
						scl <= 1'b1;
						ph <= 2'd0;
						// 发送完一字节，按事务推进（下一字节按 step+1 取）
						if (xfer == 1'b0 && step == 3'd3) begin
							state <= ST_STOP;
						end else begin
							step <= step + 1'b1;
							if (xfer == 1'b1 && step == 3'd1) begin
								state <= ST_RSTART;
							end else if (xfer == 1'b1 && step == 3'd3) begin
								// 读地址发送完毕 -> 转接收高字节（随后应答 ACK）
								bitn <= 3'd7; sh <= 8'd0; mack_lvl <= 1'b0;
								state <= ST_RCV;
							end else begin
								sh <= byte_sel(xfer, step + 3'd1); bitn <= 3'd7;
								state <= ST_BYTE;
							end
						end
					end
				end else
					tick <= tick + 1'b1;
			end
			//--------------------------------------------------------------
			ST_RSTART: begin
				if (tick == I2C_HALF - 1) begin
					tick <= 8'd0;
					case (ph)
						2'd0: begin scl <= 1'b0; sda_oe <= 1'b0; ph <= 2'd1; end
						2'd1: begin scl <= 1'b1; ph <= 2'd2; end
						2'd2: begin sda_oe <= 1'b1; ph <= 2'd3; end	// SDA 下降=RESTART
						default: begin
							scl <= 1'b0;
							step <= 3'd3; sh <= ADDR_R; bitn <= 3'd7;
							ph <= 2'd0; state <= ST_BYTE;
						end
					endcase
				end else
					tick <= tick + 1'b1;
			end
			//--------------------------------------------------------------
			ST_RCV: begin
				if (tick == I2C_HALF - 1) begin
					tick <= 8'd0;
					if (ph == 2'd0) begin
						scl <= 1'b0; sda_oe <= 1'b0;		// 释放 SDA 给从机
						ph <= 2'd1;
					end else begin
						scl <= 1'b1;
						sh <= {sh[6:0], sda_in};			// SCL 高末采样
						if (bitn == 3'd0) begin
							ph <= 2'd0; state <= ST_MACK;
						end else begin
							bitn <= bitn - 1'b1; ph <= 2'd0;
						end
					end
				end else
					tick <= tick + 1'b1;
			end
			//--------------------------------------------------------------
			ST_MACK: begin
				if (tick == I2C_HALF - 1) begin
					tick <= 8'd0;
					if (ph == 2'd0) begin
						scl <= 1'b0;
						sda_oe <= mack_lvl;				// ACK 拉低 / NACK 释放
						ph <= 2'd1;
					end else begin
						scl <= 1'b1;
						ph <= 2'd0;
						if (step == 3'd4) begin
							rcv_hi <= sh;					// 高字节完成，应答后读低字节
							step <= 3'd5; bitn <= 3'd7; sh <= 8'd0;
							mack_lvl <= 1'b1;				// 低字节后 NACK
							state <= ST_RCV;
						end else begin
							state <= ST_STOP;				// 低字节完成
						end
					end
				end else
					tick <= tick + 1'b1;
			end
			//--------------------------------------------------------------
			ST_STOP: begin
				if (tick == I2C_HALF - 1) begin
					tick <= 8'd0;
					case (ph)
						2'd0: begin scl <= 1'b0; sda_oe <= 1'b1; ph <= 2'd1; end
						2'd1: begin scl <= 1'b1; ph <= 2'd2; end
						default: begin
							sda_oe <= 1'b0;				// SDA 上升=STOP
							ph <= 2'd0;
							if (xfer == 1'b0) begin
								// 写配置完成：等转换（首次用长等待），随后读
								xfer <= 1'b1; step <= 3'd0;
								wait_target <= first_cfg ? CFG_WAIT : RD_WAIT;
								first_cfg <= 1'b0;
								wait_cnt <= 22'd0;
							end else begin
								// 读完成：结果入对应通道，切换到下一通道写配置
								case (ch)
									2'd0: begin val0 <= {rcv_hi, sh}; upd0 <= 1'b1; end
									2'd1: begin val1 <= {rcv_hi, sh}; upd1 <= 1'b1; end
									default: begin val2 <= {rcv_hi, sh}; upd2 <= 1'b1; end
								endcase
								seq <= seq + 2'd1;
								xfer <= 1'b0; step <= 3'd0;
								wait_target <= RD_WAIT; wait_cnt <= 22'd0;
							end
							state <= ST_WAIT;
						end
					endcase
				end else
					tick <= tick + 1'b1;
			end
			//--------------------------------------------------------------
			ST_WAIT: begin
				scl <= 1'b1; sda_oe <= 1'b0;
				if (wait_cnt == wait_target - 1) begin
					wait_cnt <= 22'd0; ph <= 2'd0; state <= ST_START;
				end else
					wait_cnt <= wait_cnt + 1'b1;
			end
			default: state <= ST_INIT;
			endcase
		end
	end

endmodule
