`timescale 1ns / 1ps
//=============================================================================
// st7789_drv —— ST7789 (240x240 IPS) SPI 显示驱动
//
// 硬件：1.54" 240x240 IPS，ST7789VW，8 脚 SPI
//       GND/VCC(3.3V)/SCL/SDA(MOSI)/RES/DC/CS/BLK
//       SPI mode 0（CPOL=0, CPHA=0），MSB first，8 位；DC=0 命令 / DC=1 数据
//
// 流程：RES 低 100ms -> RES 高 100ms -> 初始化命令表（58 项，含 sleep out 后
//       120ms 等待）-> 空闲 -> 接受绘制任务：设窗口(0x2A/0x2B/0x2C)后流像素
//
// 绘制任务接口（每次覆盖一个矩形窗口，像素按行优先输出）：
//   solid=1：窗口内所有像素都用 col_solid（纯色填充，最快路径）
//   solid=0：外部按 px_idx（0..npx-1）组合给出 px_in，用于画字模/任意图形
//
// 性能（SPI 12.5MHz = 50MHz/4）：窗口命令 11 字节 ≈7.0us，单像素 1.28us
//   全屏 240x240 = 73.7ms；单列竖线 n 像素 = 7.0us + 1.28us x n
//=============================================================================
module st7789_drv #(
	parameter integer CLK_HZ    = 50000000,	// 系统时钟
	parameter integer SPI_DIV   = 4,		// SCL = CLK_HZ / SPI_DIV（4 -> 12.5MHz）
	parameter integer POR_MS    = 100,		// RES 低/高电平各保持（ms）
	parameter integer SLPOUT_MS = 120,		// sleep out 后等待（ms）
	parameter integer Y_OFF     = 0			// 行坐标偏移（竖屏 0；显示错位时试 80）
)(
	input  wire        clk,
	input  wire        rst_n,
	input  wire        go,			// 1 clk 脉冲，仅在 idle 且 go 时锁存任务
	input  wire [7:0]  x0,
	input  wire [7:0]  y0,
	input  wire [7:0]  x1,
	input  wire [7:0]  y1,
	input  wire        solid,
	input  wire [15:0] col_solid,
	output wire [15:0] px_idx,		// 当前像素序号（0 起，行优先）
	input  wire [15:0] px_in,
	output wire        busy,
	output wire        ready,		// 初始化完成且空闲
	output reg         lcd_scl,
	output reg         lcd_sda,
	output reg         lcd_cs,
	output reg         lcd_dc,
	output reg         lcd_res
);

	localparam integer HALF  = SPI_DIV / 2;			// SCL 半周期 clk 数
	localparam integer MSC   = CLK_HZ / 1000;		// 1ms 的 clk 数
	localparam integer POR_C = POR_MS * MSC;
	localparam integer SLP_C = SLPOUT_MS * MSC;

	//-------------------------------------------------------------------------
	// 字节发送器：SCL 低电平期换数据，上升沿由从机采样（mode 0）
	//-------------------------------------------------------------------------
	// 启动请求由状态机独占驱动（避免多驱动 EX2000）：
	//   FSM 置 sb_start=1 -> 发送器在 sb_start && !sb_busy 时认领并置 sb_busy=1
	//   -> FSM 见 sb_busy=1 后撤销 sb_start
	reg        sb_start;
	reg  [7:0] sb_data;
	reg        sb_dc;
	reg        sb_busy;
	reg  [7:0] sb_sh;
	reg  [2:0] sb_bit;
	reg        sb_ph;
	reg  [7:0] sb_half;
	reg  [7:0] sb_half_lim;

	// 发送器空闲且无待启动请求：FSM 用此条件推进
	wire       sb_free = !sb_busy && !sb_start;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sb_busy <= 1'b0;
			sb_sh <= 8'd0; sb_bit <= 3'd0; sb_ph <= 1'b0; sb_half <= 8'd0;
			sb_half_lim <= HALF[7:0];
			lcd_scl <= 1'b0; lcd_sda <= 1'b0; lcd_dc <= 1'b0;
		end else if (sb_start && !sb_busy) begin		// 认领启动请求
			sb_busy <= 1'b1;
			sb_sh   <= sb_data;
			sb_bit  <= 3'd7;
			sb_ph   <= 1'b0;
			sb_half <= 8'd0;
			lcd_dc  <= sb_dc;
			lcd_scl <= 1'b0;
			lcd_sda <= sb_data[7];
		end else if (sb_busy) begin
			if (sb_half == sb_half_lim - 8'd1) begin
				sb_half <= 8'd0;
				if (!sb_ph) begin
					sb_ph   <= 1'b1;
					lcd_scl <= 1'b1;
				end else begin
					sb_ph   <= 1'b0;
					lcd_scl <= 1'b0;
					if (sb_bit == 3'd0)
						sb_busy <= 1'b0;
					else begin
						sb_bit  <= sb_bit - 3'd1;
						sb_sh   <= {sb_sh[6:0], 1'b0};
						lcd_sda <= sb_sh[6];
					end
				end
			end else
				sb_half <= sb_half + 8'd1;
		end
	end

	//-------------------------------------------------------------------------
	// 初始化命令表 {dc, byte}：58 项，索引 0 之后插入 SLPOUT_MS 等待
	//-------------------------------------------------------------------------
	function [8:0] init_ent;
		input [5:0] i;
		begin
			case (i)
				6'd0 : init_ent = {1'b0, 8'h11};	// SLPOUT（随后等 120ms）
				6'd1 : init_ent = {1'b0, 8'h36};	// MADCTL
				6'd2 : init_ent = {1'b1, 8'h00};	//   竖屏 RGB 顺序
				6'd3 : init_ent = {1'b0, 8'h3A};	// COLMOD
				6'd4 : init_ent = {1'b1, 8'h05};	//   RGB565
				6'd5 : init_ent = {1'b0, 8'hB2};
				6'd6 : init_ent = {1'b1, 8'h0C};
				6'd7 : init_ent = {1'b1, 8'h0C};
				6'd8 : init_ent = {1'b1, 8'h00};
				6'd9 : init_ent = {1'b1, 8'h33};
				6'd10: init_ent = {1'b1, 8'h33};
				6'd11: init_ent = {1'b0, 8'hB7};
				6'd12: init_ent = {1'b1, 8'h35};
				6'd13: init_ent = {1'b0, 8'hBB};
				6'd14: init_ent = {1'b1, 8'h32};
				6'd15: init_ent = {1'b0, 8'hC2};
				6'd16: init_ent = {1'b1, 8'h01};
				6'd17: init_ent = {1'b0, 8'hC3};
				6'd18: init_ent = {1'b1, 8'h15};
				6'd19: init_ent = {1'b0, 8'hC4};
				6'd20: init_ent = {1'b1, 8'h20};
				6'd21: init_ent = {1'b0, 8'hC6};
				6'd22: init_ent = {1'b1, 8'h0F};
				6'd23: init_ent = {1'b0, 8'hD0};
				6'd24: init_ent = {1'b1, 8'hA4};
				6'd25: init_ent = {1'b1, 8'hA1};
				6'd26: init_ent = {1'b0, 8'hE0};	// gamma+
				6'd27: init_ent = {1'b1, 8'hD0};
				6'd28: init_ent = {1'b1, 8'h08};
				6'd29: init_ent = {1'b1, 8'h0E};
				6'd30: init_ent = {1'b1, 8'h09};
				6'd31: init_ent = {1'b1, 8'h09};
				6'd32: init_ent = {1'b1, 8'h05};
				6'd33: init_ent = {1'b1, 8'h31};
				6'd34: init_ent = {1'b1, 8'h33};
				6'd35: init_ent = {1'b1, 8'h48};
				6'd36: init_ent = {1'b1, 8'h17};
				6'd37: init_ent = {1'b1, 8'h14};
				6'd38: init_ent = {1'b1, 8'h15};
				6'd39: init_ent = {1'b1, 8'h31};
				6'd40: init_ent = {1'b1, 8'h34};
				6'd41: init_ent = {1'b0, 8'hE1};	// gamma-
				6'd42: init_ent = {1'b1, 8'hD0};
				6'd43: init_ent = {1'b1, 8'h08};
				6'd44: init_ent = {1'b1, 8'h0E};
				6'd45: init_ent = {1'b1, 8'h09};
				6'd46: init_ent = {1'b1, 8'h09};
				6'd47: init_ent = {1'b1, 8'h15};
				6'd48: init_ent = {1'b1, 8'h31};
				6'd49: init_ent = {1'b1, 8'h33};
				6'd50: init_ent = {1'b1, 8'h48};
				6'd51: init_ent = {1'b1, 8'h17};
				6'd52: init_ent = {1'b1, 8'h14};
				6'd53: init_ent = {1'b1, 8'h15};
				6'd54: init_ent = {1'b1, 8'h31};
				6'd55: init_ent = {1'b1, 8'h34};
				6'd56: init_ent = {1'b0, 8'h21};	// INVON（IPS 必须反色）
				default: init_ent = {1'b0, 8'h29};	// DISPON
			endcase
		end
	endfunction

	//-------------------------------------------------------------------------
	// 主状态机
	//-------------------------------------------------------------------------
	localparam [2:0] S_POR = 3'd0, S_POR2 = 3'd1, S_INIT = 3'd2,
	                 S_IDLE = 3'd3, S_CMD = 3'd4, S_PIX = 3'd5;

	reg [2:0]  state;
	reg [23:0] dly;
	reg        slp_wait;		// 初始化中的 sleep-out 等待标志
	reg [5:0]  ii;
	reg [3:0]  ci;
	reg [15:0] npx;
	reg [15:0] pxi;
	reg        pph;				// 像素内字节相位（0=高字节 1=低字节）
	reg [7:0]  jx0, jy0, jx1, jy1;
	reg        jsolid;
	reg [15:0] jcol;

	wire [8:0] wy0 = {1'b0, jy0} + Y_OFF;
	wire [8:0] wy1 = {1'b0, jy1} + Y_OFF;
	wire [8:0] ent = init_ent(ii);

	// 窗口命令 11 字节：0x2A x0h x0l x1h x1l 0x2B y0h y0l y1h y1l 0x2C
	reg  [7:0] cb;
	reg        cd;
	always @* begin
		case (ci)
			4'd0:  begin cd = 1'b0; cb = 8'h2A; end
			4'd1:  begin cd = 1'b1; cb = 8'h00; end
			4'd2:  begin cd = 1'b1; cb = jx0;   end
			4'd3:  begin cd = 1'b1; cb = 8'h00; end
			4'd4:  begin cd = 1'b1; cb = jx1;   end
			4'd5:  begin cd = 1'b0; cb = 8'h2B; end
			4'd6:  begin cd = 1'b1; cb = {7'd0, wy0[8]}; end
			4'd7:  begin cd = 1'b1; cb = wy0[7:0]; end
			4'd8:  begin cd = 1'b1; cb = {7'd0, wy1[8]}; end
			4'd9:  begin cd = 1'b1; cb = wy1[7:0]; end
			default: begin cd = 1'b0; cb = 8'h2C; end
		endcase
	end

	wire [15:0] pix = jsolid ? jcol : px_in;
	assign px_idx = pxi;
	assign busy   = (state == S_INIT) || (state == S_CMD) || (state == S_PIX);
	assign ready  = (state == S_IDLE);

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state    <= S_POR;
			dly      <= 24'd0;
			slp_wait <= 1'b0;
			ii       <= 6'd0;
			ci       <= 4'd0;
			npx      <= 16'd0;
			pxi      <= 16'd0;
			pph      <= 1'b0;
			jx0 <= 8'd0; jy0 <= 8'd0; jx1 <= 8'd0; jy1 <= 8'd0;
			jsolid <= 1'b1; jcol <= 16'd0;
			sb_data <= 8'd0; sb_dc <= 1'b0;
			sb_start <= 1'b0;		// 必须显式复位：X 会让 sb_free 恒 X 而死锁
			lcd_res <= 1'b0;
			lcd_cs  <= 1'b1;
		end else begin
			// 字节发送器认领启动请求后撤销 sb_start（单驱动方）
			if (sb_busy) sb_start <= 1'b0;

			case (state)
			//-------------------------------------------------------------
			S_POR: begin
				lcd_res <= 1'b0;
				lcd_cs  <= 1'b1;
				if (dly == POR_C - 1) begin
					dly <= 24'd0; lcd_res <= 1'b1; state <= S_POR2;
				end else
					dly <= dly + 24'd1;
			end
			//-------------------------------------------------------------
			S_POR2: begin
				if (dly == POR_C - 1) begin
					dly <= 24'd0; ii <= 6'd0; state <= S_INIT;
				end else
					dly <= dly + 24'd1;
			end
			//-------------------------------------------------------------
			S_INIT: begin
				lcd_cs <= 1'b0;
				if (slp_wait) begin
					if (dly == SLP_C - 1) begin
						dly <= 24'd0; slp_wait <= 1'b0;
					end else
						dly <= dly + 24'd1;
				end else if (sb_free) begin
					if (ii == 6'd58) begin			// 全部发完
						lcd_cs <= 1'b1;
						state  <= S_IDLE;
					end else begin
						sb_data <= ent[7:0];
						sb_dc   <= ent[8];
						sb_start<= 1'b1;
						if (ii == 6'd0)
							slp_wait <= 1'b1;		// 0x11 之后等 120ms
						ii <= ii + 6'd1;
					end
				end
			end
			//-------------------------------------------------------------
			S_IDLE: begin
				lcd_cs <= 1'b1;
				if (go) begin
					jx0 <= x0; jy0 <= y0; jx1 <= x1; jy1 <= y1;
					jsolid <= solid; jcol <= col_solid;
					npx <= ({8'd0, x1} - {8'd0, x0} + 16'd1) *
					       ({8'd0, y1} - {8'd0, y0} + 16'd1);
					pxi <= 16'd0;
					pph <= 1'b0;
					ci  <= 4'd0;
					lcd_cs <= 1'b0;
					state <= S_CMD;
				end
			end
			//-------------------------------------------------------------
			S_CMD: begin
				if (sb_free) begin
					sb_data <= cb; sb_dc <= cd; sb_start <= 1'b1;
					if (ci == 4'd10) state <= S_PIX;
					else             ci <= ci + 4'd1;
				end
			end
			//-------------------------------------------------------------
			S_PIX: begin
				if (sb_free) begin
					if (pxi == npx) begin		// 全部像素发完
						lcd_cs <= 1'b1;
						state  <= S_IDLE;
					end else if (!pph) begin
						sb_data <= pix[15:8]; sb_dc <= 1'b1; sb_start <= 1'b1;
						pph <= 1'b1;
					end else begin
						sb_data <= pix[7:0];  sb_dc <= 1'b1; sb_start <= 1'b1;
						pph <= 1'b0;
						pxi <= pxi + 16'd1;
					end
				end
			end
			default: state <= S_POR;
			endcase
		end
	end

endmodule
