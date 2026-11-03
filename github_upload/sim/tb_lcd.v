`timescale 1ns / 1ps
//=============================================================================
// tb_lcd —— st7789_drv 协议级验证
//
// 在 BCLK(SCL) 上升沿采样 SDA，按 DC 区分命令/数据，逐字节打印，
// 并自动检查：初始化序列首尾、窗口命令、像素字节数。
//
// 检查点：
//   1) 初始化必须依次出现 0x11 / 0x36,0x00 / 0x3A,0x05 / 0x21 / 0x29
//   2) 一次 (x0=10,y0=20,x1=12,y1=23) 的纯色任务
//      -> 0x2A 00 0A 00 0C | 0x2B 00 14 00 17 | 0x2C | 3*4=12 像素*2 字节
//=============================================================================
module tb_lcd;
	reg clk = 1'b0;
	reg rst_n = 1'b0;
	always #10 clk = ~clk;			// 50MHz

	reg        go = 1'b0;
	reg [7:0]  x0 = 8'd10, y0 = 8'd20, x1 = 8'd12, y1 = 8'd23;
	reg        solid = 1'b1;
	reg [15:0] col_solid = 16'h07E0;
	wire [15:0] px_idx;
	wire        busy, ready;
	wire        scl, sda, cs, dc, res;

	st7789_drv #(
		.SPI_DIV (4), .POR_MS (1), .SLPOUT_MS (1), .Y_OFF (0)
	) u_dut (
		.clk(clk), .rst_n(rst_n),
		.go(go), .x0(x0), .y0(y0), .x1(x1), .y1(y1),
		.solid(solid), .col_solid(col_solid),
		.px_idx(px_idx), .px_in(16'h0000),
		.busy(busy), .ready(ready),
		.lcd_scl(scl), .lcd_sda(sda), .lcd_cs(cs), .lcd_dc(dc), .lcd_res(res)
	);

	//-------------------------------------------------------------------------
	// SPI 从机解码：SCL 上升沿采样
	//-------------------------------------------------------------------------
	reg [7:0] sh;
	reg [2:0] bitn = 3'd0;
	reg       is_cmd = 1'b0;		// 本次字节的 DC 极性
	integer   nbytes = 0;
	integer   ncmd = 0, ndat = 0;
	reg [7:0] cmds [0:63];
	integer   ncmp = 0;

	always @(posedge scl) begin
		if (!cs) begin
			sh   <= {sh[6:0], sda};
			is_cmd <= !dc;
			bitn <= bitn + 3'd1;
			if (bitn == 3'd7) begin
				bitn <= 3'd0;
				nbytes = nbytes + 1;
				if (!dc) begin
					$display("  CMD  %02h", {sh[6:0], sda});
					if (ncmp < 64) begin cmds[ncmp] = {sh[6:0], sda}; ncmp = ncmp + 1; end
					ncmd = ncmd + 1;
				end else begin
					$display("  DATA %02h", {sh[6:0], sda});
					ndat = ndat + 1;
				end
			end
		end
	end

	//-------------------------------------------------------------------------
	// 检查：初始化关键命令是否按序出现
	//-------------------------------------------------------------------------
	integer k, found;
	reg ok;
	reg [7:0] want [0:5];
	initial begin
		want[0] = 8'h11; want[1] = 8'h36; want[2] = 8'h3A;
		want[3] = 8'h21; want[4] = 8'h29; want[5] = 8'h00;
	end

	task check_init;
		integer j, p;
		begin
			ok = 1'b1;
			p  = 0;
			for (j = 0; j < 5; j = j + 1) begin
				found = -1;
				for (k = p; k < ncmp; k = k + 1)
					if (found < 0 && cmds[k] == want[j]) found = k;
				if (found < 0) begin
					$display("FAIL: 初始化缺少命令 %02h", want[j]);
					ok = 1'b0;
				end else
					p = found + 1;
			end
			if (ok) $display("PASS: 初始化命令序列 0x11/0x36/0x3A/0x21/0x29 齐全且有序");
		end
	endtask

	//-------------------------------------------------------------------------
	initial begin
		$display("=== tb_lcd: 上电 + 初始化 ===");
		repeat (300) @(posedge clk);
		rst_n = 1'b1;
		// 等初始化完成（POR 1ms + 1ms + 58 字节 + 1ms 等待）
		wait (ready === 1'b1);
		$display("初始化完成：命令 %0d 个，数据 %0d 个，共 %0d 字节", ncmd, ndat, nbytes);
		check_init();

		$display("\n=== 绘制任务：窗口 (10,20)-(12,23) 纯色 0x07E0 ===");
		nbytes = 0; ndat = 0; ncmd = 0;
		@(posedge clk);
		go = 1'b1;
		@(posedge clk);
		go = 1'b0;
		wait (busy === 1'b1);
		wait (ready === 1'b1);
		$display("job done: cmd=%0d data=%0d total=%0d (expect 3 cmd + 8 addr + 24 px = 32 data)",
		         ncmd, ndat, nbytes);

		if (ncmd == 3 && ndat == 32 && nbytes == 35)
			$display("PASS: window + 12 px transaction byte count correct");
		else
			$display("FAIL: byte count mismatch");

		$display("\nTB_LCD_DONE");
		$finish;
	end

	// 超时保护
	initial begin
		#20_000_000;
		$display("FAIL: 超时（初始化未完成）");
		$finish;
	end
endmodule
