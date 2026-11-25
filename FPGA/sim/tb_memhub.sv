`timescale 1ns/1ps
// Testbench: SAROO memhub + tsdram + cachebus + cacheblk against an SDRAM model,
// driven by a model of the Saturn SCU on the A-Bus (CS0, 4MB RAM cart area)
// and random STM32 traffic on port B.
//
// The A-Bus front end (synchroniser, start pulses, address register, byte
// swap) copies SSMaster.v lines 447-472 and 685-707.
//
// SCU cycle model (approximation, not from Sega's timing diagrams):
//   CS0 + RD/WR asserted on an SCU clock edge, NW wait clocks, then if EWT=1
//   extra clocks while WAIT is asserted, then data is sampled on the next
//   edge and the strobes are released. RPC=1 adds one idle clock after.
//   SCU clock 28.636MHz (34.92ns), asynchronous to the FPGA's 100MHz.
//
// Parameters (override with vsim -G):
//   NW, EWT, RPC   SCU timing (Saroo 0x3880: NW=8 EWT=1; Sega 0x2330: NW=3 EWT=0)
//   B_LOAD         STM32 traffic: 0 none, 1 light, 2 heavy
//   SIM_US         simulated time after SDRAM init, in microseconds
//   SEED           random seed
`ifdef CH8
 `define POR_DONE (u_hub._tsd.in_por == 0)
`else
 `define POR_DONE (u_hub._tsd.m_state == 1)
`endif

module tb;
	parameter int NW     = 8;
	parameter int EWT    = 1;
	parameter int RPC    = 1;	// precharge clock after reads
	parameter int WPC    = 0;	// precharge clock after writes
	parameter int B_LOAD = 0;
	parameter int SAT_IDLE_PCT = 0;	// % of Saturn accesses followed by a 0.5-3us pause
	parameter int SIM_US = 1000;
	parameter int TRAFFIC = 0;	// 0: mixed, 1: random 32-bit write/read/read (stress test 5), 2: sequential 32-bit reads
	parameter real SAMPLE_ADV = 0.0;	// ns the SCU samples read data before the end of the cycle
	parameter real WDATA_DELAY = 0.0;	// ns after the start of a write cycle until the new data is valid at the FPGA
	parameter int SEED   = 1;
	parameter string NAME = "run";
	// AMAP 0: Saturn test words all in SDRAM bank 0 (original testbench).
	// AMAP 1: like real code: 1024 words (2 KB) per bank, then the next bank, as tsdram maps addresses.
	parameter int AMAP   = 0;
	// TRAFFIC 3 (SAROO menu): the Saturn waits 1-30us in a loop that runs from the SH-2 cache (no cart
	// access), then runs from the cart: bursts of SH-2 cache-line fills (8 sequential 16-bit reads),
	// mostly the next line, sometimes a jump. B_LOAD 3: the STM32 copies from the SD card into the
	// SDRAM: runs of up to 256 sequential writes, back to back.

	localparam real TSCU = 34.92;

	// clocks / reset
	logic clk = 0;  always #5 clk = ~clk;			// mclk 100MHz
	logic sdclk;    assign sdclk = ~clk;			// SD_CLK: 180 deg (mainpll clk1)
	logic sclk = 0; initial begin #3.3; forever #(TSCU/2) sclk = ~sclk; end
	logic reset = 0; initial #100 reset = 1;		// active-low

	// ---------------- Saturn A-Bus (CS0) ----------------
	logic        SS_CS0n = 1, SS_RDn = 1, SS_WR0n = 1, SS_WR1n = 1;
	logic [23:0] SS_ADDR = 0;
	logic [15:0] scu_drv = 0;
	logic        scu_drv_en = 0;
	wire  [15:0] SS_DATA;

	logic s0 = 1, s1 = 1, s2 = 1, s3 = 1, s4 = 1;
	always @(posedge clk) begin s0 <= SS_CS0n; s1 <= s0; s2 <= s1; s3 <= s2; s4 <= s3; end
	wire ss_rd_start = (s2 == 1 && s1 == 0 && SS_RDn == 0);
	wire ss_wr_start = (s4 == 1 && s3 == 0 && (SS_WR0n == 0 || SS_WR1n == 0));

	logic [25:0] ss_ram_addr;
	always @(posedge clk) ss_ram_addr <= {2'b00, SS_ADDR};
	wire        ss_ram_cs  = ~SS_CS0n;
	wire [ 1:0] ss_mask    = {SS_WR0n, SS_WR1n};
	wire [15:0] ss_ram_din = {SS_DATA[7:0], SS_DATA[15:8]};
	wire [15:0] ss_ram_dout;
	wire        ss_ram_wait;
	wire [15:0] ss_ram_data_out = {ss_ram_dout[7:0], ss_ram_dout[15:8]};
	assign SS_DATA = scu_drv_en ? scu_drv : (SS_RDn == 0 && SS_CS0n == 0) ? ss_ram_data_out : 16'hzzzz;

	// ---------------- STM32 port ----------------
	logic        st_cs = 0, st_rd = 0, st_wr = 0;
	logic [ 1:0] st_mask = 0;
	logic [25:0] st_addr = 0;
	logic [15:0] st_din = 0;
	wire  [15:0] st_dout;
	wire         st_wait;

	// ---------------- refresh / config ----------------
	logic ext_refresh = 0;
	wire [11:0] reg_sdram = 12'b11_1100001101;	// 64MB, refresh count 0x30d (7.81us)

	// ---------------- DUT ----------------
	wire sd_cke, sd_cs, sd_ras, sd_cas, sd_we;
	wire [12:0] sd_addr;
	wire [ 1:0] sd_ba, sd_dqm;
	wire [15:0] sd_data;

	memhub u_hub(
		.reset(reset), .clk(clk),
		.cs_a(ss_ram_cs), .rd_a(ss_rd_start), .wr_a(ss_wr_start), .mask_a(ss_mask), .nwait_a(ss_ram_wait),
		.addr_a(ss_ram_addr), .wdata_a(ss_ram_din), .rdata_a(ss_ram_dout),
		.cs_b(st_cs), .rd_b(st_rd), .wr_b(st_wr), .mask_b(st_mask), .nwait_b(st_wait),
		.addr_b(st_addr), .wdata_b(st_din), .rdata_b(st_dout),
		.sd_cke(sd_cke), .sd_cs(sd_cs), .sd_ras(sd_ras), .sd_cas(sd_cas), .sd_we(sd_we),
		.sd_addr(sd_addr), .sd_ba(sd_ba), .sd_dqm(sd_dqm), .sd_data(sd_data),
		.ext_refresh(ext_refresh), .reg_sdram(reg_sdram)
	);

	sdram_model u_sdram(
		.clk(sdclk), .cke(sd_cke), .cs_n(sd_cs), .ras_n(sd_ras), .cas_n(sd_cas), .we_n(sd_we),
		.a(sd_addr), .ba(sd_ba), .dqm(sd_dqm), .dq(sd_data)
	);

	// contention on the SDRAM data bus
	int contention = 0;
	always @(posedge clk) if (u_hub.sd_oe && u_sdram.dq_oe) contention++;

	// Saturn refresh cycle: the SCU runs it as a bus cycle of its own (FC1 low,
	// no chip select) about every 15us, between accesses (see scu_refresh).
	// The pulse comes from the same edge detector as SSMaster.v (ssfc1_s*).
	logic SS_FC1 = 1, f0 = 1, f1 = 1, f2 = 1;
	always @(posedge clk) begin f0 <= SS_FC1; f1 <= f0; f2 <= f1; end
	always @* ext_refresh = (f2 == 1 && f1 == 0);
	realtime last_ext = -1e9;
	always @(posedge clk) if (ext_refresh) last_ext = $realtime;

	// ---------------- address maps ----------------
	// A (Saturn) test words w = 0..65535: bank 0, row {1, w[15:8], 0}, col w[7:0]
	// SDRAM model index {bank, row, column} of a 26-bit byte address, with the DUT's bank map (64MB part)
	function automatic int phys_index(logic [25:0] a);
`ifdef CH9
		if (u_hub._tsd.mirror) return {a[23:22], 1'b0, a[24], a[21:11], a[10:1]};	// copy 0
		return {a[23:22], a[25:24], a[21:11], a[10:1]};
`elsif CH8
		if (u_hub._tsd.BANK_MAP != 0) return {a[23:22], a[25:24], a[21:11], a[10:1]};
`endif
		return {a[12:11], a[25:13], a[10:1]};
	endfunction

	// AMAP 1: A[16:13] = w[15:12] (row), A[12:11] = w[11:10] (bank), A[10:1] = w[9:0] (column)
	// AMAP 2: the SAROO firmware's place: SDRAM 0x000000-0x01FFFF (CS0 0x22000000), A[16:1] = w
	function automatic logic [23:0] a_bus_addr(logic [15:0] w);
		if (AMAP == 1) return {2'b01, 5'b00000, w[15:12], w[11:10], w[9:0], 1'b0};
		if (AMAP == 2) return {7'b0000000, w, 1'b0};
		return {2'b01, w[15:8], 5'b00000, w[7:0], 1'b0};
	endfunction
	function automatic int a_index(logic [15:0] w);
		return phys_index({2'b00, a_bus_addr(w)});
	endfunction
	function automatic logic [15:0] swap(logic [15:0] v);
		return {v[7:0], v[15:8]};
	endfunction

`ifdef CH9
	initial u_sdram.mirror_init = 1;	// before the shadow copy below reads init values
`endif
	logic [15:0] shadow [65536];
	initial for (int w = 0; w < 65536; w++) shadow[w] = swap(u_sdram.init_val(a_index(w[15:0])));

	// ---------------- statistics ----------------
	bit final_chk = 0;
	int n_rd = 0, n_wr = 0, n_err = 0, n_late = 0, lat_n = 0;
	real lat_max = 0, lat_sum = 0;
	logic        rd_active = 0, lat_done = 0;
	logic [15:0] rd_expect;
	realtime     rd_t0;
	real         budget;

	// error classification: what was going on when the read started
	bit busy_at_start, saw_start, st_busy_at_start, bank_busy_at_start;
	int e_st_same = 0;	// CH8: STM32 errors where the Saturn's bank itself was busy at the start
	int e_busy = 0, e_nostart = 0, e_ref = 0, e_ref_ext = 0, e_st = 0, e_other = 0, e_stale = 0;
	logic [15:0] prev_smp = 16'hxxxx;
	always @(posedge clk) begin
		if (ss_rd_start && rd_active) begin
			saw_start = 1;
			busy_at_start = (u_hub.busa.state != 0);
`ifdef CH8
			bank_busy_at_start = !u_hub._tsd.bank_free[SS_ADDR[12:11]];
`endif
`ifdef CH9
			st_busy_at_start = u_hub._tsd.bg_open || (u_hub._tsd.bank_open != 0) || (u_hub.cmd_req_b != 0);
`elsif CH8
			st_busy_at_start = u_hub._tsd.t_open || (u_hub._tsd.bank_open != 0) || (u_hub.cmd_req_b != 0);
`else
			st_busy_at_start = (u_hub.state != 0 && u_hub.act_ab == 1) || (u_hub.cmd_req_b != 0);
`endif
		end
	end

	// why the last refresh was issued (only meaningful for the 'new' design)
	string   ref_why = "?", why_d = "?";
	realtime ref_t = -1e9;
	always @(posedge clk) begin
`ifndef CH9
		if (u_hub._tsd.ref_ack) begin ref_t = $realtime; ref_why = why_d; end
`endif
`ifdef HAS_REF_DEBT
		// flags as the scheduler saw them on the clock that decided
		why_d = $sformatf("ext=%0d after_sat=%0d in_hit=%0d quiet=%0d force=%0d debt=%0d",
			u_hub._tsd.ext_req, u_hub._tsd.after_sat, u_hub._tsd.in_hit, u_hub._tsd.quiet, u_hub._tsd.ref_force, u_hub._tsd.ref_debt);
`endif
	end

	// time until the bus shows the right value (any read)
	always @(posedge clk) if (rd_active && !lat_done && SS_DATA === rd_expect) begin
		real l;
		l = $realtime - rd_t0;
		lat_done = 1; lat_n++; lat_sum += l;
		if (l > lat_max) lat_max = l;
	end

`ifdef CH9
	// how often the rare change-9 paths ran (shared-word race test)
	int c_skip = 0, c_sat = 0, c_s2m = 0;
	always @(posedge clk) begin
		if (u_hub._tsd.pick == 0 && u_hub._tsd.go_tw_skip) c_skip++;
		if (u_hub._tsd.pick == 2 && u_hub._tsd.s_match) c_s2m++;
		if (u_hub._tsd.pick == 2 && u_hub._tsd.mirror && u_hub._tsd.s_req != 2'b10 && u_hub._tsd.tw_active && u_hub._tsd.s_addr[24:1] == u_hub._tsd.t_wa) c_sat++;
	end
`endif

	// ---------------- SCU master ----------------
	task automatic scu_access(bit wr, logic [15:0] w, logic [15:0] wd);
		logic [15:0] smp;
		@(posedge sclk); #2;
		SS_ADDR = a_bus_addr(w);
		SS_CS0n = 0;
		if (wr) begin
			// The new write data reaches the FPGA WDATA_DELAY ns after the cycle starts; until then the
			// previous value stays on the bus (hardware 2026-11-24: a write latched ~40ns in kept a stale bit)
			scu_drv_en = 1; SS_WR0n = 0; SS_WR1n = 0;
			if (WDATA_DELAY > 0.0) fork begin #(WDATA_DELAY); scu_drv = wd; end join_none
			else scu_drv = wd;
		end
		else begin SS_RDn = 0; rd_expect = shadow[w]; rd_t0 = $realtime; lat_done = 0; saw_start = 0; busy_at_start = 0; st_busy_at_start = 0; rd_active = 1; end
		// The SCU samples SAMPLE_ADV ns before the end of the cycle (stands in for the delays the model doesn't
		// have: bus transceivers, board, SCU input setup, FPGA pin delays). Without external wait the end of the
		// cycle is fixed, so the sample time is set from the start of the cycle (any SAMPLE_ADV works).
		if (!EWT) fork
			begin #((NW + 1) * TSCU - 2.0 - SAMPLE_ADV); smp = SS_DATA; end
		join_none
		repeat (NW) @(posedge sclk);
		if (EWT) begin
			while (ss_ram_wait == 0) @(posedge sclk);
			#(TSCU - SAMPLE_ADV);
			smp = SS_DATA;
		end
		@(posedge sclk);
		#2;
		SS_CS0n = 1; SS_RDn = 1; SS_WR0n = 1; SS_WR1n = 1; scu_drv_en = 0;
		if (wr) begin
			n_wr++; shadow[w] = wd;
		end else begin
			n_rd++; rd_active = 0;
			if (!lat_done) n_late++;
			if ((TRAFFIC != 4 || final_chk) && smp !== rd_expect) begin
				n_err++;
				if (smp === prev_smp) e_stale++;	// the value of the previous read: answered too late
				if (!saw_start)                                  e_nostart++;
				else if (busy_at_start)                          e_busy++;
				else if ($realtime - u_sdram.last_ref < 250.0) begin
					if ($realtime - last_ext < 400.0) e_ref_ext++; else e_ref++;
				end
				else if (st_busy_at_start) begin e_st++; if (bank_busy_at_start) e_st_same++; end
				else                                             e_other++;
				if (n_err <= 8) $display("%t READ ERROR w=%04h got %04h expected %04h | last refresh %.0fns before read start (%s)",
					$realtime, w, smp, rd_expect, rd_t0 - ref_t, ref_why);
			end
			prev_smp = smp;
		end
		if (RPC && !wr) @(posedge sclk);	// read precharge only (Sega 0x2330: A0RPC=1, A0WPC=0)
		if (WPC &&  wr) @(posedge sclk);
	endtask

	// SCU refresh cycle: AREF = 0x13 -> refresh enabled, 3 wait clocks
	task automatic scu_refresh();
		@(posedge sclk); #2;
		SS_FC1 = 0;
		repeat (4) @(posedge sclk);
		#2; SS_FC1 = 1;
		@(posedge sclk);
	endtask

	// ---------------- STM32 master ----------------
	int  st_ops = 0, st_rd_n = 0, st_rd_err = 0;
	logic [15:0] st_shadow [int];	// STM32 area: what each word should hold
	function automatic int b_index(logic [25:0] a);
		return phys_index(a);
	endfunction
	real st_wait_max = 0, st_wait_sum = 0;
	// mask: 1 = that byte is not written (as the FSMC byte lanes); 2'b00 = whole word
	bit st_stop = 0;	// set at the end: no new STM32 accesses, so the copies can be compared
	task automatic st_access(bit wr, logic [25:0] a, logic [15:0] d, logic [1:0] mask = 2'b00);
		realtime t0;
		real tw;
		wait (!st_stop);
		@(posedge clk);
		t0 = $realtime;
		st_cs <= 1; st_addr <= a; st_din <= d; st_mask <= wr ? mask : 2'b00;
		if (wr) st_wr <= 1; else st_rd <= 1;
		@(posedge clk); st_wr <= 0; st_rd <= 0;
		@(posedge clk);
		while (st_wait == 0) @(posedge clk);
		if (wr) begin
			logic [15:0] o;
			o = st_shadow.exists(b_index(a)) ? st_shadow[b_index(a)] : u_sdram.init_val(b_index(a));
			if (!mask[1]) o[15:8] = d[15:8];
			if (!mask[0]) o[7:0]  = d[7:0];
			st_shadow[b_index(a)] = o;
		end
		else begin
			logic [15:0] e;
			e = st_shadow.exists(b_index(a)) ? st_shadow[b_index(a)] : u_sdram.init_val(b_index(a));
			st_rd_n++;
			if (st_dout !== e) begin st_rd_err++; if (st_rd_err <= 5) $display("%t STM32 READ ERROR a=%07h got %04h expected %04h", $realtime, a, st_dout, e); end
		end
		@(posedge clk); st_cs <= 0;
		tw = $realtime - t0;
		st_ops++; st_wait_sum += tw;
		if (tw > st_wait_max) st_wait_max = tw;
	endtask

	task automatic st_access_nc(bit wr, logic [25:0] a, logic [15:0] d, logic [1:0] mask);
		wait (!st_stop);
		@(posedge clk);
		st_cs <= 1; st_addr <= a; st_din <= d; st_mask <= wr ? mask : 2'b00;
		if (wr) st_wr <= 1; else st_rd <= 1;
		@(posedge clk); st_wr <= 0; st_rd <= 0;
		@(posedge clk);
		while (st_wait == 0) @(posedge clk);
		@(posedge clk); st_cs <= 0;
		st_ops++;
	endtask
	task automatic st_read_now(logic [25:0] a, output logic [15:0] v);
		@(posedge clk);
		st_cs <= 1; st_addr <= a; st_rd <= 1;
		@(posedge clk); st_rd <= 0;
		@(posedge clk);
		while (st_wait == 0) @(posedge clk);
		v = st_dout;
		@(posedge clk); st_cs <= 0;
	endtask

	initial begin : stm32
		int unsigned gap, run;
		logic [25:0] base;
		bit wr;
		wait (reset);
		wait (`POR_DONE);
		if (B_LOAD == 6) forever begin
			// both sides on the same 16 words (TRAFFIC 4): random writes (word and byte) and reads
			logic [25:0] a;
			a = 26'h0000200 + 26'($urandom_range(15, 0) * 2);
			repeat ($urandom_range(12, 0)) @(posedge clk);
			if ($urandom % 2) st_access_nc(1, a, 16'($urandom), 2'($urandom_range(2, 0)));
			else               st_access_nc(0, a, 16'h0, 2'b00);
		end else if (B_LOAD == 5) forever begin
			// Fast scrolling through the menu (saturn_utils.c): for each new item the STM32 reads the cover
			// index at 0x4F0000, reads the cover image from cover.bin into 0x400C00 (back-to-back writes, as
			// fast as the SD card and FSMC allow), copies its 1 KB palette to 0x400100 (read + write), writes
			// the header words at 0x400004-0x40000E, then the next item comes 0.5-3 ms later.
			logic [25:0] a;
			int unsigned n;
			for (int i = 0; i < 8; i++) st_access(0, 26'h04F0000 + 26'($urandom_range(2047, 0) * 2), 16'h0);
			n = $urandom_range(24576, 4096);	// 8-48 KB image
			for (int i = 0; i < n; i++) begin
				repeat ($urandom_range(4, 0)) @(posedge clk);
				st_access(1, 26'h0400C00 + 26'(i * 2), 16'($urandom));
			end
			for (int i = 0; i < 512; i++) begin
				st_access(0, 26'h0400C00 + 26'(i * 2), 16'h0);
				st_access(1, 26'h0400100 + 26'(i * 2), 16'($urandom));
			end
			for (int i = 0; i < 6; i++) st_access(1, 26'h0400004 + 26'(i * 2), 16'($urandom));
			#($urandom_range(3000000, 500000));
		end else if (B_LOAD == 4) forever begin
			// SAROO menu cover decode (tinygif.c): the STM32 writes the image one pixel (byte) at a time
			// into SDRAM 0x401000 + y*512 + x (CS0 0x22401000), every 3-20 clocks of decoding, polls the
			// flag at 0x400000 between frames, and now and then copies a file into 0x0C0000-0x0FFFFF
			// (TMPBUFF-like area, same 4 MB as the firmware).
			for (int y = 0; y < 192; y++) begin
				for (int x = 0; x < 128; x++) begin
					logic [25:0] a;
					a = 26'h0401000 + 26'(y * 512 + x);
					repeat ($urandom_range(20, 3)) @(posedge clk);
					st_access(1, {a[25:1], 1'b0}, {2{8'($urandom)}}, a[0] ? 2'b01 : 2'b10);	// even address: low byte
				end
				if ((y % 48) == 47) st_access(0, 26'h0400000, 16'h0);	// poll the frame flag
				if (($urandom % 100) < 3) begin	// file read into the low area
					base = 26'h00C0000 + 26'($urandom_range(32767, 0) * 2);
					run = $urandom_range(256, 32);
					for (int i = 0; i < run; i++) st_access(1, base + 26'(i * 2), 16'($urandom));
				end
			end
		end else if (B_LOAD > 0) forever begin
			gap = (B_LOAD == 1) ? $urandom_range(400, 40) : (B_LOAD == 3) ? $urandom_range(200, 0) : $urandom_range(6, 0);
			repeat (gap) @(posedge clk);
			run = (B_LOAD == 3) ? $urandom_range(256, 32) : $urandom_range(16, 1);
			wr  = (B_LOAD == 3) ? (($urandom % 8) != 0) : (($urandom % 4) != 0);	// mostly writes, like sector fills
			// B area: row[9] = 0, so it never overlaps the Saturn test words
			// A24 = 1 (16-32 MB): below 32 MB, so the mirrored design doesn't fold it onto the Saturn's words
			base = {1'b0, 1'b1, 1'b0, 1'b0, 9'($urandom), 2'($urandom), 10'($urandom), 1'b0};
			for (int i = 0; i < run; i++) st_access(wr, base + 26'(i * 2), 16'($urandom));
		end
	end

	// ---------------- main ----------------
	initial begin : main
		logic [15:0] w;
		int unsigned r;
		realtime next_ref;
		void'($urandom(SEED));
		budget = (NW + 1) * TSCU;
		wait (reset);
		wait (`POR_DONE);		// SDRAM power-on sequence done
		#1000;
		w = 16'($urandom);
		next_ref = $realtime + 15000.0;
		while ($realtime < 1000.0 * SIM_US + 210000.0) begin
			if ($realtime >= next_ref) begin scu_refresh(); next_ref += 15000.0; end
			if (TRAFFIC == 1) begin
				// Like RAMCART-Stress test 5: 32-bit write A, read B, read A (each longword = 2 bus cycles,
				// high halfword first), no idle clocks
				logic [15:0] a, b, hi, lo;
				a = {16'($urandom) & 16'hFFFE};
				b = {16'($urandom) & 16'hFFFE};
				hi = 16'($urandom); lo = 16'($urandom);
				scu_access(1, a, hi); scu_access(1, a + 1, lo);
				scu_access(0, b, 16'h0); scu_access(0, b + 1, 16'h0);
				scu_access(0, a, 16'h0); scu_access(0, a + 1, 16'h0);
			end else if (TRAFFIC == 2) begin
				// Like RAMCART-Stress test 3 (verify): sequential 32-bit reads
				scu_access(0, w, 16'h0); scu_access(0, w + 1, 16'h0);
				w = w + 2;
			end else if (TRAFFIC == 4) begin
				w = 16'h0100 + 16'($urandom_range(15, 0));
				if ($urandom % 2) begin
					logic [15:0] v;
					v = 16'($urandom);
					scu_access(1, w, v);
					if ($urandom % 2) begin	// read it straight back: must see v unless the STM32 wrote it meanwhile
						shadow[w] = v;
						scu_access(0, w, 16'h0);
					end
				end else scu_access(0, w, 16'h0);
				repeat ($urandom % 3) @(posedge sclk);
			end else if (TRAFFIC == 3) begin
				int unsigned k;
				#($urandom_range(30000, 1000));			// loop in the SH-2 cache
				w = (($urandom % 4) == 0) ? 16'($urandom) & 16'hFFF8 : (w & 16'hFFF8) + 16'd8;
				k = $urandom_range(40, 1);
				for (int i = 0; i < k; i++) begin
					if ($realtime >= next_ref) begin scu_refresh(); next_ref += 15000.0; end
					for (int j = 0; j < 8; j++) scu_access(0, w + 16'(j), 16'h0);	// one cache-line fill
					r = $urandom % 100;
					if (r < 75)      w = w + 16'd8;		// next line
					else if (r < 90) w = (w + 16'($urandom_range(256, 0) * 8)) & 16'hFFF8;	// near jump
					else             w = 16'($urandom) & 16'hFFF8;	// far jump
					if (($urandom % 100) < 10) scu_access(1, 16'($urandom), 16'($urandom));	// data write
				end
			end else begin
			r = $urandom % 100;
			if (r < 55)      w = w + 1;			// sequential
			else if (r < 70) ;				// same word again
			else             w = 16'($urandom);	// random
			if (($urandom % 100) < 30) scu_access(1, w, 16'($urandom));
			else                       scu_access(0, w, 16'h0);
			repeat ($urandom % 3) @(posedge sclk);	// 0-2 idle clocks
			end
			if (($urandom % 100) < SAT_IDLE_PCT) #($urandom_range(3000, 500));	// Saturn busy elsewhere
		end
		st_stop = 1;
		#2000;
		if (TRAFFIC == 4) begin	// both sides idle: everyone must now see the same value
			int bad = 0, sat_bad;
			n_err = 0; final_chk = 1;	// the Saturn reads below are checked
			for (int i = 0; i < 16; i++) begin
				logic [15:0] ww, sv, m0, m1;
				int i0, i1;
				ww = 16'h0100 + 16'(i);
				i0 = a_index(ww);
				i1 = {i0[24:23] ^ 2'b01, 1'b1, i0[21:0]};	// copy 1 (mirrored design)
				m0 = u_sdram.rd_word(i0);
				m1 = u_sdram.rd_word(i1);
				shadow[ww] = swap(m0);
				scu_access(0, ww, 16'h0);	// twice: the second read comes from the FPGA line cache
				scu_access(0, ww, 16'h0);
				st_read_now({2'b00, a_bus_addr(ww)}, sv);
				if (sv !== m0) begin bad++; $display("SHARED w=%04h STM32 reads %04h, SDRAM %04h", ww, sv, m0); end
`ifdef CH9
				if (u_hub._tsd.mirror && m0 !== m1) begin bad++; $display("SHARED w=%04h copies %04h / %04h", ww, m0, m1); end
`endif
			end
			sat_bad = n_err;
			$display("SHARED %s | after the race: Saturn reads wrong %0d, STM32 reads or copies wrong %0d", NAME, sat_bad, bad);
`ifdef CH9
			$display("SHARED %s | Saturn wrote a word the STM32 was writing: %0d (second copy skipped %0d) | Saturn read a word whose second copy was pending: %0d", NAME, c_sat, c_skip, c_s2m);
`endif
		end
		$display("STM32 %s | accesses %0d | reads %0d READ ERRORS %0d | wait avg %.0f max %.0f ns", NAME, st_ops, st_rd_n, st_rd_err,
			(st_ops ? st_wait_sum / st_ops : 0.0), st_wait_max);
		$display("STALE %s | errors that returned the previous read's value: %0d of %0d", NAME, e_stale, n_err);
`ifdef CH9
		$display("MIRROR %s | mirrored %0d | copies differing %0d | bank-rows refreshed %0d (needed %0d for this time)", NAME, u_hub._tsd.mirror, u_sdram.mirror_mismatch(), u_hub._tsd.rr_done, int'(($realtime - 200000.0) / 1953.1));
`endif
		$display("CAUSES %s | read start missed %0d | cachebus still busy %0d | internal refresh %0d | Saturn-refresh refresh %0d | STM32 in progress %0d (same bank %0d) | other (too slow) %0d",
			NAME, e_nostart, e_busy, e_ref, e_ref_ext, e_st, e_st_same, e_other);
		$display("RESULT %s NW=%0d EWT=%0d B_LOAD=%0d | reads %0d writes %0d | READ ERRORS %0d | late %0d | read latency avg %.1f max %.1f ns (SCU samples at %.1f) | SDRAM errors %0d contention %0d | refs %0d max gap %.2f us | sdram rd %0d wr %0d",
			NAME, NW, EWT, B_LOAD, n_rd, n_wr, n_err, n_late, (lat_n ? lat_sum / lat_n : 0.0), lat_max, budget,
			u_sdram.errors, contention, u_sdram.refs, u_sdram.max_ref_gap / 1000.0, u_sdram.reads, u_sdram.writes);
		$finish;
	end
endmodule
