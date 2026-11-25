
///////////////////////////////////////////////////////
// Module: Tiny SDRAM Controller                     //
///////////////////////////////////////////////////////
//
// [ABUS_EXP] Change 9: mirrored memory, so a Saturn read never waits for the
// STM32 or for refresh.
//
// With Sega's cart timing (external WAIT off) a Saturn read has to be
// answered a fixed time after it starts. An SDRAM bank that is busy (an STM32
// access, a refresh) can't be interrupted and holds the Saturn up for up to
// ~90ns: the answer comes too late. Change 8 let the Saturn go in between an
// STM32 access in another bank, but not in the same bank, and not during a
// refresh (all banks).
//
// Change 9 (MIRROR=1, 64MB SDRAM):
// - Every word is kept twice: copy 0 in bank B, rows 0-4095, and copy 1 in
//   bank B^1, rows 4096-8191 (B from address bits A23:A22, one bank per 4 MB
//   as in change 8b). Every write goes to both copies; a read takes whichever
//   copy is in a free bank. The Saturn's 32 MB window (with AA24) and the
//   STM32's pages 0-1 are the whole logical memory; the upper half of the
//   chip holds the second copies (the STM32 firmware never selects pages 2-3;
//   if it did, they would show pages 0-1 again).
// - Background work (STM32 accesses, refresh) uses one bank at a time: a new
//   background access starts only when the bank of the last one has finished
//   its precharge. So at any moment at most one bank is busy with background
//   work, and of the two banks that hold a Saturn word, one is always free.
// - Refresh is done one bank at a time: ACTIVATE then PRECHARGE of each row of
//   each bank in turn (refreshes a row like an auto refresh does, but only in
//   that bank), 4 rows per 7.81us = all 32768 bank-rows every 64ms. The
//   all-bank AUTO REFRESH is used only during power-up.
// - The Saturn's commands always go first. What can still delay a Saturn read
//   is one command-bus clock and, after an STM32 READ, the 4-clock spacing
//   between bursts: 1-2 clocks, well inside the Sega timing budget.
// - A Saturn write is acknowledged after its first copy; the second copy is
//   written as soon as its bank is free (no background work starts until it
//   is). A Saturn read of that word meanwhile takes the copy already written.
// - An STM32 write is acknowledged after both copies. If the Saturn writes
//   the same word in between, the Saturn's value wins: the STM32's second copy
//   is skipped and t_noupd tells cachebus not to put the STM32's (older) value
//   in the line caches.
//
// MIRROR=0, or an SDRAM smaller than 64MB: single copy (no second copy has
// room). The rest stays the same (two ports, one background bank at a time,
// per-bank refresh), but then a Saturn read can meet a busy bank, as in
// change 8.
//
// Timing rules (100MHz, CL=2, BL=4), counted from the command:
//   ACT -> READ/WRITE same bank  >= 2 clocks (tRCD); Saturn write 4 (change 7)
//   ACT -> ACT other bank        >= 2 clocks (tRRD)
//   ACT -> PRECHARGE             >= 5 clocks (tRAS)
//   PRECHARGE -> ACT             >= 2 clocks (tRP)
//   READA -> ACT same bank       >= 7 clocks (burst + tRP, as before)
//   WRITEA -> ACT same bank      >= 5 clocks (tWR + tRP, as before)
//   READ -> READ                 >= 4 clocks (no burst is ever cut short)
//   READ -> WRITE                >= 7 clocks (read burst off the data bus)
// DQM is held low for the 4 clocks of a read burst; the data bus is driven
// only on the WRITE clock.

module tsdram(
	reset,
	clk,
	reg_sdram,

	ext_refresh,
	sat_cs,
	sat_start,
	sat_hit,
	stm_ok,

	s_req, s_addr, s_din, s_mask, s_ack, s_dv,
	t_req, t_go, t_addr, t_din, t_mask, t_ack, t_noupd, t_dv, t_acc,
	cmd_dout,

	sd_cke,
	sd_cs,
	sd_ras,
	sd_cas,
	sd_we,
	sd_addr,
	sd_ba,
	sd_dqm,
	sd_din,
	sd_dout,
	sd_oe
);

// s_req / t_req:
//   00: nop
//   01: write byte
//   11: write word
//   10: read  word
// A request is held until its *_ack pulse (one clock), then dropped for at
// least one clock. *_dv marks the four burst words of that port's read on
// cmd_dout.

	input reset;
	input clk;
	input [11:0] reg_sdram;

	input ext_refresh;	// Saturn refresh cycle: not needed any more (per-bank refresh)
	input sat_cs;		// Saturn CS0 active (raw, asynchronous)
	input sat_start;	// Saturn read/write start pulse (the clock before its request reaches us)
	input sat_hit;		// Saturn read served from the line cache (1-clock pulse)
	output stm_ok;		// Change 5: an STM32 access may start now

	input [ 1:0] s_req;
	input [25:0] s_addr;
	input [15:0] s_din;
	input [ 1:0] s_mask;
	output s_ack;
	output s_dv;

	input [ 1:0] t_req;
	input t_go;			// Change 5 gate from memhub
	input [25:0] t_addr;
	input [15:0] t_din;
	input [ 1:0] t_mask;
	output t_ack;
	output t_noupd;		// with t_ack: the Saturn overwrote this word meanwhile, don't cache the STM32's value
	output t_dv;
	output t_acc;		// pulse: the STM32 request was taken (its ACT went out)

	output[15:0] cmd_dout;

	output sd_cke;
	output sd_cs;
	output sd_ras;
	output sd_cas;
	output sd_we;
	output[12:0] sd_addr;
	output[ 1:0] sd_ba;
	output[ 1:0] sd_dqm;
	input [15:0] sd_din;
	output[15:0] sd_dout;
	output sd_oe;

///////////////////////////////////////////////////////
// Module Parameters                                 //
///////////////////////////////////////////////////////

	// default: MT48LC32M16A2-75 512Mb
	// bank(4) x row(8192) x col(1024)
	parameter tCK  = 10;
	parameter MIRROR = 1;

	localparam tPOR_count = (200000/tCK) + 100;

	// a counter loaded with N-1 reads 0 N clocks later
	localparam N_RD_ACT  = 4'd6;	// READA  -> ACT same bank: 7
	localparam N_WR_ACT  = 4'd4;	// WRITEA -> ACT same bank: 5
	localparam N_PRE_ACT = 4'd1;	// PRE    -> ACT same bank: 2 (tRP)
	localparam N_RRD     = 2'd1;	// ACT    -> ACT: 2
	localparam N_RCD     = 3'd1;	// ACT    -> READ/WRITE: 2
	localparam N_RCD_SW  = 3'd3;	// ACT    -> Saturn WRITE: 4 (Change 7)
	localparam N_RAS     = 3'd4;	// ACT    -> PRE: 5 (tRAS)
	localparam N_RD_RD   = 3'd3;	// READ   -> READ: 4
	localparam N_RD_WR   = 3'd6;	// READ   -> WRITE: 7


///////////////////////////////////////////////////////
// Address Map                                       //
///////////////////////////////////////////////////////
// Logical bank = A23:A22 (8MB part: A22:A21): one bank per 4 MB (change 8b).
//   64MB, mirrored: copy c in bank (A23:A22)^c, row {c, A24, A21:A11}, col A10:A1
//   64MB, single:   bank A23:A22, row {A25:A24, A21:A11},             col A10:A1
//   32MB:           bank A23:A22, row {A24, A21:A10},                  col A9:A1
//   16MB:           bank A23:A22, row A21:A10,                         col A9:A1
//    8MB:           bank A22:A21, row A20:A9,                          col A8:A1

	wire[1:0] sdtype = reg_sdram[11:10];
	wire mirror = (MIRROR!=0) && (sdtype==2'b11);

	function [12:0] f_col;
		input [1:0] t; input [25:0] a;
		f_col = (t==2'b11)? {3'b001,   a[10:1]} :
		        (t==2'b10)? {4'b0010,  a[ 9:1]} :
		        (t==2'b01)? {4'b0010,  a[ 9:1]} :
		                    {5'b00100, a[ 8:1]} ;
	endfunction
	function [1:0] f_bank;		// copy c (mirrored only)
		input [1:0] t; input m; input [25:0] a; input c;
		f_bank = (t==2'b00)? a[22:21] : (a[23:22] ^ {1'b0, (m & c)});
	endfunction
	function [12:0] f_row;
		input [1:0] t; input m; input [25:0] a; input c;
		f_row = m         ? {c,      a[24], a[21:11]} :
		        (t==2'b11)? {a[25:24],      a[21:11]} :
		        (t==2'b10)? {a[24],         a[21:10]} :
		        (t==2'b01)? {1'b0,          a[21:10]} :
		                    {1'b0,          a[20: 9]} ;
	endfunction


///////////////////////////////////////////////////////
// Bank state                                        //
///////////////////////////////////////////////////////

	reg [3:0] bank_cnt0, bank_cnt1, bank_cnt2, bank_cnt3;	// clocks before an ACT is allowed
	reg [3:0] bank_open;			// ACT done, not yet READA/WRITEA/PRE
	reg [1:0] rrd_cnt;
	reg [2:0] rd_cnt;				// READ -> READ
	reg [2:0] wr_cnt;				// READ -> WRITE

	wire [3:0] bank_free = { (bank_cnt3==0 && !bank_open[3]), (bank_cnt2==0 && !bank_open[2]),
	                         (bank_cnt1==0 && !bank_open[1]), (bank_cnt0==0 && !bank_open[0]) };


///////////////////////////////////////////////////////
// Saturn slot (S), Saturn second copy (S2)          //
///////////////////////////////////////////////////////

	reg        s_open, s_taken;
	reg [ 2:0] s_rcd;
	reg [ 1:0] s_ba;
	reg [12:0] s_row, s_col;
	reg        s_isrd, s_copy;
	reg [24:1] s_wa;			// address of the Saturn write being done

	reg        s2_pend, s2_open;	// second copy of a Saturn write: waiting / ACT done
	reg [ 2:0] s2_rcd;
	reg [ 1:0] s2_ba;
	reg [12:0] s2_row, s2_col;
	reg [15:0] s2_din;
	reg [ 1:0] s2_mask;
	reg        s2_copy;			// the copy still to be written (the other one is up to date)
	reg [24:1] s2_addr;
	wire       s2_busy = s2_pend || s2_open;

	wire       s_new    = (s_req!=2'b00) && !s_taken;
	wire       s_new_rd = (s_req==2'b10);
	wire [1:0] s_b0     = f_bank(sdtype, mirror, s_addr, 1'b0);
	wire [1:0] s_b1     = f_bank(sdtype, mirror, s_addr, 1'b1);
	wire       s_match  = mirror && s2_busy && (s_addr[24:1]==s2_addr);
	// copy 1 when: this word's copy 0 is the one still being written, or
	// (otherwise) copy 0's bank is busy
	wire       s_use1   = mirror && (s_match ? (s2_copy==1'b0) : !bank_free[s_b0]);
	wire [1:0] s_bsel   = s_use1 ? s_b1 : s_b0;
	wire       sat_waiting = s_new || s_open;


///////////////////////////////////////////////////////
// Background slot (BG): STM32 accesses, refresh     //
///////////////////////////////////////////////////////

	localparam K_TR = 2'd0, K_TW = 2'd1, K_RR = 2'd3;

	reg        bg_open;			// ACT done, command not yet
	reg [ 2:0] bg_cnt;			// clocks before its READ/WRITE (tRCD) or PRE (tRAS)
	reg [ 1:0] bg_kind;
	reg [ 1:0] bg_ba, bg_last;	// its bank / the bank of the last background access
	reg [12:0] bg_row, bg_col;

	reg        t_taken;			// the STM32 request has been started
	reg        t_seen;			// it was there last clock (decode below valid)
	reg        tw_wait2;		// STM32 write: first copy done, second to do
	reg        tw_skip;			// ... the Saturn wrote the same word meanwhile: skip the second
	reg        tw_sat;			// the Saturn wrote the same word during this STM32 write: ack with t_noupd
	reg [ 1:0] tw_ba2;
	reg [12:0] tw_row2, tw_col2;
	reg [15:0] tw_din;
	reg [ 1:0] tw_mask;
	reg        t_noupd;

	// the STM32 address is decoded one clock after its request appears (timing)
	reg  [1:0] t_b0, t_b1;
	reg [12:0] t_r0, t_r1, t_c;
	always @(posedge clk) begin
		t_b0 <= f_bank(sdtype, mirror, t_addr, 1'b0);
		t_b1 <= f_bank(sdtype, mirror, t_addr, 1'b1);
		t_r0 <= f_row (sdtype, mirror, t_addr, 1'b0);
		t_r1 <= f_row (sdtype, mirror, t_addr, 1'b1);
		t_c  <= f_col (sdtype, t_addr);
	end
	wire t_new_raw = (t_req!=2'b00) && !t_taken;
	wire t_new     = t_new_raw && t_seen;
	wire t_new_rd  = (t_req==2'b10);
	wire t_use1    = mirror && !bank_free[t_b0];	// either copy will do: take a free one
	wire [1:0] t_bsel = t_use1 ? t_b1 : t_b0;
	reg  [24:1] t_wa;			// address of the STM32 write being done (for tw_skip)
	reg         tw_active;		// an STM32 write is under way (first ACT .. ack)

	// per-bank refresh: 4 bank-rows owed per refresh interval (reg_sdram, 7.81us)
	reg[15:0] ref_count;
	reg       in_por;
	reg [6:0] rr_debt;
	reg [14:0] rr_next;			// {row, bank} to refresh next
	wire ref_tick = !in_por && (ref_count==1);

	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			ref_count <= tPOR_count[15:0];
		end else if(!in_por && ref_count<=1) begin
			ref_count <= {6'b0, reg_sdram[9:0]};
		end else if(ref_count) begin
			ref_count <= ref_count-1'b1;
		end
	end

	// a background access may start: nothing of its own still busy, no Saturn
	// access waiting or starting, no Saturn second copy to write
	wire bg_ready = !in_por && !bg_open && bank_free[bg_last] && !s2_busy && !sat_waiting && !sat_start;


///////////////////////////////////////////////////////
// Change 5 (single copy only): STM32 access slots    //
///////////////////////////////////////////////////////
// Mirrored, the STM32 can't hold up the Saturn, so it is never held back.
// Single copy: as change 8 (STM32 starts right after a Saturn cache hit or
// read while the Saturn runs fast cycles; memhub's 2us limit still applies).

	parameter  SAT_IDLE_CLKS = 10'd300;	// 3us
	localparam STM_HIT_WIN   = 3'd6;
	localparam FAST_CYCLE_CLKS = 5'd16;

	reg[1:0] cs_sync;
	always @(posedge clk)
		cs_sync <= {cs_sync[0], sat_cs};

	reg[2:0] stm_win;
	reg[9:0] sat_quiet;
	reg[4:0] cs_len;
	reg      cs_last, fast_bus;
	reg      after_sat;
	reg[2:0] sat_rd_cnt;
	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			stm_win <= 0; sat_quiet <= 0; cs_len <= 0; cs_last <= 0; fast_bus <= 0;
		end else begin
			if(sat_hit) stm_win <= STM_HIT_WIN;
			else if(stm_win) stm_win <= stm_win-1'b1;

			if(cs_sync[1]) sat_quiet <= 0;
			else if(sat_quiet!=10'h3ff) sat_quiet <= sat_quiet+1'b1;

			cs_last <= cs_sync[1];
			if(cs_sync[1]) begin
				if(cs_len!=5'h1f) cs_len <= cs_len+1'b1;
			end else begin
				if(cs_last) fast_bus <= (cs_len <= FAST_CYCLE_CLKS);
				cs_len <= 0;
			end
		end
	end

	wire sat_mode = fast_bus && (sat_quiet < SAT_IDLE_CLKS);
	assign stm_ok = mirror || !sat_mode || (stm_win!=0) || after_sat;


///////////////////////////////////////////////////////
// Command choice                                    //
///////////////////////////////////////////////////////

	// 1. Saturn READ/WRITE
	wire go_s_cmd  = s_open && s_rcd==0 && (s_isrd? (rd_cnt==0) : (wr_cnt==0));
	// 2. Saturn ACT (a write waits for the second copy of the last one)
	wire go_s_act  = s_new && !s_open && bank_free[s_bsel] && rrd_cnt==0 && (s_new_rd || !s2_busy);
	// 3. second copy of a Saturn write: WRITE, then ACT
	wire go_s2_cmd = s2_open && s2_rcd==0 && wr_cnt==0;
	wire go_s2_act = s2_pend && bank_free[s2_ba] && rrd_cnt==0;
	// 4. background command: STM32 READ (only while no Saturn access waits,
	//    unless single copy and the Saturn needs this bank) / WRITE / PRE
	wire s_needs_bg = !mirror && s_new && !s_open && (s_b0==bg_ba);
	wire go_bg_cmd = bg_open && bg_cnt==0 &&
	                 ( (bg_kind==K_RR) ||
	                   (bg_kind==K_TW && wr_cnt==0) ||
	                   (bg_kind==K_TR && rd_cnt==0 && (!sat_waiting || s_needs_bg)) );
	// 5. background ACT: second copy of an STM32 write, else a row refresh,
	//    else a new STM32 access
	wire bg_rr   = (rr_debt!=0);
	wire [1:0] bg_new_ba = tw_wait2 ? tw_ba2 : bg_rr ? rr_next[1:0] : t_bsel;
	wire go_tw_skip = bg_ready && tw_wait2 && tw_skip;
	wire go_bg_act  = bg_ready && rrd_cnt==0 && bank_free[bg_new_ba] &&
	                  ( (tw_wait2 && !tw_skip) || (!tw_wait2 && bg_rr) || (!tw_wait2 && !bg_rr && t_new && t_go) );

	wire [2:0] pick = go_s_cmd  ? 3'd1 :
	                  go_s_act  ? 3'd2 :
	                  go_s2_cmd ? 3'd3 :
	                  go_s2_act ? 3'd4 :
	                  go_bg_cmd ? 3'd5 :
	                  go_bg_act ? 3'd6 : 3'd0;


///////////////////////////////////////////////////////
// Main state                                        //
///////////////////////////////////////////////////////

	reg[ 3:0] m_cmd;
	reg[ 1:0] m_dqm;
	reg[ 1:0] m_ba;
	reg[12:0] m_addr;
	reg[15:0] m_dout;
	reg m_cke;
	reg m_oe;
	reg s_ack, t_ack, t_acc;
	reg rd_issue, rd_port;		// a READ went out, for port S (0) or T (1)
	reg[1:0] rd_dqm;			// clocks DQM stays low after a READ
	reg[15:0] rr_done;			// bank-rows refreshed (for the testbench)

	task set_bank_cnt;
		input [1:0] b; input [3:0] v;
		begin
			case(b)
			2'd0: bank_cnt0 <= v;
			2'd1: bank_cnt1 <= v;
			2'd2: bank_cnt2 <= v;
			2'd3: bank_cnt3 <= v;
			endcase
		end
	endtask

	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			in_por <= 1;
			m_cmd <= 4'b1111; m_dqm <= 2'b11; m_ba <= 0; m_addr <= 13'h1fff; m_dout <= 0;
			m_oe <= 0; m_cke <= 0;
			s_open <= 0; s_taken <= 0; s_rcd <= 0; s_ba <= 0; s_row <= 0; s_col <= 0; s_isrd <= 0; s_copy <= 0; s_wa <= 0;
			s2_pend <= 0; s2_open <= 0; s2_rcd <= 0; s2_ba <= 0; s2_row <= 0; s2_col <= 0; s2_din <= 0; s2_mask <= 0; s2_copy <= 0; s2_addr <= 0;
			bg_open <= 0; bg_cnt <= 0; bg_kind <= 0; bg_ba <= 0; bg_last <= 0; bg_row <= 0; bg_col <= 0;
			t_taken <= 0; t_seen <= 0; tw_wait2 <= 0; tw_skip <= 0; tw_ba2 <= 0; tw_row2 <= 0; tw_col2 <= 0;
			tw_din <= 0; tw_mask <= 0; t_noupd <= 0; t_wa <= 0; tw_active <= 0; tw_sat <= 0;
			bank_cnt0 <= 0; bank_cnt1 <= 0; bank_cnt2 <= 0; bank_cnt3 <= 0; bank_open <= 0;
			rrd_cnt <= 0; rd_cnt <= 0; wr_cnt <= 0;
			s_ack <= 0; t_ack <= 0; t_acc <= 0;
			rd_issue <= 0; rd_port <= 0; rd_dqm <= 0;
			rr_debt <= 0; rr_next <= 0; rr_done <= 0;
			after_sat <= 0; sat_rd_cnt <= 0;
		end else begin
			m_cmd <= 4'b0111;		// NOP
			m_dqm <= (rd_dqm!=0)? 2'b00 : 2'b11;
			if(rd_dqm) rd_dqm <= rd_dqm-1'b1;
			m_oe <= 0;
			s_ack <= 0; t_ack <= 0; t_acc <= 0; t_noupd <= 0;
			rd_issue <= 0;

			if(bank_cnt0) bank_cnt0 <= bank_cnt0-1'b1;
			if(bank_cnt1) bank_cnt1 <= bank_cnt1-1'b1;
			if(bank_cnt2) bank_cnt2 <= bank_cnt2-1'b1;
			if(bank_cnt3) bank_cnt3 <= bank_cnt3-1'b1;
			if(rrd_cnt) rrd_cnt <= rrd_cnt-1'b1;
			if(rd_cnt)  rd_cnt  <= rd_cnt-1'b1;
			if(wr_cnt)  wr_cnt  <= wr_cnt-1'b1;
			if(s_rcd)   s_rcd   <= s_rcd-1'b1;
			if(s2_rcd)  s2_rcd  <= s2_rcd-1'b1;
			if(bg_cnt)  bg_cnt  <= bg_cnt-1'b1;

			after_sat <= (sat_rd_cnt==3'd1);
			if(sat_rd_cnt) sat_rd_cnt <= sat_rd_cnt-1'b1;

			if(s_req==2'b00 && !s_open) s_taken <= 0;
			if(t_req==2'b00 && !tw_active && !(bg_open && bg_kind!=K_RR)) t_taken <= 0;
			t_seen <= t_new_raw && !(pick==3'd6 && !tw_wait2 && !bg_rr);

			// refresh debt: 4 bank-rows per interval, one paid per PRECHARGE
			if(ref_tick && !(pick==3'd5 && bg_kind==K_RR))
				rr_debt <= (rr_debt>7'd123)? 7'd127 : rr_debt+3'd4;
			else if(ref_tick)
				rr_debt <= rr_debt+3'd3;
			else if(pick==3'd5 && bg_kind==K_RR)
				rr_debt <= rr_debt-1'b1;

			// STM32 write skipped (the Saturn wrote the word after its first copy)
			if(go_tw_skip && !(pick==3'd1 || pick==3'd2 || pick==3'd3 || pick==3'd4)) begin
				tw_wait2 <= 0; tw_skip <= 0; tw_sat <= 0; tw_active <= 0;
				t_ack <= 1; t_noupd <= 1;
			end

			if(in_por) begin
				m_cmd <= 4'b1111;
				if(ref_count==500) begin
					m_cke <= 1;
				end else if(ref_count==40) begin
					m_cmd <= 4'b0010;
					m_addr <= 13'h0400;		// PRECHARGE all
				end else if(ref_count==30) begin
					m_cmd <= 4'b0001;
				end else if(ref_count==20) begin
					m_cmd <= 4'b0001;
				end else if(ref_count==10) begin
					m_cmd <= 4'b0000;
					// CL=2 RBL=4 WBL=1
					m_addr <= {3'b000,1'b1,2'b00,3'b010,4'b0010};
				end else if(ref_count==0) begin
					in_por <= 0;
				end else begin
					m_cmd <= 4'b0111;
				end
			end else begin
				case(pick)
				3'd1: begin		// Saturn READ/WRITE (auto precharge: col bit 10 = 1)
					s_open <= 0;
					s_ack <= 1;
					m_ba <= s_ba;
					m_addr <= s_col;
					bank_open[s_ba] <= 1'b0;
					if(s_isrd) begin
						m_cmd <= 4'b0101;
						m_dqm <= 2'b00;
						set_bank_cnt(s_ba, N_RD_ACT);
						rd_cnt <= N_RD_RD; wr_cnt <= N_RD_WR;
						rd_issue <= 1; rd_port <= 0; rd_dqm <= 2'd3;
						sat_rd_cnt <= 3'd7;
					end else begin
						m_cmd <= 4'b0100;
						m_dqm <= s_mask;
						m_dout <= s_din;
						m_oe <= 1;
						set_bank_cnt(s_ba, N_WR_ACT);
						if(mirror) begin	// the other copy next
							s2_pend <= 1;
							s2_copy <= ~s_copy;
							s2_ba   <= s_ba ^ 2'b01;
							s2_row  <= {~s_row[12], s_row[11:0]};
							s2_col  <= s_col;
							s2_din  <= s_din;
							s2_mask <= s_mask;
							s2_addr <= s_wa;
						end
					end
				end
				3'd2: begin		// Saturn ACT
					m_cmd <= 4'b0011;
					m_ba <= s_bsel;
					m_addr <= f_row(sdtype, mirror, s_addr, s_use1);
					bank_open[s_bsel] <= 1'b1;
					rrd_cnt <= N_RRD;
					s_open <= 1; s_taken <= 1;
					s_ba <= s_bsel;
					s_row <= f_row(sdtype, mirror, s_addr, s_use1);
					s_col <= f_col(sdtype, s_addr);
					s_isrd <= s_new_rd;
					s_copy <= s_use1;
					s_wa <= s_addr[24:1];
					s_rcd <= s_new_rd? N_RCD : N_RCD_SW;
					// the Saturn writes a word the STM32 is writing: its value wins
					// (second copy not started yet: skip it; in any case the STM32's
					// ack must not put its value in the line caches)
					if(mirror && !s_new_rd && tw_active && (s_addr[24:1]==t_wa)) begin
						tw_sat <= 1;
						if(!(bg_open && bg_kind==K_TW && tw_wait2)) tw_skip <= 1;
					end
				end
				3'd3: begin		// second copy of a Saturn write: WRITEA
					s2_open <= 0;
					m_cmd <= 4'b0100;
					m_ba <= s2_ba;
					m_addr <= s2_col;
					m_dqm <= s2_mask;
					m_dout <= s2_din;
					m_oe <= 1;
					bank_open[s2_ba] <= 1'b0;
					set_bank_cnt(s2_ba, N_WR_ACT);
				end
				3'd4: begin		// second copy of a Saturn write: ACT
					m_cmd <= 4'b0011;
					m_ba <= s2_ba;
					m_addr <= s2_row;
					bank_open[s2_ba] <= 1'b1;
					rrd_cnt <= N_RRD;
					s2_pend <= 0; s2_open <= 1;
					s2_rcd <= N_RCD;
				end
				3'd5: begin		// background READ / WRITE / PRECHARGE
					bg_open <= 0;
					m_ba <= bg_ba;
					bank_open[bg_ba] <= 1'b0;
					case(bg_kind)
					K_RR: begin
						m_cmd <= 4'b0010;
						m_addr <= 13'h0000;		// A10=0: this bank only
						set_bank_cnt(bg_ba, N_PRE_ACT);
						rr_next <= rr_next+1'b1;
						rr_done <= rr_done+1'b1;
					end
					K_TR: begin
						m_cmd <= 4'b0101;
						m_addr <= bg_col;
						m_dqm <= 2'b00;
						set_bank_cnt(bg_ba, N_RD_ACT);
						rd_cnt <= N_RD_RD; wr_cnt <= N_RD_WR;
						rd_issue <= 1; rd_port <= 1; rd_dqm <= 2'd3;
						t_ack <= 1;
					end
					default: begin	// K_TW
						m_cmd <= 4'b0100;
						m_addr <= bg_col;
						m_oe <= 1;
						set_bank_cnt(bg_ba, N_WR_ACT);
						if(tw_wait2 || !mirror) begin	// last copy written
							m_dqm <= tw_wait2 ? tw_mask : t_mask;
							m_dout <= tw_wait2 ? tw_din : t_din;
							tw_wait2 <= 0; tw_skip <= 0; tw_sat <= 0; tw_active <= 0;
							t_ack <= 1; t_noupd <= tw_sat;
						end else begin					// first copy: the other next
							m_dqm <= t_mask;
							m_dout <= t_din;
							tw_din <= t_din; tw_mask <= t_mask;
							tw_wait2 <= 1;
							tw_ba2 <= bg_ba ^ 2'b01;
							tw_row2 <= {~bg_row[12], bg_row[11:0]};
							tw_col2 <= bg_col;
						end
					end
					endcase
				end
				3'd6: begin		// background ACT
					m_cmd <= 4'b0011;
					m_ba <= bg_new_ba;
					bank_open[bg_new_ba] <= 1'b1;
					rrd_cnt <= N_RRD;
					bg_open <= 1;
					bg_ba <= bg_new_ba;
					bg_last <= bg_new_ba;
					if(tw_wait2) begin
						m_addr <= tw_row2;
						bg_kind <= K_TW; bg_row <= tw_row2; bg_col <= tw_col2;
						bg_cnt <= N_RCD;
					end else if(bg_rr) begin
						m_addr <= rr_next[14:2];
						bg_kind <= K_RR; bg_row <= rr_next[14:2];
						bg_cnt <= N_RAS;
					end else begin
						m_addr <= t_use1 ? t_r1 : t_r0;
						bg_kind <= t_new_rd ? K_TR : K_TW;
						bg_row <= t_use1 ? t_r1 : t_r0;
						bg_col <= t_c;
						bg_cnt <= N_RCD;
						t_taken <= 1; t_acc <= 1;
						if(!t_new_rd) begin tw_active <= 1; t_wa <= t_addr[24:1]; end
					end
				end
				default: ;
				endcase
			end
		end
	end


///////////////////////////////////////////////////////
// Read data                                         //
///////////////////////////////////////////////////////
// The words of a burst are on sd_din 3 to 6 clocks after the clock that chose
// the READ. READs are at least 4 clocks apart.

	reg[3:0] rdp;		// rdp[k]: a READ was chosen k+2 clocks ago
	reg[3:0] rdt;		// ... and it was for port T
	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			rdp <= 0;
			rdt <= 0;
		end else begin
			rdp <= {rdp[2:0], rd_issue};
			rdt <= {rdt[2:0], rd_port};
		end
	end

	wire[3:0] dv_win = rdp[3:0];
	wire[3:0] dv_tw  = rdp[3:0] & rdt[3:0];
	reg s_dv, t_dv;
	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			s_dv <= 0;
			t_dv <= 0;
		end else begin
			s_dv <= (dv_win!=0) && (dv_tw==0);
			t_dv <= (dv_tw!=0);
		end
	end

	wire[15:0] cmd_dout = sd_din;


///////////////////////////////////////////////////////
// SDRAM signals                                     //
///////////////////////////////////////////////////////

	assign sd_cke = m_cke;
	assign {sd_cs, sd_ras, sd_cas, sd_we} = m_cmd;
	assign sd_addr = m_addr;
	assign sd_ba = m_ba;
	assign sd_dqm = m_dqm;
	assign sd_dout = m_dout;
	assign sd_oe = m_oe;

endmodule

