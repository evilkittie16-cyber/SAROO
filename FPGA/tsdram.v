
///////////////////////////////////////////////////////
// Module: Tiny SDRAM Controller                     //
///////////////////////////////////////////////////////
//
// [ABUS_EXP] Change 8: two request ports and bank overlap.
//
// Before (changes 1-7): one request at a time, through memhub's arbiter. An
// STM32 access that had started ran to the end (ACT, READ/WRITE, precharge)
// before a Saturn request could even be looked at: up to ~100ns extra. With
// Sega's timing (external WAIT off) that answer is too late, and the Saturn
// reads the wrong value. The SAROO menu hits this all the time: the Saturn
// waits in a loop running from its own cache while the STM32 copies from the
// SD card, and its next instruction fetch from the cart meets an STM32 access.
//
// Now the Saturn (port S) and the STM32 (port T) each have their own request
// slot, and every access is ACT, READ/WRITE with auto precharge, as before.
// One command per clock, chosen in this order:
//   1. the Saturn's READ/WRITE, when its row is open long enough (tRCD)
//   2. the Saturn's ACT, into any bank that is free
//   3. the STM32's READ/WRITE (an STM32 READ only when no Saturn access is
//      waiting, unless the Saturn is waiting for the STM32's bank)
//   4. a refresh, when every bank is idle
//   5. the STM32's ACT, when no Saturn request is waiting
// So when the Saturn asks while an STM32 access is under way in another bank,
// the Saturn's ACT and READ go in between, 1-2 clocks late instead of up to
// 9. In the same bank it still has to wait for the STM32 access to finish.
//
// Timing rules kept (100MHz, CL=2, BL=4):
//   ACT -> READ/WRITE same bank  >= 2 clocks (tRCD); Saturn write 4 (change 7)
//   ACT -> ACT other bank        >= 2 clocks (tRRD)
//   READA -> ACT same bank       >= 7 clocks (burst + tRP, as before)
//   WRITEA -> ACT same bank      >= 5 clocks (tWR + tRP, as before)
//   REF -> any ACT               >= 9 clocks (tRFC, as before)
//   READ -> READ                 >= 4 clocks (no burst is ever cut short)
//   READ -> WRITE                >= 7 clocks (read burst off the data bus)
// The data bus is now driven only on the WRITE clock itself (was: until the
// controller went idle), so a READ in another bank can follow at once.

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
	t_req, t_go, t_addr, t_din, t_mask, t_ack, t_dv, t_acc,
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
// A request is held until its *_ack pulse (one clock, the clock after the
// READ/WRITE command), then dropped for at least one clock. *_dv marks the
// four burst words of that port's read on cmd_dout.

	input reset;
	input clk;
	input [11:0] reg_sdram;

	input ext_refresh;
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
	parameter tRC  = 7;
	parameter tRAS = 5;
	parameter tRCD = 2;
	parameter tCL  = 2;
	parameter tWR  = 2;
	parameter tRP  = 2;
	parameter tRFC = 7;
	parameter tCK  = 10;

	parameter BL = 4;
	parameter COL_BITS = 10;
	parameter ROW_BITS = 13;

	localparam tPOR_count = (200000/tCK) + 100;

	// clocks a counter is loaded with so the next step is allowed N clocks
	// after the command (a counter loaded with N-1 reads 0 N clocks later)
	localparam N_RD_ACT  = 4'd6;	// READA  -> ACT same bank: 7
	localparam N_WR_ACT  = 4'd4;	// WRITEA -> ACT same bank: 5
	localparam N_REF_ACT = 4'd8;	// REF    -> ACT: 9
	localparam N_RRD     = 2'd1;	// ACT    -> ACT: 2
	localparam N_RCD     = 2'd1;	// ACT    -> READ/WRITE: 2
	localparam N_RCD_SW  = 2'd3;	// ACT    -> Saturn WRITE: 4 (Change 7)
	localparam N_RD_RD   = 3'd3;	// READ   -> READ: 4
	localparam N_RD_WR   = 3'd6;	// READ   -> WRITE: 7


///////////////////////////////////////////////////////
// Address Map                                       //
///////////////////////////////////////////////////////

	// A31 - A26 [A25 - A13] [A12 A11] [A10 - A1] A0
	//           [R12 -  R0] [ B1  B0] [ C9 - C0]      64MB
	//
	// A31 - A25 [A24 - A12] [A11 A10] [ A9 - A1] A0
	//           [R12 -  R0] [ B1  B0] [ C8 - C0]      32MB
	//
	// A31 - A24 [A23 - A12] [A11 A10] [ A9 - A1] A0
	//           [R11 -  R0] [ B1  B0] [ C8 - C0]      16MB
	//
	// A31 - A23 [A22 - A11] [A10  A9] [ A8 - A1] A0
	//           [R11 -  R0] [ B1  B0] [ C7 - C0]       8MB

	wire[1:0] sdtype = reg_sdram[11:10];

	function [12:0] f_col;
		input [1:0] t; input [25:0] a;
		f_col = (t==2'b11)? {3'b001,   a[10:1]} :
		        (t==2'b10)? {4'b0010,  a[ 9:1]} :
		        (t==2'b01)? {4'b0010,  a[ 9:1]} :
		                    {5'b00100, a[ 8:1]} ;
	endfunction
	function [1:0] f_bank;
		input [1:0] t; input [25:0] a;
		f_bank = (t==2'b11)? a[12:11] :
		         (t==2'b10)? a[11:10] :
		         (t==2'b01)? a[11:10] :
		                     a[10: 9] ;
	endfunction
	function [12:0] f_row;
		input [1:0] t; input [25:0] a;
		f_row = (t==2'b11)? a[25:13] :
		        (t==2'b10)? a[24:12] :
		        (t==2'b01)? {1'b0, a[23:12]} :
		                    {1'b0, a[22:11]} ;
	endfunction


///////////////////////////////////////////////////////
// Request slots                                     //
///////////////////////////////////////////////////////

	// A slot is open from its ACT to its READ/WRITE. *_taken stays set from
	// the ACT until the port drops its request (after *_ack), so the same
	// request is never served twice.
	reg        s_open, t_open;		// ACT done, READ/WRITE not yet
	reg        s_taken, t_taken;
	reg [ 1:0] s_rcd, t_rcd;		// clocks left before READ/WRITE is allowed
	reg [ 1:0] s_ba, t_ba;
	reg [12:0] s_col, t_col;
	reg        s_isrd, t_isrd;

	wire       s_new    = (s_req!=2'b00) && !s_taken;	// Saturn request not yet started
	wire       t_new_raw = (t_req!=2'b00) && !t_taken;
	reg        t_seen;		// the STM32 request was already there last clock (its bank below is valid)
	wire       t_new    = t_new_raw && t_seen;
	wire [1:0] s_new_ba = f_bank(sdtype, s_addr);
	// The STM32's bank, row and column are decoded one clock after its request appears (timing: the
	// decode, the bank checks and the command choice don't fit in one 10ns clock). It costs the STM32 one
	// clock; the Saturn's path is not delayed.
	reg  [1:0] t_new_ba;
	reg [12:0] t_new_row, t_new_col;
	always @(posedge clk) begin
		t_new_ba  <= f_bank(sdtype, t_addr);
		t_new_row <= f_row(sdtype, t_addr);
		t_new_col <= f_col(sdtype, t_addr);
	end
	wire       s_new_rd = (s_req==2'b10);
	wire       t_new_rd = (t_req==2'b10);


///////////////////////////////////////////////////////
// Timing counters                                   //
///////////////////////////////////////////////////////

	reg [3:0] bank_cnt0, bank_cnt1, bank_cnt2, bank_cnt3;	// clocks before an ACT is allowed in that bank
	reg [3:0] bank_open;			// ACT done, READA/WRITEA not yet
	reg [1:0] rrd_cnt;
	reg [2:0] rd_cnt;				// READ -> READ
	reg [2:0] wr_cnt;				// READ -> WRITE

	wire [3:0] bank_free = { (bank_cnt3==0 && !bank_open[3]), (bank_cnt2==0 && !bank_open[2]),
	                         (bank_cnt1==0 && !bank_open[1]), (bank_cnt0==0 && !bank_open[0]) };
	wire all_idle = (bank_free==4'b1111);


///////////////////////////////////////////////////////
// Refresh scheduler (Changes 1 and 4)               //
///////////////////////////////////////////////////////
// - The timer keeps running while a refresh is postponed and counts the owed
//   refreshes in ref_debt (average rate unchanged: >= 1 per 7.81us).
// - Owed refreshes are issued only when every bank is idle, at moments when a
//   Saturn access is unlikely to arrive during the refresh:
//     after_sat : right after a Saturn READ has finished in the SDRAM;
//     in_hit    : a Saturn read is being answered from the line cache;
//     quiet     : CS0 inactive for QUIET_CLKS clocks;
//     ext_req   : a Saturn refresh cycle (FC1).
// - When REF_DEBT_MAX refreshes are owed, refresh is forced: no new ACT is
//   started until the banks are idle and the refresh is out.
// - Otherwise a waiting Saturn request always goes first (Change 1).

	localparam REF_DEBT_MAX = 3'd4;
	localparam QUIET_CLKS   = 10'd64;	// 640ns

	reg[15:0] ref_count;
	reg       ref_ack;
	reg       in_por;
	wire      ref_tick = !in_por && (ref_count==1);

	reg[2:0] ref_debt;
	reg      ext_req;
	reg      after_sat;		// the clock a Saturn READ's bank becomes idle again
	reg[2:0] sat_rd_cnt;
	reg[9:0] sat_quiet;

	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			ref_count <= tPOR_count[15:0];
		end else if(ref_ack || (!in_por && ref_count<=1)) begin
			ref_count <= {6'b0, reg_sdram[9:0]};
		end else if(ref_count) begin
			ref_count <= ref_count-1'b1;
		end
	end

	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			ref_debt <= 0;
		end else if(ref_tick && !(ref_ack && ref_debt!=0)) begin
			if(ref_debt!=3'd7) ref_debt <= ref_debt+1'b1;
		end else if(!ref_tick && ref_ack && ref_debt!=0) begin
			ref_debt <= ref_debt-1'b1;
		end
	end

	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			ext_req <= 0;
		end else if(ext_refresh) begin
			ext_req <= 1;
		end else if(ref_ack) begin
			ext_req <= 0;
		end
	end

	// Saturn bus activity, from CS0 itself
	reg[1:0] cs_sync;
	reg[1:0] hit_win;

	always @(posedge clk)
		cs_sync <= {cs_sync[0], sat_cs};

	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			hit_win <= 0;
			sat_quiet <= 0;
		end else begin
			if(sat_hit)
				hit_win <= 2'd2;
			else if(hit_win)
				hit_win <= hit_win-1'b1;

			if(cs_sync[1])
				sat_quiet <= 0;
			else if(sat_quiet!=10'h3ff)
				sat_quiet <= sat_quiet+1'b1;
		end
	end

	wire in_hit      = (hit_win!=0);
	wire ref_force   = (ref_debt>=REF_DEBT_MAX);
	wire quiet       = (sat_quiet>=QUIET_CLKS);
	wire ref_ok      = ext_req || (ref_debt!=0 && (after_sat || in_hit || quiet || ref_force));
	wire sat_waiting = s_new || s_open;
	wire do_refresh  = !in_por && all_idle && !t_open && !s_open && ref_ok && !(sat_waiting && !ref_force);
	wire ref_hold    = ref_force && !in_por;	// forced refresh owed: start no new access


///////////////////////////////////////////////////////
// Change 5: STM32 access slots                      //
///////////////////////////////////////////////////////
// While the Saturn is using the cart with fast cycles and CS0 was active
// within SAT_IDLE_CLKS, an STM32 access may only start up to STM_HIT_WIN
// clocks after a Saturn cache hit, or right after a Saturn read (after_sat).
// memhub lets a request through after STM_MAX_WAIT so it can't starve.

	parameter  SAT_IDLE_CLKS = 10'd300;	// 3us
	localparam STM_HIT_WIN   = 3'd6;

	reg[2:0] stm_win;
	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			stm_win <= 0;
		end else if(sat_hit) begin
			stm_win <= STM_HIT_WIN;
		end else if(stm_win) begin
			stm_win <= stm_win-1'b1;
		end
	end

	localparam FAST_CYCLE_CLKS = 5'd16;
	reg[4:0] cs_len;
	reg      cs_last;
	reg      fast_bus;
	always @(negedge reset or posedge clk)
	begin
		if(reset==0) begin
			cs_len <= 0; cs_last <= 0; fast_bus <= 0;
		end else begin
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
	wire stm_slot = (stm_win!=0) || after_sat;
	assign stm_ok = !sat_mode || (stm_slot && ref_debt==0 && !ext_req);


///////////////////////////////////////////////////////
// Command choice                                    //
///////////////////////////////////////////////////////

	wire s_bank_ok = bank_free[s_new_ba] && rrd_cnt==0;
	wire t_bank_ok = bank_free[t_new_ba] && rrd_cnt==0;

	// 1. Saturn READ/WRITE
	wire go_s_cmd = s_open && s_rcd==0 && (s_isrd? (rd_cnt==0) : (wr_cnt==0));
	// 2. Saturn ACT (a forced refresh still goes first)
	wire go_s_act = s_new && !s_open && s_bank_ok && !ref_hold;
	// 3. STM32 READ/WRITE. A READ holds the data bus for 4 clocks, so it waits
	//    while a Saturn access is waiting, unless that access needs this bank.
	wire s_needs_t_bank = s_new && !s_open && (s_new_ba==t_ba);
	wire t_cmd_ready = t_open && t_rcd==0 && (t_isrd? (rd_cnt==0) : (wr_cnt==0));
	wire go_t_cmd = t_cmd_ready && (!t_isrd || !sat_waiting || s_needs_t_bank);
	// 4. refresh (do_refresh above)
	// 5. STM32 ACT: not while a Saturn request is waiting.
	// Not in the clock a Saturn access starts either (its request arrives one
	// clock later), and not while a Saturn refresh cycle (FC1) is waiting for
	// its refresh (ext_req in ref_wait): that refresh has to go out during the
	// FC1 cycle, before the Saturn's next access.
	// An owed refresh that has its quiet slot also goes before new STM32
	// accesses (as before Change 8), so it goes out at the start of the
	// Saturn's pause, not at a random moment later in it.
	wire ref_wait = ext_req || (ref_debt!=0 && quiet);
	wire go_t_act = t_new && !t_open && t_go && t_bank_ok && !sat_waiting && !sat_start && !ref_hold && !ref_wait;

	wire [2:0] pick = go_s_cmd   ? 3'd1 :
	                  go_s_act   ? 3'd2 :
	                  go_t_cmd   ? 3'd3 :
	                  do_refresh ? 3'd4 :
	                  go_t_act   ? 3'd5 : 3'd0;


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
	reg rd_issue, rd_port;		// a READ went out this clock, for port S (0) or T (1)
	reg[1:0] rd_dqm;			// clocks DQM stays low after a READ

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
			m_cmd <= 4'b1111;
			m_dqm <= 2'b11;
			m_ba <= 0;
			m_addr <= 13'h1fff;
			m_dout <= 0;
			m_oe <= 0;
			m_cke <= 0;
			ref_ack <= 0;
			s_open <= 0; t_open <= 0; s_taken <= 0; t_taken <= 0; t_seen <= 0;
			s_rcd <= 0; t_rcd <= 0; s_ba <= 0; t_ba <= 0; s_col <= 0; t_col <= 0; s_isrd <= 0; t_isrd <= 0;
			bank_cnt0 <= 0; bank_cnt1 <= 0; bank_cnt2 <= 0; bank_cnt3 <= 0;
			bank_open <= 0;
			rrd_cnt <= 0; rd_cnt <= 0; wr_cnt <= 0;
			s_ack <= 0; t_ack <= 0; t_acc <= 0;
			rd_issue <= 0; rd_port <= 0; rd_dqm <= 0;
			after_sat <= 0; sat_rd_cnt <= 0;
		end else begin
			// defaults for this clock
			m_cmd <= 4'b0111;		// NOP
			// DQM has a read latency of 2: it stays low from the READ for 4
			// clocks so the whole burst comes out (before: low until idle)
			m_dqm <= (rd_dqm!=0)? 2'b00 : 2'b11;
			if(rd_dqm) rd_dqm <= rd_dqm-1'b1;
			m_oe <= 0;
			ref_ack <= 0;
			s_ack <= 0; t_ack <= 0; t_acc <= 0;
			rd_issue <= 0;

			// counters
			if(bank_cnt0) bank_cnt0 <= bank_cnt0-1'b1;
			if(bank_cnt1) bank_cnt1 <= bank_cnt1-1'b1;
			if(bank_cnt2) bank_cnt2 <= bank_cnt2-1'b1;
			if(bank_cnt3) bank_cnt3 <= bank_cnt3-1'b1;
			if(rrd_cnt) rrd_cnt <= rrd_cnt-1'b1;
			if(rd_cnt)  rd_cnt  <= rd_cnt-1'b1;
			if(wr_cnt)  wr_cnt  <= wr_cnt-1'b1;
			if(s_rcd)   s_rcd   <= s_rcd-1'b1;
			if(t_rcd)   t_rcd   <= t_rcd-1'b1;

			// after_sat: one clock, when a Saturn READ's bank is idle again
			after_sat <= (sat_rd_cnt==3'd1);
			if(sat_rd_cnt) sat_rd_cnt <= sat_rd_cnt-1'b1;

			// a port that dropped its request may be served again
			if(s_req==2'b00 && !s_open) s_taken <= 0;
			if(t_req==2'b00 && !t_open) t_taken <= 0;
			t_seen <= t_new_raw && !(pick==3'd5);

			if(in_por) begin
				m_cmd <= 4'b1111;
				if(ref_count==500) begin
					m_cke <= 1;
				end else if(ref_count==40) begin
					m_cmd <= 4'b0010;
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
					end
				end
				3'd2: begin		// Saturn ACT
					m_cmd <= 4'b0011;
					m_ba <= s_new_ba;
					m_addr <= f_row(sdtype, s_addr);
					bank_open[s_new_ba] <= 1'b1;
					rrd_cnt <= N_RRD;
					s_open <= 1; s_taken <= 1;
					s_ba <= s_new_ba;
					s_col <= f_col(sdtype, s_addr);
					s_isrd <= s_new_rd;
					s_rcd <= s_new_rd? N_RCD : N_RCD_SW;
				end
				3'd3: begin		// STM32 READ/WRITE
					t_open <= 0;
					t_ack <= 1;
					m_ba <= t_ba;
					m_addr <= t_col;
					bank_open[t_ba] <= 1'b0;
					if(t_isrd) begin
						m_cmd <= 4'b0101;
						m_dqm <= 2'b00;
						set_bank_cnt(t_ba, N_RD_ACT);
						rd_cnt <= N_RD_RD; wr_cnt <= N_RD_WR;
						rd_issue <= 1; rd_port <= 1; rd_dqm <= 2'd3;
					end else begin
						m_cmd <= 4'b0100;
						m_dqm <= t_mask;
						m_dout <= t_din;
						m_oe <= 1;
						set_bank_cnt(t_ba, N_WR_ACT);
					end
				end
				3'd4: begin		// refresh
					m_cmd <= 4'b0001;
					ref_ack <= 1;
					bank_cnt0 <= N_REF_ACT; bank_cnt1 <= N_REF_ACT;
					bank_cnt2 <= N_REF_ACT; bank_cnt3 <= N_REF_ACT;
				end
				3'd5: begin		// STM32 ACT
					m_cmd <= 4'b0011;
					m_ba <= t_new_ba;
					m_addr <= t_new_row;
					bank_open[t_new_ba] <= 1'b1;
					rrd_cnt <= N_RRD;
					t_open <= 1; t_taken <= 1; t_acc <= 1;
					t_ba <= t_new_ba;
					t_col <= t_new_col;
					t_isrd <= t_new_rd;
					t_rcd <= N_RCD;
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
// the READ (same timing as before: CL=2 plus the registered command).
// READs are at least 4 clocks apart, so at most one burst is in this window.

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

	// s_dv/t_dv are registered: high 3-6 clocks after the READ was chosen
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

