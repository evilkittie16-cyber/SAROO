
///////////////////////////////////////////////////////
// Module: Dual Master Bus system                    //
///////////////////////////////////////////////////////

module memhub(
	reset, clk,
	cs_a, rd_a, wr_a, mask_a, nwait_a, addr_a, wdata_a, rdata_a,
	cs_b, rd_b, wr_b, mask_b, nwait_b, addr_b, wdata_b, rdata_b,
	sd_cke, sd_cs, sd_ras, sd_cas, sd_we, sd_addr, sd_ba, sd_dqm, sd_data,
	ext_refresh, reg_sdram
);

///////////////////////////////////////////////////////
// Pins                                              //
///////////////////////////////////////////////////////

	// system
	input reset;
	input clk;

	// bus a
	input cs_a;
	input rd_a;
	input wr_a;
	input[1:0] mask_a;
	output nwait_a;
	input[25:0] addr_a;
	input[15:0] wdata_a;
	output[15:0] rdata_a;

	// bus b
	input cs_b;
	input rd_b;
	input wr_b;
	input[1:0] mask_b;
	output nwait_b;
	input[25:0] addr_b;
	input[15:0] wdata_b;
	output[15:0] rdata_b;

	// sdram
	output sd_cke;
	output sd_cs;
	output sd_ras;
	output sd_cas;
	output sd_we;
	output[12:0] sd_addr;
	output[ 1:0] sd_ba;
	output[ 1:0] sd_dqm;
	inout [15:0] sd_data;
	input ext_refresh;
	input [11:0] reg_sdram;

	// [SIM] declarations moved up for ModelSim (order only)
	wire[25:3] cache_addr_a;
	wire[63:0] cache_data1d_a;
	wire[ 3:0] cache_valid_a;
	wire cache_update_b;
	wire[25:3] cache_addr_b;
	wire[63:0] cache_data1d_b;
	wire[ 3:0] cache_valid_b;
	wire[15:0] cmd_dout;
	wire[15:0] sd_dout;
	wire sd_oe;

	// [ABUS_EXP] Change 8: the request manager that was here (one request at
	// a time, Saturn first) is gone. Each bus has its own request port on
	// tsdram, which overlaps them across SDRAM banks (see tsdram.v).
	wire cmd_ack_a, cmd_ack_b;
	wire data_valid_a, data_valid_b;
	wire t_acc;


///////////////////////////////////////////////////////
// bus A                                             //
///////////////////////////////////////////////////////

	wire[ 1:0] cmd_req_a;
	wire cache_invalid_a;
	wire cache_update_a;

	// [ABUS_EXP] Change 3: address, data and mask are captured by cachebus
	// the moment a request is seen (req_*), and used from then on, instead of
	// the live bus. With Sega's timing (external wait off) the Saturn may end
	// its cycle before the request reaches the SDRAM.
	wire[25:0] req_addr_a;
	wire[15:0] req_wdata_a;
	wire[ 1:0] req_mask_a;
	wire[25:0] req_addr_b;
	wire[15:0] req_wdata_b;
	wire[ 1:0] req_mask_b;
	wire hit_a, hit_b;		// [ABUS_EXP] Change 4: cache-hit pulses (hit_b unused)
	wire stm_ok;			// [ABUS_EXP] Change 5: from tsdram

	cachebus busa(reset, clk,
		cs_a, rd_a, wr_a, nwait_a, addr_a, wdata_a, rdata_a,
		cmd_req_a, cmd_ack_a, cache_invalid_a, cache_update_a,
		cache_addr_a, cache_data1d_a, cache_valid_a,
		mask_a, req_addr_a, req_wdata_a, req_mask_a, hit_a,
		data_valid_a, cmd_dout	// [ABUS_EXP] Change 6a: fill data straight to the bus
	);

	cacheblk _cha(reset, clk,
		cache_addr_a, cache_data1d_a, cache_valid_a, data_valid_a, cmd_dout,
		cache_update_a, req_mask_a, req_addr_a, req_wdata_a,
		cache_update_b, req_mask_b, req_addr_b, req_wdata_b,
		cache_invalid_a, req_addr_a
	);


///////////////////////////////////////////////////////
// bus B                                             //
///////////////////////////////////////////////////////

	wire[ 1:0] cmd_req_b;
	wire cache_invalid_b;

	cachebus busb(reset, clk,
		cs_b, rd_b, wr_b, nwait_b, addr_b, wdata_b, rdata_b,
		cmd_req_b, cmd_ack_b, cache_invalid_b, cache_update_b,
		cache_addr_b, cache_data1d_b, cache_valid_b,
		mask_b, req_addr_b, req_wdata_b, req_mask_b, hit_b,
		data_valid_b, cmd_dout
	);

	cacheblk _chb(reset, clk,
		cache_addr_b, cache_data1d_b, cache_valid_b, data_valid_b, cmd_dout,
		cache_update_b, req_mask_b, req_addr_b, req_wdata_b,
		cache_update_a, req_mask_a, req_addr_a, req_wdata_a,
		cache_invalid_b, req_addr_b
	);


///////////////////////////////////////////////////////
// [ABUS_EXP] Change 5: STM32 gate                    //
///////////////////////////////////////////////////////
// STM32 requests start only when tsdram says it is safe (stm_ok), or after
// waiting STM_MAX_WAIT clocks so they can't starve.

	parameter  STM_GATE     = 1;		// 0: Change 5 off (for comparison)
	localparam STM_MAX_WAIT = 8'd200;	// 2us
	reg[7:0] b_wait;
	wire b_force = (b_wait==STM_MAX_WAIT);
	wire b_go    = (STM_GATE==0) || stm_ok || b_force;

	always @(negedge reset or posedge clk)
	begin
		if(reset==0)
			b_wait <= 0;
		else if(cmd_req_b==0 || t_acc)
			b_wait <= 0;
		else if(!b_force)
			b_wait <= b_wait+1'b1;
	end


///////////////////////////////////////////////////////
// sdram controller                                  //
///////////////////////////////////////////////////////

	tsdram _tsd(
		reset, clk, reg_sdram,
		ext_refresh, cs_a, (rd_a | wr_a), hit_a, stm_ok,
		cmd_req_a, req_addr_a, req_wdata_a, req_mask_a, cmd_ack_a, data_valid_a,
		cmd_req_b, b_go, req_addr_b, req_wdata_b, req_mask_b, cmd_ack_b, data_valid_b, t_acc,
		cmd_dout,
		sd_cke, sd_cs, sd_ras, sd_cas, sd_we, sd_addr, sd_ba, sd_dqm, sd_data, sd_dout, sd_oe
	);

	assign sd_data = (sd_oe)? sd_dout : 16'hzzzz;


endmodule

