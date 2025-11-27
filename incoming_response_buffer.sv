// ============================================================================
// incoming_response_buffer.sv
// ----------------------------------------------------------------------------
// 8-entry FIFO for AXI R beats.
//
// r_in  : R from AXI slave          (r_if.receiver)
// r_out : R to r_ordering_unit      (r_if.sender)
//
// Behavior:
//   - r_in.ready = 1 when FIFO not full
//   - r_out.valid = 1 when FIFO not empty
//   - push on  r_in.valid  & r_in.ready
//   - pop  on  r_out.valid & r_out.ready
//
// Control uses only bitwise &, |, ~ (no &&, ||, !).
// ============================================================================

module incoming_response_buffer #(
	parameter int ID_WIDTH    = 4,
	parameter int DATA_WIDTH  = 64,
	parameter int RESP_WIDTH  = 2,
	parameter int DEPTH       = 8
)(
	input  logic clk,
	input  logic rst,          // async active-high

	// R from AXI slave
	r_if.receiver r_in,

	// R toward r_ordering_unit
	r_if.sender   r_out        // *** FIXED: removed extra comma ***
);

	// -------------------------------------------------------------
	// R entry (one beat)
	// -------------------------------------------------------------
	typedef struct packed {
		logic [ID_WIDTH-1:0]    id;
		logic [DATA_WIDTH-1:0]  data;
		logic [RESP_WIDTH-1:0]  resp;
		logic                   last;
	} r_entry_t;

	localparam int PTR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
	localparam int CNT_W = $clog2(DEPTH + 1);

	// FIFO storage
	r_entry_t         mem     [0:DEPTH-1];
	logic [PTR_W-1:0] wr_ptr_q;
	logic [PTR_W-1:0] rd_ptr_q;
	logic [CNT_W-1:0] count_q;

	// Status flags
	logic empty;
	logic full;

	// -------------------------------------------------------------
	// Empty / full flags
	// -------------------------------------------------------------
	always_comb begin
		empty = (count_q == {CNT_W{1'b0}});
		full  = (count_q == CNT_W'(DEPTH));   // *** FIXED ***
	end

	// -------------------------------------------------------------
	// Handshake control
	// -------------------------------------------------------------
	logic push;
	logic pop;

	always_comb begin
		r_in.ready  = ~full;
		r_out.valid = ~empty;

		push = r_in.valid  & r_in.ready;
		pop  = r_out.valid & r_out.ready;
	end

	// -------------------------------------------------------------
	// Output data (combinational)
	// -------------------------------------------------------------
	always_comb begin
		if (empty) begin
			r_out.id   = {ID_WIDTH{1'b0}};
			r_out.data = {DATA_WIDTH{1'b0}};
			r_out.resp = {RESP_WIDTH{1'b0}};
			r_out.last = 1'b0;
		end
		else begin
			r_out.id   = mem[rd_ptr_q].id;
			r_out.data = mem[rd_ptr_q].data;
			r_out.resp = mem[rd_ptr_q].resp;
			r_out.last = mem[rd_ptr_q].last;
		end
	end

	// -------------------------------------------------------------
	// Sequential logic
	// -------------------------------------------------------------
	always_ff @(posedge clk or posedge rst) begin
		if (rst) begin
			wr_ptr_q <= '0;
			rd_ptr_q <= '0;
			count_q  <= '0;
		end
		else begin

			// -------------------------
			// PUSH
			// -------------------------
			if (push) begin
				mem[wr_ptr_q].id   <= r_in.id;
				mem[wr_ptr_q].data <= r_in.data;
				mem[wr_ptr_q].resp <= r_in.resp;
				mem[wr_ptr_q].last <= r_in.last;

				// pointer wrap
				if (wr_ptr_q == PTR_W'(DEPTH-1))
					wr_ptr_q <= '0;
				else
					wr_ptr_q <= wr_ptr_q + {{(PTR_W-1){1'b0}}, 1'b1};
			end

			// -------------------------
			// POP
			// -------------------------
			if (pop) begin
				if (rd_ptr_q == PTR_W'(DEPTH-1))
					rd_ptr_q <= '0;
				else
					rd_ptr_q <= rd_ptr_q + {{(PTR_W-1){1'b0}}, 1'b1};
			end

			// -------------------------
			// COUNT
			// -------------------------
			if (push & (~pop)) begin
				count_q <= count_q + {{(CNT_W-1){1'b0}}, 1'b1};
			end
			else if ((~push) & pop) begin
				count_q <= count_q - {{(CNT_W-1){1'b0}}, 1'b1};
			end
		end
	end

endmodule