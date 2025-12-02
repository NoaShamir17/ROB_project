// ============================================================================
// outgoing_response_buffer.sv
// ----------------------------------------------------------------------------
// 8-entry FIFO for AXI R beats on the outgoing side.
//
// r_in  : R from r_ordering_unit  (r_if.receiver)
// r_out : R to AXI master          (r_if.sender)
//
// Behavior:
//   - When not full, r_in.ready = 1 and we accept beats on
//       r_in.valid & r_in.ready (push).
//   - When not empty, r_out.valid = 1 and we provide the oldest beat.
//       On r_out.valid & r_out.ready we pop that beat.
//
// Control uses only bitwise &, |, ~ (no &&, ||, !).
// ============================================================================

module outgoing_response_buffer #(
  	parameter int ID_WIDTH        = 32,
  	parameter int DATA_WIDTH      = 64,
  	parameter int RESP_WIDTH      = 2,   // AXI RRESP width (OKAY/SLVERR/DECERR...)
	parameter int DEPTH       = 8
)(
	input  logic clk,
	input  logic rst,          // async, active-high

	// R from r_ordering_unit
	r_if.receiver r_in,

	// R toward AXI master
	r_if.sender   r_out        // *** FIXED: removed extra comma ***
);

	// -------------------------------------------------------------
	// One R entry record (single beat)
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
	// Empty / full
	// -------------------------------------------------------------
	always_comb begin
		empty = (count_q == {CNT_W{1'b0}});
		full  = (count_q == CNT_W'(DEPTH));   // *** FIXED ***
	end

	// -------------------------------------------------------------
	// Handshake and fire signals
	// -------------------------------------------------------------
	logic push;
	logic pop;

	// r_in sees ready when FIFO not full
	always_comb begin
		r_in.ready = ~full;
	end

	// r_out sees valid when FIFO not empty
	always_comb begin
		r_out.valid = ~empty;
	end

	// Bitwise-only fire logic
	always_comb begin
		push = r_in.valid  & r_in.ready;
		pop  = r_out.valid & r_out.ready;
	end

	// -------------------------------------------------------------
	// Head entry → r_out (combinational)
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
	// Sequential: write, read, count, pointers
	// -------------------------------------------------------------
	always_ff @(posedge clk or posedge rst) begin
		if (rst) begin
			wr_ptr_q <= '0;
			rd_ptr_q <= '0;
			count_q  <= '0;
		end
		else begin

			// --------------------
			// WRITE (push)
			// --------------------
			if (push) begin
				mem[wr_ptr_q].id   <= r_in.id;
				mem[wr_ptr_q].data <= r_in.data;
				mem[wr_ptr_q].resp <= r_in.resp;
				mem[wr_ptr_q].last <= r_in.last;

				// Wrap-around
				if (wr_ptr_q == PTR_W'(DEPTH-1))
					wr_ptr_q <= '0;
				else
					wr_ptr_q <= wr_ptr_q + {{(PTR_W-1){1'b0}}, 1'b1};
			end

			// --------------------
			// READ (pop)
			// --------------------
			if (pop) begin
				if (rd_ptr_q == PTR_W'(DEPTH-1))
					rd_ptr_q <= '0;
				else
					rd_ptr_q <= rd_ptr_q + {{(PTR_W-1){1'b0}}, 1'b1};
			end

			// --------------------
			// COUNT update
			// --------------------
			if (push & (~pop)) begin
				count_q <= count_q + {{(CNT_W-1){1'b0}}, 1'b1};
			end
			else if ((~push) & pop) begin
				count_q <= count_q - {{(CNT_W-1){1'b0}}, 1'b1};
			end
			// push & pop → no change
		end
	end

endmodule