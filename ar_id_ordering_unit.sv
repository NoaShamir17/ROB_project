// ============================================================================
// ar_id_ordering_unit.sv
// ----------------------------------------------------------------------------
// ROLE
//   Ordering stage between ar_in and ar_out.
//   • Captures one AR request.
//   • Requests Unique ID from allocator.
//   • When UID is granted, forwards request with:
//       ar_out.id = unique_id
//   • Holds output stable until downstream handshake.
//   • Blocks new input when tag-map is full, out-buffer is full, or entry is
//     occupied.
// ============================================================================
module ar_id_ordering_unit #(
  parameter int ID_WIDTH    = 4,
  parameter int ADDR_WIDTH  = 32,
  parameter int LEN_WIDTH   = 8,
  parameter int SIZE_WIDTH  = 3,
  parameter int BURST_WIDTH = 2,
  parameter int QOS_WIDTH   = 4
)(
  input  logic clk,
  input  logic rst,

  // Upstream (from IncomingRequestBuffer)
  ar_if.receiver ar_in,

  // Downstream (to OutgoingRequestBuffer / slave)
  ar_if.sender   ar_out,

  // Tag-map / allocator handshake
  output logic                  alloc_req,        // request Unique ID
  input  logic                  alloc_gnt,        // allocator granted Unique ID
  output logic [ID_WIDTH-1:0]   alloc_in_id,      // original ID

  // Unique ID from allocator (valid when alloc_gnt == 1)
  input  logic [ID_WIDTH-1:0]   unique_id,

  // Backpressure from tag-map (no more UID slots)
  input  logic                  tag_map_full
);

  // ========================================================================
  // STATE + FLAGS
  // ========================================================================
  localparam ST_REQ = 1'b0;   // idle, no request captured
  localparam ST_GNT = 1'b1;   // have request, waiting UID and/or sending

  logic state_q, state_d;

  // Whether we already latched a Unique ID for the captured request
  logic                 uid_valid_q;
  logic                 uid_valid_d;
  logic [ID_WIDTH-1:0]  uid_q;      // latched Unique ID
  logic [ID_WIDTH-1:0]  uid_d;

  // ========================================================================
  // LATCHED AR FIELDS (from ar_in)
  // ========================================================================
  logic [ID_WIDTH-1:0]    id_q;
  logic [ADDR_WIDTH-1:0]  addr_q;
  logic [LEN_WIDTH-1:0]   len_q;
  logic [SIZE_WIDTH-1:0]  size_q;
  logic [BURST_WIDTH-1:0] burst_q;
  logic [QOS_WIDTH-1:0]   qos_q;

  // ========================================================================
  // HANDSHAKES (pure wires)
  // ========================================================================
  wire hs_in  = ar_in.valid  & ar_in.ready;
  wire hs_out = ar_out.valid & ar_out.ready;
  wire gnt    = alloc_gnt;

  // ========================================================================
  // OUTPUTS / HANDSHAKES (pure assigns)
  // ========================================================================

  // Upstream ready: only when idle, tag-map not full, and outgoing buffer not full
  assign ar_in.ready =
      (state_q == ST_REQ)
    & (~tag_map_full)
    & (ar_out.ready);

  // Allocator request / original ID
  assign alloc_req   = (state_q == ST_GNT) & (~uid_valid_q);
  assign alloc_in_id = id_q;

  // Downstream valid: we have a captured request AND a valid UID
  assign ar_out.valid = (state_q == ST_GNT) & uid_valid_q;

  // AR payload toward downstream (combinational from latched regs)
  assign ar_out.id    = uid_q;    // internal unique ID
  assign ar_out.addr  = addr_q;
  assign ar_out.len   = len_q;
  assign ar_out.size  = size_q;
  assign ar_out.burst = burst_q;
  assign ar_out.qos   = qos_q;

  // ========================================================================
  // NEXT-STATE + UID LATCH LOGIC
  // ========================================================================
  always_comb begin
    // default: hold values
    state_d     = state_q;
    uid_valid_d = uid_valid_q;
    uid_d       = uid_q;

    // ST_REQ: idle, no captured request
    if (state_q == ST_REQ) begin
      uid_valid_d = 1'b0;

      // New request captured
      if (hs_in) begin
        state_d = ST_GNT;
      end
    end
    else begin
      // ST_GNT: we have a captured request

      // If we do not yet have UID and allocator grants one, latch it
      if ((uid_valid_q == 1'b0) & gnt) begin
        uid_d       = unique_id;
        uid_valid_d = 1'b1;
      end

      // Once UID is valid and downstream accepts, return to idle
      if (uid_valid_q & hs_out) begin
        state_d     = ST_REQ;
        uid_valid_d = 1'b0;
      end
    end
  end

  // ========================================================================
  // SEQ: STATE + REGISTERS
  // ========================================================================
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      state_q     <= ST_REQ;
      uid_valid_q <= 1'b0;
      uid_q       <= '0;

      id_q        <= '0;
      addr_q      <= '0;
      len_q       <= '0;
      size_q      <= '0;
      burst_q     <= '0;
      qos_q       <= '0;
    end
    else begin
      state_q     <= state_d;
      uid_valid_q <= uid_valid_d;
      uid_q       <= uid_d;

      // Capture AR fields only when we handshake input
      if (hs_in) begin
        id_q     <= ar_in.id;
        addr_q   <= ar_in.addr;
        len_q    <= ar_in.len;
        size_q   <= ar_in.size;
        burst_q  <= ar_in.burst;
        qos_q    <= ar_in.qos;
      end
    end
  end

endmodule
