// ============================================================================
// ar_id_ordering_unit.sv
// ----------------------------------------------------------------------------
// ROLE
//   Single-entry AR (read-address) ordering stage between ar_in (receiver)
//   and ar_out (sender). Accepts one request from ar_in, requests a Unique ID
//   from the tag-map/allocator, and forwards the request to ar_out with the
//   UNIQUE ID replacing the original AXI ID. Also drives ar_out.tagid with
//   the same UNIQUE ID so downstream blocks can track it.
//
//   States (1 bit):
//     • ST_REQ = 0 (IDLE)  : accept a new AR when tag-map not full
//     • ST_GNT = 1 (ISSUE) : hold request; when alloc_gnt & ar_out handshake,
//                            return to IDLE
//
// INTERFACES (ar_if):
//   sender   : output valid,id,addr,len,size,burst,qos,tagid ; input ready
//   receiver : input  valid,id,addr,len,size,burst,qos,tagid ; output ready
//
// POLICY
//   • Bitwise only (& | ~), no && ||.
//   • All AR fields here: valid, id, addr, len, size, burst, qos, tagid, ready.
//   • unique_id (from tag-map) replaces original id on ar_out.id and ar_out.tagid.
//   • ar_in.ready is gated with ~tag_map_full to avoid over-admitting.
//
// ----------------------------------------------------------------------------

module ar_id_ordering_unit #(
  parameter ID_WIDTH   = 4,
  parameter ADDR_WIDTH = 32,
  parameter LEN_WIDTH  = 8,   // typical AXI LEN field width
  parameter USER_QOS_W = 4    // qos width (as provided by your ar_if)
)(
  input  logic clk,
  input  logic rst,

  // Upstream (previous stage / master)
  ar_if.receiver ar_in,

  // Downstream (next stage / slave)
  ar_if.sender   ar_out,

  // Tag-map / allocator handshake
  output logic                alloc_req,      // request a Unique ID for held req
  input  logic                alloc_gnt,      // allocator granted the Unique ID
  output logic [ID_WIDTH-1:0] alloc_in_id,    // original ARID to be mapped

  // Unique ID from tag-map (used when alloc_gnt=1)
  input  logic [ID_WIDTH-1:0] unique_id,

  // Backpressure: when tag-map is full, block new accepts
  input  logic                tag_map_full
);

  // ==========================================================================
  // STATE MACHINE: 1 bit (compact)
  // ==========================================================================
  localparam ST_REQ = 1'b0; // IDLE
  localparam ST_GNT = 1'b1; // ISSUE
  logic state_q, state_d;

  // ==========================================================================
  // LATCHED FIELDS (captured on ar_in handshake)
  // ==========================================================================
  logic [ID_WIDTH-1:0]   id_q;     // original AXI ID (for alloc_in_id visibility)
  logic [ADDR_WIDTH-1:0] addr_q;
  logic [LEN_WIDTH-1:0]  len_q;
  logic [2:0]            size_q;
  logic [1:0]            burst_q;
  logic [USER_QOS_W-1:0] qos_q;
  logic [ID_WIDTH-1:0]   tagid_q;  // captured from ar_in (not forwarded)

  // ==========================================================================
  // HANDSHAKES (bitwise only)
  // ==========================================================================
  wire hs_in  = ar_in.valid  & ar_in.ready;   // accept from upstream
  wire hs_out = ar_out.valid & ar_out.ready;  // present to downstream
  wire gnt    = alloc_gnt;

  // ==========================================================================
  // SEQ: STATE + CAPTURE
  // ==========================================================================
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      state_q <= ST_REQ;

      id_q     <= '0;
      addr_q   <= '0;
      len_q    <= '0;
      size_q   <= '0;
      burst_q  <= '0;
      qos_q    <= '0;
      tagid_q  <= '0;
    end
    else begin
      state_q <= state_d;

      // Capture one AR when upstream handshake completes
      if (hs_in) begin
        id_q     <= ar_in.id;      // original ID (only for allocator visibility)
        addr_q   <= ar_in.addr;
        len_q    <= ar_in.len;
        size_q   <= ar_in.size;
        burst_q  <= ar_in.burst;
        qos_q    <= ar_in.qos;
        tagid_q  <= ar_in.tagid;   // kept for debug/coverage if needed
      end
    end
  end

  // ==========================================================================
  // NEXT-STATE (purely combinational)
  // ==========================================================================
  always_comb begin
    state_d = state_q;

    if (state_q == ST_REQ) begin
      // IDLE: accept a new request when ar_in.valid and we assert ar_in.ready
      if (hs_in)
        state_d = ST_GNT;
    end
    else begin
      // ISSUE: wait for allocator grant and downstream acceptance
      if (gnt & hs_out)
        state_d = ST_REQ;
    end
  end

  // ==========================================================================
  // OUTPUTS / HANDSHAKES
  // ==========================================================================
  // Upstream READY: only IDLE and tag-map not full
  assign ar_in.ready = (state_q == ST_REQ) & ~tag_map_full;

  // Ask allocator while we are holding a request
  assign alloc_req   = (state_q == ST_GNT);
  assign alloc_in_id = id_q;                // tell allocator which original ID

  // Present to downstream only when granted
  assign ar_out.valid = (state_q == ST_GNT) & gnt;

  // Replace original ID with Unique ID; also drive tagid with the Unique ID
  assign ar_out.id     = unique_id;         // *** replaced ID ***
  assign ar_out.tagid  = unique_id;         // *** propagate tag for tracking ***

  // Forward remaining AR fields verbatim from the captured values
  assign ar_out.addr   = addr_q;
  assign ar_out.len    = len_q;
  assign ar_out.size   = size_q;
  assign ar_out.burst  = burst_q;
  assign ar_out.qos    = qos_q;

endmodule
