// ============================================================================
// ar_id_ordering_unit.sv
// ----------------------------------------------------------------------------
// ROLE
//   This block ensures AXI per-ID ordering at the AR (Read Address) stage.
//   It accepts read requests from the master side (through s_ar interface),
//   assigns / requests a Unique ID allocation, and forwards requests to the
//   slave side (through m_ar interface) only when ordering rules allow it.
//
//   It effectively acts as a "traffic cop" between master and slave:
//     - Accepts one request when ready
//     - Holds it until it is safe to issue downstream
//     - Waits for ID allocator to grant a unique tag
//     - Then releases it to the slave
//
//   Internally uses a single-bit FSM for handshaking control.
//   FSM States:
//     ST_REQ = 0 → IDLE  (waiting for a new request from master)
//     ST_GNT = 1 → ISSUE (holding request, waiting for allocator + downstream ready)
//
//   Interfaces:
//     • s_ar — slave-side view (from master → this block)
//     • m_ar — master-side view (from this block → slave)
//     • alloc_* — interface to ID allocator logic
//
//   This module forms part of the front path of the Read Order Buffer (ROB).
//
// ----------------------------------------------------------------------------
// DESIGN POLICY
//   • Fully bitwise logic (no logical operators like && or ||).
//   • All transfers obey VALID/READY handshake semantics.
//   • Resets clear internal storage safely.


module ar_id_ordering_unit #(
  parameter ID_WIDTH   = 4,
  parameter ADDR_WIDTH = 32
)(
  input  logic clk,
  input  logic rst,

  // ---------------- Interface toward master ----------------
  // s_ar : The AXI Read Address interface from the master
  ar_if.slave  s_ar,

  // ---------------- Interface toward slave -----------------
  // m_ar : The AXI Read Address interface to the slave
  ar_if.master m_ar,

  // ---------------- Allocation handshake -------------------
  output logic                alloc_req,    // Request new Unique ID
  input  logic                alloc_gnt,    // Allocator granted unique ID
  output logic [ID_WIDTH-1:0] alloc_in_id   // Original AXI ID to map
);

  // ==========================================================================
  // STATE MACHINE (1-bit binary encoding)
  // ==========================================================================
  localparam ST_REQ = 1'b0;  // IDLE: waiting for master request
  localparam ST_GNT = 1'b1;  // ISSUE: sending downstream request
  logic state_q, state_d;

  // ==========================================================================
  // INTERNAL STORAGE (latched request metadata)
  // ==========================================================================
  logic [ID_WIDTH-1:0]   id_q;
  logic [ADDR_WIDTH-1:0] addr_q;

  // ==========================================================================
  // HANDSHAKE SHORTCUTS
  // ==========================================================================
  wire hs_master = s_ar.valid & s_ar.ready; // Master handshake (input accepted)
  wire hs_slave  = m_ar.valid & m_ar.ready; // Slave handshake (output accepted)
  wire gnt       = alloc_gnt;               // Allocator grant

  // ==========================================================================
  // SEQUENTIAL LOGIC: STATE + DATA LATCHING
  // ==========================================================================
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      state_q <= ST_REQ;      // start IDLE
      id_q    <= '0;
      addr_q  <= '0;
    end
    else begin
      state_q <= state_d;

      // Capture the AR request from master when handshake occurs
      if (hs_master) begin
        id_q   <= s_ar.id;    // original AXI ID
        addr_q <= s_ar.addr;  // address to be read
      end
    end
  end

  // ==========================================================================
  // NEXT-STATE LOGIC
  // ==========================================================================
  always_comb begin
    state_d = state_q;

    if (state_q == ST_REQ) begin
      // --------------------------------------------------------------
      // STATE: IDLE
      // --------------------------------------------------------------
      // Wait until master sends a valid AR (s_ar.valid & s_ar.ready)
      // Once we accept it → move to ISSUE and request a Unique ID
      if (hs_master)
        state_d = ST_GNT;
    end
    else begin
      // --------------------------------------------------------------
      // STATE: ISSUE
      // --------------------------------------------------------------
      // While in ISSUE, we are waiting for:
      //   (1) Allocator grant (alloc_gnt)
      //   (2) Downstream slave handshake (m_ar.valid & m_ar.ready)
      //
      // Only after BOTH occur → we return to IDLE.
      if (gnt & hs_slave)
        state_d = ST_REQ;
    end
  end

  // ==========================================================================
  // OUTPUT / HANDSHAKE CONTROL
  // ==========================================================================

  // Master-side READY: we can accept a new request only when IDLE
  assign s_ar.ready = (state_q == ST_REQ);

  // Allocator interface: request a new UID while issuing
  assign alloc_req   = (state_q == ST_GNT);
  assign alloc_in_id = id_q; // send the original AXI ID to allocator

  // Slave-side VALID: assert only when allocator granted + still issuing
  assign m_ar.valid = (state_q == ST_GNT) & gnt;

  // Forward the AR fields to slave (the rest can be passed as needed)
  assign m_ar.id   = id_q;     // in final system → will be replaced by Unique ID
  assign m_ar.addr = addr_q;

  // Other fields (ARLEN, ARSIZE, ARBURST, etc.) can be extended as needed
  // within the ar_if interface; this skeleton focuses on ordering and handshakes.

endmodule
