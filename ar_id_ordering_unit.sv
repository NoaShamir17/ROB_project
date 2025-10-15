// ============================================================================
// ar_id_ordering_unit
// ----------------------------------------------------------------------------
// ROLE IN SYSTEM:
//   Receives AR requests from the master, obtains a unique internal tag (UID)
//   for the original ARID via an external allocator, and forwards the request
//   to the slave with ARID replaced by that UID.
//
// PROTOCOL NOTES (AXI-AR):
//   • s_ar.valid/ready: handshake from master into this unit.
//   • m_ar.valid/ready: handshake from this unit into the slave.
//   • This unit never reorders; it accepts one request at a time.
//   • Back-pressure: if a request is in flight (awaiting UID and/or slave
//     acceptance), s_ar.ready is deasserted to stall the master.
//
// LATENCY / THROUGHPUT:
//   • Latency includes allocator response time and the next m_ar handshake.
//   • Throughput = one request per allocator grant + slave accept.
//   • For higher throughput, place a FIFO in front (master side) or behind
//     (toward the slave) and duplicate allocator requests per entry.
//
// RESET BEHAVIOR:
//   • On reset, the unit returns to idle, ready to accept a new request.
//   • No partially accepted request is retained across reset.
//
// INTEGRATION POINTS:
//   • Connect alloc_req/alloc_in_id to your tag allocator.
//   • When alloc_gnt is high, unique_id must hold a stable UID for the
//     captured request until the downstream handshake completes.
//   • Extend pass-through fields (e.g., ARQOS/ARCACHE/ARUSER) where noted.
//
// SAFETY / CORNER CASES COVERED:
//   • If the allocator grants but the slave is not ready, the request remains
//     pending (m_ar.valid stays asserted) until m_ar.ready.
//   • If the slave is ready but the allocator hasn’t granted yet, no request
//     is driven to the slave.
//   • The master is only acknowledged (ready) at idle, preventing overrun.
// ============================================================================

module ar_id_ordering_unit #(
    parameter int ID_WIDTH   = 4,   // width of master’s ARID
    parameter int UID_WIDTH  = 8,   // width of unique_id from allocator
    parameter int ADDR_WIDTH = 32,  // width of address field
    parameter int LEN_WIDTH  = 8    // width of burst length field
)(
    input  logic clk,
    input  logic rst,

    // ---------------- AXI Read Address channel from master ----------------
    ar_if.slave  s_ar,

    // ---------------- AXI Read Address channel to slave -------------------
    ar_if.master m_ar,

    // ---------------- Allocator interface ---------------------------------
    output logic                 alloc_req,    // request UID for captured ARID
    output logic [ID_WIDTH-1:0]  alloc_in_id,  // original ARID sent to allocator
    input  logic                 alloc_gnt,    // allocator indicates UID is ready
    input  logic [UID_WIDTH-1:0] unique_id     // the granted UID (stable when gnt)
);

    // =========================================================================
    // STATE MACHINE (one-hot)
    //   state_q[0] : IDLE  (WAIT_REQ) — accept a new AR request
    //   state_q[1] : ISSUE (WAIT_GNT) — waiting for allocator and forwarding
    // Rationale: one-hot keeps next-state equations shallow and easy to time.
    // =========================================================================
    localparam int ST_REQ = 0; // idle / waiting for master request
    localparam int ST_GNT = 1; // waiting for UID grant and downstream accept

    logic [1:0] state_q, state_d;

    // =========================================================================
    // PIPELINED HOLD OF AR FIELDS
    //   Captured at the moment we accept the master’s request, then held
    //   until we forward it with the translated ID.
    // =========================================================================
    logic [ADDR_WIDTH-1:0] addr_q;
    logic [LEN_WIDTH-1:0]  len_q;
    logic [2:0]            size_q;
    logic [1:0]            burst_q;
    logic [ID_WIDTH-1:0]   id_q;

    // =========================================================================
    // SEQUENTIAL: state & captures
    // =========================================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // IDLE on reset
            state_q <= 2'b01;
        end else begin
            state_q <= state_d;
        end

        // Capture AR fields exactly on master handshake
        if (s_ar.valid & s_ar.ready) begin
            addr_q  <= s_ar.addr;
            len_q   <= s_ar.len;
            size_q  <= s_ar.size;
            burst_q <= s_ar.burst;
            id_q    <= s_ar.id;
        end
    end

    // =========================================================================
    // MASTER SIDE: back-pressure policy
    //   • Ready only when idle → strictly one request in flight.
    //   • Keeps upstream simple and AXI-compliant.
    // =========================================================================
    assign s_ar.ready = state_q[ST_REQ];

    // =========================================================================
    // ALLOCATOR SIDE: request/ID pass-through
    //   • Assert request while in ISSUE state.
    //   • Original ID is whatever we captured from the master.
    // =========================================================================
    assign alloc_req   = state_q[ST_GNT];
    assign alloc_in_id = id_q;

    // =========================================================================
    // SLAVE SIDE: drive translated request
    //   • Valid once a UID is available and we are in ISSUE state.
    //   • ARID is replaced by unique_id; other fields are forwarded as-is.
    //   • If downstream stalls, valid remains asserted until ready.
    // =========================================================================
    assign m_ar.valid = (state_q[ST_GNT] & alloc_gnt);
    assign m_ar.id    = unique_id;
    assign m_ar.addr  = addr_q;
    assign m_ar.len   = len_q;
    assign m_ar.size  = size_q;
    assign m_ar.burst = burst_q;
    // NOTE: add ARQOS/ARCACHE/ARLOCK/ARPROT/ARUSER/etc. if present in ar_if.

    // =========================================================================
    // HANDSHAKE SHORT-NAMES (clarity only)
    // =========================================================================
    wire hs_master = s_ar.valid & s_ar.ready; // accepted from master
    wire hs_slave  = m_ar.valid & m_ar.ready; // delivered to slave
    wire gnt       = alloc_gnt;               // UID is available this cycle

    // =========================================================================
    // NEXT-STATE: single-cycle flow control
    //   IDLE  → ISSUE : on master handshake
    //   ISSUE → IDLE  : when both UID is granted and slave accepts
    //   Otherwise remain in current state.
    // =========================================================================
    always_comb begin
        state_d = 2'b00;

        // IDLE bit (ST_REQ)
        state_d[ST_REQ] =
              (state_q[ST_REQ] & ~hs_master)   // keep waiting for a request
            | (state_q[ST_GNT] &  (gnt & hs_slave)); // finished issuing → go idle

        // ISSUE bit (ST_GNT)
        state_d[ST_GNT] =
              (state_q[ST_REQ] &  hs_master)   // got a new request → start issuing
            | (state_q[ST_GNT] & ~(gnt & hs_slave)); // keep issuing until both done
    end

endmodule
