// ============================================================================
// ar_id_ordering_unit
// ----------------------------------------------------------------------------
// PURPOSE:
//   * Accepts AXI read address requests (AR channel) from the master.
//   * Sends the ORIGINAL ARID to allocator_tag_map to request a unique_id.
//   * When allocator grants, forwards the request to the slave with
//     ARID replaced by the unique_id.
//   * Ensures AXI VALID/READY rules are respected in both directions.
//
// LIMITATIONS:
//   * Handles one request at a time (no internal queue).
//   * If multiple requests arrive back-to-back, master is stalled until
//     the current request is granted and sent forward.
//   * For more throughput, a small FIFO can be added later.
// ----------------------------------------------------------------------------

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
    output logic                 alloc_req,    // pulse: ask allocator for UID
    output logic [ID_WIDTH-1:0]  alloc_in_id,  // original ARID sent to allocator
    input  logic                 alloc_gnt,    // allocator says "UID ready"
    input  logic [UID_WIDTH-1:0] unique_id     // allocator’s granted unique ID
);

    // ---------------- FSM encoding ----------------
    typedef enum logic [1:0] {
        WAIT_REQ,   // wait for new request from master
        WAIT_GNT    // waiting for allocator grant + forwarding to slave
    } state_e;

    state_e state_q, state_d; // state register + next-state

    // ---------------- Latches for AR fields ----------------
    // When we handshake with master (s_ar.valid && s_ar.ready),
    // capture the request fields here so they stay stable
    // until allocator returns a UID.
    logic [ADDR_WIDTH-1:0] addr_q;
    logic [LEN_WIDTH-1:0]  len_q;
    logic [2:0]            size_q;
    logic [1:0]            burst_q;
    logic [ID_WIDTH-1:0]   id_q;

    // ========================================================================
    // SEQUENTIAL BLOCK
    // ========================================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state_q <= WAIT_REQ;
        end else begin
            state_q <= state_d;
        end

        // Capture AR fields on master handshake
        if (s_ar.valid && s_ar.ready) begin
            addr_q  <= s_ar.addr;
            len_q   <= s_ar.len;
            size_q  <= s_ar.size;
            burst_q <= s_ar.burst;
            id_q    <= s_ar.id;
        end
    end

    // ========================================================================
    // MASTER-SIDE HANDSHAKE (s_ar)
    // ========================================================================
    // Ready is asserted only in WAIT_REQ, meaning we can only take one request
    // at a time. Once we’ve accepted a request, we stop accepting until it is
    // fully granted and forwarded.
    assign s_ar.ready = (state_q == WAIT_REQ);

    // ========================================================================
    // ALLOCATOR HANDSHAKE
    // ========================================================================
    // alloc_req is asserted in WAIT_GNT to request a unique_id for the
    // captured ARID. The allocator returns alloc_gnt + unique_id asynchronously.
    assign alloc_req   = (state_q == WAIT_GNT);
    assign alloc_in_id = id_q;

    // ========================================================================
    // SLAVE-SIDE DRIVE (m_ar)
    // ========================================================================
    // Once allocator grants, we can drive a valid request to the slave.
    // ARID is replaced by unique_id (internal translation).
    assign m_ar.valid = (state_q == WAIT_GNT) && alloc_gnt;
    assign m_ar.id    = unique_id;   // translated ID
    assign m_ar.addr  = addr_q;      // original address
    assign m_ar.len   = len_q;       // burst length
    assign m_ar.size  = size_q;      // transfer size
    assign m_ar.burst = burst_q;     // burst type
    // NOTE: if you have more fields (ARQOS/ARCACHE/ARUSER), add them here too.

    // ========================================================================
    // NEXT-STATE LOGIC
    // ========================================================================
    always_comb begin
        state_d = state_q; // default: stay

        case (state_q)
            // ---------------- WAIT_REQ ----------------
            // Wait for a master request. When handshake occurs,
            // move to WAIT_GNT to request a UID.
            WAIT_REQ: begin
                if (s_ar.valid && s_ar.ready) begin
                    state_d = WAIT_GNT;
                end
            end

            // ---------------- WAIT_GNT ----------------
            // Wait for allocator to grant a UID.
            // Once granted and slave accepts (m_ar.valid && m_ar.ready),
            // return to WAIT_REQ to accept the next request.
            WAIT_GNT: begin
                if (alloc_gnt && m_ar.valid && m_ar.ready) begin
                    state_d = WAIT_REQ;
                end
            end
        endcase
    end

endmodule
