// ============================================================================
// response_park — Parking lot for complete read responses (indexed by UID)
// ----------------------------------------------------------------------------
// ROLE
//   Holds one whole response per UID until higher-level ordering logic
//   explicitly requests that UID. This block does not choose order; it only
//   stores, hands out on demand, and releases on free.
//
// DESIGN INTENT / ARCH GUIDANCE
//   • One-slot-per-UID model simplifies hazard reasoning: no cross-UID mixing,
//     no implicit arbitration inside this module.
//   • Capacity policy keeps one headroom slot in the wider system
//     (MAX_REQ-1). This is useful in fabrics that can otherwise deadlock
//     when all buffers are “almost full” at the same time.
//   • Output protocol is a request/grant *pulse*. The consumer should capture
//     out_* on alloc_gnt and treat the slot as reserved until issuing FREE.
//   • Reservation (“checked out”) lets the consumer use out_* for multiple
//     cycles without losing ownership, even if another allocator request is
//     presented for the same UID.
//
// LATENCY / THROUGHPUT
//   • Enqueue: single-cycle acceptance when ready.
//   • Allocate: single-cycle grant when present.
//   • No internal multi-entry queueing per UID; throughput scales with how
//     many distinct UIDs are used concurrently.
//   • If burst/multi-beat per UID is needed, replace each slot with a small
//     FIFO (see “Extension hooks” notes below).
//
// BACKPRESSURE / FLOW CONTROL
//   • ENQUEUE side: in_ready deasserts if pool is full or target UID occupied.
//     This prevents silent overwrite and keeps accounting exact.
//   • ALLOCATE side: grant occurs only if the requested UID is present.
//     No “next-best” selection occurs here.
//   • FREE side: slot becomes reusable immediately; usage counter is updated
//     only if the slot actually held a parked response.
//
// CORNER-CASE BEHAVIOR
//   • Simultaneous enqueue/allocate/free to different UIDs: all operate
//     independently; order is defined by the sequence inside the comb block.
//   • Same-UID races in the *same* cycle are resolved by the stated comb
//     priority (ENQUEUE → ALLOCATE → FREE). Adjust if your fabric requires
//     a different precedence (e.g., FREE before ENQUEUE on same UID).
//   • Free is accepted when the slot is either occupied or reserved. This
//     allows returning a slot that was granted to the consumer in a prior
//     cycle even if the producer already cleared validity.
//   • Reset clears both occupancy and reservation. No response survives reset.
//
// ============================================================================

module response_park #(
  parameter int NUM_ROWS   = 4,
  parameter int NUM_COLS   = 4,
  // Total UIDs; capacity policy is MAX_REQ-1 (keeps headroom system-wide)
  parameter int MAX_REQ    = NUM_ROWS*NUM_COLS,

  // Payload of a “whole response”
  parameter int DATA_WIDTH = 256,
  parameter int RESP_WIDTH = 2,
  parameter int ID_WIDTH   = 4,
  parameter int TAG_WIDTH  = 4
)(
  input  logic clk,
  input  logic rst,

  // =================== ENQUEUE: producer → park ============================
  // Accepts a full response for a specific UID. Ready only when:
  //   (1) pool not full, and (2) that UID slot is currently empty.
  input  logic                                         in_valid,
  output logic                                         in_ready,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] in_uid,
  input  logic [DATA_WIDTH-1:0]                        in_data,
  input  logic [RESP_WIDTH-1:0]                        in_resp,
  input  logic [ID_WIDTH-1:0]                          in_orig_id,
  input  logic [TAG_WIDTH-1:0]                         in_tagid,

  // =================== ALLOCATE: arbiter → park ============================
  // Requests read-access to a specific UID. If present, a one-cycle grant
  // provides the payload and reserves the slot for the consumer.
  input  logic                                         alloc_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] alloc_uid,

  // =================== FREE: consumer → park ===============================
  // Releases the slot once the consumer is fully done with the data.
  input  logic                                         free_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] id_to_release,

  // =================== OUTPUT: granted payload =============================
  // Valid for one cycle when alloc_gnt is asserted; consumer should latch.
  output logic                                         alloc_gnt,
  output logic [DATA_WIDTH-1:0]                        out_data,
  output logic [RESP_WIDTH-1:0]                        out_resp,
  output logic [ID_WIDTH-1:0]                          out_orig_id,
  output logic [TAG_WIDTH-1:0]                         out_tagid,

  // Free acknowledgement (pulse)
  output logic                                         free_ack
);

  localparam int UID_W = $clog2(NUM_ROWS)+$clog2(NUM_COLS);

  // Per-UID storage and flags
  //   v_q  : slot holds a parked response
  //   co_q : slot is reserved by a consumer (after grant) until freed
  logic                    v_q   [MAX_REQ],  v_d   [MAX_REQ];
  logic                    co_q  [MAX_REQ],  co_d  [MAX_REQ];
  logic [DATA_WIDTH-1:0]   data_q[MAX_REQ],  data_d[MAX_REQ];
  logic [RESP_WIDTH-1:0]   resp_q[MAX_REQ],  resp_d[MAX_REQ];
  logic [ID_WIDTH-1:0]     oid_q [MAX_REQ],  oid_d [MAX_REQ];
  logic [TAG_WIDTH-1:0]    tag_q [MAX_REQ],  tag_d [MAX_REQ];

  // Pool usage counter (counts how many v_q are set)
  logic [$clog2(MAX_REQ):0] used_cnt_q, used_cnt_d;

  // Pool summaries (used for backpressure and defensive checks)
  wire full  = (used_cnt_q == MAX_REQ-1); // enforce one-slot headroom policy
  wire empty = (used_cnt_q == '0);

  // ENQUEUE readiness: advertise acceptance only when capacity exists
  // and the addressed UID slot is free.
  assign in_ready = (~full) & (~v_q[in_uid]);

  // --------------------------------------------------------------------------
  // COMBINATIONAL CONTROL
  // Priority inside this block:
  //   1) ENQUEUE (accept new data)
  //   2) ALLOCATE (grant + reserve)
  //   3) FREE (release reservation/occupancy)
  //
  // If your environment requires different same-cycle resolution on the
  // *same* UID (e.g., prefer FREE before ENQUEUE), reorder these sections.
  // --------------------------------------------------------------------------
  always_comb begin
    // Default: no handshakes, zeroed outputs, hold state
    alloc_gnt = 1'b0;
    free_ack  = 1'b0;

    out_data    = '0;
    out_resp    = '0;
    out_orig_id = '0;
    out_tagid   = '0;

    used_cnt_d = used_cnt_q;

    // Hold all slots by default
    for (int i = 0; i < MAX_REQ; i++) begin
      v_d   [i] = v_q   [i];
      co_d  [i] = co_q  [i];
      data_d[i] = data_q[i];
      resp_d[i] = resp_q[i];
      oid_d [i] = oid_q [i];
      tag_d [i] = tag_q [i];
    end

    // ===== 1) ENQUEUE: accept a new parked response for in_uid ==============
    // Acceptance requires in_valid and in_ready the same cycle.
    // On accept:
    //   • Mark slot occupied, clear reservation, write payload.
    //   • Increment usage count.
    if (in_valid & in_ready) begin
      v_d   [in_uid] = 1'b1;
      co_d  [in_uid] = 1'b0;
      data_d[in_uid] = in_data;
      resp_d[in_uid] = in_resp;
      oid_d [in_uid] = in_orig_id;
      tag_d [in_uid] = in_tagid;
      used_cnt_d     = used_cnt_q + 1;
    end

    // ===== 2) ALLOCATE: hand out the requested UID if present ===============
    // If the slot is occupied:
    //   • Pulse grant and drive out_* for that UID in the same cycle.
    //   • Mark slot reserved; occupancy is not cleared here. Consumer must free.
    if (alloc_req & v_q[alloc_uid]) begin
      alloc_gnt    = 1'b1;
      out_data     = data_q[alloc_uid];
      out_resp     = resp_q[alloc_uid];
      out_orig_id  = oid_q [alloc_uid];
      out_tagid    = tag_q [alloc_uid];
      co_d[alloc_uid] = 1'b1; // keep slot reserved across multiple cycles
    end

    // ===== 3) FREE: release a slot after consumer is done ===================
    // Free is accepted when the slot is either occupied or reserved.
    // Effects:
    //   • Pulse free_ack.
    //   • Decrement usage if the slot contributed to the count (occupied).
    //   • Clear both occupied and reserved flags.
    // Note:
    //   • Guard against underflow in case of mis-integration.
    if (free_req & (v_q[id_to_release] | co_q[id_to_release])) begin
      free_ack = 1'b1;

      if ((~empty) & v_q[id_to_release])
        used_cnt_d = used_cnt_q - 1;

      v_d [id_to_release] = 1'b0;
      co_d[id_to_release] = 1'b0;
    end
  end

  // --------------------------------------------------------------------------
  // SEQUENTIAL STATE
  //   • On reset: clear pool, reservations, payloads, and usage counter.
  //   • Otherwise: commit next-state computed above.
  // --------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      used_cnt_q <= '0;
      for (int i = 0; i < MAX_REQ; i++) begin
        v_q  [i] <= 1'b0;
        co_q [i] <= 1'b0;
        data_q[i] <= '0;
        resp_q[i] <= '0;
        oid_q [i] <= '0;
        tag_q [i] <= '0;
      end
    end else begin
      used_cnt_q <= used_cnt_d;
      for (int i = 0; i < MAX_REQ; i++) begin
        v_q  [i] <= v_d  [i];
        co_q [i] <= co_d [i];
        data_q[i] <= data_d[i];
        resp_q[i] <= resp_d[i];
        oid_q [i]  <= oid_d [i];
        tag_q [i]  <= tag_d [i];
      end
    end
  end


endmodule
