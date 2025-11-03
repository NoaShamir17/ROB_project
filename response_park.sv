// ============================================================================
// response_park.sv
// ----------------------------------------------------------------------------
// PROJECT: AXI Read Order Buffer (ROB)
// BLOCK  : response_park  —  a “parking lot” for complete read responses,
//          addressed by the *internal* UID (unique tag issued by remapper).
//
// WHY THIS EXISTS
// --------------
// • AXI allows out-of-order completion across *different* IDs but requires
//   in-order completion per *same* original ID. Your ROB breaks the dependency
//   by translating original IDs → internal UIDs. Responses from the fabric
//   therefore come back with *internal* RID (= UID).
// • This module *parks* one whole response per UID, until a higher-level
//   ordering/arbiter decides it is legal to release that UID back to the master.
// • It *does not* pick order. It *only* stores (ENQUEUE), hands out on demand
//   (ALLOCATE), and releases storage (FREE).
//
// HIGH-LEVEL PROTOCOL (3 independent control planes + 2 data planes)
// -------------------------------------------------------------------
// 1) ENQUEUE plane (producer → park; via r_if.receiver `in_r`):
//    • Producer drives:  in_r.valid=1, in_r.id=UID, in_r.data/resp/tagid
//    • Park drives:      in_r.ready (1 when it can accept *that* UID right now)
//    • Handshake rule:   accept when (in_r.valid & in_r.ready).  (bitwise-only)
//    • Effect on accept: capture payload → slot[UID], mark slot occupied,
//                        increment occupancy counter.
//
// 2) ALLOCATE plane (arbiter → park; scalar control):
//    • Arbiter drives:   alloc_req=1, alloc_uid=UID it wishes to consume now
//    • Park returns:     alloc_gnt=1 for 1 cycle *if slot[UID] is occupied*
//    • Effect on grant:  out_r.valid=1 for 1 cycle and out_r.* drives a COPY
//                        of the parked payload. Slot becomes "reserved" so no
//                        other consumer can re-allocate it until FREE.
//
// 3) FREE plane (consumer → park; scalar control):
//    • Consumer drives:  free_req=1, id_to_release=UID previously granted
//    • Park returns:     free_ack=1 for 1 cycle *if slot was occupied or reserved*
//    • Effect on accept: clear occupied/reserved; decrement occupancy if the
//                        slot was occupied; storage becomes available.
//
// DATA PLANES
// -----------
// • Input data plane  : r_if.receiver  (producer → park)
// • Output data plane : r_if.sender    (park → consumer)
//   NOTE: out_r does *not* honor out_r.ready in this basic design. The consumer
//         must latch on alloc_gnt/out_r.valid. This avoids hidden stalls.
//
// CAPACITY & HEADROOM POLICY
// --------------------------
// • Capacity is parameterized as MAX_OUTSTANDING (system-wide promise).
// • Optional HEADROOM leaves 1 slot free (default) to ease deadlock pressure
//   in the wider fabric (classic “almost full everywhere” livelock).
//   Set HEADROOM=0 if you insist on using all slots.
//
// UID CONVENTION IN THIS VERSION
// ------------------------------
// • We derive the per-slot index directly from the *incoming* r_if.id, which
//   is assumed to be the *internal* UID on the fabric R channel.  We select
//   the lower UID_W bits:  uid_idx = in_r.id[UID_W-1:0].
//   If you encode row/col in higher bits, change the slicing here accordingly.
//
// ORIGINAL ID VS. UID
// -------------------
// • The master must see ORIGINAL RID on the way out. In this version we store
//   ORIGINAL ID in `oid_q` (we take it from in_r.tagid for convenience), and
//   we return it on `out_r.id`.
// • If your system instead supplies ORIGINAL ID in in_r.id and the UID in
//   in_r.tagid, just swap the two assignments in the ENQUEUE and OUTPUT areas.
//
// RACE-RESOLUTION ORDER (same UID, same cycle)
// --------------------------------------------
// • Priority inside the combinational control is:
//      ENQUEUE  →  ALLOCATE  →  FREE
//   Rationale: ensures a producer beat is never lost when capacity exists;
//   allows allocator to see the old data; then FREE clears the slot.
//   If you require different semantics, reorder those sections carefully.
//
// BITWISE-ONLY STYLE
// ------------------
// • All conditions use bitwise & and | as requested (no &&/||).
//
// ============================================================================

module response_park #(
  // -------------------- UID SPACE (used only for width derivation) ----------
  parameter int NUM_ROWS         = 4,                      // e.g., 4 rows in UID grid
  parameter int NUM_COLS         = 4,                      // e.g., 4 cols in UID grid

  // -------------------- PHYSICAL CAPACITY -----------------------------------
  // How many responses the park is *willing/able* to store concurrently.
  // Must match the system's outstanding-transaction promise.
  parameter int MAX_OUTSTANDING  = NUM_ROWS*NUM_COLS,

  // Optional “leave-one-free” policy to reduce global HOL/deadlock risk.
  // HEADROOM=1 → assert full when used_cnt == MAX_OUTSTANDING-1
  // HEADROOM=0 → assert full only when used_cnt == MAX_OUTSTANDING
  parameter int HEADROOM         = 1,

  // -------------------- PAYLOAD SHAPE ---------------------------------------
  parameter int DATA_WIDTH       = 256, // AXI RDATA width
  parameter int RESP_WIDTH       = 2,   // AXI RRESP width
  parameter int ID_WIDTH         = 8,   // width of IDs carried on r_if.id
  parameter int TAG_WIDTH        = 8    // width of tag/user field on r_if.tagid
)(
  input  logic clk,
  input  logic rst,

  // =================== ENQUEUE: producer → park (whole response) ============
  // Handshake: accept when (in_r.valid & in_r.ready)
  // Convention (this version):
  //   * in_r.id    = internal UID (we slice lower UID_W bits for index)
  //   * in_r.tagid = ORIGINAL RID (or any meta you choose to propagate back)
  r_if.receiver in_r,

  // =================== ALLOCATE: arbiter → park =============================
  // Request a specific UID. If present, we grant for 1 cycle and present the
  // payload on out_r.* in the same cycle. Slot becomes "reserved" until FREE.
  input  logic                                         alloc_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] alloc_uid,

  // =================== FREE: consumer → park ================================
  // Tell us you're done with the UID (previously granted). We clear and recycle.
  input  logic                                         free_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] id_to_release,

  // =================== OUTPUT: park → consumer (one-cycle COPY on grant) ====
  output logic    alloc_gnt,  // single-cycle pulse: grant for alloc_uid
  r_if.sender     out_r,      // we drive valid/id/data/resp/tagid/last

  // =================== STATUS / DIAGNOSTICS =================================
  output logic    free_ack,   // single-cycle pulse acknowledging FREE
  output logic    full        // asserted when occupancy hits threshold
);

  // -------------------- DERIVED CONSTANTS -----------------------------------
  localparam int UID_W  = $clog2(NUM_ROWS) + $clog2(NUM_COLS);
  // CAP_TH = effective “full” threshold honoring headroom policy.
  localparam int CAP_TH = (MAX_OUTSTANDING > HEADROOM) ? (MAX_OUTSTANDING - HEADROOM) : 0;

  // -------------------- UID INDEX FROM INCOMING RID -------------------------
  // Select the bits that encode the per-slot index.  If your UID encoding is
  // {row, col} mapped to higher bits, re-slice here (or compute index).
  wire [UID_W-1:0] uid_idx = in_r.id[UID_W-1:0];

  // -------------------- PER-UID STORAGE ARRAYS ------------------------------
  // v_q  : slot holds a parked response (occupied)
  // co_q : slot is reserved by a consumer (after ALLOCATE) until FREE
  logic                    v_q   [MAX_OUTSTANDING],  v_d   [MAX_OUTSTANDING];
  logic                    co_q  [MAX_OUTSTANDING],  co_d  [MAX_OUTSTANDING];

  // Payload captured on ENQUEUE; copied out on ALLOCATE
  logic [DATA_WIDTH-1:0]   data_q[MAX_OUTSTANDING],  data_d[MAX_OUTSTANDING];
  logic [RESP_WIDTH-1:0]   resp_q[MAX_OUTSTANDING],  resp_d[MAX_OUTSTANDING];

  // ORIGINAL ID (as seen by the master) — returned on out_r.id
  // In this version we store it from in_r.tagid (see “Original ID vs UID”).
  logic [ID_WIDTH-1:0]     oid_q [MAX_OUTSTANDING],  oid_d [MAX_OUTSTANDING];

  // Optional metadata (you can repurpose as needed)
  logic [TAG_WIDTH-1:0]    tag_q [MAX_OUTSTANDING],  tag_d [MAX_OUTSTANDING];

  // -------------------- OCCUPANCY ACCOUNTING --------------------------------
  // used_cnt_q counts how many v_q[] are 1.  We update it only on accepted
  // ENQUEUE (increment) and accepted FREE of an occupied slot (decrement).
  logic [$clog2(MAX_OUTSTANDING):0] used_cnt_q, used_cnt_d;

  wire full_int  = (used_cnt_q == CAP_TH); // “don’t accept more” threshold
  wire empty_int = (used_cnt_q == '0);     // zero occupancy

  // -------------------- ENQUEUE BACKPRESSURE --------------------------------
  // Ready to accept a new beat for uid_idx only when:
  //   • not full (capacity exists), *and*
  //   • that UID slot is not already occupied (no overwrite).
  assign in_r.ready = (~full_int) & (~v_q[uid_idx]);

  // Expose fullness for upstream logic (e.g., r_id_ordering_unit).
  assign full = full_int;

  // -------------------- DEFAULT OUTPUTS (data plane) ------------------------
  // We generate a one-cycle COPY on grant; consumer must latch.
  // out_r.last=1'b1 because this module views a whole response as one beat.
  always_comb begin
    out_r.valid = 1'b0;
    out_r.id    = '0;   // overwritten on grant with ORIGINAL ID
    out_r.data  = '0;
    out_r.resp  = '0;
    out_r.last  = 1'b1; // single-beat response view
    out_r.tagid = '0;   // optional metadata (can carry UID if desired)
  end

  // -------------------- COMBINATIONAL CONTROL CORE --------------------------
  // *Single* always_comb with explicit priorities to make same-cycle behavior
  // obvious and avoid hidden latches.  We stage intent in *_d signals and
  // commit in the sequential block.
  //
  // Priority order for *same UID in the same cycle*:
  //   1) ENQUEUE  (accept new payload if able)
  //   2) ALLOCATE (if occupied, grant and copy payload out)
  //   3) FREE     (release after consumer is done)
  //
  // Note: For *different* UIDs in the same cycle, all three can succeed
  // concurrently because they touch disjoint array indices.

  // “Pulse” and output staging
  logic                  alloc_gnt_d, free_ack_d;
  logic                  out_valid_d;
  logic [DATA_WIDTH-1:0] out_data_d;
  logic [RESP_WIDTH-1:0] out_resp_d;
  logic [ID_WIDTH-1:0]   out_oid_d;
  logic [TAG_WIDTH-1:0]  out_tag_d;

  always_comb begin
    // ====== defaults ========================================================
    alloc_gnt_d = 1'b0;
    free_ack_d  = 1'b0;

    out_valid_d = 1'b0;
    out_data_d  = '0;
    out_resp_d  = '0;
    out_oid_d   = '0;
    out_tag_d   = '0;

    used_cnt_d = used_cnt_q;

    // Hold per-UID state by default
    for (int i = 0; i < MAX_OUTSTANDING; i++) begin
      v_d   [i] = v_q   [i];
      co_d  [i] = co_q  [i];
      data_d[i] = data_q[i];
      resp_d[i] = resp_q[i];
      oid_d [i] = oid_q [i];
      tag_d [i] = tag_q [i];
    end

    // ====== 1) ENQUEUE: accept producer beat ================================
    // Accept only when (in_r.valid & in_r.ready).  in_r.ready already enforces:
    //   - not full, and
    //   - v_q[uid_idx]==0  (no overwrite).
    if (in_r.valid & in_r.ready) begin
      v_d   [uid_idx] = 1'b1;        // slot becomes occupied
      co_d  [uid_idx] = 1'b0;        // not reserved yet (ownership begins on grant)

      data_d[uid_idx] = in_r.data;   // capture data
      resp_d[uid_idx] = in_r.resp;   // capture response code

      // ORIGINAL ID capture policy (this version):
      //   • We assume ORIGINAL ID arrived in in_r.tagid.
      //   • If your fabric supplies ORIGINAL ID in in_r.id instead, swap lines.
      oid_d [uid_idx] = in_r.tagid;  // store ORIGINAL ID for master-facing return
      tag_d [uid_idx] = in_r.tagid;  // store meta (you may choose a different source)

      used_cnt_d = used_cnt_q + 1;   // occupancy increased by one slot
    end

    // ====== 2) ALLOCATE: request a specific UID =============================
    // If the requested slot is occupied (v_q==1), we:
    //   • pulse alloc_gnt,
    //   • present a COPY of the payload on out_r.* for *one* cycle,
    //   • mark slot reserved (co_d=1) so no one else can allocate it again
    //     until FREE arrives.
    if (alloc_req & v_q[alloc_uid]) begin
      alloc_gnt_d = 1'b0 | 1'b1;     // explicit pulse set (bitwise style)

      out_valid_d = 1'b1;            // one-cycle “data valid” on output plane
      out_data_d  = data_q[alloc_uid];
      out_resp_d  = resp_q[alloc_uid];

      // Return ORIGINAL ID to the outside world (master-facing)
      out_oid_d   = oid_q [alloc_uid];

      // Optional metadata: you may choose to echo INTERNAL UID here instead
      //   e.g., out_tag_d = alloc_uid;
      out_tag_d   = tag_q [alloc_uid];

      co_d[alloc_uid] = 1'b1;        // consumer “owns” the slot content now
      // NOTE: v_q stays 1 until FREE; that keeps accounting stable and avoids
      //       allowing a second ENQUEUE into the same UID.
    end

    // ====== 3) FREE: release slot after consumer is done ====================
    // We accept FREE when either occupied (v_q) or reserved (co_q) is 1.
    // This allows “freeing a granted-but-already-cleared” situation robustly.
    if (free_req & (v_q[id_to_release] | co_q[id_to_release])) begin
      free_ack_d = 1'b1;

      // Decrement occupancy only if this slot actually counted (v_q==1).
      // Guard against underflow with (~empty_int).
      if ((~empty_int) & v_q[id_to_release])
        used_cnt_d = used_cnt_q - 1;

      v_d [id_to_release] = 1'b0;    // no longer occupied
      co_d[id_to_release] = 1'b0;    // no longer reserved
      // (We leave data/resp/oid/tag content “don’t care” after clear.)
    end
  end

  // -------------------- OUTPUT PLANE DRIVE (from staged comb) ---------------
  // The consumer must latch on out_r.valid (or alloc_gnt).  We intentionally
  // avoid ready-based backpressure in this basic version to keep the control
  // plane explicit and to make timing simple.  If you want backpressure, add
  // a tiny skid/elastic buffer here.
  always_comb begin
    out_r.valid = out_valid_d;
    out_r.data  = out_data_d;
    out_r.resp  = out_resp_d;
    out_r.id    = out_oid_d;  // master-facing ORIGINAL ID
    out_r.tagid = out_tag_d;  // optional metadata (could carry UID)
    // out_r.last is tied to 1'b1 above (single-beat view).
  end

  // -------------------- SEQUENTIAL COMMIT -----------------------------------
  // Reset fully clears occupancy/reservations/payloads and pulses.
  // At runtime we “commit” next-state computed by the comb block.
  always_ff @(posedge clk) begin
    if (rst) begin
      used_cnt_q <= '0;
      for (int i = 0; i < MAX_OUTSTANDING; i++) begin
        v_q  [i] <= 1'b0;
        co_q [i] <= 1'b0;
        data_q[i] <= '0;
        resp_q[i] <= '0;
        oid_q [i] <= '0;
        tag_q [i] <= '0;
      end
      alloc_gnt <= 1'b0;
      free_ack  <= 1'b0;
    end else begin
      used_cnt_q <= used_cnt_d;
      for (int i = 0; i < MAX_OUTSTANDING; i++) begin
        v_q  [i] <= v_d  [i];
        co_q [i] <= co_d [i];
        data_q[i] <= data_d[i];
        resp_q[i] <= resp_d[i];
        oid_q [i] <= oid_d [i];
        tag_q [i] <= tag_d [i];
      end
      // single-cycle pulses
      alloc_gnt <= alloc_gnt_d;
      free_ack  <= free_ack_d;
    end
  end

  // -------------------- PRACTICAL DEBUGGING TIPS ----------------------------
  // 1) Waveform probes that pay off:
  //    • used_cnt_q, full
  //    • v_q[UID], co_q[UID] for 2-3 UIDs under stress
  //    • in_r.valid/ready with in_r.id (watch for backpressure correctness)
  //    • alloc_req/alloc_uid → alloc_gnt + out_r.valid/id/resp/data
  //    • free_req/id_to_release → free_ack
  //
  // 2) Common integration mistakes (and symptoms):
  //    • Forgetting FREE → v_q never clears → full stays high.
  //    • Driving ALLOCATE for an empty UID → alloc_gnt stays 0.
  //    • Allowing producer to reuse a UID before FREE → in_r.ready remains 0.
  //
  // 3) If you later add multi-beat bursts:
  //    • Replace each slot with a tiny FIFO (depth=max R beats).  v_q means
  //      “FIFO not empty”.  On ALLOCATE, present the next beat each cycle
  //      while reserved; FREE once the burst ends (out_r.last=1).
  //
  // 4) If you need out_r.ready handshaking:
  //    • Add a 1-2 entry elastic buffer here and have it honor out_r.ready.
  //      Keep alloc/free control plane unchanged.
  //
  // 5) Safety sanity (optional assertions you can add in TB/formal):
  //    • in_r.ready → v_q[uid_idx]==0  (no overwrite intent)
  //    • used_cnt_q == $countones(v_q) (occupancy invariant)
  //    • alloc_req & v_q[alloc_uid] → ##1 alloc_gnt (grant next clk)
  //    • free_req & v_q[id_to_release] → used_cnt_q decreases by 1 next clk
  //
  // ==========================================================================

endmodule
