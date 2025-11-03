// ============================================================================
// response_park.sv
// ----------------------------------------------------------------------------
// ROLE (WHAT THIS BLOCK DOES)
//   • This block is a "parking lot" for *complete* read responses, indexed by a
//     unique internal UID (row/col encoded externally).
//   • Each UID has exactly one slot here (one whole response per UID).
//   • The block does not perform any ordering decisions by itself. An upstream
//     "ordering / allocation unit" tells it which UID to hand out.
//   • The consumer requests a given UID via ALLOCATE. If present, we grant
//     (one-cycle pulse), drive a copy of the payload on an r_if sender, and mark
//     the slot as "reserved" until the consumer issues FREE.
//
// KEY MOTIVATIONS (WHY THIS DESIGN)
//   • One-slot-per-UID simplifies hazards: no internal arbitration is needed.
//   • A system-wide "headroom" policy can be enforced by not allowing occupancy
//     to reach the declared maximum. This avoids classic fabric deadlocks where
//     everything is "almost full" simultaneously.
//   • The interface is split into three concerns:
//       1) ENQUEUE (producer → park)     : push a complete response into a UID.
//       2) ALLOCATE (arbiter → park)     : request a specific UID's payload.
//       3) FREE (consumer  → park)       : release the slot after using it.
//   • ENQUEUE and OUTPUT payloads are carried via your r_if interface.
//
// ASSUMPTIONS / SCOPE
//   • "Whole response" is a single beat at this level (out_r.last = 1'b1).
//     If multi-beat is required in the future, replace each per-UID slot with
//     a small FIFO (see “EXTENSION HOOKS” notes in comments below).
//   • This module is synchronous (one clock domain). Reset clears all state.
//
// TIMING / LATENCY
//   • ENQUEUE acceptance: same cycle as in_r.valid & in_r.ready.
//   • ALLOCATE grant: same cycle as alloc_req & slot_present.
//   • FREE release: same cycle as free_req & (slot was occupied or reserved).
//
// FLOW CONTROL / BACKPRESSURE
//   • ENQUEUE backpressure: in_r.ready deasserts if capacity is at threshold
//     (i.e., "full") OR if the target UID is already occupied.
//   • ALLOCATE: no backpressure from out_r.ready — we issue a grant pulse and
//     a one-cycle valid beat. The consumer should latch immediately.
//   • FREE: slot becomes immediately available; occupancy counter updates only
//     if the slot was actually contributing to occupancy.
//
// CORNER-CASE BEHAVIOR
//   • Simultaneous ENQUEUE/ALLOCATE/FREE to *different* UIDs: all okay.
//   • Same-UID simultaneous ops: resolved by explicit priority in the comb
//     block: ENQUEUE → ALLOCATE → FREE (see "COMBINATIONAL CONTROL").
//     If a different precedence is desired (e.g., FREE before ENQUEUE),
//     re-order the three sections accordingly.
//   • FREE is accepted even if the slot is only reserved (i.e., after grant).
//   • Reset clears both occupancy ("v") and reservation ("co") flags.
// ============================================================================

module response_park #(
  // -------------------- SIZING / CAPACITY -----------------------------------
  // NUM_ROWS/NUM_COLS define the UID space externally (row/col encoded UID).
  // They are used only to derive UID width locally (no internal row/col logic).
  parameter int NUM_ROWS         = 4,                      // e.g., 4 rows
  parameter int NUM_COLS         = 4,                      // e.g., 4 cols
  // PHYSICAL CAPACITY: number of entries we are *willing and able* to store.
  // This must match the system promise of how many responses can be outstanding.
  parameter int MAX_OUTSTANDING  = NUM_ROWS*NUM_COLS,      // compile-time cap
  // Optional headroom. When HEADROOM=1 (default), we declare "full" at
  // MAX_OUTSTANDING-1 to leave one slot free system-wide (deadlock avoidance).
  // Set HEADROOM=0 if you want to utilize all MAX_OUTSTANDING slots.
  parameter int HEADROOM         = 1,

  // -------------------- PAYLOAD SHAPE ---------------------------------------
  parameter int DATA_WIDTH       = 256, // RDATA width (bus width)
  parameter int RESP_WIDTH       = 2,   // RRESP width (AXI)
  parameter int ID_WIDTH         = 4,   // original ID width (back to master)
  parameter int TAG_WIDTH        = 4    // optional tagging/user metadata
)(
  input  logic clk,
  input  logic rst,

  // =================== ENQUEUE PATH (producer → park) =======================
  // The producer provides a *whole* response beat with an associated UID.
  // We accept it when in_r.valid & in_r.ready. The UID is passed out-of-band.
  // NOTE: If UID is derivable from r_if.id in your system, you can refactor
  //       away in_uid and compute it here; we keep it explicit to avoid any
  //       UID-decode here.
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] in_uid,
  r_if.receiver                                        in_r,   // producer side

  // =================== ALLOCATE (arbiter → park) ============================
  // The arbiter asks for a specific UID. If present, we grant for 1 cycle and
  // drive a COPY of the payload on out_r. We also mark the slot as "reserved".
  input  logic                                         alloc_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] alloc_uid,

  // =================== FREE (consumer → park) ===============================
  // The consumer tells us when it has finished using the handed-out UID. We
  // clear "reserved" and "occupied" and restore capacity for that slot.
  input  logic                                         free_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] id_to_release,

  // =================== OUTPUT PATH (park → consumer) ========================
  // When we grant, we present the payload for EXACTLY one cycle with valid=1.
  // The consumer should latch. We also output a one-cycle alloc_gnt pulse.
  output logic                                         alloc_gnt, // 1-cycle pulse
  r_if.sender                                          out_r,      // consumer side

  // =================== STATUS / DIAGNOSTICS =================================
  // free_ack : one-cycle pulse acknowledging FREE
  // full     : asserted when occupancy reached the CAP_TH threshold
  output logic                                         free_ack,
  output logic                                         full
);

  // -------------------- DERIVED CONSTANTS -----------------------------------
  localparam int UID_W  = $clog2(NUM_ROWS) + $clog2(NUM_COLS);
  // CAP_TH is the effective threshold where we assert "full".
  // Example: MAX_OUTSTANDING=16, HEADROOM=1 → CAP_TH=15 (do not fill the last).
  localparam int CAP_TH = (MAX_OUTSTANDING > HEADROOM) ? (MAX_OUTSTANDING - HEADROOM) : 0;

  // -------------------- PER-UID STORAGE -------------------------------------
  // v_q  : "occupied"  — the slot currently holds a parked response (data+meta)
  // co_q : "reserved"  — the slot is reserved by a consumer (after grant)
  //
  // For each UID slot we store:
  //   • data_q : data payload      (RDATA)
  //   • resp_q : response code     (RRESP)
  //   • oid_q  : original AXI ID   (RID seen by the master)
  //   • tag_q  : optional tag/user (ARUSER/RUSER-style metadata)
  logic                    v_q   [MAX_OUTSTANDING],  v_d   [MAX_OUTSTANDING];
  logic                    co_q  [MAX_OUTSTANDING],  co_d  [MAX_OUTSTANDING];
  logic [DATA_WIDTH-1:0]   data_q[MAX_OUTSTANDING],  data_d[MAX_OUTSTANDING];
  logic [RESP_WIDTH-1:0]   resp_q[MAX_OUTSTANDING],  resp_d[MAX_OUTSTANDING];
  logic [ID_WIDTH-1:0]     oid_q [MAX_OUTSTANDING],  oid_d [MAX_OUTSTANDING];
  logic [TAG_WIDTH-1:0]    tag_q [MAX_OUTSTANDING],  tag_d [MAX_OUTSTANDING];

  // -------------------- OCCUPANCY ACCOUNTING --------------------------------
  // used_cnt_q : counts how many "occupied" slots exist (sum of v_q[])
  // full_int   : asserted when used_cnt_q == CAP_TH (honors HEADROOM policy)
  // empty_int  : asserted when used_cnt_q == 0
  logic [$clog2(MAX_OUTSTANDING):0] used_cnt_q, used_cnt_d;

  wire full_int  = (used_cnt_q == CAP_TH);
  wire empty_int = (used_cnt_q == '0);

  // -------------------- ENQUEUE BACKPRESSURE --------------------------------
  // We are ready to accept the producer's beat (in_r.valid & in_r.ready) only
  // if:
  //   (A) We have capacity (not full_int), AND
  //   (B) The destination UID slot is not already occupied (no overwrite).
  // Note: This is safe even if a concurrent FREE happens for the same UID;
  //       precedence is ENQUEUE → ALLOCATE → FREE (see comb block below).
  assign in_r.ready = (~full_int) & (~v_q[in_uid]);

  // Expose "full" for upstream backpressure logic (e.g., your ordering unit).
  assign full = full_int;

  // -------------------- DEFAULT OUTPUTS -------------------------------------
  // We drive r_if sender with clean defaults and assert valid only on grant.
  // "last" is 1'b1 here because we model a whole response as a single beat.
  // If you later add per-UID FIFOs for multi-beat, "last" should be driven
  // by that FIFO's end-of-burst condition.
  // IMPORTANT: Outgoing r_if does not provide READY-based flow control here —
  //            the consumer must latch on alloc_gnt/out_r.valid.
  //            (You can add a tiny elastic buffer if you need out_r.ready.)
  always_comb begin
    out_r.valid = 1'b0;
    out_r.id    = '0;
    out_r.data  = '0;
    out_r.resp  = '0;
    out_r.last  = 1'b1; // single-beat response view
    out_r.tagid = '0;
  end

  // -------------------- COMBINATIONAL CONTROL -------------------------------
  // PRIORITY (for potential same-UID races in a single cycle):
  //   1) ENQUEUE — accept new data if in_r.valid & in_r.ready
  //   2) ALLOCATE — if UID occupied, pulse alloc_gnt and present payload
  //   3) FREE — release the slot, clear reserved/occupied (with accounting)
  //
  // RATIONALE:
  //   • ENQUEUE first prevents losing a producer beat when capacity is present.
  //   • ALLOCATE second allows immediate grant if a slot is occupied.
  //   • FREE last guarantees that if all three hit same UID in same cycle, the
  //     allocator still sees the old data (handed out), and FREE clears after.
  //
  // You can reorder these sections if your fabric requires different semantics
  // on same-UID same-cycle events (e.g., FREE before ENQUEUE).
  logic            alloc_gnt_d, free_ack_d;
  logic [DATA_WIDTH-1:0] out_data_d;
  logic [RESP_WIDTH-1:0] out_resp_d;
  logic [ID_WIDTH-1:0]   out_oid_d;
  logic [TAG_WIDTH-1:0]  out_tag_d;
  logic                  out_valid_d;

  always_comb begin
    // ------------------ Default holds ---------------------------------------
    alloc_gnt_d = 1'b0;
    free_ack_d  = 1'b0;

    out_valid_d = 1'b0;
    out_data_d  = '0;
    out_resp_d  = '0;
    out_oid_d   = '0;
    out_tag_d   = '0;

    used_cnt_d = used_cnt_q;

    // Hold all per-UID states by default
    for (int i = 0; i < MAX_OUTSTANDING; i++) begin
      v_d   [i] = v_q   [i];
      co_d  [i] = co_q  [i];
      data_d[i] = data_q[i];
      resp_d[i] = resp_q[i];
      oid_d [i] = oid_q [i];
      tag_d [i] = tag_q [i];
    end

    // ------------------ 1) ENQUEUE ------------------------------------------
    // Accept a producer beat if in_r.valid & in_r.ready.
    // Effects on acceptance:
    //   • v_d[in_uid]  = 1
    //   • co_d[in_uid] = 0  (a fresh store is not "reserved" yet)
    //   • write data/resp/oid/tag
    //   • used_cnt_d++ (only if the slot was previously not occupied, which
    //     is guaranteed here by the in_r.ready gate above).
    if (in_r.valid & in_r.ready) begin
      v_d   [in_uid] = 1'b1;
      co_d  [in_uid] = 1'b0;
      data_d[in_uid] = in_r.data;
      resp_d[in_uid] = in_r.resp;
      oid_d [in_uid] = in_r.id;
      tag_d [in_uid] = in_r.tagid;
      used_cnt_d     = used_cnt_q + 1;
    end

    // ------------------ 2) ALLOCATE -----------------------------------------
    // If the requested UID is occupied, we grant and drive a 1-cycle valid
    // on the output r_if with a COPY of the stored payload.
    // We also mark the slot as "reserved" (consumer-owned) until FREE arrives.
    if (alloc_req & v_q[alloc_uid]) begin
      alloc_gnt_d = 1'b1;

      out_valid_d = 1'b1;
      out_data_d  = data_q[alloc_uid];
      out_resp_d  = resp_q[alloc_uid];
      out_oid_d   = oid_q [alloc_uid];
      out_tag_d   = tag_q [alloc_uid];

      // Reservation ensures the consumer "owns" the slot's content even if
      // someone else tries to allocate the same UID in subsequent cycles.
      // Occupancy (v_q) remains asserted until FREE; this keeps accounting
      // stable and prevents double-enqueue into the same UID.
      co_d[alloc_uid] = 1'b1;
    end

    // ------------------ 3) FREE ---------------------------------------------
    // FREE is accepted when the slot is either occupied (v_q=1) OR reserved
    // (co_q=1). This supports a consumer that frees after a prior ALLOCATE,
    // even if the producer had already cleared validity.
    // Effects on acceptance:
    //   • free_ack pulse (one cycle)
    //   • if the slot was occupied, used_cnt_d--
    //   • clear both v_d and co_d
    if (free_req & (v_q[id_to_release] | co_q[id_to_release])) begin
      free_ack_d = 1'b1;

      // Defensive guard against underflow:
      if ((~empty_int) & v_q[id_to_release])
        used_cnt_d = used_cnt_q - 1;

      v_d [id_to_release] = 1'b0;
      co_d[id_to_release] = 1'b0;
    end
  end

  // -------------------- DRIVE OUTPUT r_if FROM COMB RESULTS -----------------
  // Note: We generate out_r.valid in comb (1-cycle pulse on grant) and simply
  // forward the comb-selected payload. This assumes the consumer latches on
  // out_r.valid or alloc_gnt. If you need backpressure, insert a small
  // skid/elastic buffer here that honors out_r.ready (not present in this
  // design by intent).
  always_comb begin
    out_r.valid = out_valid_d;
    out_r.data  = out_data_d;
    out_r.resp  = out_resp_d;
    out_r.id    = out_oid_d;
    out_r.tagid = out_tag_d;
    // out_r.last is tied to 1'b1 earlier (single-beat response view).
  end

  // -------------------- SEQUENTIAL STATE ------------------------------------
  // Reset behavior:
  //   • Clear occupancy/reservations and payloads.
  //   • Clear counters and pulses.
  // Runtime:
  //   • Commit next-states from the combinational logic.
  //   • alloc_gnt and free_ack are issued as 1-cycle pulses.
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
      // pulses low on reset
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
      // register pulses (1-cycle each when asserted in comb)
      alloc_gnt <= alloc_gnt_d;
      free_ack  <= free_ack_d;
    end
  end

  // -------------------- EXTENSION HOOKS / DESIGN NOTES ----------------------
  // 1) MULTI-BEAT RESPONSES:
  //    • Replace each per-UID slot {data_q, resp_q, oid_q, tag_q} with a small
  //      FIFO (depth = max burst beats). "v_q" would indicate "FIFO not empty".
  //      "co_q" remains as a reservation bit (optionally merged with a FIFO
  //      state). out_r.last becomes FIFO's last-beat flag.
  //
  // 2) OUTPUT BACKPRESSURE:
  //    • If a consumer requires READY-based flow control, add a 1- or 2-entry
  //      elastic buffer at the output. You can feed it with the same
  //      out_valid_d/data and let it honor out_r.ready externally.
  //
  // 3) DIFFERENT SAME-CYCLE PRIORITIES:
  //    • If your fabric must prefer FREE over ENQUEUE/ALLOCATE on the same UID,
  //      reorder the three sections in the comb block. Ensure accounting updates
  //      remain consistent with your chosen precedence.
  //
  // 4) ERROR HANDLING / ASSERTS:
  //    • You may add assertions:
  //        - $onehot0({in_r.valid & in_r.ready, alloc_req & v_q[alloc_uid],
  //                    free_req & (v_q[id_to_release] | co_q[id_to_release])})
  //          per-UID if you want to prove mutual exclusion by policy.
  //        - Coverpoints on full, empty, and mixed patterns to gain confidence.
  //
  // 5) FORMAL PROPERTIES:
  //    • in_r.ready must deassert whenever the slot is occupied (no overwrite).
  //    • used_cnt_q equals the number of v_q bits set (invariant).
  //    • If alloc_req & v_q[alloc_uid], alloc_gnt pulses and out_r.valid=1.
  //    • After FREE of an occupied UID, used_cnt_q decreases by 1 on the next
  //      cycle (unless reset).
  //
  // 6) INTEGRATION TIP:
  //    • Connect your upstream "r_id_ordering_unit" as producer (in_r.*),
  //      and your "return arbiter" as consumer (uses alloc_req/alloc_uid to
  //      request a UID; latches out_r payload on alloc_gnt; calls FREE when done).
  //
  // 7) SECURITY / ISOLATION:
  //    • Because slots are accessed strictly by UID, cross-UID leakage is
  //      structurally prevented. If multiple masters share the UID namespace,
  //      ensure your higher-level mapping separates them or add master tags.
  //
  // 8) PARAMETER SANITY:
  //    • If HEADROOM >= MAX_OUTSTANDING, CAP_TH becomes 0, which forces full
  //      to assert immediately when used_cnt_q==0; avoid that pathological case.
  // ==========================================================================

endmodule
