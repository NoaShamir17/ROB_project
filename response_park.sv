// ============================================================================
// response_park.sv  (single-consumer, no alloc_uid)
// ----------------------------------------------------------------------------
// PROJECT: AXI Read Order Buffer (ROB)
// BLOCK  : response_park  —  a “parking lot” for complete read responses,
//          addressed by the *internal* UID (unique tag issued by remapper).
//
// SINGLE-CONSUMER, FLOW-GUIDED ALLOCATE
// -------------------------------------
// • In this variant there is exactly one consumer. We REMOVE alloc_uid.
// • Allocation (ALLOCATE) always targets the UID that is arriving now on
//   in_r.id (uid_idx). If the slot for uid_idx is occupied, we grant.
// • This greatly simplifies integration but means you cannot pick an *older*
//   parked UID while a *different* UID is currently arriving.
//
// ORDER OF CONTROL PLANES (for same UID, same cycle)
// --------------------------------------------------
// Priority: ENQUEUE → ALLOCATE → FREE
// (Allocator looks at v_q, i.e., “old” state; it will *not* see a same-cycle
// enqueue for the same uid_idx.)
// ============================================================================

module response_park #(
  // -------------------- UID SPACE (width derivation only) -------------------
  parameter int NUM_ROWS         = 4,
  parameter int NUM_COLS         = 4,

  // -------------------- PHYSICAL CAPACITY -----------------------------------
  parameter int MAX_OUTSTANDING  = NUM_ROWS*NUM_COLS,

  // Leave-one-free policy to ease global deadlock pressure
  parameter int HEADROOM         = 1,

  // -------------------- PAYLOAD SHAPE ---------------------------------------
  parameter int DATA_WIDTH       = 256, // AXI RDATA
  parameter int RESP_WIDTH       = 2,   // AXI RRESP
  parameter int ID_WIDTH         = 8,   // width of ORIGINAL ID (master-facing)
  parameter int TAG_WIDTH        = 8    // width of tag/user (meta)
)(
  input  logic clk,
  input  logic rst,

  // =================== ENQUEUE: producer → park (whole response) ============
  // Handshake: accept when (in_r.valid & in_r.ready)
  // Convention in this variant:
  //   * in_r.id    = internal UID (RID from fabric) — we slice lower UID_W bits
  //   * in_r.tagid = ORIGINAL RID to echo back to the master on out_r.id
  r_if.receiver in_r,

  // =================== ALLOCATE: single consumer, no UID arg ================
  // The *only* control is "please allocate now"; target UID = uid_idx.
  input  logic alloc_req,

  // =================== FREE: consumer → park ================================
  input  logic                                         free_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] id_to_release,

  // =================== OUTPUT: park → consumer (one-cycle COPY on grant) ====
  output logic    alloc_gnt,  // single-cycle pulse when we grant current uid_idx
  r_if.sender     out_r,      // valid/id/data/resp/tagid/last

  // =================== STATUS / DIAGNOSTICS =================================
  output logic    free_ack,   // single-cycle pulse acknowledging FREE
  output logic    full        // asserted when occupancy hits threshold
);

  // -------------------- DERIVED CONSTANTS -----------------------------------
  localparam int UID_W  = $clog2(NUM_ROWS) + $clog2(NUM_COLS);
  localparam int CAP_TH = (MAX_OUTSTANDING > HEADROOM) ? (MAX_OUTSTANDING - HEADROOM) : 0;

  // -------------------- UID INDEX FROM INCOMING RID -------------------------
  // If your UID encodes {row,col} in higher bits — adjust slicing here.
  wire [UID_W-1:0] uid_idx = in_r.id[UID_W-1:0];

  // -------------------- PER-UID STORAGE ARRAYS ------------------------------
  // v_q : slot occupied (parked response exists)
  // co_q: slot reserved by consumer (after ALLOCATE) until FREE
  logic                    v_q   [MAX_OUTSTANDING],  v_d   [MAX_OUTSTANDING];
  logic                    co_q  [MAX_OUTSTANDING],  co_d  [MAX_OUTSTANDING];

  logic [DATA_WIDTH-1:0]   data_q[MAX_OUTSTANDING],  data_d[MAX_OUTSTANDING];
  logic [RESP_WIDTH-1:0]   resp_q[MAX_OUTSTANDING],  resp_d[MAX_OUTSTANDING];

  // ORIGINAL ID (master-facing). In this variant, taken from in_r.tagid.
  logic [ID_WIDTH-1:0]     oid_q [MAX_OUTSTANDING],  oid_d [MAX_OUTSTANDING];

  // Optional metadata (kept equal to tagid here; repurpose as needed)
  logic [TAG_WIDTH-1:0]    tag_q [MAX_OUTSTANDING],  tag_d [MAX_OUTSTANDING];

  // -------------------- OCCUPANCY -------------------------------------------
  logic [$clog2(MAX_OUTSTANDING):0] used_cnt_q, used_cnt_d;

  wire full_int  = (used_cnt_q == CAP_TH);
  wire empty_int = (used_cnt_q == '0);

  // -------------------- ENQUEUE BACKPRESSURE --------------------------------
  // Accept only if not full and slot[uid_idx] is currently empty.
  assign in_r.ready = (~full_int) & (~v_q[uid_idx]);
  assign full       = full_int;

  // -------------------- OUTPUT DEFAULTS (data plane) ------------------------
  always_comb begin
    out_r.valid = 1'b0;
    out_r.id    = '0;      // overwritten on grant with ORIGINAL ID
    out_r.data  = '0;
    out_r.resp  = '0;
    out_r.last  = 1'b1;    // single-beat semantics here
    out_r.tagid = '0;      // optional meta echo
  end

  // -------------------- CONTROL CORE (bitwise-only) -------------------------
  logic                  alloc_gnt_d, free_ack_d;
  logic                  out_valid_d;
  logic [DATA_WIDTH-1:0] out_data_d;
  logic [RESP_WIDTH-1:0] out_resp_d;
  logic [ID_WIDTH-1:0]   out_oid_d;
  logic [TAG_WIDTH-1:0]  out_tag_d;

  always_comb begin
    // ===== defaults =========================================================
    alloc_gnt_d = 1'b0;
    free_ack_d  = 1'b0;

    out_valid_d = 1'b0;
    out_data_d  = '0;
    out_resp_d  = '0;
    out_oid_d   = '0;
    out_tag_d   = '0;

    used_cnt_d = used_cnt_q;

    // Hold state by default
    for (int i = 0; i < MAX_OUTSTANDING; i++) begin
      v_d   [i] = v_q   [i];
      co_d  [i] = co_q  [i];
      data_d[i] = data_q[i];
      resp_d[i] = resp_q[i];
      oid_d [i] = oid_q [i];
      tag_d [i] = tag_q [i];
    end

    // ===== 1) ENQUEUE =======================================================
    if (in_r.valid & in_r.ready) begin
      v_d   [uid_idx] = 1'b1;         // occupy slot
      co_d  [uid_idx] = 1'b0;         // not reserved yet

      data_d[uid_idx] = in_r.data;    // capture payload
      resp_d[uid_idx] = in_r.resp;

      // ORIGINAL ID capture policy (echo back to master)
      oid_d [uid_idx] = in_r.tagid;
      tag_d [uid_idx] = in_r.tagid;

      used_cnt_d = used_cnt_q + 1;
    end

    // ===== 2) ALLOCATE (single-consumer, targets *current* uid_idx) ========
    // Grant only if the *current* uid_idx slot is already occupied.
    // Note: We look at v_q (old state), so same-cycle new ENQUEUE won’t be
    // visible to allocator — by design.
    if (alloc_req & v_q[uid_idx]) begin
      alloc_gnt_d = 1'b1;

      out_valid_d = 1'b1;             // one-cycle copy on the output plane
      out_data_d  = data_q[uid_idx];
      out_resp_d  = resp_q[uid_idx];
      out_oid_d   = oid_q [uid_idx];  // ORIGINAL ID back to master
      out_tag_d   = tag_q [uid_idx];

      co_d[uid_idx] = 1'b1;           // mark as reserved until FREE
      // v_q stays 1 until FREE to block overwrite
    end

    // ===== 3) FREE ==========================================================
    if (free_req & (v_q[id_to_release] | co_q[id_to_release])) begin
      free_ack_d = 1'b1;

      if ((~empty_int) & v_q[id_to_release])
        used_cnt_d = used_cnt_q - 1;

      v_d [id_to_release] = 1'b0;
      co_d[id_to_release] = 1'b0;
      // payload left don't-care after clear
    end
  end

  // -------------------- DRIVE OUTPUT PLANE ----------------------------------
  always_comb begin
    out_r.valid = out_valid_d;
    out_r.data  = out_data_d;
    out_r.resp  = out_resp_d;
    out_r.id    = out_oid_d;  // ORIGINAL ID to master
    out_r.tagid = out_tag_d;  // optional meta
    // out_r.last tied to 1'b1 above
  end

  // -------------------- SEQ COMMIT ------------------------------------------
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
        tag_q[i] <= tag_d[i];
      end
      alloc_gnt <= alloc_gnt_d;  // 1-cycle pulse
      free_ack  <= free_ack_d;   // 1-cycle pulse
    end
  end

  // -------------------- DEBUG TIPS ------------------------------------------
  // Probe: used_cnt_q/full, v_q[uid], co_q[uid], in_r.valid/ready/id,
  //        alloc_req → alloc_gnt/out_r.valid/id/data, free_req/id_to_release.
  //
  // Common gotchas:
  // • Forgetting FREE → v_q never clears → full stays high.
  // • Expecting allocator to “see” same-cycle enqueue → it won’t (by design).
  //
  // Extending to multi-beat bursts:
  // • Per-UID tiny FIFO; v_q means “FIFO not empty”; FREE on last beat.
  //
  // Adding out_r.ready backpressure:
  // • Add a 1–2 entry elastic buffer; leave alloc/free planes unchanged.
  // ==========================================================================

endmodule
