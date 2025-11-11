// ============================================================================
// response_park.sv  —  Single-consumer, UID-only parking lot for responses
// ----------------------------------------------------------------------------
// ROLE
//   • Park exactly one whole response per internal UID (one slot per UID).
//   • ENQUEUE: accept (UID, data, resp) when capacity exists and the slot is empty.
//   • ALLOCATE: on alloc_req, present a 1-cycle COPY of the slot for the *current*
//                uid_idx (derived from in_r.id). Grant is ONE-SHOT per reservation.
//   • FREE: release a previously granted UID slot (one-shot ack).
//
// ONE-SHOT CONTROL PULSES
//   • alloc_gnt: asserted for exactly 1 cycle when (alloc_req & v_q[uid] & ~co_q[uid]).
//                co_q is set on grant, preventing repeated grants while alloc_req=1.
//   • free_ack : asserted for exactly 1 cycle when (free_req & (v_q|co_q)); we clear
//                v_q/co_q immediately, so the condition is false on the next cycle.
//
// LIMITS
//   • No original-ID handling (out_r.id does NOT carry ARID; we expose UID or zeros).
//   • No out_r.ready backpressure; consumer must latch on alloc_gnt/out_r.valid.
//   • Single consumer (no alloc_uid port).
// ============================================================================

module response_park #(
  // -------------------- UID SPACE (width derivation only) -------------------
  parameter int NUM_ROWS         = 4,
  parameter int NUM_COLS         = 4,

  // -------------------- PHYSICAL CAPACITY -----------------------------------
  parameter int MAX_OUTSTANDING  = NUM_ROWS*NUM_COLS,

  // Leave-one-free policy to reduce system-level deadlock/livelock pressure
  parameter int HEADROOM         = 1,

  // -------------------- PAYLOAD SHAPE ---------------------------------------
  parameter int DATA_WIDTH       = 256, // AXI RDATA
  parameter int RESP_WIDTH       = 2,   // AXI RRESP

  // Output-ID policy (no original ID here). Drive UID (zero-extended) or zeros.
  parameter int ID_WIDTH         = 8,
  parameter bit DRIVE_UID_ON_OUTPUT = 1'b0
)(
  input  logic clk,
  input  logic rst,

  // =================== ENQUEUE: producer → park =============================
  // in_r.id    = internal UID (RID from fabric) — lower UID_W bits → uid_idx
  // in_r.data  = response data (single-beat view)
  // in_r.resp  = response code
  // in_r.tagid = ignored
  r_if.receiver in_r,

  // =================== ALLOCATE: single consumer ============================
  input  logic alloc_req,  // request to present current uid_idx if available

  // =================== FREE: consumer → park ================================
  input  logic                                         free_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] id_to_release,

  // =================== OUTPUT: park → consumer ==============================
  // One-cycle COPY on grant; consumer must latch.
  output logic    alloc_gnt,  // ONE-SHOT pulse on grant of current uid_idx
  r_if.sender     out_r,

  // =================== STATUS ==============================================
  output logic    free_ack,   // ONE-SHOT pulse on successful free
  output logic    full        // asserted when occupancy reaches threshold
);

  // -------------------- DERIVED CONSTANTS -----------------------------------
  localparam int UID_W  = $clog2(NUM_ROWS) + $clog2(NUM_COLS);
  localparam int CAP_TH = (MAX_OUTSTANDING > HEADROOM) ? (MAX_OUTSTANDING - HEADROOM) : 0;

  // -------------------- UID INDEX FROM INCOMING RID -------------------------
  wire [UID_W-1:0] uid_idx = in_r.id[UID_W-1:0];

  // -------------------- PER-UID STORAGE -------------------------------------
  // v_q : slot occupied (parked response exists)
  // co_q: slot reserved by consumer (after ALLOCATE) until FREE
  logic                    v_q   [MAX_OUTSTANDING];
  logic                    co_q  [MAX_OUTSTANDING];
  logic [DATA_WIDTH-1:0]   data_q[MAX_OUTSTANDING];
  logic [RESP_WIDTH-1:0]   resp_q[MAX_OUTSTANDING];

  // -------------------- OCCUPANCY -------------------------------------------
  logic [$clog2(MAX_OUTSTANDING):0] used_cnt_q;

  wire full_int  = (used_cnt_q == CAP_TH);
  wire empty_int = (used_cnt_q == '0);

  // -------------------- ENQUEUE BACKPRESSURE --------------------------------
  // Accept only if not full and the slot is empty.
  assign in_r.ready = (~full_int) & (~v_q[uid_idx]);
  assign full       = full_int;

  // -------------------- OUTPUT DEFAULTS -------------------------------------
  // We treat each parked response as single-beat here; last=1.
  always_comb begin
    out_r.valid = 1'b0;
    out_r.data  = '0;
    out_r.resp  = '0;
    out_r.last  = 1'b1;
    out_r.tagid = '0;

    if (DRIVE_UID_ON_OUTPUT) begin
      if (ID_WIDTH >= UID_W) out_r.id = { {(ID_WIDTH-UID_W){1'b0}}, uid_idx };
      else                   out_r.id = uid_idx[ID_WIDTH-1:0];
    end else begin
      out_r.id = '0;
    end
  end

  // -------------------- ONE-SHOT PULSE INTENT (COMB) ------------------------
  // Grant only when OCCUPIED and NOT RESERVED → one-shot until FREE.
  wire can_grant = alloc_req & v_q[uid_idx] & (~co_q[uid_idx]);

  // Free-ack one-shot whenever either occupied or reserved exists for that UID.
  wire can_free  = free_req & (v_q[id_to_release] | co_q[id_to_release]);

  // Staged output data on grant (copy from arrays).
  logic [DATA_WIDTH-1:0] out_data_d;
  logic [RESP_WIDTH-1:0] out_resp_d;
  logic [ID_WIDTH-1:0]   out_id_d;

  // Select ID policy for the output on grant (UID or zeros).
  wire [ID_WIDTH-1:0] uid_ext =
      (ID_WIDTH >= UID_W) ? { {(ID_WIDTH-UID_W){1'b0}}, uid_idx } : uid_idx[ID_WIDTH-1:0];
  wire [ID_WIDTH-1:0] out_id_policy = (DRIVE_UID_ON_OUTPUT ? uid_ext : {ID_WIDTH{1'b0}});

  // Default staged outputs
  always_comb begin
    out_data_d = '0;
    out_resp_d = '0;
    out_id_d   = out_id_policy;

    if (can_grant) begin
      out_data_d = data_q[uid_idx];
      out_resp_d = resp_q[uid_idx];
      // out_id_d already computed by policy
    end
  end

  // -------------------- SEQUENTIAL COMMIT (UPDATE ONLY ACTIVE UID) ----------
  always_ff @(posedge clk) begin
    if (rst) begin
      used_cnt_q <= '0;
      for (int i = 0; i < MAX_OUTSTANDING; i++) begin
        v_q  [i] <= 1'b0;
        co_q [i] <= 1'b0;
        data_q[i] <= '0;
        resp_q[i] <= '0;
      end
      // Clear one-shot pulses
      alloc_gnt <= 1'b0;
      free_ack  <= 1'b0;

      // Clear outputs
      out_r.valid <= 1'b0;
      out_r.data  <= '0;
      out_r.resp  <= '0;
      // out_r.id/tagid/last are driven in comb defaults
    end else begin
      // ---------- default: drop pulses unless asserted this cycle -----------
      alloc_gnt <= 1'b0;
      free_ack  <= 1'b0;

      // ---------- ENQUEUE: accept new payload ------------------------------
      if (in_r.valid & in_r.ready) begin
        v_q  [uid_idx] <= 1'b1;
        co_q [uid_idx] <= 1'b0;
        data_q[uid_idx] <= in_r.data;
        resp_q[uid_idx] <= in_r.resp;
        used_cnt_q <= used_cnt_q + 1;
      end

      // ---------- ALLOCATE: ONE-SHOT grant, reserve until FREE -------------
      if (can_grant) begin
        alloc_gnt   <= 1'b1;      // one-cycle pulse
        out_r.valid <= 1'b1;      // one-cycle data valid
        out_r.data  <= out_data_d;
        out_r.resp  <= out_resp_d;
        // out_r.id/tagid driven by comb defaults/policy
        co_q[uid_idx] <= 1'b1;    // mark reserved; prevents re-grant
      end else begin
        out_r.valid <= 1'b0;      // default when no grant
      end

      // ---------- FREE: ONE-SHOT ack and clear slot -------------------------
      if (can_free) begin
        free_ack <= 1'b1;         // one-cycle pulse
        if ((~empty_int) & v_q[id_to_release]) begin
          used_cnt_q <= used_cnt_q - 1;
        end
        v_q [id_to_release] <= 1'b0;
        co_q[id_to_release] <= 1'b0;
      end
    end
  end

  // -------------------- INTEGRATION NOTES -----------------------------------
  // • alloc_gnt is guaranteed one-shot per reservation because co_q gates re-grant.
  // • free_ack is one-shot because v_q/co_q are cleared in the same cycle.
  // • If alloc_req is held high across cycles, you’ll get only the first grant;
  //   subsequent grants require FREE (which clears co_q).
  // • Make sure upstream honors in_r.ready to avoid overwrite.
  // ==========================================================================

endmodule
