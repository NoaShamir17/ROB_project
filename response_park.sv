// ============================================================================
// response_park.sv — Single-consumer, UID-only parking lot for WHOLE BURSTS
// ----------------------------------------------------------------------------
// ROLE (unchanged policy, updated payload to whole-burst):
//   • Park exactly one whole response per internal UID (one slot per UID).
//   • ENQUEUE (from slave R beats): accept all beats; commit slot atomically
//     on LAST beat (one whole-burst image per UID).
//   • ALLOCATE: on alloc_req, present a 1-cycle COPY of the slot for the
//     current uid_idx (from RID). Grant is ONE-SHOT (co_q gate).
//   • FREE: clear a previously granted UID slot (one-shot ack).
//
// OUTPUTS:
//   • out_r: single-cycle flagging (valid/last=1), no backpressure (as before).
//   • Sideband “whole burst” outputs carry the actual burst fields matching FIFO:
//       out_id[7:0], out_resp[1:0], out_nbeats[NBEATS_W-1:0], out_payload[W-1:0].
//     Consumer latches them on alloc_gnt.
//
// CONTROL:
//   • Bitwise-only (~, &, |). No &&, ||, ! in comb paths.
//   • One-shot pulses via co_q gating.
// ============================================================================

module response_park #(
  // UID space
  parameter int NUM_ROWS         = 4,
  parameter int NUM_COLS         = 4,
  parameter int MAX_OUTSTANDING  = NUM_ROWS*NUM_COLS,
  parameter int HEADROOM         = 1,

  // AXI data shape
  parameter int DATA_WIDTH       = 256,  // AXI RDATA
  parameter int RESP_WIDTH       = 2,    // AXI RRESP
  parameter int MAX_BEATS        = 32,   // max beats per burst

  // Output-ID policy for out_r.id (sideband uses fixed 8-bit id)
  parameter int ID_WIDTH         = 8,
  parameter bit DRIVE_UID_ON_OUTPUT = 1'b0
)(
  input  logic clk,
  input  logic rst,

  // =================== ENQUEUE (slave → park) ===============================
  // Beat-level AXI R input (we aggregate to a whole burst per UID).
  r_if.receiver s_r,

  // =================== ALLOCATE / FREE =====================================
  input  logic alloc_req,  // request to present current uid_idx (from s_r.id)
  input  logic free_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] id_to_release,

  // =================== OUTPUT ===============================================
  // One-cycle grant pulse; consumer must latch sideband fields this cycle.
  output logic                        alloc_gnt,  // ONE-SHOT on grant
  r_if.sender                         out_r,      // one-cycle notify (last=1)

  // Sideband whole-burst copy (matches FIFO record fields)
  output logic [7:0]                  out_id,       // zero-extended UID
  output logic [RESP_WIDTH-1:0]       out_resp,     // burst-level resp (last-beat)
  output logic [$clog2(MAX_BEATS+1)-1:0] out_nbeats,// number of beats
  output logic [MAX_BEATS*DATA_WIDTH-1:0] out_payload, // flattened payload

  // =================== STATUS ==============================================
  output logic                        free_ack,   // ONE-SHOT on successful free
  output logic                        full
);

  // -------------------- DERIVED CONSTANTS -----------------------------------
  localparam int UID_W   = $clog2(NUM_ROWS) + $clog2(NUM_COLS);
  localparam int CAP_TH  = (MAX_OUTSTANDING > HEADROOM) ? (MAX_OUTSTANDING - HEADROOM) : 0;
  localparam int NBEATS_W= $clog2(MAX_BEATS + 1);

  // -------------------- UID INDEX FROM RID ----------------------------------
  wire [UID_W-1:0] uid_idx = s_r.id[UID_W-1:0];

  // -------------------- PER-UID STORAGE (WHOLE BURST) -----------------------
  logic                     v_q   [MAX_OUTSTANDING];   // slot occupied
  logic                     co_q  [MAX_OUTSTANDING];   // slot reserved (after grant)
  logic [7:0]               id_q  [MAX_OUTSTANDING];   // stored (zero-extended) UID
  logic [RESP_WIDTH-1:0]    resp_q[MAX_OUTSTANDING];   // last-beat resp for burst
  logic [NBEATS_W-1:0]      nbeats_q[MAX_OUTSTANDING]; // number of beats
  logic [MAX_BEATS*DATA_WIDTH-1:0] payload_q[MAX_OUTSTANDING]; // whole payload

  // -------------------- OCCUPANCY -------------------------------------------
  logic [$clog2(MAX_OUTSTANDING):0] used_cnt_q;

  wire full_int  = (used_cnt_q == CAP_TH);
  wire empty_int = (used_cnt_q == '0);

  // -------------------- ENQUEUE BACKPRESSURE (beat-level) -------------------
  // Accept all non-last beats. For LAST beat, stall if slot already full or park is full.
  wire slot_busy  = v_q[uid_idx];
  wire stall_last = s_r.last & (slot_busy | full_int);
  assign s_r.ready = ~stall_last;

  assign full = full_int;

  // -------------------- COLLECTOR (aggregate one burst per UID) -------------
  // Per-UID collector could be multi-burst; here we keep single in-flight burst.
  // For simplicity, we use a single collector (serial per UID arrival).
  logic                  collecting_q;
  logic [7:0]            rid8_q;
  logic [NBEATS_W-1:0]   beats_q;
  logic [RESP_WIDTH-1:0] last_resp_q;
  logic [MAX_BEATS*DATA_WIDTH-1:0] payload_acc_q;

  wire take_beat = s_r.valid & s_r.ready;
  wire [NBEATS_W-1:0] beats_inc = beats_q + {{(NBEATS_W-1){1'b0}},1'b1};

  always_ff @(posedge clk) begin
    if (rst) begin
      collecting_q  <= 1'b0;
      rid8_q        <= 8'b0;
      beats_q       <= {NBEATS_W{1'b0}};
      last_resp_q   <= {RESP_WIDTH{1'b0}};
      payload_acc_q <= {MAX_BEATS*DATA_WIDTH{1'b0}};

      used_cnt_q    <= '0;
      for (int i = 0; i < MAX_OUTSTANDING; i++) begin
        v_q[i]       <= 1'b0;
        co_q[i]      <= 1'b0;
        id_q[i]      <= 8'b0;
        resp_q[i]    <= {RESP_WIDTH{1'b0}};
        nbeats_q[i]  <= {NBEATS_W{1'b0}};
        payload_q[i] <= {MAX_BEATS*DATA_WIDTH{1'b0}};
      end

      alloc_gnt     <= 1'b0;
      free_ack      <= 1'b0;
      out_r.valid   <= 1'b0;
      out_r.data    <= '0;
      out_r.resp    <= '0;
      out_r.last    <= 1'b1;
      out_r.tagid   <= '0;
      out_id        <= 8'b0;
      out_resp      <= {RESP_WIDTH{1'b0}};
      out_nbeats    <= {NBEATS_W{1'b0}};
      out_payload   <= {MAX_BEATS*DATA_WIDTH{1'b0}};
    end
    else begin
      // drop one-shot pulses by default
      alloc_gnt   <= 1'b0;
      free_ack    <= 1'b0;
      out_r.valid <= 1'b0; // single-cycle notify when grant occurs

      // Accept beats and build the in-flight burst image
      if (take_beat) begin
        if (collecting_q == 1'b0) begin
          collecting_q <= 1'b1;
          // zero-extend UID to 8b for storage
          rid8_q       <= (UID_W >= 8) ? s_r.id[7:0] : { {(8-UID_W){1'b0}}, s_r.id[UID_W-1:0] };
          beats_q      <= {{(NBEATS_W-1){1'b0}},1'b1};
          payload_acc_q[0 +: DATA_WIDTH] <= s_r.data;
          last_resp_q  <= s_r.resp;
        end
        else begin
          payload_acc_q[beats_q*DATA_WIDTH +: DATA_WIDTH] <= s_r.data;
          beats_q      <= beats_inc;
          last_resp_q  <= s_r.resp;
        end

        // On LAST beat accepted → commit atomically to slot (UID)
        if (s_r.last) begin
          collecting_q <= 1'b0;

          // Write the UID slot
          id_q     [uid_idx] <= rid8_q;
          resp_q   [uid_idx] <= last_resp_q;
          nbeats_q [uid_idx] <= beats_inc; // total beats = previous + 1
          payload_q[uid_idx] <= payload_acc_q;

          // If previously empty, bump occupancy
          if (~v_q[uid_idx]) begin
            used_cnt_q <= used_cnt_q + 1;
          end
          v_q [uid_idx] <= 1'b1;
          co_q[uid_idx] <= 1'b0; // clear any stale reservation on rewrite
        end
      end

      // -------------------- ALLOCATE (ONE-SHOT) ------------------------------
      // Grant only when requested, occupied and not already reserved.
      // can_grant = alloc_req & v_q[uid] & ~co_q[uid]
      if (alloc_req & v_q[uid_idx] & (~co_q[uid_idx])) begin
        alloc_gnt   <= 1'b1;       // one-cycle grant pulse
        out_r.valid <= 1'b1;       // one-cycle notify (no ready)
        // Populate sideband copy (consumer must latch this cycle)
        out_id      <= id_q     [uid_idx];
        out_resp    <= resp_q   [uid_idx];
        out_nbeats  <= nbeats_q [uid_idx];
        out_payload <= payload_q[uid_idx];

        // Drive out_r single-beat view (notify semantics)
        out_r.data  <= {DATA_WIDTH{1'b0}}; // not used; sideband carries payload
        out_r.resp  <= resp_q[uid_idx];
        out_r.last  <= 1'b1;
        if (DRIVE_UID_ON_OUTPUT) begin
          if (ID_WIDTH >= UID_W) out_r.id <= { {(ID_WIDTH-UID_W){1'b0}}, uid_idx };
          else                   out_r.id <= uid_idx[ID_WIDTH-1:0];
        end else begin
          out_r.id <= {ID_WIDTH{1'b0}};
        end

        // Mark reserved to block re-grants until FREE
        co_q[uid_idx] <= 1'b1;
      end

      // -------------------- FREE (ONE-SHOT) ---------------------------------
      if (free_req & (v_q[id_to_release] | co_q[id_to_release])) begin
        free_ack <= 1'b1;
        if ((~empty_int) & v_q[id_to_release]) begin
          used_cnt_q <= used_cnt_q - 1;
        end
        v_q [id_to_release] <= 1'b0;
        co_q[id_to_release] <= 1'b0;
        // Optional: zero contents (not required functionally)
      end
    end
  end

  // -------------------- CONSTANT OUT_R FIELDS -------------------------------
  // Keep tagid zero; last is asserted only on notify cycle above.
  // out_r.id policy handled in sequential on grant; otherwise hold zeros.

endmodule
