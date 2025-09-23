// ============================================================================
// r_ordering_unit  (Read channel per-ID in-order guard + free handshake)
// ---------------------------------------------------------------------------
// PURPOSE (big-picture):
//   * AXI requires: For a given ORIGINAL ID (RID at the master), responses on
//     the R channel must be returned in the SAME ORDER the requests were issued.
//     Different IDs may interleave arbitrarily.
//   * Internally, this design uses a unique tag {row,col} per transaction:
//       - 'row' = the bucket currently representing ONE ORIGINAL ID (while active)
//       - 'col' = a per-row slot assigned by the allocator for each outstanding
//                 request of that ORIGINAL ID.
//   * This block accepts ONE beat at a time from the slave-side R channel, but
//     ONLY if that beat's {row,col} matches the HEAD transaction of that row,
//     i.e. the oldest issued request that has not yet retired. That guarantees
//     per-ID in-order retirement.
//   * When the LAST beat of the head transaction is TAKEN by the master,
//     we immediately (a) POP the head from the per-row order FIFO and
//                  (b) enqueue a FREE {row,col} into a small "free outbox"
//                      that holds frees until the allocator acks them.
//
// HOW THIS VARIANT INTEGRATES WITH allocator_tag_map.sv:
//   * AR path (allocation / issue order):
//       - We use the allocator's `alloc_gnt` + `unique_id={row,col}` as the
//         single source of truth for "a request was just ISSUED". On each grant
//         we PUSH `col` into that row's tiny issue-order FIFO.
//   * R path (restoring ORIGINAL RID):
//       - For any accepted R beat, `restored_id` (from allocator/tag_map) is the
//         ORIGINAL RID corresponding to the internal {row,col} of THAT beat.
//         We capture it into a 1-beat output skid and present it to the master.
//   * Free path (recycling slots):
//       - We present frees as packed {row,col} on `free_unique_id` with `free_req`
//         held HIGH (level-valid) until the allocator asserts `free_ack`.
//         A tiny free outbox guarantees no free is lost if the allocator stalls.
//
// DESIGN CHOICES:
//   * Minimal buffering on R input: non-head-of-row beats are NOT accepted
//     (ready=0) -> simplest, correct, small area. The upstream/fabric can feed
//     legal beats first to avoid head-of-line stalls.
//   * Single-beat output skid: clean handling of downstream backpressure.
//   * POP order FIFO immediately on LAST beat being accepted by the master;
//     do NOT wait for allocator ack (keeps R ordering progress independent).
// ============================================================================

module r_ordering_unit #(
    // ---------------- Parameterization (geometry & widths) -------------------
    parameter int NUM_ROWS           = 4,   // # of ORIGINAL IDs concurrently active
    parameter int NUM_COLS           = 4,   // # of concurrent requests per SAME ORIGINAL ID

    // AXI R-channel field widths (must match your r_if)
    parameter int ID_WIDTH           = 4,   // ORIGINAL RID width seen by master
    parameter int DATA_WIDTH         = 64,
    parameter int RESP_WIDTH         = 2,
    parameter int TAG_WIDTH          = 4,

    // Depth of the small "free outbox" queue (>=1; 2-4 is usually enough)
    parameter int FREE_OUTBOX_DEPTH  = 4
)(
    input  logic clk,
    input  logic rst,

    // ------------------------ Slave-side R (input) ---------------------------
    // INTERNAL RID = packed {row,col}. We will ONLY accept head-of-row beats.
    r_if.receiver r_in_if,

    // ------------------------ Master-side R (output) -------------------------
    // ORIGINAL RID + payload driven toward the master (via 1-beat skid).
    r_if.sender   r_out_if,

    // ------------------------ Allocator/tag_map integration ------------------
    // AR path (issue events): ONE pulse per successful allocation/issue.
    input  logic                                            alloc_gnt,   // 1-cycle pulse on allocation grant
    input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0]    unique_id,   // packed {row,col} for that AR issue

    // Free path (retirement): we push frees and hold until ack (no drop).
    output logic                                            free_req,
    output logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0]    free_unique_id, // packed {row,col} being freed
    input  logic                                            free_ack,

    // Restored ORIGINAL RID for the INTERNAL {row,col} of the *accepted* beat.
    // Your top computes this using tag_map and drives it combinationally here.
    input  logic [ID_WIDTH-1:0]                             restored_id
);

    // ---------------- Local width aliases (for readability) ------------------
    localparam int ROW_W = $clog2(NUM_ROWS);
    localparam int COL_W = $clog2(NUM_COLS);
    localparam int CNT_W = $clog2(NUM_COLS+1);

    // Outbox pointer width; depth=1 => force 1 bit to avoid $clog2(1)=0
    localparam int FQ_W  = (FREE_OUTBOX_DEPTH <= 1) ? 1 : $clog2(FREE_OUTBOX_DEPTH);

    // =========================================================================
    // STATE: Per-row "issue order" FIFO
    // ---------------------------------
    // We store ONLY 'col' values, one per issued request on that row, in order.
    // For row r:
    //   per_row_col_queue[r][i]  : the 'col' value stored at FIFO slot i
    //   fifo_head_idx_q[r]       : POP pointer (oldest col to enforce NOW)
    //   fifo_tail_idx_q[r]       : PUSH pointer (where next issued col goes)
    //   fifo_count_q[r]          : # of outstanding transactions for row r
    //
    // Invariants:
    //   0 <= fifo_count_q[r] <= NUM_COLS
    //   If fifo_count_q[r]==0, head/tail contents are don't-care.
    //   On alloc_gnt:   push 1 col into row FIFO
    //   On retire:      pop  1 col from row FIFO (immediately at RLAST accept)
    // =========================================================================
    logic [COL_W-1:0] per_row_col_queue [NUM_ROWS][NUM_COLS];

    logic [COL_W-1:0] fifo_head_idx_q [NUM_ROWS], fifo_head_idx_d [NUM_ROWS];
    logic [COL_W-1:0] fifo_tail_idx_q [NUM_ROWS], fifo_tail_idx_d [NUM_ROWS];
    logic [CNT_W-1:0] fifo_count_q    [NUM_ROWS], fifo_count_d    [NUM_ROWS];

    // =========================================================================
    // DECODE: Split INTERNAL RID {row,col} from r_in_if.id
    // =========================================================================
    logic [ROW_W-1:0] in_row_idx;   // row index carried by incoming beat
    logic [COL_W-1:0] in_col_idx;   // col index carried by incoming beat

    // For the incoming beat's row: what col is currently at the HEAD?
    logic [COL_W-1:0] head_col_value_for_row;
    logic             is_head_transaction;

    // =========================================================================
    // OUTPUT SKID (1 beat)
    // --------------------
    // Stores exactly ONE accepted input beat until the master takes it.
    // This isolates downstream backpressure from the slave-side handshake.
    // =========================================================================
    logic                  out_valid_q,   out_valid_d;     // skid has a valid beat
    logic [ID_WIDTH-1:0]   out_orig_id_q, out_orig_id_d;   // ORIGINAL RID to the master
    logic [DATA_WIDTH-1:0] out_data_q,    out_data_d;
    logic [RESP_WIDTH-1:0] out_resp_q,    out_resp_d;
    logic                  out_last_q,    out_last_d;      // LAST flag (end of burst)
    logic [TAG_WIDTH-1:0]  out_tagid_q,   out_tagid_d;

    // Track INTERNAL {row,col} of the beat currently in the skid
    // (used when retiring to generate a FREE event).
    logic [ROW_W-1:0]      out_row_idx_q, out_row_idx_d;
    logic [COL_W-1:0]      out_col_idx_q, out_col_idx_d;

    // "Retire" event fires when the skid's current beat is LAST and the master
    // actually takes it (valid & ready). We POP the per-row FIFO immediately
    // and enqueue a FREE {row,col} into the outbox.
    logic retire_fire;

    // =========================================================================
    // SINGLE WRITE PORT for per_row_col_queue
    // --------------------------------------
    // At most one write per cycle (on alloc_gnt). We compute row/slot/value.
    // =========================================================================
    logic                  fifo_wr_en;
    logic [ROW_W-1:0]      fifo_wr_row;
    logic [COL_W-1:0]      fifo_wr_idx;
    logic [COL_W-1:0]      fifo_wr_data;

    // =========================================================================
    // FREE OUTBOX (queue of pending frees)
    // -----------------------------------
    // - freeq_head_q : POP pointer (oldest free to present to allocator)
    // - freeq_tail_q : PUSH pointer (where next free is stored)
    // - freeq_count_q: # of pending frees (0..FREE_OUTBOX_DEPTH)
    // - freeq_row_q[k], freeq_col_q[k] : queued {row,col} frees
    //
    // We present the HEAD free as {free_unique_id, free_req=1} and HOLD it
    // level-valid until the allocator asserts free_ack. This guarantees no
    // free is lost, even if allocator stalls a few cycles.
    // =========================================================================
    logic [ROW_W-1:0] freeq_row_q [FREE_OUTBOX_DEPTH], freeq_row_d [FREE_OUTBOX_DEPTH];
    logic [COL_W-1:0] freeq_col_q [FREE_OUTBOX_DEPTH], freeq_col_d [FREE_OUTBOX_DEPTH];

    logic [FQ_W-1:0]  freeq_head_q,  freeq_head_d;     // POP pointer
    logic [FQ_W-1:0]  freeq_tail_q,  freeq_tail_d;     // PUSH pointer
    logic [$clog2(FREE_OUTBOX_DEPTH+1)-1:0] freeq_count_q, freeq_count_d;

    // =========================================================================
    // COMBINATIONAL
    // =========================================================================
    integer r_i;

    always_comb begin
        // --------------------------------------------------------------------
        // 1) DECODE incoming internal ID {row,col} from slave-side R beat
        // --------------------------------------------------------------------
        in_row_idx = r_in_if.id[ROW_W+COL_W-1 : COL_W];
        in_col_idx = r_in_if.id[COL_W-1      : 0   ];

        // --------------------------------------------------------------------
        // 2) DEFAULTS: hold all state (skid, per-row FIFOs, free outbox)
        // --------------------------------------------------------------------
        // Skid defaults
        out_valid_d   = out_valid_q;
        out_orig_id_d = out_orig_id_q;
        out_data_d    = out_data_q;
        out_resp_d    = out_resp_q;
        out_last_d    = out_last_q;
        out_tagid_d   = out_tagid_q;
        out_row_idx_d = out_row_idx_q;
        out_col_idx_d = out_col_idx_q;

        // Per-row FIFO pointers/counts: hold unless updated below
        for (r_i = 0; r_i < NUM_ROWS; r_i++) begin
            fifo_head_idx_d[r_i] = fifo_head_idx_q[r_i];
            fifo_tail_idx_d[r_i] = fifo_tail_idx_q[r_i];
            fifo_count_d   [r_i] = fifo_count_q   [r_i];
        end

        // Free outbox defaults
        freeq_head_d  = freeq_head_q;
        freeq_tail_d  = freeq_tail_q;
        freeq_count_d = freeq_count_q;
        for (int unsigned k = 0; k < FREE_OUTBOX_DEPTH; k++) begin
            freeq_row_d[k] = freeq_row_q[k];
            freeq_col_d[k] = freeq_col_q[k];
        end

        // No write to per_row_col_queue unless alloc_gnt below
        fifo_wr_en   = 1'b0;
        fifo_wr_row  = '0;
        fifo_wr_idx  = '0;
        fifo_wr_data = '0;

        // --------------------------------------------------------------------
        // 3) HEAD-OF-ROW calculation & gating policy for r_in_if.ready
        //    * We only accept beats for the HEAD {row,head_col} of that row's
        //      issue-order FIFO.
        //    * Also require a free slot in the output skid (either empty or
        //      draining in the same cycle).
        // --------------------------------------------------------------------
        if (fifo_count_q[in_row_idx] != '0) begin
            // At least 1 pending transaction for this row -> fetch its HEAD col
            head_col_value_for_row = per_row_col_queue[in_row_idx][fifo_head_idx_q[in_row_idx]];
        end else begin
            // Row FIFO empty; value is irrelevant (we won't accept anyway)
            head_col_value_for_row = '0;
        end

        // A beat is eligible if the row has pending txns AND its col matches HEAD
        is_head_transaction = (fifo_count_q[in_row_idx] != '0) &
                              (in_col_idx == head_col_value_for_row);

        // We only assert ready for legal HEAD beats and when skid can take a beat
        r_in_if.ready = is_head_transaction & (~out_valid_q | r_out_if.ready);

        // --------------------------------------------------------------------
        // 4) DRIVE master-side R interface directly from the skid registers
        //    The skid isolates timing and downstream backpressure.
        // --------------------------------------------------------------------
        r_out_if.valid = out_valid_q;
        r_out_if.id    = out_orig_id_q;   // ORIGINAL RID (restored)
        r_out_if.data  = out_data_q;
        r_out_if.resp  = out_resp_q;
        r_out_if.last  = out_last_q;
        r_out_if.tagid = out_tagid_q;

        // "Retire" occurs when the skid's beat is LAST and the master takes it.
        // We will POP the per-row FIFO and enqueue a FREE on this event.
        retire_fire = out_valid_q & r_out_if.ready & out_last_q;

        // --------------------------------------------------------------------
        // 5) SKID behavior
        //    (a) If current skid beat is accepted by master, clear its valid.
        //    (b) If we accept a new input beat from slave, load skid with it.
        // --------------------------------------------------------------------
        if (out_valid_q & r_out_if.ready) begin
            // Master consumed current skid beat -> skid becomes empty
            out_valid_d = 1'b0;
        end

        if (r_in_if.valid && r_in_if.ready) begin
            // Accept new beat into skid; capture ORIGINAL RID + payload
            out_valid_d   = 1'b1;
            out_orig_id_d = restored_id;    // ORIGINAL RID (tag_map for THIS beat)
            out_data_d    = r_in_if.data;
            out_resp_d    = r_in_if.resp;
            out_last_d    = r_in_if.last;
            out_tagid_d   = r_in_if.tagid;

            // Remember internal coordinates for FREE on retire
            out_row_idx_d = in_row_idx;
            out_col_idx_d = in_col_idx;
        end

        // --------------------------------------------------------------------
        // 6) Per-row order FIFO update:
        //    POP on retire (immediate), PUSH on alloc_gnt (AR issue).
        //    NOTE: POP must NOT wait for allocator ack; that would stall R path.
        // --------------------------------------------------------------------

        // POP: we've retired the HEAD {row,col}; advance head and decrement count
        if (retire_fire) begin
            // head++ (wrap)
            if (fifo_head_idx_q[out_row_idx_q] == (NUM_COLS-1))
                fifo_head_idx_d[out_row_idx_q] = '0;
            else
                fifo_head_idx_d[out_row_idx_q] = fifo_head_idx_q[out_row_idx_q] + 1;

            // count-- (guard against negative)
            if (fifo_count_q[out_row_idx_q] != '0)
                fifo_count_d[out_row_idx_q] = fifo_count_q[out_row_idx_q] - 1;
        end

        // PUSH: AR issuer granted a new {row,col} -> append 'col' at row's TAIL
        if (alloc_gnt) begin
            logic [ROW_W-1:0] push_row;
            logic [COL_W-1:0] push_col;
            push_row = unique_id[ROW_W+COL_W-1 : COL_W];
            push_col = unique_id[COL_W-1      : 0];

            fifo_wr_en   = 1'b1;
            fifo_wr_row  = push_row;
            fifo_wr_idx  = fifo_tail_idx_q[push_row];  // write at current tail
            fifo_wr_data = push_col;

            // tail++ (wrap)
            if (fifo_tail_idx_q[push_row] == (NUM_COLS-1))
                fifo_tail_idx_d[push_row] = '0;
            else
                fifo_tail_idx_d[push_row] = fifo_tail_idx_q[push_row] + 1;

            // count++ (bounded by NUM_COLS)
            if (fifo_count_q[push_row] < NUM_COLS)
                fifo_count_d[push_row] = fifo_count_q[push_row] + 1;
            // else: overflow => issuer bug; consider an assertion
        end

        // --------------------------------------------------------------------
        // 7) FREE OUTBOX (handshake to allocator)
        //    - Present the HEAD free (if any) level-valid until free_ack.
        //    - On retire: enqueue a new free {row,col} at TAIL (if space).
        //    - On ack: POP one from HEAD.
        // --------------------------------------------------------------------

        // Present current HEAD free to allocator
        if (freeq_count_q != 0) begin
            free_req       = 1'b1;
            free_unique_id = { freeq_row_q[freeq_head_q], freeq_col_q[freeq_head_q] };
        end else begin
            free_req       = 1'b0;
            free_unique_id = '0;
        end

        // POP a presented free on allocator ack
        if ((freeq_count_q != 0) & free_ack) begin
            // head++ (wrap)
            if (freeq_head_q == FQ_W'(FREE_OUTBOX_DEPTH-1))
                freeq_head_d = '0;
            else
                freeq_head_d = freeq_head_q + FQ_W'(1);

            // count--
            freeq_count_d = freeq_count_q - 1;
        end

        // PUSH a new free when we retire a txn (LAST beat taken by master)
        if (retire_fire) begin
            if (freeq_count_q < FREE_OUTBOX_DEPTH) begin
                // write {row,col} at tail
                freeq_row_d[freeq_tail_q] = out_row_idx_q;
                freeq_col_d[freeq_tail_q] = out_col_idx_q;

                // tail++ (wrap)
                if (freeq_tail_q == FQ_W'(FREE_OUTBOX_DEPTH-1))
                    freeq_tail_d = '0;
                else
                    freeq_tail_d = freeq_tail_q + FQ_W'(1);

                // count++
                freeq_count_d = freeq_count_q + 1;
            end
            // else: outbox overflow => allocator stalled too long; assert in sim
        end
    end // always_comb

    // =========================================================================
    // SEQUENTIAL: commit all state on clk
    // =========================================================================
    integer rr;
    always_ff @(posedge clk) begin
        if (rst) begin
            // ---- Reset skid ----
            out_valid_q   <= 1'b0;
            out_orig_id_q <= '0;
            out_data_q    <= '0;
            out_resp_q    <= '0;
            out_last_q    <= 1'b0;
            out_tagid_q   <= '0;
            out_row_idx_q <= '0;
            out_col_idx_q <= '0;

            // ---- Reset per-row FIFOs ----
            for (rr = 0; rr < NUM_ROWS; rr++) begin
                fifo_head_idx_q[rr] <= '0;  // POP starts at 0
                fifo_tail_idx_q[rr] <= '0;  // PUSH starts at 0
                fifo_count_q   [rr] <= '0;  // empty
            end
            // Note: per_row_col_queue contents are don't-care when count==0.

            // ---- Reset free outbox ----
            freeq_head_q  <= '0;
            freeq_tail_q  <= '0;
            freeq_count_q <= '0;
            for (int unsigned k = 0; k < FREE_OUTBOX_DEPTH; k++) begin
                freeq_row_q[k] <= '0;
                freeq_col_q[k] <= '0;
            end

        end else begin
            // ---- Commit skid ----
            out_valid_q   <= out_valid_d;
            out_orig_id_q <= out_orig_id_d;
            out_data_q    <= out_data_d;
            out_resp_q    <= out_resp_d;
            out_last_q    <= out_last_d;
            out_tagid_q   <= out_tagid_d;
            out_row_idx_q <= out_row_idx_d;
            out_col_idx_q <= out_col_idx_d;

            // ---- Commit per-row FIFOs ----
            for (rr = 0; rr < NUM_ROWS; rr++) begin
                fifo_head_idx_q[rr] <= fifo_head_idx_d[rr];
                fifo_tail_idx_q[rr] <= fifo_tail_idx_d[rr];
                fifo_count_q   [rr] <= fifo_count_d   [rr];
            end

            // ---- Commit free outbox ----
            freeq_head_q  <= freeq_head_d;
            freeq_tail_q  <= freeq_tail_d;
            freeq_count_q <= freeq_count_d;
            for (int unsigned k = 0; k < FREE_OUTBOX_DEPTH; k++) begin
                freeq_row_q[k] <= freeq_row_d[k];
                freeq_col_q[k] <= freeq_col_d[k];
            end

            // ---- Single write to per_row_col_queue (on alloc_gnt) ----
            if (fifo_wr_en) begin
                per_row_col_queue[fifo_wr_row][fifo_wr_idx] <= fifo_wr_data;
            end
        end
    end

    // =========================================================================
    // (Optional) ASSERTIONS — enable in simulation/formal
    // =========================================================================
    // 1) Accept only head-of-row beats (protocol ordering)
    // assert property (@(posedge clk) disable iff (rst)
    //   (r_in_if.valid && r_in_if.ready) |->
    //     (fifo_count_q[in_row_idx] != 0) &&
    //     (in_col_idx == per_row_col_queue[in_row_idx][fifo_head_idx_q[in_row_idx]])
    // );

    // 2) Never POP when row FIFO empty
    // assert property (@(posedge clk) disable iff (rst)
    //   !(retire_fire && (fifo_count_q[out_row_idx_q] == 0))
    // );

    // 3) Never overflow a row FIFO (issuer must obey NUM_COLS)
    // assert property (@(posedge clk) disable iff (rst)
    //   !(alloc_gnt && (fifo_count_q[unique_id[ROW_W+COL_W-1:COL_W]] == NUM_COLS))
    // );

    // 4) Free outbox capacity must be sufficient (tune FREE_OUTBOX_DEPTH)
    // assert property (@(posedge clk) disable iff (rst)
    //   !(retire_fire && (freeq_count_q == FREE_OUTBOX_DEPTH))
    // );

endmodule


