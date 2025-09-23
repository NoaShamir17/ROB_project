// // ============================================================================
// // allocator + tag map (cyclic FREE queues per row, no scanning)
// // ----------------------------------------------------------------------------
// // High-level goal
// //  - We translate ORIGINAL AXI IDs (ARID/RID space) into UNIQUE *internal*
// //    IDs of the form {row,col}. "row" is pinned (sticky) to an ORIGINAL ID
// //    while any column in that row is busy. "col" is the per-row slot.
// //
// // Why this structure?
// //  - AXI requires in-order responses *per ORIGINAL ID*, but allows
// //    out-of-order completion across different IDs. The matrix {row,col}
// //    lets the fabric run many requests in parallel, while we still know
// //    which ORIGINAL ID to restore on the way back (via tag_map).
// //  - A tiny cyclic FREE-queue per row gives O(1) "choose next col":
// //      * ALLOC: pop head  (no linear search or priority-encode)
// //      * FREE : push tail (returns the slot for re-use in cyclic order)
// //    This avoids LSB-first scans and exactly matches the "keep advancing
// //    pointer; don't reuse a hole early" policy you asked for.
// //
// // Behavior summary
// //  - When a request arrives for 'in_id', we:
// //      1) Try to *reuse* the row already bound to that ORIGINAL ID.
// //      2) Otherwise, grab the first completely unused row and bind it to in_id.
// //      3) Allocate a column by POPPING the row's FREE queue head.
// //      4) Produce unique_id={row,col} and remember tag_map[row][col]=in_id.
// //  - When the response retires (free_req with free_unique_id={row,col}):
// //      1) Immediately expose restored_id = tag_map[row][col] (combinational).
// //      2) Return 'col' by PUSHING it to that row's FREE queue tail.
// //      3) If all columns of the row are free again, release the row binding.
// //
// // Notes
// //  - All updates are synchronous (always_ff). The "grant" and "restored_id"
// //    are combinational outputs for simple top-level plumbing.
// //  - No modulo operator is used in datapath; wrap uses a simple compare+add.
// //  - The code is heavily commented for readability; area/timing is still tiny
// //    for typical NUM_ROWS/NUM_COLS in a student project.
// // ============================================================================

// module allocator_tag_map #(
//     // Width of the ORIGINAL AXI ID space (what the master uses on ARID/RID)
//     parameter int ID_WIDTH = 4,

//     // Geometry: total capacity = NUM_ROWS * NUM_COLS
//     // - NUM_ROWS  : how many *different* ORIGINAL IDs can be active at once
//     // - NUM_COLS  : how many *concurrent* txns per SAME ORIGINAL ID
//     parameter int NUM_ROWS = 4,
//     parameter int NUM_COLS = 4
// )(
//     // -------- Clock / Reset --------
//     input  logic clk,   // single synchronous clock
//     input  logic rst,   // synchronous active-high reset -----(positive or negative?)

//     // -------- Allocate interface (from AR path) --------
//     // alloc_req  : "I need a {row,col} for this in_id"
//     // in_id      : ORIGINAL ID of the incoming request
//     // alloc_gnt  : 1-cycle pulse when we *successfully* allocated
//     // unique_id  : the INTERNAL tag {row,col} to send downstream on ARID
//     input  logic                alloc_req,
//     input  logic [ID_WIDTH-1:0] in_id,
//     output logic                alloc_gnt,
//     output logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] unique_id,

//     // -------- Free interface (from R retirement) --------
//     // free_req        : "please free this INTERNAL {row,col}"
//     // free_unique_id  : the INTERNAL tag {row,col} observed on the last beat
//     // restored_id     : combinational ORIGINAL ID for that {row,col}
//     // free_ack        : 1-cycle pulse when we *accepted* the free
//     input  logic                free_req,
//     input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] free_unique_id, //$clog2???
//     output logic [ID_WIDTH-1:0] restored_id,
//     output logic                free_ack
// );

//     // ---------------- Local widths ----------------
//     localparam int ROW_W = $clog2(NUM_ROWS);
//     localparam int COL_W = $clog2(NUM_COLS);
//     localparam int CNT_W = $clog2(NUM_COLS+1);

//     // =========================================================================
//     //                            STATE REGISTERS
//     // =========================================================================

//     // Row binding bitmap and owner IDs
//     logic [NUM_ROWS-1:0]             row_is_bound_q, row_is_bound_d;
//     logic [ID_WIDTH-1:0]             bound_orig_id_q [NUM_ROWS], bound_orig_id_d [NUM_ROWS];

//     // Tag map: ORIGINAL ID recorded for each INTERNAL slot {row,col}
//     logic [ID_WIDTH-1:0]             tag_map_q       [NUM_ROWS][NUM_COLS],
//                                      tag_map_d       [NUM_ROWS][NUM_COLS];

//     // Per-row FREE ring queues (store column indices), head/tail/count
//     logic [COL_W-1:0]                free_queue_slot_q [NUM_ROWS][NUM_COLS],
//                                      free_queue_slot_d [NUM_ROWS][NUM_COLS];
//     logic [COL_W-1:0]                pnt_allocate_q    [NUM_ROWS], pnt_allocate_d    [NUM_ROWS]; // was free_head_idx_*
//     logic [COL_W-1:0]                pnt_free_q        [NUM_ROWS], pnt_free_d        [NUM_ROWS]; // was free_tail_idx_*
//     logic [CNT_W-1:0]                free_count_q      [NUM_ROWS], free_count_d      [NUM_ROWS];

//     // =========================================================================
//     //                           ALLOCATION PATH (COMB)
//     // =========================================================================

//     // Existing bound row for this in_id?
//     logic             have_row_hit;
//     logic [ROW_W-1:0] hit_row_idx;

//     always_comb begin
//         have_row_hit = 1'b0;
//         hit_row_idx  = '0;
//         for (int unsigned r = 0; r < NUM_ROWS; r++) begin //use generate
//             if (row_is_bound_q[r] & (bound_orig_id_q[r] == in_id)) begin
//                 have_row_hit = 1'b0 | 1'b1; // force 2-state
//                 hit_row_idx  = r[ROW_W-1:0];
//                 break;
//             end
//         end
//     end

//     // First unbound row?
//     logic             have_unbound_row;
//     logic [ROW_W-1:0] first_unbound_row_idx;

//     always_comb begin
//         have_unbound_row     = 1'b0;
//         first_unbound_row_idx= '0;
//         for (int unsigned r = 0; r < NUM_ROWS; r++) begin
//             if (!row_is_bound_q[r]) begin
//                 have_unbound_row      = 1'b0 | 1'b1;
//                 first_unbound_row_idx = r[ROW_W-1:0];
//                 break; //no break
//             end
//         end
//     end

//     // Choose row: prefer existing binding, else take first free row
//     logic [ROW_W-1:0] chosen_row_idx;
//     always_comb begin
//         chosen_row_idx = have_row_hit ? hit_row_idx : first_unbound_row_idx;
//     end

//     // Column availability and chosen column from row's FREE queue
//     logic             has_free_col_in_chosen_row;
//     logic [COL_W-1:0] chosen_col_idx;

//     always_comb begin
//         if ( (have_row_hit | have_unbound_row) & (free_count_q[chosen_row_idx] != '0) ) begin
//             has_free_col_in_chosen_row = 1'b1;
//             chosen_col_idx             = free_queue_slot_q[chosen_row_idx][pnt_allocate_q[chosen_row_idx]];
//         end else begin
//             has_free_col_in_chosen_row = 1'b0;
//             chosen_col_idx             = '0;
//         end
//     end

//     // Simple internal condition as a net (assign is fine here)
//     wire can_allocate_row = (have_row_hit | have_unbound_row);

//     // =========================================================================
//     //                              FREE PATH (COMB)
//     // =========================================================================

//     // Decode {row,col} from free_unique_id (as nets; assign is fine)
//     wire [ROW_W-1:0] free_row_idx = free_unique_id[ROW_W+COL_W-1 : COL_W];
//     wire [COL_W-1:0] free_col_idx = free_unique_id[COL_W-1 : 0];

//     // =========================================================================
//     //                        OUTPUTS & NEXT-STATE (COMB)
//     // =========================================================================

//     always_comb begin
//         // ---------- Default outputs ----------
//         alloc_gnt   = 1'b0;
//         unique_id   = '0;
//         restored_id = tag_map_q[free_row_idx][free_col_idx]; // comb restore
//         // free_ack is registered below; leave it unchanged in comb

//         // ---------- Default next-state (hold) ----------
//         row_is_bound_d = row_is_bound_q;
//         for (int unsigned r = 0; r < NUM_ROWS; r++) begin
//             bound_orig_id_d [r] = bound_orig_id_q [r];
//             pnt_allocate_d  [r] = pnt_allocate_q  [r];
//             pnt_free_d      [r] = pnt_free_q      [r];
//             free_count_d    [r] = free_count_q    [r];
//             for (int unsigned c = 0; c < NUM_COLS; c++) begin
//                 free_queue_slot_d[r][c] = free_queue_slot_q[r][c];
//                 tag_map_d        [r][c] = tag_map_q        [r][c];
//             end
//         end

//         // ---------- Allocation commit ----------
//         if (alloc_req & can_allocate_row & has_free_col_in_chosen_row) begin
//             alloc_gnt = 1'b1;
//             unique_id = {chosen_row_idx, chosen_col_idx};

//             // Bind row if it was unbound
//             if (~have_row_hit) begin
//                 row_is_bound_d[chosen_row_idx]  = 1'b1;
//                 bound_orig_id_d[chosen_row_idx] = in_id;
//             end

//             // POP from row's FREE queue: advance ALLOCATE pointer with wrap, decrement count
//             if (pnt_allocate_q[chosen_row_idx] == COL_W'(NUM_COLS-1))
//                 pnt_allocate_d[chosen_row_idx] = '0;
//             else
//                 pnt_allocate_d[chosen_row_idx] = pnt_allocate_q[chosen_row_idx] + COL_W'(1);

//             if (free_count_q[chosen_row_idx] != '0)
//                 free_count_d[chosen_row_idx] = free_count_q[chosen_row_idx] - CNT_W'(1);

//             // Record ORIGINAL ID for this INTERNAL slot
//             tag_map_d[chosen_row_idx][chosen_col_idx] = in_id;
//         end

//         // ---------- Free commit ----------
//         if (free_req) begin
//             // PUSH freed column into row's FREE queue at FREE pointer
//             free_queue_slot_d[free_row_idx][pnt_free_q[free_row_idx]] = free_col_idx;

//             // Advance FREE pointer with wrap
//             if (pnt_free_q[free_row_idx] == COL_W'(NUM_COLS-1))
//                 pnt_free_d[free_row_idx] = '0;
//             else
//                 pnt_free_d[free_row_idx] = pnt_free_q[free_row_idx] + COL_W'(1);

//             // Increment FREE count
//             if (free_count_q[free_row_idx] != CNT_W'(NUM_COLS))
//                 free_count_d[free_row_idx] = free_count_q[free_row_idx] + CNT_W'(1);

//             // If after increment the row is fully free, release binding
//             if (free_count_d[free_row_idx] == CNT_W'(NUM_COLS)) begin
//                 row_is_bound_d[free_row_idx]  = 1'b0;
//                 bound_orig_id_d[free_row_idx] = '0;
//                 // Note: tag_map entries are left as don't-cares until overwritten on next alloc
//             end
//         end
//     end

//     // =========================================================================
//     //                         SEQUENTIAL (REGISTER) LOGIC
//     // =========================================================================

//     always_ff @(posedge clk) begin
//         if (rst) begin
//             row_is_bound_q <= '0;
//             for (int unsigned r = 0; r < NUM_ROWS; r++) begin
//                 bound_orig_id_q[r] <= '0;

//                 // Initialize FREE queues: queue = [0,1,2,...,NUM_COLS-1]
//                 pnt_allocate_q[r] <= '0;
//                 pnt_free_q    [r] <= '0;
//                 free_count_q  [r] <= CNT_W'(NUM_COLS);

//                 for (int unsigned c = 0; c < NUM_COLS; c++) begin
//                     free_queue_slot_q[r][c] <= COL_W'(c);
//                     tag_map_q        [r][c] <= '0;
//                 end
//             end
//             free_ack <= 1'b0;

//         end else begin
//             row_is_bound_q <= row_is_bound_d;
//             for (int unsigned r = 0; r < NUM_ROWS; r++) begin
//                 bound_orig_id_q[r] <= bound_orig_id_d[r];
//                 pnt_allocate_q [r] <= pnt_allocate_d [r];
//                 pnt_free_q     [r] <= pnt_free_d     [r];
//                 free_count_q   [r] <= free_count_d   [r];
//                 for (int unsigned c = 0; c < NUM_COLS; c++) begin
//                     free_queue_slot_q[r][c] <= free_queue_slot_d[r][c];
//                     tag_map_q        [r][c] <= tag_map_d        [r][c];
//                 end
//             end

//             // Free handshake acknowledge (mirror; make ready/valid if you need decoupling)
//             free_ack <= free_req;
//         end
//     end

//     // =========================================================================
//     // (Optional) Assertions for sim/formal
//     // =========================================================================
//     // // Free-count is within 0..NUM_COLS
//     // for (genvar r = 0; r < NUM_ROWS; r++) begin : A_RANGE
//     //   assert property (@(posedge clk) disable iff (rst)
//     //     free_count_q[r] <= NUM_COLS
//     //   );
//     // end

// endmodule




// ============================================================================
// allocator_tag_map
// ----------------------------------------------------------------------------
// Purpose
//  - Map ORIGINAL AXI IDs (in_id) to UNIQUE internal tags {row,col} so that
//    the datapath can run many requests concurrently but still restore the
//    per-ORIGINAL-ID ordering on return.
//  - Each original ID "sticks" to a row while that row has any busy columns.
//  - Each row has a tiny cyclic FREE queue of columns: ALLOC pops head,
//    FREE pushes tail. No scans or modulo operators.
//
// Key properties
//  - O(1) allocate/free within a row (just pointer bump + compare for wrap).
//  - In-order per original ID is preserved externally by using the sticky row.
//  - State arrays are updated only on events (alloc/free), reducing toggling.
//
// Interface
//  - alloc_req/in_id     -> request to allocate a unique {row,col}
//  - alloc_gnt/unique_id -> single-cycle grant + internal tag
//  - free_req/free_id    -> request to free that internal {row,col}
//  - restored_id         -> original ID for the free_id (comb, guarded)
//  - free_ack            -> mirrors free_req (single-cycle ack)
//
// Notes
//  - Single clock, synchronous active-high reset.
//  - No always_* inside generate loops. One always_comb + one always_ff.
//  - Widths derived from geometry; ports use UID_W computed from parameters.
// ============================================================================

module allocator_tag_map #(
    // External AXI ID width (ARID/RID)
    parameter int ID_WIDTH = 4,

    // Geometry: NUM_ROWS * NUM_COLS total slots
    // - NUM_ROWS: max distinct ORIGINAL IDs active at once
    // - NUM_COLS: max concurrent txns per SAME ORIGINAL ID
    parameter int NUM_ROWS = 4,
    parameter int NUM_COLS = 4,

    // ---- Derived widths (as parameters so they can be used in the port list) ----
    // Guard against zero-width vectors by forcing 1 when size==1
    parameter int ROW_W = (NUM_ROWS > 1) ? $clog2(NUM_ROWS) : 1,
    parameter int COL_W = (NUM_COLS > 1) ? $clog2(NUM_COLS) : 1,
    parameter int UID_W = ROW_W + COL_W,                 // packed {row,col}
    parameter int CNT_W = $clog2(NUM_COLS+1)             // counts 0..NUM_COLS
)(
    // -------- Clock / Reset --------
    input  logic clk,
    input  logic rst, // synchronous, active-high

    // -------- Allocate interface (AR path) --------
    input  logic                alloc_req,              // request an internal tag
    input  logic [ID_WIDTH-1:0] in_id,                  // ORIGINAL AXI ID
    output logic                alloc_gnt,              // 1-cycle grant when allocated
    output logic [UID_W-1:0]    unique_id,              // internal {row,col}

    // -------- Free interface (R retirement) --------
    input  logic                free_req,               // request to free internal tag
    input  logic [UID_W-1:0]    free_unique_id,         // internal {row,col} to free
    output logic [ID_WIDTH-1:0] restored_id,            // ORIGINAL ID for that tag (comb)
    output logic                free_ack                // mirrors free_req
);

    // =========================================================================
    //                            STATE REGISTERS
    // =========================================================================
    // Row binding (whether row is in-use and by which original ID)
    logic [NUM_ROWS-1:0] row_is_bound_q, row_is_bound_d;
    logic [ID_WIDTH-1:0] bound_orig_id_q [NUM_ROWS];
    logic [ID_WIDTH-1:0] bound_orig_id_d [NUM_ROWS];

    // Tag map: for each {row,col}, remember ORIGINAL ID to restore on free
    // Note: We write only the single entry that changes on an alloc event.
    logic [ID_WIDTH-1:0] tag_map_q [NUM_ROWS][NUM_COLS];

    // Per-row cyclic FREE queues (store column indices at each queue slot)
    // Note: We write only the single entry that changes on a free event.
    logic [COL_W-1:0]    free_queue_slot_q [NUM_ROWS][NUM_COLS];

    // Per-row pointers + count
    logic [COL_W-1:0]    pnt_allocate_q [NUM_ROWS], pnt_allocate_d [NUM_ROWS]; // head (POP on alloc)
    logic [COL_W-1:0]    pnt_free_q     [NUM_ROWS], pnt_free_d     [NUM_ROWS]; // tail (PUSH on free)
    logic [CNT_W-1:0]    free_count_q   [NUM_ROWS], free_count_d   [NUM_ROWS]; // how many free cols remain

    // =========================================================================
    //                              FREE PATH DECODE
    // =========================================================================
    // Decode {row,col} from free_unique_id; used both for restore and free
    wire [ROW_W-1:0] free_row_idx = free_unique_id[ROW_W+COL_W-1 : COL_W];
    wire [COL_W-1:0] free_col_idx = free_unique_id[COL_W-1 : 0];

    // =========================================================================
    //                      COMBINATIONAL: NEXT-STATE + OUTPUTS
    // =========================================================================
    // Per-element write-enables so arrays update only on events
    logic tag_we  [NUM_ROWS][NUM_COLS];   // write enables for tag_map_q
    logic slot_we [NUM_ROWS][NUM_COLS];   // write enables for free_queue_slot_q

    always_comb begin
        // ---------- Default outputs ----------
        alloc_gnt   = 1'b0;
        unique_id   = '0;

        // Expose restore only when a free is requested (avoids spurious reads)
        restored_id = free_req ? tag_map_q[free_row_idx][free_col_idx] : '0;

        // ---------- Default next-state (hold) ----------
        row_is_bound_d = row_is_bound_q;
        for (int r = 0; r < NUM_ROWS; r++) begin
            bound_orig_id_d [r] = bound_orig_id_q [r];
            pnt_allocate_d  [r] = pnt_allocate_q  [r];
            pnt_free_d      [r] = pnt_free_q      [r];
            free_count_d    [r] = free_count_q    [r];
        end

        // Clear all per-element write-enables (arrays hold unless enabled)
        for (int r = 0; r < NUM_ROWS; r++) begin
            for (int c = 0; c < NUM_COLS; c++) begin
                tag_we [r][c] = 1'b0;
                slot_we[r][c] = 1'b0;
            end
        end

        // ---------- Row selection ----------
        // Find existing bound row for this in_id (first match) and first unbound row.
        // No breaks: we set each only once using the "if not found yet" style.
        logic             have_row_hit;            have_row_hit         = 1'b0;
        logic [ROW_W-1:0] hit_row_idx;             hit_row_idx          = '0;
        logic             have_unbound_row;        have_unbound_row     = 1'b0;
        logic [ROW_W-1:0] first_unbound_row_idx;   first_unbound_row_idx= '0;

        for (int r = 0; r < NUM_ROWS; r++) begin
            if (!have_row_hit && row_is_bound_q[r] && (bound_orig_id_q[r] == in_id)) begin
                have_row_hit = 1'b1;
                hit_row_idx  = ROW_W'(r);
            end
            if (!have_unbound_row && !row_is_bound_q[r]) begin
                have_unbound_row      = 1'b1;
                first_unbound_row_idx = ROW_W'(r);
            end
        end

        // Choose row: prefer existing binding, else the first unbound row
        logic [ROW_W-1:0] chosen_row_idx;
        chosen_row_idx     = have_row_hit ? hit_row_idx : first_unbound_row_idx;
        logic              can_allocate_row;  can_allocate_row = (have_row_hit || have_unbound_row);

        // Check column availability and peek chosen column from row's FREE queue
        logic             chosen_row_has_free;  chosen_row_has_free = 1'b0;
        logic [COL_W-1:0] chosen_col_idx;       chosen_col_idx      = '0;
        if (can_allocate_row && (free_count_q[chosen_row_idx] != '0)) begin
            chosen_row_has_free = 1'b1;
            chosen_col_idx      = free_queue_slot_q[chosen_row_idx][pnt_allocate_q[chosen_row_idx]];
        end

        // ---------- Allocation commit (event) ----------
        if (alloc_req && can_allocate_row && chosen_row_has_free) begin
            alloc_gnt = 1'b1;
            unique_id = {chosen_row_idx, chosen_col_idx};

            // Bind row if it was not already bound to this in_id
            if (!(have_row_hit && (hit_row_idx == chosen_row_idx))) begin
                row_is_bound_d[chosen_row_idx]  = 1'b1;
                bound_orig_id_d[chosen_row_idx] = in_id;
            end

            // POP the row's FREE queue head (advance allocate pointer with wrap)
            if (pnt_allocate_q[chosen_row_idx] == COL_W'(NUM_COLS-1))
                pnt_allocate_d[chosen_row_idx] = '0;
            else
                pnt_allocate_d[chosen_row_idx] = pnt_allocate_q[chosen_row_idx] + COL_W'(1);

            // Decrement count
            if (free_count_q[chosen_row_idx] != '0)
                free_count_d[chosen_row_idx] = free_count_q[chosen_row_idx] - CNT_W'(1);

            // Write exactly the single tag_map entry for {row=chosen_row_idx, col=chosen_col_idx}
            tag_we[chosen_row_idx][chosen_col_idx] = 1'b1; // actual write occurs in always_ff
        end

        // ---------- Free commit (event) ----------
        if (free_req) begin
            // Write exactly the single FREE-queue slot at the current free pointer
            slot_we[free_row_idx][pnt_free_q[free_row_idx]] = 1'b1; // write in always_ff

            // Advance FREE pointer with wrap
            if (pnt_free_q[free_row_idx] == COL_W'(NUM_COLS-1))
                pnt_free_d[free_row_idx] = '0;
            else
                pnt_free_d[free_row_idx] = pnt_free_q[free_row_idx] + COL_W'(1);

            // Increment FREE count
            if (free_count_q[free_row_idx] != CNT_W'(NUM_COLS))
                free_count_d[free_row_idx] = free_count_q[free_row_idx] + CNT_W'(1);

            // If fully free after increment, release the row binding
            if ( (free_count_q[free_row_idx] != CNT_W'(NUM_COLS)) &&
                 ((free_count_q[free_row_idx] + CNT_W'(1)) == CNT_W'(NUM_COLS)) ) begin
                row_is_bound_d [free_row_idx] = 1'b0;
                bound_orig_id_d[free_row_idx] = '0;
                // tag_map contents are don't-care until overwritten on next alloc
            end
        end
    end // always_comb

    // =========================================================================
    //                              SEQUENTIAL
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            // Global reset: rows unbound; all columns free in cyclic order
            row_is_bound_q <= '0;
            for (int r = 0; r < NUM_ROWS; r++) begin
                bound_orig_id_q[r] <= '0;
                pnt_allocate_q[r]  <= '0;
                pnt_free_q    [r]  <= '0;
                free_count_q  [r]  <= CNT_W'(NUM_COLS);  // all columns free initially
                for (int c = 0; c < NUM_COLS; c++) begin
                    free_queue_slot_q[r][c] <= COL_W'(c); // queue initialized to [0..NUM_COLS-1]
                    tag_map_q        [r][c] <= '0;
                end
            end
            free_ack <= 1'b0;

        end else begin
            // Row-level metadata and pointers
            row_is_bound_q <= row_is_bound_d;
            for (int r = 0; r < NUM_ROWS; r++) begin
                bound_orig_id_q[r] <= bound_orig_id_d[r];
                pnt_allocate_q [r] <= pnt_allocate_d [r];
                pnt_free_q     [r] <= pnt_free_d     [r];
                free_count_q   [r] <= free_count_d   [r];

                // ----- Selective array writes (only on events) -----
                for (int c = 0; c < NUM_COLS; c++) begin
                    if (slot_we[r][c]) begin
                        // FREE event writes the freed column into the queue at tail
                        free_queue_slot_q[r][c] <= free_col_idx;
                    end
                    if (tag_we[r][c]) begin
                        // ALLOC event records ORIGINAL ID into the chosen {row,col}
                        tag_map_q[r][c] <= in_id;
                    end
                    // else: no assignment -> register retains its value
                end
            end

            // Mirror free_req as an acknowledge (simple ready/valid)
            free_ack <= free_req;
        end
    end

endmodule
