module ID_remapping_and_reordering #(
    parameter ID_WIDTH        = 4,    // AXI ID width (original RID width from master)
    parameter ADDR_WIDTH      = 32,   // AXI ARADDR width
    parameter LEN_WIDTH       = 8,    // AXI ARLEN width
    parameter TAG_WIDTH       = 4,    // Internal tag width (if you keep a tag separate from RID)
    parameter FIFO_DEPTH      = 16,   // Depth for buffering requests/beats (if you add FIFOs)
    parameter MAX_OUTSTANDING = 16    // Total outstanding transactions supported (matrix slots)
)(
    input  logic clk,            // Clock
    input  logic rst,            // Synchronous active-high reset

    ar_if.receiver ar_in_if,        // Incoming AR from AXI master (RID = ORIGINAL ID)
    ar_if.sender   ar_out_if        // Outgoing AR toward internal logic / slave (RID = INTERNAL {row,col})
    r_if.sender   r_out_if      // Outgoing R to AXI master (RID = ORIGINAL ID restored)
    r_if.receiver r_in_if    // Incoming R from AXI slave (RID = INTERNAL {row,col})
);
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// NOTE: The port list as written will NOT compile (missing commas).
// You need commas between interface ports, e.g.:
//   ar_if.sender   ar_out_if,       // ...
//   r_if.sender    r_out_if,        // ...
//   r_if.receiver  r_in_if
// Leaving it as-is per your request (“comments only”), but be aware.

// ----------------------------------------------------------------------------
// PURPOSE OF THIS MODULE (what this block should do):
// 1) On AR path:
//    - Receive AR from master with original RID in ar_in_if.id
//    - Choose a ROW (e.g., ar_in_if.id[ROW_W-1:0]) and request an INTERNAL tag
//      from a global allocator (matrix) => returns {row,col}
//    - Rewrite ARID to INTERNAL {row,col} and drive ar_out_if.*
//    - On AR fire (ar_out_if.valid && ar_out_if.ready), WRITE tag-map:
//        map[{row,col}] = original RID
//
// 2) On R path:
//    - Receive R from slave with INTERNAL RID = {row,col} on r_in_if.id
//    - READ tag-map[{row,col}] to get ORIGINAL RID
//    - Rewrite RID back to ORIGINAL and drive r_out_if.* to master
//    - On last beat commit to master (r_out_if.valid&&ready&&last) FREE {row,col}
//      back to the allocator and INVALIDATE the map entry.
//
// 3) Optional buffering using FIFO_DEPTH if you need elasticity.
//
// ----------------------------------------------------------------------------
// INTERNAL REPRESENTATIONS YOU’LL LIKELY NEED (not currently declared here):
// - Parameters for matrix shape:
//     localparam int NUM_ROWS = <power of two>; localparam int NUM_COLS = MAX_OUTSTANDING/NUM_ROWS;
//     localparam int ROW_W = $clog2(NUM_ROWS); localparam int COL_W = $clog2(NUM_COLS);
// - Global allocator (one instance):
//     alloc_req, alloc_row → alloc_gnt, alloc_id_row, alloc_id_col
//     free_req, free_row, free_col  ← free_gnt, free_misorder
// - Tag map (one instance):
//     wr_en, wr_row, wr_col, wr_orig_id
//     rd_en, rd_row, rd_col → rd_hit, rd_orig_id
//     inv_en, inv_row, inv_col
// - Row selection policy:
//     row_sel = ar_in_if.id[ROW_W-1:0]  (or use a small hash + row_full)
//
// ----------------------------------------------------------------------------

    // Struct to bundle incoming AR fields into one packed value
    typedef struct packed {
        logic [ID_WIDTH-1:0]   id;     // AXI RID (ORIGINAL on input; INTERNAL on output)
        logic [ADDR_WIDTH-1:0] addr;   // ARADDR
        logic [LEN_WIDTH-1:0]  len;    // ARLEN (beats-1)
        logic [2:0]            size;   // ARSIZE
        logic [1:0]            burst;  // ARBURST
        logic [3:0]            qos;    // ARQOS (optional)
        logic [TAG_WIDTH-1:0]  tagid;  // Optional internal tag (sideband; not required if you use RID only)
    } ar_req_t;
    // ^ Use this only if you want to stage AR as a single packed word (FIFO-friendly).
    //   If not using, you can delete later. For now this is a placeholder.

    
    logic push_req, pop_req;            // Control signals for write/read
    // ^ These names suggest FIFO control (push_req to enqueue AR, pop_req to dequeue).
    //   Decide: are you buffering AR requests internally? If yes, instantiate a FIFO:
    //     fifo #( .W($bits(ar_req_t)), .DEPTH(FIFO_DEPTH) ) u_ar_fifo (...)
    //   and drive push_req when ar_in_if fires; drive pop_req when allocator grants and downstream ready.

    logic matrix_full;
    // ^ Typically derived from allocator row_full[] or a global fullness metric.
    //   Use it to backpressure ar_in_if when you cannot allocate a slot.

    logic [MAX_OUTSTANDING-1:0] used_rows;
    // ^ Bitmap for debug/telemetry (which slots are in use).
    //   If you implement a true matrix with {row,col}, you’ll keep per-row head/tail
    //   and per-slot allocated bits. This flat vector can mirror that state for visibility.

    // =============================== WHAT TO DO WHERE =========================
    // === AR PATH (master → this → slave) ===
    // 1) Backpressure:
    //    ar_in_if.ready should be 1 only when you can both:
    //      (a) accept into your local AR FIFO (or direct pass) AND
    //      (b) successfully allocate a tag from the allocator soon (or immediately).
    //
    // 2) Allocate INTERNAL tag (RID = {row,col}):
    //    - Pick row_sel from original RID: logic [ROW_W-1:0] row_sel = ar_in_if.id[ROW_W-1:0];
    //      (or via a row selector that avoids full rows)
    //    - Drive alloc_req=1 when ar_in_if.valid && ar_out_if.ready (or when you pop from AR FIFO).
    //    - Wait for alloc_gnt=1, then stamp ar_out_if.id = {alloc_id_row, alloc_id_col}
    //
    // 3) Map write on AR fire:
    //    - On (ar_out_if.valid && ar_out_if.ready) pulse:
    //         wr_en=1; wr_row=alloc_id_row; wr_col=alloc_id_col; wr_orig_id=ar_in_if.id (original)
    //    - This creates the lookup for the R path.

    // === R PATH (slave → this → master) ===
    // 4) Map read per beat:
    //    - Split internal RID from r_in_if.id into row=MSBs, col=LSBs
    //    - Drive rd_en when you intend to accept the beat; use rd_orig_id to set r_out_if.id
    //    - If rd_hit==0, don’t accept (r_in_if.ready=0) and flag an error (map miss)
    //
    // 5) Free on retire:
    //    - On (r_out_if.valid && r_out_if.ready && r_out_if.last) pulse:
    //         free_req=1; free_row=row; free_col=col
    //    - When free_gnt=1, also inv_en=1 for the same {row,col} to clear the map entry.

    // === OPTIONAL BUFFERS ===
    // - Use FIFO_DEPTH to add:
    //    * An AR FIFO (type ar_req_t) between ar_in_if and allocate/stamp
    //    * Per-(row,col) beat buffers if you reorder at beat-level (often not needed for simple flows)

    // === ASSERTIONS (recommended) ===
    // - AR stamp stable under stall: when ar_out_if.valid && !ready → $stable(ar_out_if fields)
    // - Map hit for any accepted R beat
    // - Free misorder never happens (if allocator provides free_misorder, assert it is 0)

    // ================================ WIRING SKETCH ===========================
    // (for reference; declare and connect in your top or here if you keep it integrated)
    //
    // localparam int NUM_ROWS = <choose>, NUM_COLS = MAX_OUTSTANDING/NUM_ROWS;
    // localparam int ROW_W = $clog2(NUM_ROWS), COL_W = $clog2(NUM_COLS);
    //
    // // Allocator instance (single, global)
    // rob_id_matrix_fifo_rows #(.ROW_W(ROW_W), .COL_W(COL_W)) u_alloc (...);
    //
    // // Tag map instance (single, global)
    // rob_tag_map #(.ROW_W(ROW_W), .COL_W(COL_W), .ID_W(ID_WIDTH)) u_map (...);
    //
    // // Row selection (optional smarter policy)
    // arid_row_selector #(.NUM_ROWS(NUM_ROWS), .ID_W(ID_WIDTH)) u_sel (...);

endmodule
