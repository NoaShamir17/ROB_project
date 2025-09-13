// ============================================================================
// same_id_ordering_unit
// ----------------------------------------------------------------------------
// PURPOSE (read-only AXI):
// Enforce "in-order per ORIGINAL ID" on the R channel using the INTERNAL RID
// {row,col}.  We do this by:
//
//   • Keeping a small FIFO *per row* that stores the columns (col) in the
//     order transactions were allocated (one push per AR dispatch).
//   • On R beats from the slave (RID={row,col}), we only accept the beat if
//     {row,col} == HEAD(row) of that per-row FIFO. Otherwise we stall.
//   • When the LAST beat of HEAD(row) is *accepted by the master*, we pop the
//     FIFO head and pulse a "free" for that {row,col} back to the allocator.
//
// This guarantees AXI's "in-order per ID" rule in the simplest way possible,
// at the cost of stalling out-of-order returns for the same ID.
// ----------------------------------------------------------------------------
// INTEGRATION CONTRACT:
//   - Upstream (slave side): r_in_if.receiver (RID = INTERNAL {row,col})
//   - Downstream (master side): r_out_if.sender (RID = ORIGINAL, already
//     restored by a tag_map in your top; we take restored_id_in as input).
//   - From AR path (top): an "allocation event" per AR dispatch, carrying the
//     INTERNAL {row,col} that was allocated for that request. This pushes
//     the column into the per-row FIFO (preserving allocation order).
//   - To allocator: a free pulse + {row,col} when we retire a transaction
//     (i.e., when RLAST for the head is accepted by the master).
// ----------------------------------------------------------------------------
// DESIGN CHOICES (for clarity):
//   - We stall non-head beats (no per-slot beat buffers). This is the simplest
//     correct ordering policy and keeps code very readable.
//   - All state is tiny: per-row head/tail pointers + counts + a small storage
//     array of columns (width COL_W). Max depth per row = NUM_COLS.
//   - One-beat "skid" on the output: we register the outgoing beat to the
//     master, so backpressure is handled cleanly.
// ============================================================================

module same_id_ordering_unit #(
    // Geometry must match your allocator / tag_map
    parameter int NUM_ROWS  = 4,     // number of rows (one per ORIGINAL ID while active)
    parameter int NUM_COLS  = 4,     // max outstanding per row
    // R channel field widths (must match your r_if)
    parameter int ID_WIDTH  = 4,     // ORIGINAL RID width (what master expects)
    parameter int DATA_WIDTH= 64,
    parameter int RESP_WIDTH= 2,
    parameter int TAG_WIDTH = 4
)(
    input  logic clk,
    input  logic rst,

    // ------------------------ Slave-side R (input) -------------------------
    // Incoming beats from the slave/interconnect. RID is INTERNAL {row,col}.
    r_if.receiver r_in_if,

    // ------------------------ Master-side R (output) -----------------------
    // Outgoing beats to the master. RID must be ORIGINAL (restored by tag_map).
    r_if.sender   r_out_if,

    // ------------------------ Restored ID (from tag_map) -------------------
    // The ORIGINAL ID corresponding to the INTERNAL {row,col} of *THIS* r_in_if beat.
    // Your top should compute: restored_id_in = tag_map[{r_row,r_col}]
    input  logic [ID_WIDTH-1:0] restored_id_in,

    // ------------------------ Allocation events (from AR path) -------------
    // Pulse 'alloc_evt_valid' *once per AR dispatch to the slave*, with the
    // INTERNAL {row,col} that was allocated for that request.
    input  logic                 alloc_evt_valid,
    input  logic [$clog2(NUM_ROWS)-1:0] alloc_evt_row,
    input  logic [$clog2(NUM_COLS)-1:0] alloc_evt_col,

    // ------------------------ Free back to allocator -----------------------
    // We pulse 'free_req' exactly when the LAST beat of the head {row,col}
    // is accepted by the master, freeing that slot in the allocator.
    output logic                 free_req,
    output logic [$clog2(NUM_ROWS)-1:0] free_row,
    output logic [$clog2(NUM_COLS)-1:0] free_col
);
    // ---------------- Local widths ----------------
    localparam int ROW_W = $clog2(NUM_ROWS);
    localparam int COL_W = $clog2(NUM_COLS);

    // ---------------- Per-row FIFO of "allocated columns (col) in order" ---
    // Each row keeps the *sequence* of cols allocated on that row (the order
    // requests were issued). We only release responses in that order.
    //
    // Storage: col_fifo[row][index] holds a COL_W entry
    logic [COL_W-1:0] col_fifo [NUM_ROWS][NUM_COLS];

    // Pointers and count per row
    logic [COL_W-1:0] head_ptr_q [NUM_ROWS], head_ptr_d [NUM_ROWS]; // points to current head index
    logic [COL_W-1:0] tail_ptr_q [NUM_ROWS], tail_ptr_d [NUM_ROWS]; // points to next write index
    logic [$clog2(NUM_COLS+1)-1:0] cnt_q [NUM_ROWS], cnt_d [NUM_ROWS]; // how many entries in FIFO

    // ---------------- Decode INTERNAL RID from r_in_if.id ------------------
    // INTERNAL RID is packed as {row, col}. We must split it out.
    wire [ROW_W-1:0] r_row = r_in_if.id[ROW_W+COL_W-1:COL_W];
    wire [COL_W-1:0] r_col = r_in_if.id[COL_W-1:0];

    // ---------------- What is the "head column" for this row right now? ----
    // If FIFO is not empty (cnt>0), the head column value is at col_fifo[row][head_ptr]
    logic [COL_W-1:0] head_col_for_row;
    assign head_col_for_row = (cnt_q[r_row] != '0) ? col_fifo[r_row][head_ptr_q[r_row]] : '0;

    // ---------------- Is this incoming beat for the HEAD of its row? -------
    // We only want to accept beats for the head-of-queue transaction per row.
    wire is_head_of_row = (cnt_q[r_row] != '0) && (r_col == head_col_for_row);

    // ---------------- Output (to master) one-beat skid registers -----------
    // We register the outgoing R beat so we can apply clean backpressure.
    logic                  r_out_valid_q, r_out_valid_d;
    logic [ID_WIDTH-1:0]   r_out_id_q,    r_out_id_d;    // ORIGINAL RID (restored_id_in)
    logic [DATA_WIDTH-1:0] r_out_data_q,  r_out_data_d;
    logic [RESP_WIDTH-1:0] r_out_resp_q,  r_out_resp_d;
    logic                  r_out_last_q,  r_out_last_d;
    logic [TAG_WIDTH-1:0]  r_out_tagid_q, r_out_tagid_d;

    // We also need to remember the INTERNAL {row,col} of this *output beat*,
    // so that when LAST is accepted we know which FIFO to pop and what to free.
    logic [ROW_W-1:0]      r_out_row_q,   r_out_row_d;
    logic [COL_W-1:0]      r_out_col_q,   r_out_col_d;

    // ---------------- Ready/Valid handshake logic --------------------------
    // Input is ready if:
    //   (a) the incoming beat is for the HEAD of its row   (is_head_of_row)
    //   (b) AND we can place/replace the output register   (~r_out_valid_q || r_out_if.ready)
    //
    // This enforces in-order per row *without* buffering non-head beats.
    assign r_in_if.ready = is_head_of_row && (~r_out_valid_q || r_out_if.ready);

    // Drive the interface outputs from our registered signals
    assign r_out_if.valid = r_out_valid_q;
    assign r_out_if.id    = r_out_id_q;     // ORIGINAL RID (restored_id_in)
    assign r_out_if.data  = r_out_data_q;
    assign r_out_if.resp  = r_out_resp_q;
    assign r_out_if.last  = r_out_last_q;
    assign r_out_if.tagid = r_out_tagid_q;

    // ---------------- Combinational next-state for output regs -------------
    always_comb begin
        // Hold by default
        r_out_valid_d = r_out_valid_q;
        r_out_id_d    = r_out_id_q;
        r_out_data_d  = r_out_data_q;
        r_out_resp_d  = r_out_resp_q;
        r_out_last_d  = r_out_last_q;
        r_out_tagid_d = r_out_tagid_q;
        r_out_row_d   = r_out_row_q;
        r_out_col_d   = r_out_col_q;

        // If current output beat was accepted by master, clear valid
        if (r_out_valid_q && r_out_if.ready) begin
            r_out_valid_d = 1'b0;
        end

        // Launch a new output beat when we accept a beat from the slave
        if (r_in_if.valid && r_in_if.ready) begin
            r_out_valid_d = 1'b1;
            r_out_id_d    = restored_id_in; // ORIGINAL RID from tag_map
            r_out_data_d  = r_in_if.data;   // pass-through
            r_out_resp_d  = r_in_if.resp;
            r_out_last_d  = r_in_if.last;
            r_out_tagid_d = r_in_if.tagid;
            // track INTERNAL {row,col} for this output beat
            r_out_row_d   = r_row;
            r_out_col_d   = r_col;
        end
    end

    // ---------------- Free pulse & FIFO pop on LAST accept -----------------
    // We generate free for exactly *one* slot per cycle: when an output beat
    // with LAST is accepted by the master. Then we pop the head FIFO entry
    // for that row.
    //
    // Note: Because we only ever accept HEAD beats from a row, the sequence
    // of beats observed at the master is guaranteed to be in the same order
    // as allocation for that row. So "pop on LAST" == "retire the txn".
    wire retire_fire = (r_out_valid_q & r_out_if.ready & r_out_last_q); // this cycle we retire a txn

    // Free outputs (combinational)
    assign free_req = retire_fire;
    assign free_row = r_out_row_q;
    assign free_col = r_out_col_q;

    // ---------------- Per-row FIFO push/pop next-state logic ---------------
    // Default: hold current pointers and counts
    genvar gr;
    generate
        for (gr = 0; gr < NUM_ROWS; gr++) begin : G_ROW_INIT
            // We'll assign defaults below in a single always_comb
        end
    endgenerate

    // We update all rows' head/tail/cnt together in one block for clarity.
    integer r_i;
    always_comb begin
        // Default: hold current state
        for (r_i = 0; r_i < NUM_ROWS; r_i++) begin
            head_ptr_d[r_i] = head_ptr_q[r_i];
            tail_ptr_d[r_i] = tail_ptr_q[r_i];
            cnt_d    [r_i]  = cnt_q    [r_i];
        end

        // ---------------- PUSH on alloc event ----------------
        if (alloc_evt_valid) begin
            // Push alloc_evt_col into the FIFO of alloc_evt_row at tail_ptr
            col_fifo[alloc_evt_row][tail_ptr_q[alloc_evt_row]] = alloc_evt_col;
            // Advance tail and count (wrap by modulo NUM_COLS)
            tail_ptr_d[alloc_evt_row] = (tail_ptr_q[alloc_evt_row] + 1) % NUM_COLS;
            // Simple overflow guard: count must be < NUM_COLS (can add assertion)
            if (cnt_q[alloc_evt_row] != NUM_COLS[$clog2(NUM_COLS+1)-1:0]) begin
                cnt_d[alloc_evt_row] = cnt_q[alloc_evt_row] + 1;
            end
            // else: overflow (shouldn't happen if allocator never exceeds per-row capacity)
        end

        // ---------------- POP on retire (LAST accepted by master) ----------
        if (retire_fire) begin
            // Pop from the FIFO for r_out_row_q (must be non-empty)
            // Advance head and decrement count
            head_ptr_d[r_out_row_q] = (head_ptr_q[r_out_row_q] + 1) % NUM_COLS;
            if (cnt_q[r_out_row_q] != '0) begin
                cnt_d[r_out_row_q] = cnt_q[r_out_row_q] - 1;
            end
        end
    end

    // ---------------- Sequential: registers update -------------------------
    integer rr;
    always_ff @(posedge clk) begin
        if (rst) begin
            // Reset output skid regs
            r_out_valid_q <= 1'b0;
            r_out_id_q    <= '0;
            r_out_data_q  <= '0;
            r_out_resp_q  <= '0;
            r_out_last_q  <= 1'b0;
            r_out_tagid_q <= '0;
            r_out_row_q   <= '0;
            r_out_col_q   <= '0;

            // Reset per-row FIFOs
            for (rr = 0; rr < NUM_ROWS; rr++) begin
                head_ptr_q[rr] <= '0;
                tail_ptr_q[rr] <= '0;
                cnt_q    [rr]  <= '0;
                // Note: we do not clear col_fifo contents on reset; cnt=0 makes them ignored.
            end
        end else begin
            // Commit output regs
            r_out_valid_q <= r_out_valid_d;
            r_out_id_q    <= r_out_id_d;
            r_out_data_q  <= r_out_data_d;
            r_out_resp_q  <= r_out_resp_d;
            r_out_last_q  <= r_out_last_d;
            r_out_tagid_q <= r_out_tagid_d;
            r_out_row_q   <= r_out_row_d;
            r_out_col_q   <= r_out_col_d;

            // Commit per-row FIFO pointers and counts
            for (rr = 0; rr < NUM_ROWS; rr++) begin
                head_ptr_q[rr] <= head_ptr_d[rr];
                tail_ptr_q[rr] <= tail_ptr_d[rr];
                cnt_q    [rr]  <= cnt_d    [rr];
            end
        end
    end

    // ---------------- Optional safety assertions (for sim/formal) ----------
    // initial begin
    //   // Never pop from empty
    //   assert property (@(posedge clk) disable iff (rst)
    //       !(retire_fire && (cnt_q[r_out_row_q] == 0))
    //   ) else $error("Pop from empty FIFO on row %0d", r_out_row_q);
    //
    //   // Never overflow (allocator should limit outstanding per row)
    //   assert property (@(posedge clk) disable iff (rst)
    //       !(alloc_evt_valid && (cnt_q[alloc_evt_row] == NUM_COLS))
    //   ) else $error("Per-row FIFO overflow on row %0d", alloc_evt_row);
    // end

endmodule




