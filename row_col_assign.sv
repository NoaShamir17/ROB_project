// module row_col_assign #(
//     parameter ID_WIDTH        = 4,    // AXI ID width (orig ID width)
//     parameter MAX_OUTSTANDING = 16    // Number of slots to manage (N)
// )(
//     input  logic clk,            // Clock
//     input  logic rst,            // Synchronous active-high reset
//     input  logic [ID_WIDTH-1:0] in_id,

//     output logic [2 * $clog2(MAX_OUTSTANDING) - 1:0] unique_id,
//     //output logic valid_id
//     );

// /*
//     typedef struct packed {
//         logic used;     // 1 = slot is allocated, 0 = free
//         logic [ID_WIDTH-1:0] id; // stored original master ID for this row
//         logic [$clog2(MAX_OUTSTANDING)-1:0] available_col; // next available column in this row
//     } row_variables_t;

//     // ---------------- THE TABLE: array of per-slot records, size MAX_OUTSTANDING.
//     row_variables_t row_variables [0:MAX_OUTSTANDING-1];

// /////////////////////////////////////////////////////////////////////////////////
// // Main clocked block: runs on every rising edge of clk
// always_ff @(posedge clk) begin
//   if (rst) begin
//     // ---------------- RESET BEHAVIOR ----------------
//     // Clear outputs so nothing is valid after reset
//     valid_id  <= 1'b0;
//     unique_id <= '0;

//     // Clear the entire slot table on reset
//     for (int j = 0; j < MAX_OUTSTANDING; j++) begin
//       row_variables[j].used      <= 1'b0; // no slot marked as used
//       row_variables[j].id        <= '0;   // clear stored ID
//       row_variables[j].available_col <= '0;   // convention: '0 means FREE in your code
//     end
//   end else begin
//     // ---------------- NORMAL OPERATION ----------------
//     valid_id  <= 1'b0;

//     // Loop over all slots in the table on this clock edge
//     for (int j = 0; j < MAX_OUTSTANDING; j++) begin
//       // If we haven’t already found a match, and this slot is used,
//       // and the stored ID equals the incoming ID...
//       if (row_variables[j].used & (row_variables[j].id == in_id)) begin
//         // Build the unique_id:
//         //   - Upper bits are the index 'j' of the slot that matched
//         //   - Lower bits are row_variables[j].available_col_col (in your convention)
//         unique_id <= {
//           j[$clog2(MAX_OUTSTANDING)-1:0],
//           row_variables[j].available_col[$clog2(MAX_OUTSTANDING)-1:0]
//         };

//         // Mark the output as valid, since we found a hit
//         valid_id <= 1'b1;
//         row_variables[j].available_col <= (row_variables[j].available_col + 1)%MAX_OUTSTANDING; // keep available_col as is

//       end
//       else begin
//         //this doesnt work!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//         if (&row_variables[j-1:0].used & ~row_variables[j].used) begin // if row is the first one available_col
//           unique_id <= {
//             j[$clog2(MAX_OUTSTANDING)-1:0],
//             row_variables[j].available_col[$clog2(MAX_OUTSTANDING)-1:0]
//           };
//           valid_id <= 1'b1;
//           row_variables[j].used      <= 1'b1; // mark as used
//           row_variables[j].id        <= in_id; // store incoming id
//           row_variables[j].available_col <= (row_variables[j].available_col + 1)%MAX_OUTSTANDING; // increment available_col
//         end
//       end
//     end
//   end
// end


// */

// logic [MAX_OUTSTANDING-1:0] used_rows;
// logic [$clog2(MAX_OUTSTANDING)-1:0] row;
// logic [$clog2(MAX_OUTSTANDING)-1:0] col;
// logic id_allocated;

// typedef struct packed {
//         logic [ID_WIDTH-1:0] id; // stored original master ID for this row
//         logic [$clog2(MAX_OUTSTANDING)-1:0] available_col; // next available column in this row
//     } row_variables_t;

// row_variables_t row_variables [0:MAX_OUTSTANDING-1];


// always_comb begin 
//     row = '0;
//     col = '0;
//     id_allocated = 1'b0;


//     for (int i = 0; i < MAX_OUTSTANDING; i++) begin
//         if (used_rows[i] & (row_variables[i].id == in_id)) begin //if master id is already in the table
//             row = i;
//             col = row_variables[i].available_col;
//             id_allocated = 1'b1;
//         end
//     end
//     if (!id_allocated) begin //if master id is not in the table
//         for (int j = 0; j < MAX_OUTSTANDING; j++) begin
//             if (&used_rows[j-1:0] & ~used_rows[j]) begin //find first free row
//                 row = j;
//                 col = row_variables[j].available_col;
//                 //mark the row as used and store the id
//                 used_rows[j] = 1'b1;
//                 row_variables[j].used      = 1'b1; // mark as used
//                 row_variables[j].id        = in_id; // store incoming id
//                 break;
//             end
//         end
//     end
//     unique_id = {row, col};

    
// end

// always_ff @(posedge clk) begin
//     if (rst) begin
//         used_rows <= '0;
//         unique_id <= '0;
//     end else begin
//         //FSM
        
//     end

// end 




// endmodule


// Row/Col allocator + tag map
// - Reuses a row per original ID (if any column of that row is still busy, the row "belongs" to that ID)
// - Each row has a bitmap of used/free columns to track outstanding transactions
// - A (row,col) -> original ID tag_map lets us restore the original ID when freeing
module row_col_assign #(
    parameter int ID_WIDTH = 4,     // Width of the ORIGINAL AXI ID (as seen on ARID/RID)
    // Choose a matrix geometry: total capacity = NUM_ROWS * NUM_COLS
    parameter int NUM_ROWS = 4,     // Number of distinct rows (each row binds to one original ID while used)
    parameter int NUM_COLS = 4      // Number of columns per row (how many concurrent txns for same ID)
)(
    // -------- Clock / Reset --------
    input  logic clk,               // Single synchronous clock
    input  logic rst,               // Synchronous active-high reset

    // -------- Allocate interface --------
    input  logic                alloc_req,   // 1: request to allocate a slot for 'in_id' this cycle
    input  logic [ID_WIDTH-1:0] in_id,       // The original ID we want to serve/allocate for
    output logic                alloc_gnt,   // 1-cycle pulse: allocation succeeded this cycle
    // Unique identifier we hand to the fabric/slave; pack row then col
    output logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] unique_id, // {row, col}

    // -------- Free interface --------
    input  logic                free_req,        // 1: request to free the slot addressed by 'free_unique_id'
    input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] free_unique_id, // {row,col} of the returning txn
    output logic [ID_WIDTH-1:0] restored_id,     // The ORIGINAL ID for that {row,col}, combinational lookup
    output logic                free_ack         // 1-cycle pulse acknowledging we consumed free_req
);
    // ---------------- Local widths ----------------
    // Pre-compute bit widths for indexing rows/cols based on parameters.
    localparam int ROW_W = $clog2(NUM_ROWS);     // Bits required to index a row
    localparam int COL_W = $clog2(NUM_COLS);     // Bits required to index a column

    // ---------------- State (always_ff) ----------------
    // All *_q are the current registered state; *_d are next-state (computed in always_comb).

    // row_used_q[r] == 1 if row r is currently bound to some original ID (i.e., at least one col is used)
    logic [NUM_ROWS-1:0] row_used_q, row_used_d;

    // row_id_q[r] holds which original ID is bound to row r (valid only when row_used_q[r]==1)
    logic [ID_WIDTH-1:0] row_id_q   [NUM_ROWS], row_id_d   [NUM_ROWS];

    // col_used_q[r][c] == 1 if column c of row r is currently allocated (outstanding)
    logic [NUM_COLS-1:0] col_used_q [NUM_ROWS], col_used_d [NUM_ROWS];

    // tag_map_q[r][c] stores the ORIGINAL ID that was allocated to {r,c} (for quick restore on free)
    logic [ID_WIDTH-1:0] tag_map_q  [NUM_ROWS][NUM_COLS], tag_map_d [NUM_ROWS][NUM_COLS];

    // ---------------- Helpers ----------------
    // Priority encoder: returns index of first '1' bit in v (LSB first).
    // Fixed 32-bit body for simplicity; we'll zero-extend inputs shorter than 32.
    function automatic [31:0] first_one(input logic [31:0] v);
        int k;                          // loop variable (synthesizes to a small priority mux tree)
        begin
            first_one = '0;             // default = 0 (if no bits are set, result will be 0)
            for (k = 0; k < 32; k++)    // check bits from LSB to MSB
                if (v[k]) begin
                    first_one = k;      // capture the first 1-bit index
                    break;              // stop at first hit
                end
        end
    endfunction

    // ---------------- Allocate (always_comb) ----------------
    // Combinational search signals for row selection
    logic             have_row_hit;     // 1 if an existing row is already bound to in_id
    logic [ROW_W-1:0] hit_row_idx;      // which row matched in_id
    logic             have_free_row;    // 1 if there exists a row not currently used by any ID
    logic [ROW_W-1:0] free_row_idx;     // the first free row index

    // Search an existing row bound to in_id (sticky binding)
    always_comb begin
        have_row_hit = 1'b0;            // default: assume no match
        hit_row_idx  = '0;              // default index value
        // Linear scan across rows; small NUM_ROWS → cheap logic
        for (int r = 0; r < NUM_ROWS; r++) begin
            // A "hit" if the row is used AND the bound ID equals in_id
            if (row_used_q[r] && (row_id_q[r] == in_id)) begin
                have_row_hit = 1'b1;    // remember we found a matching row
                hit_row_idx  = r[ROW_W-1:0]; // capture its index (truncate int->logic vector)
                break;                  // stop on first match
            end
        end
    end

    // Find first completely free row (no ID bound yet)
    always_comb begin
        have_free_row = 1'b0;           // default: assume none free
        free_row_idx  = '0;             // default index value
        for (int r = 0; r < NUM_ROWS; r++) begin
            if (!row_used_q[r]) begin   // a row is "free" if it's not used by any ID
                have_free_row = 1'b1;   // mark that a free row exists
                free_row_idx  = r[ROW_W-1:0]; // capture that row index
                break;                  // choose the first such row (LSB priority)
            end
        end
    end

    // For whichever row we will use (hit row or new row), pick the first free column
    logic [ROW_W-1:0]    chosen_row;        // the row we plan to allocate in this cycle
    logic [NUM_COLS-1:0] free_mask_for_row; // bit=1 means column is free
    logic                has_free_col_in_row; // 1 if any column in chosen_row is free
    logic [COL_W-1:0]    chosen_col;        // the first free column index we pick

    always_comb begin
        // Policy: prefer reusing an existing row (keeps per-ID ordering localized).
        // If no bound row exists, fall back to the first free row.
        if (have_row_hit)   chosen_row = hit_row_idx;
        else                chosen_row = free_row_idx;

        // Build a "free" mask from the used bitmap (invert used to get free)
        free_mask_for_row   = ~col_used_q[chosen_row];

        // Reduction-OR: true if at least one free column bit is 1
        has_free_col_in_row = |free_mask_for_row;

        // Priority-encode the first free column (LSB first).
        // We zero-extend free_mask_for_row up to 32 bits to match the helper function port,
        // then slice the result back down to COL_W bits.
        chosen_col = first_one({{(32-NUM_COLS){1'b0}}, free_mask_for_row})[COL_W-1:0];
    end

    // We can bind a row if either we found a hit or we found a free row.
    wire can_bind_row = have_row_hit || have_free_row;

    // Grant allocation only when:
    // - alloc_req is asserted,
    // - we have a row to bind (existing or free),
    // - and the chosen row actually has a free column.
    assign alloc_gnt = alloc_req && can_bind_row && has_free_col_in_row;

    // Pack the chosen row/col into a single "unique" tag (internal RID to send downstream)
    assign unique_id = {chosen_row, chosen_col}; // MSBs = row, LSBs = col

    // ---------------- Free decode (always_comb) ----------------
    // Unpack the incoming free request's unique_id into row / col indices
    wire [ROW_W-1:0] free_row = free_unique_id[ROW_W+COL_W-1:COL_W]; // upper bits are row
    wire [COL_W-1:0] free_col = free_unique_id[COL_W-1:0];           // lower bits are col

    // Combinationally restore the ORIGINAL ID for that slot using tag_map_q.
    // (Freeing itself is done in always_ff so the state change is synchronous.)
    assign restored_id = tag_map_q[free_row][free_col];

    // ---------------- Next-state logic (always_comb) ----------------
    // Compute the next state (*_d) from the current state (*_q) + this cycle's actions.
    always_comb begin
        // Start by copying current state -> next state (default "hold" behavior)
        row_used_d = row_used_q;
        for (int r = 0; r < NUM_ROWS; r++) begin
            row_id_d  [r] = row_id_q  [r];
            col_used_d[r] = col_used_q[r];
            for (int c = 0; c < NUM_COLS; c++) begin
                tag_map_d[r][c] = tag_map_q[r][c];
            end
        end

        // -------- Allocation commit (if we granted this cycle) --------
        if (alloc_gnt) begin
            // If we are taking a previously-free row, bind it to 'in_id' now.
            if (!have_row_hit) begin
                row_used_d[chosen_row] = 1'b1; // mark row as used by some ID
                row_id_d  [chosen_row] = in_id; // store which ID owns this row
            end
            // Occupy the chosen column in that row, and remember which original ID it belongs to.
            col_used_d[chosen_row][chosen_col] = 1'b1; // mark the column as allocated
            tag_map_d [chosen_row][chosen_col] = in_id; // save ORIGINAL ID for restore on free
        end

        // -------- Free commit (if we accepted a free this cycle) --------
        if (free_req) begin
            col_used_d[free_row][free_col] = 1'b0; // free that one column slot
            // If that was the last used column in the row, release the row binding entirely.
            if (col_used_d[free_row] == '0) begin
                row_used_d[free_row] = 1'b0; // row no longer owned by any ID
                row_id_d [free_row]  = '0;   // clear row's bound ID (tidy; not strictly required)
            end
            // Note: tag_map_d can be left as-is; the next allocation will overwrite it.
        end
    end

    // ---------------- State registers & handshakes (always_ff) ----------------
    // Synchronous update of all state; also generate free_ack as a 1-cycle pulse.
    always_ff @(posedge clk) begin
        if (rst) begin
            // Reset all state to empty/zero
            row_used_q <= '0;
            for (int r = 0; r < NUM_ROWS; r++) begin
                row_id_q  [r] <= '0;
                col_used_q[r] <= '0;
                for (int c = 0; c < NUM_COLS; c++) begin
                    tag_map_q[r][c] <= '0;
                end
            end
            free_ack <= 1'b0;           // nothing to ack during reset
        end else begin
            // Commit the next-state signals into the registers
            row_used_q <= row_used_d;
            for (int r = 0; r < NUM_ROWS; r++) begin
                row_id_q  [r] <= row_id_d  [r];
                col_used_q[r] <= col_used_d[r];
                for (int c = 0; c < NUM_COLS; c++) begin
                    tag_map_q[r][c] <= tag_map_d[r][c];
                end
            end

            // Handshake: acknowledge we consumed a free_req this cycle.
            // (Here we assume free_req is already qualified by the caller; we just mirror it as a pulse.)
            free_ack <= free_req;
        end
    end

    // ---------------- Synthesis-time sanity (optional) ----------------
    // You can enable assertions in simulation/formal to catch misconfigs:
    // initial begin
    //     assert (NUM_ROWS > 0 && NUM_COLS > 0)
    //         else $error("NUM_ROWS and NUM_COLS must be > 0");
    //     assert ((1<<ROW_W) >= NUM_ROWS && (1<<COL_W) >= NUM_COLS)
    //         else $error("clog2 width insufficient");
    // end
endmodule


    




    

