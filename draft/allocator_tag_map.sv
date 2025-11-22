module allocator_tag_map #(
    // External AXI ID width (ARID/RID)
    parameter int ID_WIDTH = 4,
    parameter int MAX_OUTSTANDING = 16,
    parameter int NUM_ROWS  = $bits(MAX_OUTSTANDING),
    parameter int NUM_COLS  = $bits(MAX_OUTSTANDING)
)(
    input  logic                 clk,
    input  logic                 rst, // synchronous active-high

    // alloc interface
    input  logic                 alloc_req,
    input  logic [ID_WIDTH-1:0]  in_orig_id,
    output logic                 alloc_gnt,
    output logic [UID_W-1:0]     unique_id,
    output logic                 tag_map_full, //TODO: check if needed - if so, implement

    // free interface
    input  logic                 free_req,
    input  logic [UID_W-1:0]     free_unique_id,
    output logic [ID_WIDTH-1:0]  restored_id,
    output logic                 free_ack
);

    // internal derived parameters
    localparam int ROW_W = $bits(NUM_ROWS);
    localparam int COL_W = $bits(NUM_COLS);
    localparam int UID_W = ROW_W + COL_W;
    localparam int PER_ROW_CNT_W = $bits(NUM_COLS+1);

    // ---------------------------------------------------------------------
    // state
    // ---------------------------------------------------------------------
    logic [NUM_ROWS-1:0]        row_is_bound_current, row_is_bound_next; // row is bound to some original ID
    logic [ID_WIDTH-1:0]        bound_orig_id_current [NUM_ROWS];     // original ID bound to each row
    logic [ID_WIDTH-1:0]        bound_orig_id_next [NUM_ROWS];

    // tag map
    logic [ID_WIDTH-1:0]        tag_map_q [NUM_ROWS][NUM_COLS];
    // write-enable for tag map (single entry on alloc)
    logic                       tag_we   [NUM_ROWS][NUM_COLS];

    // per-row next-available column pointer (monotonic wrap)
    logic [COL_W-1:0]           available_col_current [NUM_ROWS];
    logic [COL_W-1:0]           available_col_next [NUM_ROWS];

    // per-row outstanding count (how many in-flight requests)
    logic [CNT_W-1:0]           used_count_current [NUM_ROWS];
    logic [CNT_W-1:0]           used_count_next [NUM_ROWS];

    //

    // decode free_unique_id
    wire [ROW_W-1:0]            free_row_idx = free_unique_id[ROW_W+COL_W-1 : COL_W];
    wire [COL_W-1:0]            free_col_idx = free_unique_id[COL_W-1 : 0];


    // row selection signals
    logic             have_row_hit; // does any row match in_orig_id
    logic [ROW_W-1:0] hit_row_idx;      // index of matching row (if any)
    logic             have_unbound_row; // is there any unbound row
    logic [ROW_W-1:0] first_unbound_row_idx; // index of first unbound row (if any)
    logic             bound_row_found;
    // ---------------------------------------------------------------------
    // combinational next-state + outputs
    // ---------------------------------------------------------------------
    always_comb begin
        // defaults
        alloc_gnt   = 1'b0;
        unique_id   = '0;
        restored_id = free_req ? tag_map_q[free_row_idx][free_col_idx] : '0;
        free_ack    = 1'b0; // registered in always_ff below (mirror)

        // default next-state = hold current
        row_is_bound_next = row_is_bound_current;
        for (int r = 0; r < NUM_ROWS; r++) begin
            bound_orig_id_next[r]  = bound_orig_id_current[r];
            available_col_next[r]  = available_col_current[r];
            used_count_next[r]     = used_count_current[r];
        end

        // clear tag write enables (selective write on alloc)
        for (int r = 0; r < NUM_ROWS; r++) begin
            for (int c = 0; c < NUM_COLS; c++) begin
                tag_we[r][c] = 1'b0;
            end
        end
        // clear row selection signals
        have_row_hit        = 1'b0; hit_row_idx = '0;
        have_unbound_row    = 1'b0; first_unbound_row_idx = '0;
        bound_row_found = 1'b0;

        // --- row selection: find hit row and first free row ---
        //TODO: change loop - without flags!
        for (int r = 0; r < NUM_ROWS; r++) begin
            bound_row_found = row_is_bound_current[r] & (~|(bound_orig_id_current[r] ^ in_orig_id));
            if (~have_row_hit & bound_row_found) begin
                have_row_hit = 1'b1;
                hit_row_idx  = ROW_W'(r);
            end
            if (~have_unbound_row & ~row_is_bound_current[r]) begin
                have_unbound_row = 1'b1;
                first_unbound_row_idx = ROW_W'(r);
            end
        end

        logic [ROW_W-1:0] chosen_row_idx;
        chosen_row_idx = have_row_hit ? hit_row_idx : first_unbound_row_idx;
        logic can_choose_row = have_row_hit | have_unbound_row;

        //TODO: consider removing this part
        // Consider a simultaneous free in the same row: that frees a slot before allocation.
        // Compute effective used count for chosen row *if* free happens there this cycle.
        logic [CNT_W-1:0] eff_used_chosen;
        eff_used_chosen = used_count_current[chosen_row_idx];
        if (free_req && (free_row_idx == chosen_row_idx)) begin
            // avoid underflow
            if (eff_used_chosen != '0) eff_used_chosen = eff_used_chosen - CNT_W'(1);
        end

        // allow allocation only if chosen row has at least one free slot after accounting for same-cycle frees
        logic chosen_row_has_capacity;
        chosen_row_has_capacity = (eff_used_chosen < CNT_W'(NUM_COLS));

        // chosen column (monotonic pointer)
        logic [COL_W-1:0] chosen_col_idx;
        chosen_col_idx = available_col_current[chosen_row_idx];

        // ---------- Allocation event ----------
        if (alloc_req && can_choose_row && chosen_row_has_capacity) begin
            alloc_gnt = 1'b1;
            unique_id = {chosen_row_idx, chosen_col_idx};

            // If row was unbound, bind it
            if (!(have_row_hit && (hit_row_idx == chosen_row_idx))) begin
                row_is_bound_next[chosen_row_idx]  = 1'b1;
                bound_orig_id_next[chosen_row_idx] = in_orig_id;
            end

            // increment used_count for chosen row (will be written in always_ff)
            used_count_next[chosen_row_idx] = used_count_current[chosen_row_idx] + CNT_W'(1);

            // advance available_col (wrap)
            if (available_col_current[chosen_row_idx] == COL_W'(NUM_COLS-1))
                available_col_next[chosen_row_idx] = '0;
            else
                available_col_next[chosen_row_idx] = available_col_current[chosen_row_idx] + COL_W'(1);

            // write tag_map for chosen slot
            tag_we[chosen_row_idx][chosen_col_idx] = 1'b1;
        end

        // ---------- Free event ----------
        if (free_req) begin
            // decrement used_count for free_row (avoid underflow)
            if (used_count_current[free_row_idx] != '0)
                used_count_next[free_row_idx] = used_count_current[free_row_idx] - CNT_W'(1);
            else
                used_count_next[free_row_idx] = used_count_current[free_row_idx];

            // if row becomes empty after this free (considering alloc may also occur same cycle)
            // note: if an alloc to same row occurs in same cycle we already incremented used_count_next above
            // (alloc handling precedence in code guarantees used_count_next reflects both events)
            if (used_count_next[free_row_idx] == '0) begin
                row_is_bound_next[free_row_idx]  = 1'b0;
                bound_orig_id_next[free_row_idx] = '0;
            end
        end
    end // always_comb

    // ---------------------------------------------------------------------
    // sequential: registers update
    // ---------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            row_is_bound_current <= '0;
            for (int r = 0; r < NUM_ROWS; r++) begin
                bound_orig_id_current[r] <= '0;
                available_col_current[r] <= '0;
                used_count_current[r]    <= '0;
                for (int c = 0; c < NUM_COLS; c++) begin
                    tag_map_q[r][c] <= '0;
                end
            end
            free_ack <= 1'b0;
        end else begin
            row_is_bound_current <= row_is_bound_next;
            for (int r = 0; r < NUM_ROWS; r++) begin
                bound_orig_id_current[r] <= bound_orig_id_next[r];
                available_col_current[r] <= available_col_next[r];
                used_count_current[r]    <= used_count_next[r];
                for (int c = 0; c < NUM_COLS; c++) begin
                    if (tag_we[r][c]) begin
                        tag_map_q[r][c] <= in_orig_id; // tag write uses in_orig_id from the allocation request
                    end
                    // else leave tag_map_q as-is (don't overwrite on free)
                end
            end

            // simple free handshake mirror
            free_ack <= free_req;
        end
    end

endmodule
