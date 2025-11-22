=======================
Commented code by Gil - look for "GIL ERROR"
=======================

module id_allocate_and_restore_unit #(
    parameter int ID_WIDTH = 4,
    parameter int MAX_OUTSTANDING = 16,
    parameter int NUM_ROWS = $clog2(MAX_OUTSTANDING), // GIL ERROR - MaxOutstanding is a value already, not a vector. As a vector it is 32bits wide...
    parameter int NUM_COLS = $clog2(MAX_OUTSTANDING)  //  ...  I guess you wanted log or something like that. There is a function for that $clog2()
)(
    input  logic                 clk,
    input  logic                 rst, // synchronous active-high

    // alloc interface
    input  logic                 alloc_req,
    input  logic [ID_WIDTH-1:0]  in_orig_id,
    output logic                 alloc_gnt,
    output logic [ID_WIDTH-1:0]     unique_id,
    output logic                 id_matrix_full, //TODO: check if needed - if so, implement

    // free interface
    input  logic                 free_req,
    input  logic [ID_WIDTH-1:0]  unique_id_to_free,
    output logic [ID_WIDTH-1:0]  restored_id,
    output logic                 free_ack
);
    localparam int ROW_W = $bits(NUM_ROWS); // GIL ERROR - you did not mean bits. change to what you really meant.
    localparam int COL_W = $bits(NUM_COLS); // same...
    localparam int PER_ROW_CNT_W = $clog2(NUM_COLS+1); // GIL: Error - NumCol is already a number, not a vector to extract value from
    localparam int TOT_REQ_CNT_W = $clog2(MAX_OUSTSTANDING + 1); // GIL Error - same as above. think what you wanted to get from there at all...
							// GIL - and syntax error - OUSTST instead of OUTST...	

    logic [ID_WIDTH-1:0]       id_matrix [NUM_ROWS][NUM_COLS];
    logic                      id_matrix_we [NUM_ROWS][NUM_COLS]; // write-enable for id matrix (single entry on alloc)

    // ---------------------------------------------------------------------
    // state of each row in id matrix
    // (bound means row is bound to some original ID)
    // ---------------------------------------------------------------------
    logic [NUM_ROWS-1:0]        row_is_bound_current, row_is_bound_next; // row is bound to some original ID
    logic [ID_WIDTH-1:0]        bound_orig_id_current [NUM_ROWS];     // original ID bound to each row
    logic [ID_WIDTH-1:0]        bound_orig_id_next [NUM_ROWS];

    // per-row next-available column pointer (monotonic wrap)
    logic [COL_W-1:0]           available_col_current [NUM_ROWS];
    logic [COL_W-1:0]           available_col_next [NUM_ROWS];

    //---------couters---------
    // per-row outstanding count (how many in-flight requests)
    logic [PER_ROW_CNT_W-1:0]           used_count_current [NUM_ROWS];
    logic [PER_ROW_CNT_W-1:0]           used_count_next [NUM_ROWS];
    // total outstanding requests count
    logic [TOT_REQ_CNT_W-1:0]           tot_used_count_current;
    logic [TOT_REQ_CNT_W-1:0]           tot_used_count_next;

    assign id_matrix_full = (used_count_current == TOT_REQ_CNT_W'(MAX_OUTSTANDING));

    // ---------------------------------------------------------------------
    // row selection logic
    // ---------------------------------------------------------------------

    // row and col selection signals
    logic             have_row_hit; // does any row match in_orig_id
    logic [ROW_W-1:0] hit_row_idx;      // index of matching row (if any)
    logic [NUM_ROWS-1:0] row_bound_to_in_id; // index of row bound to in_orig_id is high
    logic             have_unbound_row; // is there any unbound row
    logic [ROW_W-1:0] first_unbound_row_idx; // index of first unbound row (if any)
    logic [NUM_ROWS-1:0] first_unbound_row; // index of first unbound row is high
    logic [ROW_W-1:0] chosen_row_idx;
    logic [COL_W-1:0] chosen_col_idx;

        
    // allocation signals
    assign alloc_gnt = alloc_req & (have_row_hit | have_unbound_row) & ~id_matrix_full;
    assign unique_id = {(ID_WIDTH-ROW_W-COL_W){1'b0},chosen_row_idx, chosen_col_idx};//unique_id is zero-padded to ID_WIDTH

    // slot selection combinational logic
    always_comb begin : slot_selection_logic
        // defaults
        have_row_hit        = 1'b0;
        hit_row_idx         = '0;
        have_unbound_row    = 1'b0;
        first_unbound_row_idx = '0;


        // default next-state = hold current
        row_is_bound_next = row_is_bound_current;
        for (int r = 0; r < NUM_ROWS; r++) begin
            bound_orig_id_next[r]  = bound_orig_id_current[r];
            available_col_next[r]  = available_col_current[r];
        end

        // clear matrix write enables (selective write on alloc)
        for (int r = 0; r < NUM_ROWS; r++) begin
            for (int c = 0; c < NUM_COLS; c++) begin
                id_matrix_we[r][c] = 1'b0;                   // GIL ERROR - you assign id_matrix_we here in combi area and later on in sequential (flop) area. 
							// Need to decide what you really meant - combi (always comb , or state (always_ff)	
            end
        end

        // scan all rows for hit / unbound
        for (int r = 0; r < NUM_ROWS; r++) begin
            //first - find the row bound to in_orig_id (if any)
            row_bound_to_in_id[r] = row_is_bound_current[r] & (~|(bound_orig_id_current[r] ^ in_orig_id));
            if (row_bound_to_in_id[r]) begin
                hit_row_idx  = ROW_W'(r);
            end

            //second - find first unbound row
            if(r == 0) begin
                first_unbound_row[0] = ~row_is_bound_current[0];
            end
            else begin
                first_unbound_row[r] = ~row_is_bound_current[r] & (&row_is_bound_current[r-1:0]);
            end

            if (first_unbound_row[r]) begin
                first_unbound_row_idx = ROW_W'(r);
            end
        end

        have_row_hit = |row_bound_to_in_id;
        have_unbound_row = |row_is_bound_current; // GIL ERROR - if judging the name - you wanted "nobody" and not "anybody" so you needed = ~&row_is_bound_current;
        chosen_row_idx = have_row_hit ? hit_row_idx : first_unbound_row_idx;
        chosen_col_idx = available_col_current[chosen_row_idx];

        // on alloc grant, update state
        if (alloc_gnt) begin
            //if row was unbound, bind it
            if (~have_row_hit) begin
                row_is_bound_next[chosen_row_idx] = 1'b1;
                bound_orig_id_next[chosen_row_idx] = in_orig_id;
            end

            // update available col pointer
            if(available_col_current[chosen_row_idx] == (NUM_COLS - 1)) begin
                available_col_next[chosen_row_idx] = '0;
            end
            else begin
                available_col_next[chosen_row_idx] = available_col_current[chosen_row_idx] + COL_W'(1);
            end

            // set id matrix write enable
            id_matrix_we[chosen_row_idx][chosen_col_idx] = 1'b1;

        end

    end




    // ---------------------------------------------------------------------
    // free logic
    // ---------------------------------------------------------------------

    // decode unique_id_to_free
    logic [ROW_W-1:0] free_row_idx;
    logic [COL_W-1:0] free_col_idx;

    assign free_row_idx = unique_id_to_free[ROW_W+COL_W-1 : COL_W];
    assign free_col_idx = unique_id_to_free[COL_W-1 : 0];
    assign restored_id = free_req ? id_matrix[free_row_idx][free_col_idx] : '0;

    always_comb begin : free_logic
        // default
        free_ack    = 1'b0; // registered in always_ff below (mirror)

        // on free request, update state
        if (free_req) begin
            free_ack = 1'b1;

            // if this was the last outstanding in the row, unbind the row
            if (used_count_current[free_row_idx] == PER_ROW_CNT_W'(1)) begin
                row_is_bound_next[free_row_idx] = 1'b0;
                bound_orig_id_next[free_row_idx] = '0;
                available_col_next[free_row_idx] = '0;
            end
        end
    end

    // -----------------------------------------------------
    //counter logic
    // -----------------------------------------------------

    always_comb begin : counter_logic
        // default next-state = hold current
        tot_used_count_next = tot_used_count_current;
        for (int r = 0; r < NUM_ROWS; r++) begin
            used_count_next[r]     = used_count_current[r];
        end

        // on alloc grant, increment counters
        if (alloc_gnt) begin
            tot_used_count_next = tot_used_count_current + TOT_REQ_CNT_W'(1);
            used_count_next[chosen_row_idx] = used_count_current[chosen_row_idx] + PER_ROW_CNT_W'(1);
        end

        // on free request, decrement counters
        if (free_req) begin
            tot_used_count_next = tot_used_count_current - TOT_REQ_CNT_W'(1);
            used_count_next[unique_id_to_free[ROW_W+COL_W-1 : COL_W]] =
            used_count_current[unique_id_to_free[ROW_W+COL_W-1 : COL_W]] - PER_ROW_CNT_W'(1);
        end
    end

    // ---------------------------------------------------------------------
    // sequential: registers update
    // ---------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            row_is_bound_current <= '0;
            tot_used_count_current <= '0;
            for (int r = 0; r < NUM_ROWS; r++) begin
                bound_orig_id_current[r] <= '0;
                available_col_current[r] <= '0;
                used_count_current[r]    <= '0;
                for (int c = 0; c < NUM_COLS; c++) begin
                    id_matrix[r][c] <= '0;
                    id_matrix_we[r][c] <= 1'b0;  // GIL ERROR - see my comment at the combi above...(one of the assignment are a bug).
                end
            end
            free_ack <= 1'b0;
        end else begin
            row_is_bound_current <= row_is_bound_next;
            tot_used_count_current <= tot_used_count_next;
            for (int r = 0; r < NUM_ROWS; r++) begin
                bound_orig_id_current[r] <= bound_orig_id_next[r];
                available_col_current[r] <= available_col_next[r];
                used_count_current[r]    <= used_count_next[r];
                for (int c = 0; c < NUM_COLS; c++) begin
                    if (id_matrix_we[r][c]) begin
                        id_matrix[r][c] <= in_orig_id; // tag write uses in_orig_id from the allocation request
                    end
                    // else leave tag_map_q as-is (don't overwrite on free)
                end
            end

            // simple free handshake mirror
            free_ack <= free_req;
        end
    end

endmodule



=========================================
OLD CODE:
=========================================
module id_allocate_and_restore_unit #(
    parameter int ID_WIDTH = 4,
    parameter int MAX_OUTSTANDING = 16,
    parameter int NUM_ROWS = $bits(MAX_OUTSTANDING),
    parameter int NUM_COLS = $bits(MAX_OUTSTANDING)
)(
    input  logic                 clk,
    input  logic                 rst, // synchronous active-high

    // alloc interface
    input  logic                 alloc_req,
    input  logic [ID_WIDTH-1:0]  in_orig_id,
    output logic                 alloc_gnt,
    output logic [ID_WIDTH-1:0]     unique_id,
    output logic                 id_matrix_full, //TODO: check if needed - if so, implement

    // free interface
    input  logic                 free_req,
    input  logic [ID_WIDTH-1:0]  unique_id_to_free,
    output logic [ID_WIDTH-1:0]  restored_id,
    output logic                 free_ack
);
    localparam int ROW_W = $bits(NUM_ROWS);
    localparam int COL_W = $bits(NUM_COLS);
    localparam int PER_ROW_CNT_W = $bits(NUM_COLS+1);
    localparam int TOT_REQ_CNT_W = $bits(MAX_OUSTSTANDING + 1);

    logic [ID_WIDTH-1:0]       id_matrix [NUM_ROWS][NUM_COLS];
    logic                      id_matrix_we [NUM_ROWS][NUM_COLS]; // write-enable for id matrix (single entry on alloc)

    // ---------------------------------------------------------------------
    // state of each row in id matrix
    // (bound means row is bound to some original ID)
    // ---------------------------------------------------------------------
    logic [NUM_ROWS-1:0]        row_is_bound_current, row_is_bound_next; // row is bound to some original ID
    logic [ID_WIDTH-1:0]        bound_orig_id_current [NUM_ROWS];     // original ID bound to each row
    logic [ID_WIDTH-1:0]        bound_orig_id_next [NUM_ROWS];

    // per-row next-available column pointer (monotonic wrap)
    logic [COL_W-1:0]           available_col_current [NUM_ROWS];
    logic [COL_W-1:0]           available_col_next [NUM_ROWS];

    //---------couters---------
    // per-row outstanding count (how many in-flight requests)
    logic [PER_ROW_CNT_W-1:0]           used_count_current [NUM_ROWS];
    logic [PER_ROW_CNT_W-1:0]           used_count_next [NUM_ROWS];
    // total outstanding requests count
    logic [TOT_REQ_CNT_W-1:0]           tot_used_count_current;
    logic [TOT_REQ_CNT_W-1:0]           tot_used_count_next;

    assign id_matrix_full = (used_count_current == TOT_REQ_CNT_W'(MAX_OUTSTANDING));

    // ---------------------------------------------------------------------
    // row selection logic
    // ---------------------------------------------------------------------

    // row and col selection signals
    logic             have_row_hit; // does any row match in_orig_id
    logic [ROW_W-1:0] hit_row_idx;      // index of matching row (if any)
    logic [NUM_ROWS-1:0] row_bound_to_in_id; // index of row bound to in_orig_id is high
    logic             have_unbound_row; // is there any unbound row
    logic [ROW_W-1:0] first_unbound_row_idx; // index of first unbound row (if any)
    logic [NUM_ROWS-1:0] first_unbound_row; // index of first unbound row is high
    logic [ROW_W-1:0] chosen_row_idx;
    logic [COL_W-1:0] chosen_col_idx;

        
    // allocation signals
    assign alloc_gnt = alloc_req & (have_row_hit | have_unbound_row) & ~id_matrix_full;
    assign unique_id = {(ID_WIDTH-ROW_W-COL_W){1'b0},chosen_row_idx, chosen_col_idx};//unique_id is zero-padded to ID_WIDTH

    // slot selection combinational logic
    always_comb begin : slot_selection_logic
        // defaults
        have_row_hit        = 1'b0;
        hit_row_idx         = '0;
        have_unbound_row    = 1'b0;
        first_unbound_row_idx = '0;


        // default next-state = hold current
        row_is_bound_next = row_is_bound_current;
        for (int r = 0; r < NUM_ROWS; r++) begin
            bound_orig_id_next[r]  = bound_orig_id_current[r];
            available_col_next[r]  = available_col_current[r];
        end

        // clear matrix write enables (selective write on alloc)
        for (int r = 0; r < NUM_ROWS; r++) begin
            for (int c = 0; c < NUM_COLS; c++) begin
                id_matrix_we[r][c] = 1'b0;
            end
        end

        // scan all rows for hit / unbound
        for (int r = 0; r < NUM_ROWS; r++) begin
            //first - find the row bound to in_orig_id (if any)
            row_bound_to_in_id[r] = row_is_bound_current[r] & (~|(bound_orig_id_current[r] ^ in_orig_id));
            if (row_bound_to_in_id[r]) begin
                hit_row_idx  = ROW_W'(r);
            end

            //second - find first unbound row
            if(r == 0) begin
                first_unbound_row[0] = ~row_is_bound_current[0];
            end
            else begin
                first_unbound_row[r] = ~row_is_bound_current[r] & (&row_is_bound_current[r-1:0]);
            end

            if (first_unbound_row[r]) begin
                first_unbound_row_idx = ROW_W'(r);
            end
        end

        have_row_hit = |row_bound_to_in_id;
        have_unbound_row = |row_is_bound_current;
        chosen_row_idx = have_row_hit ? hit_row_idx : first_unbound_row_idx;
        chosen_col_idx = available_col_current[chosen_row_idx];

        // on alloc grant, update state
        if (alloc_gnt) begin
            //if row was unbound, bind it
            if (~have_row_hit) begin
                row_is_bound_next[chosen_row_idx] = 1'b1;
                bound_orig_id_next[chosen_row_idx] = in_orig_id;
            end

            // update available col pointer
            if(available_col_current[chosen_row_idx] == (NUM_COLS - 1)) begin
                available_col_next[chosen_row_idx] = '0;
            end
            else begin
                available_col_next[chosen_row_idx] = available_col_current[chosen_row_idx] + COL_W'(1);
            end

            // set id matrix write enable
            id_matrix_we[chosen_row_idx][chosen_col_idx] = 1'b1;

        end

    end




    // ---------------------------------------------------------------------
    // free logic
    // ---------------------------------------------------------------------

    // decode unique_id_to_free
    logic [ROW_W-1:0] free_row_idx;
    logic [COL_W-1:0] free_col_idx;

    assign free_row_idx = unique_id_to_free[ROW_W+COL_W-1 : COL_W];
    assign free_col_idx = unique_id_to_free[COL_W-1 : 0];
    assign restored_id = free_req ? id_matrix[free_row_idx][free_col_idx] : '0;

    always_comb begin : free_logic
        // default
        free_ack    = 1'b0; // registered in always_ff below (mirror)

        // on free request, update state
        if (free_req) begin
            free_ack = 1'b1;

            // if this was the last outstanding in the row, unbind the row
            if (used_count_current[free_row_idx] == PER_ROW_CNT_W'(1)) begin
                row_is_bound_next[free_row_idx] = 1'b0;
                bound_orig_id_next[free_row_idx] = '0;
                available_col_next[free_row_idx] = '0;
            end
        end
    end

    // -----------------------------------------------------
    //counter logic
    // -----------------------------------------------------

    always_comb begin : counter_logic
        // default next-state = hold current
        tot_used_count_next = tot_used_count_current;
        for (int r = 0; r < NUM_ROWS; r++) begin
            used_count_next[r]     = used_count_current[r];
        end

        // on alloc grant, increment counters
        if (alloc_gnt) begin
            tot_used_count_next = tot_used_count_current + TOT_REQ_CNT_W'(1);
            used_count_next[chosen_row_idx] = used_count_current[chosen_row_idx] + PER_ROW_CNT_W'(1);
        end

        // on free request, decrement counters
        if (free_req) begin
            tot_used_count_next = tot_used_count_current - TOT_REQ_CNT_W'(1);
            used_count_next[unique_id_to_free[ROW_W+COL_W-1 : COL_W]] =
            used_count_current[unique_id_to_free[ROW_W+COL_W-1 : COL_W]] - PER_ROW_CNT_W'(1);
        end
    end

    // ---------------------------------------------------------------------
    // sequential: registers update
    // ---------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            row_is_bound_current <= '0;
            tot_used_count_current <= '0;
            for (int r = 0; r < NUM_ROWS; r++) begin
                bound_orig_id_current[r] <= '0;
                available_col_current[r] <= '0;
                used_count_current[r]    <= '0;
                for (int c = 0; c < NUM_COLS; c++) begin
                    id_matrix[r][c] <= '0;
                    id_matrix_we[r][c] <= 1'b0;
                end
            end
            free_ack <= 1'b0;
        end else begin
            row_is_bound_current <= row_is_bound_next;
            tot_used_count_current <= tot_used_count_next;
            for (int r = 0; r < NUM_ROWS; r++) begin
                bound_orig_id_current[r] <= bound_orig_id_next[r];
                available_col_current[r] <= available_col_next[r];
                used_count_current[r]    <= used_count_next[r];
                for (int c = 0; c < NUM_COLS; c++) begin
                    if (id_matrix_we[r][c]) begin
                        id_matrix[r][c] <= in_orig_id; // tag write uses in_orig_id from the allocation request
                    end
                    // else leave tag_map_q as-is (don't overwrite on free)
                end
            end

            // simple free handshake mirror
            free_ack <= free_req;
        end
    end

endmodule