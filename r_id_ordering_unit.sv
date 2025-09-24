
//~~~~~~~~~~~~~~~~Noa's notes~~~~~~~~~~~~~~~~~~~~~~
/*
i need a module named r_id_ordering_unit that tracks the order of respones to release, and also implemets the release itself.
the order tracking is done as follows:
each row has a variable named release_idx, which is the index of the column that
is next in line to be released.
when a response comes in, if its column index matches the release_idx of its row, 
it is released and the release_idx of that row is incremented (wraps around to 0 if needed).
if it does not match, it is held until it matches. 
each cycle the module checks if any of the storesd responses can be released.
when a response is released, a free request is sent to the allocator with the {row,col} of that transaction.

when a response is set to wait until it's relese, it's held in a different module which serves as
a memory unit for the respy687onses that are waiting to be released. it is limited to a certain depth, marked by the parameter MAX_OUSTSTANDING.
this modules name is r_response_waiting_memory.
the inputs are the metadata of the response to be stored (row,col,original_id,data,resp,last,tagid) and a write enable signal.
another input is the id (either the original or the unique id) of the response to be released, and a release enable signal.
the outputs are the metadata of the response to be released, and a signal indicating if the memory is full (to prevent writing when full).

in each cycle, at most one response can be released (if multiple are ready, only one is released and the rest wait).


**a bug can occur if for example, a response of original id "A" has come, but it needs to wait until the request before it (also of original id "A") arrives.
now lets assume that in the next cycle, the needed response arrives, so it can be released on this 
cycle, and then in the next cycle when the second response can be released, another response of 
original id "B" for example arrives and can also be released immediately. so we need to choose 
who to release first.

suggestion: we can add an FSM with different states, each state handles a different amount of requests ready to be released.


*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ============================================================================
// r_id_ordering_unit
// ----------------------------------------------------------------------------
// Purpose:
//   - Enforce in-order release of responses per ORIGINAL ID row
//   - Allocate unique IDs {row,col} via allocator_tag_map
//   - If response arrives out-of-order, store in r_response_waiting_memory
//   - Each cycle, release at most one response (direct > waiting)
// ----------------------------------------------------------------------------
module r_id_ordering_unit #(
    parameter int ID_WIDTH         = 4,   // original ID width
    parameter int MAX_OUTSTANDING  = 16,
    parameter int NUM_ROWS = MAX_OUTSTANDING,
    parameter int NUM_COLS = MAX_OUTSTANDING,
    parameter int UID_W = $bit(NUM_ROWS) + $bit(NUM_COLS)

)(
    input  logic clk,
    input  logic rst,

    // ---------------- Incoming response ----------------
    input  logic                 resp_valid,
    input  logic [UID_W-1:0]     resp_uid,      // {row,col} from allocator
    r_if.receiver                r_in,          // metadata of incoming response

    // ---------------- Outgoing response ----------------
    output logic                 release_valid,
    r_if.sender                  r_out,         // metadata to master (ordered)

    // ---------------- Waiting memory interface ----------------
    output logic                 wm_write_en,   // store out-of-order response
    output logic [UID_W-1:0]     wm_write_uid,
    r_if.sender                  wm_write_data,

    output logic                 wm_release_en, // request release of stored response
    output logic [UID_W-1:0]     wm_release_uid,
    r_if.receiver                wm_release_data, // data returned when released

    input  logic                 wm_full        // block writes when full
);

    // =========================================================================
    // Internal state
    // =========================================================================

    // Release pointers per row (wrap-around COL_W bits)

    localparam int ROW_W = $clog2(NUM_ROWS);
    localparam int COL_W = $clog2(NUM_COLS);
    logic [COL_W-1:0] release_idx [NUM_ROWS];

    // Bookkeeping: which {row,col} are waiting in wm
    logic waiting_map [NUM_ROWS][NUM_COLS];

    // Decode uid
    wire [ROW_W-1:0] resp_row = resp_uid[UID_W-1:COL_W];
    wire [COL_W-1:0] resp_col = resp_uid[COL_W-1:0];


    // ---------------- Allocator interaction ----------------
    logic                 free_req,
    logic [UID_W-1:0]     free_uid,
    logic [ID_WIDTH-1:0]  restored_id,   // allocator translates {row,col}→orig_id


    // =========================================================================
    // Arbitration logic
    // =========================================================================

    logic direct_hit;   // freshly arrived response is in-order
    logic wm_hit;       // some waiting response is in-order
    logic [UID_W-1:0] wm_hit_uid; // first waiting candidate (priority encode)
    logic [NUM_ROWS-1:0] wm_hit_vec; // one-hot per row if that row has a hit
    logic [NUM_ROWS-1:0] wm_first_row; // vector marking first row with a hit (only one bit set to 1)

    //check if direct hit
    assign direct_hit = resp_valid & ~|(resp_col ^ release_idx[resp_row]);

    //check if waiting memory has a hit
    // Priority-encode first row with a hit
    always_comb begin
        // defaults
        wm_hit_uid = '0;

        for (r = 0; r < NUM_ROWS; r++) begin : GEN_WM_HIT_VEC
            wm_hit_vec[r] = waiting_map[r][release_idx[r]];
            wm_first_row[r] = wm_hit_vec[r] & ~|wm_hit_vec[r-1:0];
            if (wm_first_row[r]) begin
                // only one bit is set in wm_first_row
                // so we can use it to get the first row with a hit
                wm_hit_uid = {r[ROW_W-1:0], release_idx[r]};
            end
        end 
    end

    assign wm_hit = |wm_hit_vec;

    
   

    // =========================================================================
    // Release path
    // =========================================================================

    always_comb begin
        // Defaults
        release_valid   = 1'b0;
        r_out.valid     = 1'b0;
        r_out.id        = '0;
        r_out.data      = '0;
        r_out.resp      = '0;
        r_out.last      = 1'b0;
        r_out.tagid     = '0;
        free_req        = 1'b0;
        free_uid        = '0;
        wm_write_en     = 1'b0;
        wm_write_uid    = '0;
        wm_release_en   = 1'b0;
        wm_release_uid  = '0;

        // -------- Priority: direct release > wm release --------
        if (direct_hit) begin
            release_valid   = 1'b1;
            r_out.valid     = 1'b1;
            r_out.id        = restored_id; // TODO: take this from allocator

            r_out.data      = r_in.data;
            r_out.resp      = r_in.resp;
            r_out.last      = r_in.last;
            r_out.tagid     = r_in.tagid;
            free_req        = 1'b1; //TODO: input to allocator
            free_uid        = resp_uid; //TODO: input to allocator

        end else if (wm_hit) begin
            release_valid   = 1'b1;
            r_out.valid     = 1'b1;
            r_out.id        = wm_release_data.id;
            r_out.data      = wm_release_data.data;
            r_out.resp      = wm_release_data.resp;
            r_out.last      = wm_release_data.last;
            r_out.tagid     = wm_release_data.tagid;
            free_req        = 1'b1;
            free_uid        = wm_hit_uid;
            wm_release_en   = 1'b1;
            wm_release_uid  = wm_hit_uid;

        end else if (resp_valid && !direct_hit && !wm_full) begin
            // Out-of-order arrival → store in waiting memory
            wm_write_en  = 1'b1;
            wm_write_uid = resp_uid;
            wm_write_data.valid = 1'b1;
            wm_write_data.id    = r_in.id;
            wm_write_data.data  = r_in.data;
            wm_write_data.resp  = r_in.resp;
            wm_write_data.last  = r_in.last;
            wm_write_data.tagid = r_in.tagid;
        end
    end

    // =========================================================================
    // Sequential state update
    // =========================================================================

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int r = 0; r < NUM_ROWS; r++) begin
                release_idx[r] <= '0;
                for (int c = 0; c < NUM_COLS; c++)
                    waiting_map[r][c] <= 1'b0;
            end
        end else begin
            // Direct release pointer update
            if (direct_hit) begin
                if (release_idx[resp_row] == COL_W'(NUM_COLS-1))
                    release_idx[resp_row] <= '0;
                else
                    release_idx[resp_row] <= release_idx[resp_row] + 1'b1;
            end
            // WM release pointer update
            else if (wm_hit) begin
                logic [ROW_W-1:0] wm_row = wm_hit_uid[UID_W-1:COL_W];
                logic [COL_W-1:0] wm_col = wm_hit_uid[COL_W-1:0];
                waiting_map[wm_row][wm_col] <= 1'b0;
                if (release_idx[wm_row] == COL_W'(NUM_COLS-1))
                    release_idx[wm_row] <= '0;
                else
                    release_idx[wm_row] <= release_idx[wm_row] + 1'b1;
            end
            // Mark waiting entry on store
            if (wm_write_en) begin
                waiting_map[resp_row][resp_col] <= 1'b1;
            end
        end
    end

endmodule
