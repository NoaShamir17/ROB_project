module r_ordering_unit #(
    parameter int ID_WIDTH         = 4,   // original ID width
    parameter int MAX_OUTSTANDING  = 16,
    parameter int NUM_ROWS = MAX_OUTSTANDING,
    parameter int NUM_COLS = MAX_OUTSTANDING,
    parameter int MAX_LEN = 8, //max beats per response
)(
    input  logic clk,
    input  logic rst,

    // ---------------- Incoming response ----------------
    r_if.receiver                r_in,          // incoming response from incoming response buffer

    // ---------------- Outgoing response ----------------
    r_if.sender                  r_out,         // outgoing ordered response to outgoing response buffer
   
    // ---------------- Allocator interaction ----------------
    output logic                 allocator_free_req,
    output logic [ID_WIDTH-1:0]  uid_to_restore,
    input logic [ID_WIDTH-1:0]  restored_id,   // allocator translates {row,col}→orig_id


    // ---------------- Response memory (rm) interface ----------------
    r_if.sender                  r_store, //id is unique_id
                                            //valid  means request to store data
                                            //ready means memory can accept data

    output logic [ID_WIDTH-1:0]  rm_release_uid,
    r_if.receiver                r_release, // data returned when released
                                                  // valid means data from memory is valid
                                                  //ready means request to release data

);

    // =========================================================================
    // Internal state
    // =========================================================================

    // Release pointers per row (wrap-around COL_W bits)

    localparam int ROW_W = $clog2(NUM_ROWS);
    localparam int COL_W = $clog2(NUM_COLS);
    localparam int BEAT_CNT_W = $clog2(MAX_LEN);
    logic [COL_W-1:0] release_idx [NUM_ROWS];

    // Bookkeeping: which {row,col} are waiting in rm
    logic waiting_map [NUM_ROWS][NUM_COLS];
    logic [BEAT_CNT_W-1:0] waiting_beats_count [NUM_ROWS][NUM_COLS];

    genvar row, col;
    generate
        for (row = 0; row < NUM_ROWS; row = row + 1) begin : gen_waiting_map_row
            for (col = 0; col < NUM_COLS; col = col + 1) begin : gen_waiting_map_col
                waiting_map[row][col] = |(waiting_beats_count[row][col] ^ {BEAT_CNT_W{1'b0}}); // high when beats count is non-zero
            end
        end
    endgenerate


    // Decode uid
    logic [ROW_W-1:0] resp_row = r_in.id[ID_WIDTH-1:COL_W];
    logic [COL_W-1:0] resp_col = r_in.id[COL_W-1:0];

    // =========================================================================
    // Arbitration logic - direct or memory hit
    // direct hit = incoming response can be sent directly out
    // rm hit = some response is waiting in memory and can be sent out
    // =========================================================================

    logic direct_hit;   // freshly arrived response is in-order
    logic rm_hit;       // some waiting response is in-order
    logic [ID_WIDTH-1:0] rm_hit_uid; // first waiting candidate (priority encode)
    logic [NUM_ROWS-1:0] rm_hit_vec; // one-hot per row if that row has a hit
    logic [NUM_ROWS-1:0] rm_first_row; // vector marking first row with a hit (only one bit set to 1)
    logic [ID_WIDTH-1:0] rm_uid [NUM_ROWS-1:0]; // uid per row for hit checking
    
    
    //----------direct hit logic----------
    // direct hit if resp_col matches release_idx of that row
    // AND if not waiting in rm (no earlier beats pending)
    assign direct_hit = r_in.valid & 
                         ~|(resp_col ^ release_idx[resp_row]) &
                         ~waiting_map[resp_row][resp_col]; // direct hit if resp_col matches release_idx of that row
    

    //----------waiting memory hit logic----------
    // Priority-encode first row with a hit
    always_comb begin
        // defaults
        rm_hit_uid = '0;

        for (r = 0; r < NUM_ROWS; r++) begin : gen_rm_hit_vec
            rm_hit_vec[r] = waiting_map[r][release_idx[r]]; // high when that row has a hit

            //first row logic
            if (r == 0) begin
                rm_first_row[0] = rm_hit_vec[0];
            end
            else begin
                //high when this row has a hit and no prior row has a hit
                rm_first_row[r] = rm_hit_vec[r] & (~|rm_hit_vec[r-1:0]);
            end

            //uid per row
            rm_uid[r] = {r[ROW_W-1:0], release_idx[r]};

            //choose the uid from the first row with a hit
            rm_hit_uid = rm_hit_uid | (rm_uid[r] & {ID_WIDTH{rm_first_row[r]}});

        end 
    end

    assign rm_hit = |rm_hit_vec;
    logic [ROW_W-1:0] rm_row = rm_hit_uid[ID_WIDTH-1:COL_W];
    logic [COL_W-1:0] rm_col = rm_hit_uid[COL_W-1:0];
                    

    // =========================================================================
    // Release and store path
    // =========================================================================

    //-----handshake signals-----
    logic hs_in = r_in.valid & r_in.ready;
    logic hs_out = r_out.valid & r_out.ready;
    logic hs_store = r_store.valid & r_store.ready;
    logic hs_release = r_release.valid & r_release.ready;

    //-----allocator free request-----
    assign allocator_free_req = hs_out & r_out.last; // free when last beat is sent out

    always_comb begin : data_path
        //------------ Default assignments ----------------

        // handshake signals
        r_in.ready = 1'b0;
        r_out.valid = 1'b0;
        r_store.valid = 1'b0;
        r_release.ready = 1'b0;

        // allocator signals
        uid_to_restore     = '0;

        // rm release signals
        rm_release_uid = '0;

        // response output signals
        // default to X so simulation can catch incorrect use (x-propagation)
        r_out.id        = 'x;               // sized to ID_WIDTH, all bits X
        r_out.data      = 'x;               // sized to data width, all bits X
        r_out.resp      = 'x;               // sized to resp width, all bits X
        r_out.last      = 1'bx;             // single-bit X

        // response memory store signals
        r_store.id      = 'x;               // sized to ID_WIDTH
        r_store.data    = 'x;
        r_store.resp    = 'x;
        r_store.last    = 1'bx;
        //----------------------------------------------------------

        // ---------------- Release logic ---------------------
        // -------- Priority: direct release > rm release ----------
        if (direct_hit) begin
            // Direct hit - release immediately

            //handshake signals
            r_in.ready    = r_out.ready; // ready when output is ready
            r_out.valid   = r_in.valid;  // valid when input is valid

            //allocator signals
            uid_to_restore = r_in.id;
            
            // response output signals
            r_out.id        = restored_id;
            r_out.data      = r_in.data;
            r_out.resp      = r_in.resp;
            r_out.last      = r_in.last;

        end else begin
            // No direct hit - store in response memory

            // handshake signals
            r_in.ready    = r_store.ready; // ready when store is ready
            r_store.valid = r_in.valid;   // valid when input is valid

            // response memory store signals
            r_store.id      = r_in.id;
            r_store.data    = r_in.data;
            r_store.resp    = r_in.resp;
            r_store.last    = r_in.last;

            if (rm_hit) begin
                // rm hit - release from response memory

                //handshake signals
                r_release.ready = r_out.ready; // ready when output is ready
                r_out.valid     = r_release.valid; // valid when release data is valid

                // response memory release signals
                rm_release_uid = rm_hit_uid;

                // allocator signals
                uid_to_restore = rm_hit_uid;
            
                // response output signals
                r_out.id        = restored_id;
                r_out.data      = r_release.data;
                r_out.resp      = r_release.resp;
                r_out.last      = r_release.last;

             end
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
                    waiting_beats_count[r][c] <= '0;
            end
        end else begin
            if (hs_out) begin
                //handshake on output - update pointers and waiting map
                if (direct_hit & hs_in) begin
                    // Increment release pointer for that row 
                    // wrap-around handled by bit-width
                    release_idx[resp_row] <= release_idx[resp_row] + 1'b1;
                end
                else if (rm_hit & hs_release) begin
                    // Increment release pointer for that row 
                    release_idx[rm_row] <= release_idx[rm_row] + 1'b1;
                    // Decrement waiting beats count for that entry
                    waiting_beats_count[resp_row][resp_col] <= waiting_beats_count[resp_row][resp_col] - 1'b1;
                end
            end

            // Mark waiting entry on store
            if (hs_store) begin
                // Increment waiting beats count for that entry
                waiting_beats_count[resp_row][resp_col] <= waiting_beats_count[resp_row][resp_col] + 1'b1;
            end
        end
    end

endmodule
