module r_reodering_unit_FSM #(
    parameter int ID_WIDTH = 4,
    parameter int NUM_ROWS = MAX_OUTSTANDING,
    parameter int NUM_COLS = MAX_OUTSTANDING
)(
    input  logic clk,
    input  logic rst,

    // ---------------- Incoming response ----------------
    r_if.receiver                r_in,          // metadata of incoming response

    // ---------------- Outgoing response ----------------
    r_if.sender                  r_out, 
    
// ---------------- Allocator interaction ----------------
    output logic                 allocator_free_req,
    output logic [ID_WIDTH-1:0]  allocator_free_uid,
    input logic                 allocator_free_ack,
    input logic [ID_WIDTH-1:0]  restored_id,   // allocator translates {row,col}→orig_id

    // ---------------- Response memory (rm) interface ----------------
    //output logic                 rm_store_req,   // store out-of-order response
    //input logic                  rm_store_done,    // request successfully stored
    r_if.sender                  r_store, //id is unique_id
                                            //valid is store_req (request to store data)
                                            //ready is store_ack (memory can accept data)

    //output logic                 rm_release_en, // request release of stored response
    output logic [ID_WIDTH-1:0]  rm_release_uid,
    r_if.receiver                r_release, // data returned when released
                                                  // valid means data from memory is valid
                                                  //ready is release_req (request to release data)

    input  logic                 rm_full        // metadata to master (ordered)
);


    // =========================================================================
    // Internal state
    // =========================================================================

    // Release pointers per row (wrap-around COL_W bits)

    localparam int ROW_W = $bits(NUM_ROWS);
    localparam int COL_W = $bits(NUM_COLS);
    logic [COL_W-1:0] release_idx [NUM_ROWS];

    // Bookkeeping: which {row,col} are waiting in wm
    logic waiting_map [NUM_ROWS][NUM_COLS];

    // Decode uid
    wire [ROW_W-1:0] resp_row = r_in.id[ID_WIDTH-1:COL_W];
    wire [COL_W-1:0] resp_col = r_in.id[COL_W-1:0];


    //---------handshakes----------
    logic in_hs = r_in.valid & r_in.ready; // incoming response accepted
    logic out_hs = r_out.valid & r_out.ready; // outgoing response accepted
    logic release_hs = r_release.valid & r_release.ready; // released response accepted
    logic store_hs = r_store.valid & r_store.ready; // store request accepted

    typedef enum logic [1:0] {
        ST_IDLE,  
        ST_ISSUE
    } state_t;


    state_t state_current, state_next;

  // ==========================================================================
  // LATCHED FIELDS (captured on r_in handshake)
  // ==========================================================================

    logic [ID_WIDTH-1:0]   id_q;                 // transaction ID for this data beat
    logic [DATA_WIDTH-1:0] data_q;               // actual data payload being transferred
    logic [RESP_WIDTH-1:0] resp_q;               // response code (OKAY, SLVERR, etc.)
    logic                  last_q;               // high means this is the last beat in a burst
    logic [TAG_WIDTH-1:0]  tagid_q;              // internal tag used for tracking/reordering


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


    //----------direct hit logic----------
    assign direct_hit = r_in.valid & (~|(resp_col ^ release_idx[resp_row]));

    //----------waiting memory hit logic----------
    alway_comb begin : memory_hit_logic
        //defaults
        rm_hit_uid = '0;
        
        for (r = 0; r < NUM_ROWS; r++) begin
            rm_hit_vec[r] = waiting_map[r][release_idx[r]];

            //choose the first row with a hit
            if (r == 0) begin
                rm_first_row[0] = rm_hit_vec[0];
            end
            else begin
                rm_first_row[r] = rm_hit_vec[r] & (~|rm_hit_vec[r-1:0]);
            end

            if (rm_first_row[r]) begin
                // only one bit is set in wm_first_row
                // so we can use it to get the first row with a hit
                rm_hit_uid = {(ID_WIDTH-ROW_W-COL_W){1'b0},ROW_W'(r), release_idx[r]};
            end
        end
    end
    assign rm_hit = |rm_hit_vec;

    always_comb begin : state_logic
        // Default assignments
        state_next = state_current;

        //handshake signals
        r_in.ready = 1'b0;
        r_out.valid = 1'b0;
        r_store.valid = 1'b0;
        r_release.ready = 1'b0;

        //alloactor signals
        allocator_free_req = 1'b0;
        allocator_free_uid = '0;

        //rm signals
        rm_release_uid = '0;


        case (state_current)
            ST_IDLE: begin
                if(r_in.valid) begin
                    if (direct_hit) begin
                        // Direct hit - forward response immediately
                        r_out.valid = 1'b1;
                        r_in.ready = r_out.ready;

                        if (out_hs) begin
                            // Update release index for the row
                            release_idx[resp_row] = release_idx[resp_row] + 1;

                            // If last beat, free the UID in allocator
                            if (r_in.last) begin
                                allocator_free_req = 1'b1;
                                allocator_free_uid = r_in.id;
                            end
                        end
                    end
                    else begin
                        // No direct hit - store in memory
                        r_store.valid = 1'b1;
                        r_in.ready = r_store.ready;

                        if (store_hs) begin
                            // Mark the {row,col} as occupied in waiting map
                            waiting_map[resp_row][resp_col] = 1'b1;
                        end
                    end
                end
            end

            ST_READY: begin
                if (r_in.valid & r_in.ready) begin
                    state_next = ST_STORE;
                end
            end

            ST_STORE: begin
                // After storing, go to RELEASE state
                state_next = ST_RELEASE;
            end

            ST_RELEASE: begin
                if (r_release.valid) begin
                    // After releasing, go back to IDLE state
                    state_next = ST_IDLE;
                end
            end

            default: begin
                state_next = ST_IDLE;
            end
        endcase
    end


    // -----------Forward data to output and store in memory-----------
    assign r_out.id    = id_q
    assign r_out.data  = data_q;
    assign r_out.resp  = resp_q;
    assign r_out.last  = last_q;
    assign r_out.tagid = tagid_q;

    assign r_store.id    = id_q;
    assign r_store.data  = data_q;
    assign r_store.resp  = resp_q;
    assign r_store.last  = last_q;
    assign r_store.tagid = tagid_q;
endmodule