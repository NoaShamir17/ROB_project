// ============================================================================
// response_memory.sv
// ----------------------------------------------------------------------------
// ROLE
//   • Per-UID storage of R beats.
//   • r_in writes beats into FIFO[r_in.id].
//   • Start-of-burst for a UID is detected internally when that UID's
//     beat_cnt == 0 and we accept a beat. alloc_req/uid_to_alloc are kept
//     as inputs for compatibility / debug but are not relied on here,
//     because in your system alloc_req = r_store.valid (every beat).
//   • free_req/uid_to_free + r_out provide a *combinational* view of the
//     head beat of FIFO[uid_to_free].
//   • free_ack goes high whenever there is a valid beat for this UID
//     (free_ack = r_out.valid). The actual pop still happens on
//     (r_out.valid & r_out.ready) inside the sequential logic.
// ============================================================================

module response_memory #(
    parameter int NUM_UIDS   = 16,
    parameter int MAX_BEATS  = 8,
    parameter int ID_WIDTH   = 4,
    parameter int DATA_WIDTH = 64,
    parameter int RESP_WIDTH = 2
)(
    input  logic clk,
    input  logic rst_n,

    // Incoming beats from fabric
    r_if.receiver r_in,

    // Outgoing beats to ordering/master
    r_if.sender   r_out,

    // Control: start of burst for some UID
    // NOTE: In your system these are wired from r_store:
    //   alloc_req    = r_store.valid
    //   uid_to_alloc = r_store.id
    // We do NOT use them for the actual "new burst" decision (see below).
    //input  logic [ID_WIDTH-1:0] uid_to_alloc,
    //input  logic                alloc_req,

    // Control: pop one beat for some UID
    // Typically:
    //   free_req    = r_release.ready
    //   uid_to_free = rm_release_uid
    input  logic [ID_WIDTH-1:0] uid_to_free
    //input  logic                free_req,

    // Pop indication to r_ordering_unit
    // Architecturally: "there is a valid beat for this UID"
    //output logic                free_ack
);

    // --------------------------------------------------------------------
    // Local params and state
    // --------------------------------------------------------------------
    localparam int CNT_W = $clog2(MAX_BEATS + 1);

    // data_mem / resp_mem / last_mem indexed by [uid][index]
    logic [DATA_WIDTH-1:0] data_mem [NUM_UIDS][MAX_BEATS];
    logic [RESP_WIDTH-1:0] resp_mem [NUM_UIDS][MAX_BEATS];
    logic                  last_mem [NUM_UIDS][MAX_BEATS];

    // FIFO pointers and counters per UID
    logic [CNT_W-1:0] wptr     [NUM_UIDS];
    logic [CNT_W-1:0] rptr     [NUM_UIDS];
    logic [CNT_W-1:0] beat_cnt [NUM_UIDS];

    // Derived empty/full per UID
    logic fifo_empty [NUM_UIDS];
    logic fifo_full  [NUM_UIDS];

    genvar g;
    generate
        for (g = 0; g < NUM_UIDS; g = g + 1) begin : GEN_FLAGS
            always_comb begin
                fifo_empty[g] = (beat_cnt[g] == {CNT_W{1'b0}});
                fifo_full[g]  = (beat_cnt[g] == MAX_BEATS[CNT_W-1:0]);
            end
        end
    endgenerate

    // Convenience
    wire [ID_WIDTH-1:0] in_uid = r_in.id;

    // --------------------------------------------------------------------
    // WRITE SIDE: r_in.valid / r_in.ready
    // --------------------------------------------------------------------
    // Policy:
    //   • Allow a beat as long as its FIFO is not full.
    //   • "New burst" for a UID is detected when beat_cnt[in_uid] == 0
    //     and we accept a beat. We reset that UID's FIFO pointers then.
    //     This avoids relying on alloc_req, which is 1 for every beat
    //     in your mapping (r_store.valid).
    // --------------------------------------------------------------------

    always_comb begin
        r_in.ready = 1'b1;
        if (r_in.valid && fifo_full[in_uid]) begin
            r_in.ready = 1'b0;
        end
    end

    wire accept_in = r_in.valid & r_in.ready;

    // --------------------------------------------------------------------
    // READ / FREE SIDE: free_req + uid_to_free → r_out + free_ack
    // --------------------------------------------------------------------
    // Combinational view of the head of FIFO[uid_to_free]:
    //   can_pop      = FIFO not empty
    //   r_out.valid  = free_req & can_pop
    //   r_out.*      = mem[uid_to_free][rptr[uid_to_free]]
    //
    // According to your spec:
    //   free_ack = r_out.valid
    // i.e. "there is a valid beat available for that UID".
    // The actual pop (advancing rptr / decrementing beat_cnt) still happens
    // only when valid & ready are both 1.
    // --------------------------------------------------------------------

    wire can_pop = ~fifo_empty[uid_to_free];

    always_comb begin
        r_out.valid = can_pop; //free_req & can_pop;
        r_out.id    = uid_to_free;

        if (r_out.valid) begin
            r_out.data = data_mem[uid_to_free][rptr[uid_to_free]];
            r_out.resp = resp_mem[uid_to_free][rptr[uid_to_free]];
            r_out.last = last_mem[uid_to_free][rptr[uid_to_free]];
        end
        else begin
            r_out.data = {DATA_WIDTH{1'b0}};
            r_out.resp = {RESP_WIDTH{1'b0}};
            r_out.last = 1'b0;
        end
    end

    // free_ack now reflects "there is valid data" for this UID
    //assign free_ack = r_out.valid;

    // --------------------------------------------------------------------
    // SEQUENTIAL: commit writes and pops
    // --------------------------------------------------------------------
    integer i;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NUM_UIDS; i = i + 1) begin
                wptr[i]     <= {CNT_W{1'b0}};
                rptr[i]     <= {CNT_W{1'b0}};
                beat_cnt[i] <= {CNT_W{1'b0}};
            end
        end
        else begin
            // ---------------- WRITE PATH ----------------
            if (accept_in) begin
                // NEW BURST detection:
                // If this UID currently has zero beats stored, this is the
                // first beat of a new burst – reset pointers before writing.
                if (beat_cnt[in_uid] == {CNT_W{1'b0}}) begin
                    wptr[in_uid]     <= {CNT_W{1'b0}};
                    rptr[in_uid]     <= {CNT_W{1'b0}};
                    // beat_cnt[in_uid] already 0 here
                end

                // Write data/resp/last at current wptr
                data_mem[in_uid][wptr[in_uid]] <= r_in.data;
                resp_mem[in_uid][wptr[in_uid]] <= r_in.resp;
                last_mem[in_uid][wptr[in_uid]] <= r_in.last;

                // Advance write pointer (wrap)
                if (wptr[in_uid] == (MAX_BEATS[CNT_W-1:0] - {{(CNT_W-1){1'b0}}, 1'b1})) begin
                    wptr[in_uid] <= {CNT_W{1'b0}};
                end
                else begin
                    wptr[in_uid] <= wptr[in_uid] + {{(CNT_W-1){1'b0}}, 1'b1};
                end

                // Increment count
                beat_cnt[in_uid] <= beat_cnt[in_uid] + {{(CNT_W-1){1'b0}}, 1'b1};
            end

            // ---------------- POP PATH -------------------
            // Pop happens on real handshake: valid & ready
            if (r_out.valid && r_out.ready) begin
                // Pop from FIFO[uid_to_free]

                // Advance read pointer (wrap)
                if (rptr[uid_to_free] == (MAX_BEATS[CNT_W-1:0] - {{(CNT_W-1){1'b0}}, 1'b1})) begin
                    rptr[uid_to_free] <= {CNT_W{1'b0}};
                end
                else begin
                    rptr[uid_to_free] <= rptr[uid_to_free] + {{(CNT_W-1){1'b0}}, 1'b1};
                end

                // Decrement count
                beat_cnt[uid_to_free] <= beat_cnt[uid_to_free] - {{(CNT_W-1){1'b0}}, 1'b1};
            end
        end
    end

endmodule
