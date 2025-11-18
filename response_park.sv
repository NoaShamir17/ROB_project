// ============================================================================
// response_park.sv
// ----------------------------------------------------------------------------
// ROLE
//   Per-UID parking lot for AXI-R beats, implemented as a set of small FIFOs:
//   • Each UID has its own FIFO (depth = MAX_BEATS).
//   • We can WRITE new beats for any UID while READing (draining) that UID.
//   • Completion of a UID is defined by: seen RLAST for that UID AND its FIFO
//     has been fully drained (count == 0).
//   • When a UID completes, we pulse uid_freed_valid + uid_freed_uid so the
//     allocator / ordering unit can recycle the UID.
//
// INTERFACES
//   - r_in  : r_if.receiver
//            * id   = UID (internal tag)
//            * data = RDATA from slave fabric
//            * resp = RRESP
//            * last = RLAST
//   - r_out : r_if.sender
//            * data / resp / last streamed towards r_id_ordering_unit
//            * id   = active UID (ordering unit will map UID → original ID)
//
// CONTROL
//   - drain_start, drain_uid: select which UID to start draining.
//   - uid_freed_valid / uid_freed_uid: pulse when a UID is completely drained.
//
// NOTES
//   • No LEN on the R side; we rely only on RLAST.
//   • Backpressure: r_in.ready deasserts if the target UID FIFO is full.
//   • Only bitwise ops (&, |, ~, !) used in control expressions (no && ||).
// ============================================================================

module response_park #(
    // Number of internal UIDs (tags)
    parameter int NUM_UIDS   = 16,
    // Max beats per UID (FIFO depth)
    parameter int MAX_BEATS  = 8,

    // R channel field widths
    parameter int ID_WIDTH   = $clog2(NUM_UIDS),
    parameter int DATA_WIDTH = 64,
    parameter int RESP_WIDTH = 2
)(
    input  logic clk,
    input  logic rst,

    // Incoming beats from the fabric side (id = UID)
    r_if.receiver r_in,

    // Outgoing beats towards the reorder / master side
    r_if.sender   r_out,

    // Drain control from r_id_ordering_unit
    input  logic [ID_WIDTH-1:0] drain_uid,
    input  logic                drain_start,   // 1-cycle pulse: start draining this UID

    // Status back to ordering unit / allocator
    output logic                uid_freed_valid, // 1-cycle pulse when UID finished
    output logic [ID_WIDTH-1:0] uid_freed_uid,   // which UID just completed

    // Overall fullness indicator (optional use by upstream)
    output logic                full            // high when all slots are occupied
);
    // ------------------------------------------------------------------------
    // Local parameters and type helpers
    // ------------------------------------------------------------------------
    localparam int UID_W   = ID_WIDTH;
    localparam int CNT_W   = $clog2(MAX_BEATS + 1);
    localparam int TOT_CAP = NUM_UIDS * MAX_BEATS;
    localparam int TOT_W   = $clog2(TOT_CAP + 1);

    // ------------------------------------------------------------------------
    // Per-UID storage: data + resp + simple FIFO bookkeeping
    // ------------------------------------------------------------------------
    // data_mem[uid][beat_index]
    logic [DATA_WIDTH-1:0] data_mem [NUM_UIDS][MAX_BEATS];
    logic [RESP_WIDTH-1:0] resp_mem [NUM_UIDS][MAX_BEATS];

    // Write/read pointers and counters per UID
    logic [CNT_W-1:0] wptr      [NUM_UIDS]; // next write index
    logic [CNT_W-1:0] rptr      [NUM_UIDS]; // next read index
    logic [CNT_W-1:0] beat_cnt  [NUM_UIDS]; // number of valid beats in FIFO

    // Have we already seen RLAST for this UID?
    logic              seen_last[NUM_UIDS];

    // Global occupancy (for "full" status)
    logic [TOT_W-1:0] total_used;

    // Convenience wires: index from incoming R channel
    wire [UID_W-1:0] in_uid  = r_in.id;

    // Per-UID empty/full flags (derived)
    logic fifo_empty [NUM_UIDS];
    logic fifo_full  [NUM_UIDS];

    genvar g_uid;
    generate
        for (g_uid = 0; g_uid < NUM_UIDS; g_uid = g_uid + 1) begin : GEN_FLAGS
            always_comb begin
                fifo_empty[g_uid] = (beat_cnt[g_uid] == {CNT_W{1'b0}});
                fifo_full[g_uid]  = (beat_cnt[g_uid] == MAX_BEATS[CNT_W-1:0]);
            end
        end
    endgenerate

    // ------------------------------------------------------------------------
    // Write side: accept beats from r_in into the appropriate UID FIFO
    // ------------------------------------------------------------------------
    // Backpressure: only accept if target UID FIFO is not full
    always_comb begin
        // Default: ready when not explicitly blocked
        r_in.ready = 1'b1;

        if (r_in.valid) begin
            // If this UID FIFO is full, we must stall
            if (fifo_full[in_uid]) begin
                r_in.ready = 1'b0;
            end
        end
    end

    wire accept_in = r_in.valid & r_in.ready;

    // ------------------------------------------------------------------------
    // Drain side state: which UID we are currently draining, and when
    // ------------------------------------------------------------------------
    logic              draining;
    logic [UID_W-1:0]  active_uid;

    // Convenience wires for the active UID
    wire [CNT_W-1:0] active_count = beat_cnt[active_uid];
    wire             active_empty = (active_count == {CNT_W{1'b0}});

    // ------------------------------------------------------------------------
    // r_out (read side) combinational logic
    // ------------------------------------------------------------------------
    always_comb begin
        // Defaults: no valid data
        r_out.valid = 1'b0;
        r_out.data  = {DATA_WIDTH{1'b0}};
        r_out.resp  = {RESP_WIDTH{1'b0}};
        r_out.last  = 1'b0;
        r_out.id    = active_uid; // UID; reorder unit can ignore or use it

        if (draining & (~active_empty)) begin
            // There is at least one beat to send for active_uid
            r_out.valid = 1'b1;
            r_out.data  = data_mem[active_uid][rptr[active_uid]];
            r_out.resp  = resp_mem[active_uid][rptr[active_uid]];

            // "Last" when we have already seen RLAST for this UID AND
            // there is exactly one beat left in its FIFO.
            if (seen_last[active_uid] & (active_count == {{(CNT_W-1){1'b0}}, 1'b1})) begin
                r_out.last = 1'b1;
            end
        end
    end

    wire accept_out = r_out.valid & r_out.ready;

    // ------------------------------------------------------------------------
    // Sequential state update (write + drain + bookkeeping)
    // ------------------------------------------------------------------------
    integer i;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all per-UID state
            for (i = 0; i < NUM_UIDS; i = i + 1) begin
                wptr[i]       <= {CNT_W{1'b0}};
                rptr[i]       <= {CNT_W{1'b0}};
                beat_cnt[i]   <= {CNT_W{1'b0}};
                seen_last[i]  <= 1'b0;
            end

            total_used      <= {TOT_W{1'b0}};
            draining        <= 1'b0;
            active_uid      <= {UID_W{1'b0}};
            uid_freed_valid <= 1'b0;
            uid_freed_uid   <= {UID_W{1'b0}};
            full            <= 1'b0;
        end
        else begin
            // Default: no freed UID this cycle
            uid_freed_valid <= 1'b0;

            // ------------------------------------------------------------
            // Start draining a new UID when requested
            // ------------------------------------------------------------
            if (drain_start & (~draining)) begin
                active_uid <= drain_uid;
                draining   <= 1'b1;
            end

            // ------------------------------------------------------------
            // WRITE path: new beat accepted from r_in
            // ------------------------------------------------------------
            if (accept_in) begin
                // Write into FIFO of in_uid at its current write pointer
                data_mem[in_uid][wptr[in_uid]] <= r_in.data;
                resp_mem[in_uid][wptr[in_uid]] <= r_in.resp;

                // Advance write pointer (simple wrap-around modulo MAX_BEATS)
                if (wptr[in_uid] == (MAX_BEATS[CNT_W-1:0] - {{(CNT_W-1){1'b0}}, 1'b1})) begin
                    wptr[in_uid] <= {CNT_W{1'b0}};
                end
                else begin
                    wptr[in_uid] <= wptr[in_uid] + {{(CNT_W-1){1'b0}}, 1'b1};
                end

                // Increment per-UID count
                beat_cnt[in_uid] <= beat_cnt[in_uid] + {{(CNT_W-1){1'b0}}, 1'b1};

                // Mark that we have seen RLAST for this UID when applicable
                if (r_in.last) begin
                    seen_last[in_uid] <= 1'b1;
                end

                // Update global used count
                total_used <= total_used + {{(TOT_W-1){1'b0}}, 1'b1};
            end

            // ------------------------------------------------------------
            // READ path: beat consumed on r_out
            // ------------------------------------------------------------
            if (accept_out) begin
                // Advance read pointer for active_uid
                if (rptr[active_uid] == (MAX_BEATS[CNT_W-1:0] - {{(CNT_W-1){1'b0}}, 1'b1})) begin
                    rptr[active_uid] <= {CNT_W{1'b0}};
                end
                else begin
                    rptr[active_uid] <= rptr[active_uid] + {{(CNT_W-1){1'b0}}, 1'b1};
                end

                // Decrement per-UID count
                beat_cnt[active_uid] <= beat_cnt[active_uid] - {{(CNT_W-1){1'b0}}, 1'b1};

                // Decrement global used count
                total_used <= total_used - {{(TOT_W-1){1'b0}}, 1'b1};

                // If this was the last beat for this UID, consider it fully freed
                if (r_out.last) begin
                    // UID is now fully drained: clear flags and counters
                    seen_last[active_uid] <= 1'b0;
                    wptr[active_uid]      <= {CNT_W{1'b0}};
                    rptr[active_uid]      <= {CNT_W{1'b0}};

                    // End of draining state
                    draining              <= 1'b0;

                    // Pulse "UID freed" for allocator / ordering unit
                    uid_freed_valid       <= 1'b1;
                    uid_freed_uid         <= active_uid;
                end
            end

            // ------------------------------------------------------------
            // Global "full" flag (all slots occupied)
            // ------------------------------------------------------------
            if (total_used == TOT_CAP[TOT_W-1:0]) begin
                full <= 1'b1;
            end
            else begin
                full <= 1'b0;
            end
        end
    end

endmodule
