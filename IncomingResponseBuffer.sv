// ============================================================================
// IncomingResponseBuffer
// ----------------------------------------------------------------------------
// • r_if.receiver s_r  : receives R-beats from AXI slave
// • fifo (whole-burst) : aggregates each burst (kind=1) into one entry
// • r_if.sender   out_r: streams bursts forward to r_id_ordering_unit
// ----------------------------------------------------------------------------
// Behavior:
//   - Accept all beats except: when current beat has RLAST=1 and FIFO is full.
//   - Push to FIFO exactly when accepting LAST beat (atomic commit).
//   - Stream stored bursts strictly in FIFO order; pop only after LAST beat xfer.
// Constraints:
//   - Bitwise-only comb control (~, &, |). No &&, ||, !.
// ============================================================================
module IncomingResponseBuffer #(
    parameter ID_WIDTH       = 4,
    parameter TAG_WIDTH      = 4,
    parameter int DATA_WIDTH = 64,
    parameter int MAX_BEATS  = 32,
    parameter int FIFO_DEPTH = 16
)(
    input  logic clk,
    input  logic rst,                 // synchronous active-high reset

    // From AXI slave (receiver)
    r_if.receiver s_r,

    // To internal pipeline / ordering unit (sender)
    r_if.sender   out_r
);

    // ------------------------------------------------------------------------
    // Local params
    // ------------------------------------------------------------------------
    localparam int NBEATS_W = $clog2(MAX_BEATS + 1);

    // ------------------------------------------------------------------------
    // FIFO signals
    // ------------------------------------------------------------------------
    logic fifo_empty, fifo_full;
    logic fifo_push, fifo_pop;
    logic fifo_alloc_gnt, fifo_free_ack;

    // FIFO write bundle (response entry, kind=1)
    logic        din_kind;
    logic [7:0]  din_id;
    logic [31:0] din_addr;
    logic [7:0]  din_len;
    logic [2:0]  din_size;
    logic [1:0]  din_burst;
    logic [3:0]  din_qos;
    logic [1:0]  din_rresp;
    logic [NBEATS_W-1:0]             din_nbeats;
    logic [MAX_BEATS*DATA_WIDTH-1:0] din_payload;
    logic [7:0]  din_tag;

    // FIFO read bundle
    logic        dout_kind;
    logic [7:0]  dout_id;
    logic [31:0] dout_addr;
    logic [7:0]  dout_len;
    logic [2:0]  dout_size;
    logic [1:0]  dout_burst;
    logic [3:0]  dout_qos;
    logic [1:0]  dout_rresp;
    logic [NBEATS_W-1:0]             dout_nbeats;
    logic [MAX_BEATS*DATA_WIDTH-1:0] dout_payload;
    logic [7:0]  dout_tag;

    // ------------------------------------------------------------------------
    // Collector: aggregate incoming beats into a full burst
    // ------------------------------------------------------------------------
    logic                        collecting_q;
    logic [ID_WIDTH-1:0]         rid_q;
    logic [NBEATS_W-1:0]         beats_q;
    logic [1:0]                  last_rresp_q;
    logic [MAX_BEATS*DATA_WIDTH-1:0] payload_q;

    // Accept rule: stall only when LAST beat and FIFO is full
    wire stall_last = s_r.last & fifo_full;
    assign s_r.ready = ~stall_last;

    // Beat handshake
    wire take_beat = s_r.valid & s_r.ready;
    wire last_beat = s_r.last;

    // Increment helper
    wire [NBEATS_W-1:0] beats_inc = beats_q + {{(NBEATS_W-1){1'b0}}, 1'b1};

    // Atomic commit: push exactly when LAST beat is accepted
    assign fifo_push = take_beat & last_beat;

    // FIFO write mapping (kind=1 response)
    always_comb begin
        din_kind    = 1'b1;
        din_id      = {{(8-ID_WIDTH){1'b0}}, rid_q};
        din_addr    = 32'b0;
        din_len     = 8'b0;
        din_size    = 3'b0;
        din_burst   = 2'b0;
        din_qos     = 4'b0;
        din_rresp   = last_rresp_q;
        din_nbeats  = beats_q;
        din_payload = payload_q;
        din_tag     = {{(8-TAG_WIDTH){1'b0}}, rid_q[TAG_WIDTH-1:0]};
    end

    // Collector sequential
    always_ff @(posedge clk) begin
        if (rst) begin
            collecting_q <= 1'b0;
            rid_q        <= {ID_WIDTH{1'b0}};
            beats_q      <= {NBEATS_W{1'b0}};
            last_rresp_q <= 2'b00;
            payload_q    <= {MAX_BEATS*DATA_WIDTH{1'b0}};
        end
        else begin
            if (take_beat) begin
                // First beat of burst
                if (collecting_q == 1'b0) begin
                    collecting_q <= 1'b1;
                    rid_q        <= s_r.id[ID_WIDTH-1:0];
                    beats_q      <= {{(NBEATS_W-1){1'b0}}, 1'b1};
                    payload_q[0 +: DATA_WIDTH] <= s_r.data;
                    last_rresp_q <= s_r.resp;
                end
                // Subsequent beats
                else begin
                    payload_q[beats_q*DATA_WIDTH +: DATA_WIDTH] <= s_r.data;
                    beats_q      <= beats_inc;
                    last_rresp_q <= s_r.resp;
                end

                // Close on last beat (fifo_push asserted this cycle)
                if (last_beat) begin
                    collecting_q <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Whole-burst FIFO
    // ------------------------------------------------------------------------
    fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .MAX_BEATS  (MAX_BEATS),
        .DEPTH      (FIFO_DEPTH)
    ) u_inresp_fifo (
        .clk         (clk),
        .rst         (rst),
        .wr_en       (fifo_push),
        .rd_en       (fifo_pop),
        .alloc_gnt   (fifo_alloc_gnt),
        .free_ack    (fifo_free_ack),

        .din_kind    (din_kind),
        .din_id      (din_id),
        .din_addr    (din_addr),
        .din_len     (din_len),
        .din_size    (din_size),
        .din_burst   (din_burst),
        .din_qos     (din_qos),
        .din_rresp   (din_rresp),
        .din_nbeats  (din_nbeats),
        .din_payload (din_payload),
        .din_tag     (din_tag),

        .dout_kind    (dout_kind),
        .dout_id      (dout_id),
        .dout_addr    (dout_addr),
        .dout_len     (dout_len),
        .dout_size    (dout_size),
        .dout_burst   (dout_burst),
        .dout_qos     (dout_qos),
        .dout_rresp   (dout_rresp),
        .dout_nbeats  (dout_nbeats),
        .dout_payload (dout_payload),
        .dout_tag     (dout_tag),

        .empty        (fifo_empty),
        .full         (fifo_full)
    );

    // ------------------------------------------------------------------------
    // Stream stored bursts to the next stage (out_r)
    // ------------------------------------------------------------------------
    logic                sending_q;
    logic [NBEATS_W-1:0] ob_idx_q;
    logic [NBEATS_W-1:0] ob_last_idx;

    assign ob_last_idx = (dout_nbeats - {{(NBEATS_W-1){1'b0}}, 1'b1});

    // Start streaming when idle and FIFO has data
    wire can_start_send = (~sending_q) & (~fifo_empty);

    // Beat transfer handshake on out_r
    wire beat_xfer = out_r.valid & out_r.ready;
    wire at_last   = (ob_idx_q == ob_last_idx);

    // Pop FIFO after last beat accepted
    assign fifo_pop = beat_xfer & at_last;

    // Drive out_r (bitwise-only comb)
    always_comb begin
        out_r.valid = sending_q;
        out_r.id    = {ID_WIDTH{1'b0}};
        out_r.data  = {DATA_WIDTH{1'b0}};
        out_r.resp  = 2'b00;
        out_r.last  = 1'b0;

        if (sending_q) begin
            out_r.valid = 1'b1;
            out_r.id    = dout_id[ID_WIDTH-1:0];
            out_r.data  = dout_payload[ob_idx_q*DATA_WIDTH +: DATA_WIDTH];
            out_r.resp  = dout_rresp;
            out_r.last  = (ob_idx_q == ob_last_idx);
        end
    end

    // Sending state
    always_ff @(posedge clk) begin
        if (rst) begin
            sending_q <= 1'b0;
            ob_idx_q  <= {NBEATS_W{1'b0}};
        end
        else begin
            if (can_start_send) begin
                sending_q <= 1'b1;
                ob_idx_q  <= {NBEATS_W{1'b0}};
            end
            else if (beat_xfer) begin
                if (at_last) begin
                    sending_q <= 1'b0;
                    ob_idx_q  <= {NBEATS_W{1'b0}};
                end
                else begin
                    ob_idx_q  <= ob_idx_q + {{(NBEATS_W-1){1'b0}}, 1'b1};
                end
            end
        end
    end

endmodule
