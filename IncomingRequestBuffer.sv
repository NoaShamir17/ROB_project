// ============================================================================
// IncomingRequestBuffer
// ----------------------------------------------------------------------------
// • Accepts AR* from master (in_if).
// • If downstream can accept now and FIFO is empty → BYPASS straight to out_if.
// • Else → push a *whole-burst request* (kind=0) into fifo (depth=FIFO_DEPTH).
// • Output policy: drain FIFO first; bypass only when FIFO is empty.
// • Bitwise-only control in comb logic (~, &, |). No &&, ||, !.
// • Uses your "whole-burst" fifo with 'kind' field and AR/R superset I/O.
// ============================================================================

module IncomingRequestBuffer #(
    parameter ID_WIDTH    = 4,     // AXI ID width
    parameter ADDR_WIDTH  = 32,    // Memory address width
    parameter LEN_WIDTH   = 8,     // AXI burst length width
    parameter TAG_WIDTH   = 4,     // Internal tag width
    parameter FIFO_DEPTH  = 16,    // FIFO depth
    // fifo payload sizing (not used for requests, but fifo ports require them)
    parameter int DATA_WIDTH = 64,
    parameter int MAX_BEATS  = 32
)(
    input  logic clk,              // Clock
    input  logic rst,              // Synchronous active-high reset

    ar_if.receiver in_if,          // From AXI master (AR channel)
    ar_if.sender   out_if          // To ar_ordering_unit (next stage)
);

    // -----------------------------
    // Local widths
    // -----------------------------
    localparam int NBEATS_W = $clog2(MAX_BEATS + 1);

    // -----------------------------
    // FIFO signals (whole-burst API)
    // -----------------------------
    logic fifo_empty, fifo_full;
    logic fifo_push, fifo_pop;
    logic fifo_alloc_gnt, fifo_free_ack;

    // FIFO write-side (din_*) — we drive these from in_if.*
    logic        din_kind;        // 0=request (AR)
    logic [7:0]  din_id;
    logic [31:0] din_addr;
    logic [7:0]  din_len;
    logic [2:0]  din_size;
    logic [1:0]  din_burst;
    logic [3:0]  din_qos;
    logic [1:0]  din_rresp;       // unused for requests
    logic [NBEATS_W-1:0] din_nbeats;      // could be len+1; not used by AR logic downstream
    logic [MAX_BEATS*DATA_WIDTH-1:0] din_payload; // unused for requests
    logic [7:0]  din_tag;

    // FIFO read-side (dout_*): feeds out_if on pop
    logic        dout_kind;
    logic [7:0]  dout_id;
    logic [31:0] dout_addr;
    logic [7:0]  dout_len;
    logic [2:0]  dout_size;
    logic [1:0]  dout_burst;
    logic [3:0]  dout_qos;
    logic [1:0]  dout_rresp;        // not used for requests
    logic [NBEATS_W-1:0] dout_nbeats;
    logic [MAX_BEATS*DATA_WIDTH-1:0] dout_payload;
    logic [7:0]  dout_tag;

    // -----------------------------
    // Output load policy
    // -----------------------------
    // Output can accept a new record when: ~valid | ready
    wire can_load_out = (~out_if.valid) | (out_if.ready);

    // Serve FIFO first; only bypass when FIFO empty and can load now
    wire serve_fifo   = (~fifo_empty) & can_load_out;
    wire serve_bypass = ( fifo_empty) & in_if.valid & can_load_out;

    // -----------------------------
    // Backpressure to master
    // -----------------------------
    // Ready if we will bypass now OR FIFO has space to enqueue.
    assign in_if.ready = serve_bypass | (~fifo_full);

    // -----------------------------
    // FIFO write/read enables
    // -----------------------------
    // Push when input is valid, we are NOT bypassing this cycle, and FIFO has room.
    assign fifo_push = in_if.valid & (~serve_bypass) & (~fifo_full);

    // Pop when serving FIFO (we’ll capture dout_* into out_if)
    assign fifo_pop  = serve_fifo;

    // -----------------------------
    // Drive FIFO din_* from in_if (as a REQUEST burst, kind=0)
    // -----------------------------
    // Map widths to fifo’s fixed ports:
    always_comb begin
        // mark this entry as a REQUEST whole-burst
        din_kind    = 1'b0; // request

        // ID: map into 8 bits (zero-extend if ID_WIDTH < 8)
        din_id      = {{(8-ID_WIDTH){1'b0}}, in_if.id};

        // Address (truncate/zero-extend to 32 bits consistently)
        din_addr    = in_if.addr[31:0];

        // Length, size, burst, qos
        din_len     = {{(8-LEN_WIDTH){1'b0}}, in_if.len};
        din_size    = in_if.size;
        din_burst   = in_if.burst;
        din_qos     = in_if.qos;

        // Not relevant for requests
        din_rresp   = 2'b00;

        // You may set nbeats = len+1 to reflect intended beats; safe default below
        // Avoid dependency on arithmetic saturation vs MAX_BEATS — keep conservative.
        din_nbeats  = {NBEATS_W{1'b0}}; // not used downstream for AR; keep zero

        // Payload not used for requests
        din_payload = {MAX_BEATS*DATA_WIDTH{1'b0}};

        // Internal tag — widen to 8 bits for FIFO field
        din_tag     = {{(8-TAG_WIDTH){1'b0}}, in_if.tagid};
    end

    // -----------------------------
    // Instantiate whole-burst FIFO
    // -----------------------------
    fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .MAX_BEATS  (MAX_BEATS),
        .DEPTH      (FIFO_DEPTH)
    ) u_burst_fifo (
        .clk         (clk),
        .rst         (rst),            // fifo expects async posedge rst; fine to reuse
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

        .empty       (fifo_empty),
        .full        (fifo_full)
    );

    // -----------------------------
    // Output register load
    // -----------------------------
    // Priority: serve FIFO, else bypass, else drop valid if consumed, else hold.
    always_ff @(posedge clk) begin
        if (rst) begin
            out_if.valid <= 1'b0;
            out_if.id    <= '0;
            out_if.addr  <= '0;
            out_if.len   <= '0;
            out_if.size  <= '0;
            out_if.burst <= '0;
            out_if.qos   <= '0;
            out_if.tagid <= '0;
        end
        else begin
            if (serve_fifo) begin
                // drain one request entry from FIFO into output
                out_if.valid <= 1'b1;
                // map back widths (truncate to interface widths where needed)
                out_if.id    <= dout_id[ID_WIDTH-1:0];
                out_if.addr  <= {{(ADDR_WIDTH>32)?(ADDR_WIDTH-32):0{1'b0}}, dout_addr[31:0]};
                out_if.len   <= dout_len[LEN_WIDTH-1:0];
                out_if.size  <= dout_size;
                out_if.burst <= dout_burst;
                out_if.qos   <= dout_qos;
                out_if.tagid <= dout_tag[TAG_WIDTH-1:0];
            end
            else if (serve_bypass) begin
                // pass request straight through (FIFO empty and consumer can load)
                out_if.valid <= 1'b1;
                out_if.id    <= in_if.id;
                out_if.addr  <= in_if.addr;
                out_if.len   <= in_if.len;
                out_if.size  <= in_if.size;
                out_if.burst <= in_if.burst;
                out_if.qos   <= in_if.qos;
                out_if.tagid <= in_if.tagid;
            end
            else if (out_if.valid & out_if.ready) begin
                // consumer took current entry; no new entry loaded → drop valid
                out_if.valid <= 1'b0;
            end
            // else: hold out_if.* as-is
        end
    end

endmodule
