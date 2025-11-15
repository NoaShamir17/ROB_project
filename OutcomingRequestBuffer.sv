// ============================================================================
// OutcomingRequestBuffer
// ----------------------------------------------------------------------------
// • Input: AR request from internal logic (in_if)
// • Output: AR channel to AXI slave (m_if)
// • Policy: drain FIFO first; bypass only when FIFO empty
// • Uses your whole-burst fifo (kind=0 for requests)
// • Bitwise-only control (~, &, |) in comb logic
// ============================================================================
module OutcomingRequestBuffer #(
    parameter ID_WIDTH    = 4,     // AXI ID width (to slave)
    parameter ADDR_WIDTH  = 32,    // Address width (to slave)
    parameter LEN_WIDTH   = 8,     // AXI burst length width
    parameter TAG_WIDTH   = 4,     // Internal tag width (passed through if you need it on ARUSER)
    parameter FIFO_DEPTH  = 16,    // FIFO depth
    // fifo payload sizing (ports exist but not used for AR)
    parameter int DATA_WIDTH = 64,
    parameter int MAX_BEATS  = 32
)(
    input  logic clk,
    input  logic rst,              // synchronous active-high for this block

    // From ar_ordering_unit (producer of AR requests)
    ar_if.receiver in_if,

    // To AXI slave (we act as master on AR channel)
    ar_if.sender   m_if
);

    localparam int NBEATS_W = $clog2(MAX_BEATS + 1);

    // -----------------------------
    // FIFO interface (whole-burst)
    // -----------------------------
    logic fifo_empty, fifo_full;
    logic fifo_push, fifo_pop;
    logic fifo_alloc_gnt, fifo_free_ack;

    // Write-side to fifo (din_*): from in_if (kind=REQUEST)
    logic        din_kind;
    logic [7:0]  din_id;
    logic [31:0] din_addr;
    logic [7:0]  din_len;
    logic [2:0]  din_size;
    logic [1:0]  din_burst;
    logic [3:0]  din_qos;
    logic [1:0]  din_rresp;
    logic [NBEATS_W-1:0] din_nbeats;
    logic [MAX_BEATS*DATA_WIDTH-1:0] din_payload;
    logic [7:0]  din_tag;

    // Read-side from fifo (dout_*): drives m_if on pop
    logic        dout_kind;
    logic [7:0]  dout_id;
    logic [31:0] dout_addr;
    logic [7:0]  dout_len;
    logic [2:0]  dout_size;
    logic [1:0]  dout_burst;
    logic [3:0]  dout_qos;
    logic [1:0]  dout_rresp;
    logic [NBEATS_W-1:0] dout_nbeats;
    logic [MAX_BEATS*DATA_WIDTH-1:0] dout_payload;
    logic [7:0]  dout_tag;

    // -----------------------------
    // Output load policy
    // -----------------------------
    // Can load new AR into m_if when not holding one or slave is ready
    wire can_load_out = (~m_if.valid) | (m_if.ready);

    // Drain FIFO first; bypass only when FIFO empty and output can load
    wire serve_fifo   = (~fifo_empty) & can_load_out;
    wire serve_bypass = ( fifo_empty) & in_if.valid & can_load_out;

    // -----------------------------
    // Backpressure to producer
    // -----------------------------
    // Ready if we will bypass now OR FIFO has space
    assign in_if.ready = serve_bypass | (~fifo_full);

    // -----------------------------
    // FIFO enables (bitwise)
    // -----------------------------
    assign fifo_push = in_if.valid & (~serve_bypass) & (~fifo_full);
    assign fifo_pop  = serve_fifo;

    // -----------------------------
    // Drive fifo din_* from in_if (REQUEST entry, kind=0)
    // -----------------------------
    always_comb begin
        din_kind    = 1'b0; // request

        // widen to fifo port sizes
        din_id      = {{(8-ID_WIDTH){1'b0}}, in_if.id};
        din_addr    = in_if.addr[31:0];
        din_len     = {{(8-LEN_WIDTH){1'b0}}, in_if.len};
        din_size    = in_if.size;
        din_burst   = in_if.burst;
        din_qos     = in_if.qos;

        din_rresp   = 2'b00;                 // not used for requests
        din_nbeats  = {NBEATS_W{1'b0}};      // not used downstream on AR
        din_payload = {MAX_BEATS*DATA_WIDTH{1'b0}}; // unused on AR

        din_tag     = {{(8-TAG_WIDTH){1'b0}}, in_if.tagid};
    end

    // -----------------------------
    // FIFO instance
    // -----------------------------
    fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .MAX_BEATS  (MAX_BEATS),
        .DEPTH      (FIFO_DEPTH)
    ) u_ar_fifo (
        .clk         (clk),
        .rst         (rst),          // fifo has async posedge; acceptable here
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
    // Drive AR to slave (m_if)
    // -----------------------------
    // Priority: serve FIFO, else bypass, else drop valid if consumed
    always_ff @(posedge clk) begin
        if (rst) begin
            m_if.valid <= 1'b0;
            m_if.id    <= '0;
            m_if.addr  <= '0;
            m_if.len   <= '0;
            m_if.size  <= '0;
            m_if.burst <= '0;
            m_if.qos   <= '0;
            m_if.tagid <= '0; // if your AR user/tag goes out; keep if needed
        end
        else begin
            if (serve_fifo) begin
                m_if.valid <= 1'b1;
                m_if.id    <= dout_id[ID_WIDTH-1:0];
                m_if.addr  <= {{(ADDR_WIDTH>32)?(ADDR_WIDTH-32):0{1'b0}}, dout_addr[31:0]};
                m_if.len   <= dout_len[LEN_WIDTH-1:0];
                m_if.size  <= dout_size;
                m_if.burst <= dout_burst;
                m_if.qos   <= dout_qos;
                m_if.tagid <= dout_tag[TAG_WIDTH-1:0];
            end
            else if (serve_bypass) begin
                m_if.valid <= 1'b1;
                m_if.id    <= in_if.id;
                m_if.addr  <= in_if.addr;
                m_if.len   <= in_if.len;
                m_if.size  <= in_if.size;
                m_if.burst <= in_if.burst;
                m_if.qos   <= in_if.qos;
                m_if.tagid <= in_if.tagid;
            end
            else if (m_if.valid & m_if.ready) begin
                m_if.valid <= 1'b0;
            end
            // else: hold current AR until slave accepts
        end
    end

endmodule
