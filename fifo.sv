// ============================================================================
// fifo.sv
// ----------------------------------------------------------------------------
// FIFO for storing *whole bursts* (each entry = one full request OR response)
// ----------------------------------------------------------------------------
// • 16-entry default depth
// • Each entry stores AR*/R* fields and full RDATA payload if kind=RESPONSE
// • Bitwise-only control (~, &, |)
// • One-cycle strobes: alloc_gnt (push accepted), free_ack (pop accepted)
// • Safe push+pop in same cycle
// • dout reflects current head entry (combinational)
// ============================================================================

module fifo #(
    parameter int DATA_WIDTH = 64,     // width of each beat (RDATA)
    parameter int MAX_BEATS  = 32,     // max beats per burst (ARLEN+1)
    parameter int DEPTH      = 16      // number of bursts stored
)(
    input  logic clk,
    input  logic rst,                  // asynchronous, active-high

    // Handshake controls
    input  logic wr_en,                // push request (valid burst)
    input  logic rd_en,                // pop request
    output logic alloc_gnt,            // 1 for one cycle when burst accepted
    output logic free_ack,             // 1 for one cycle when burst released

    // === Write data ===
    // Generic "whole burst" record fields:
    input  logic        din_kind,      // 0=request (AR), 1=response (R)
    input  logic [7:0]  din_id,        // ARID or RID (as relevant)
    input  logic [31:0] din_addr,      // ARADDR (unused for R)
    input  logic [7:0]  din_len,       // ARLEN / beats-1
    input  logic [2:0]  din_size,      // ARSIZE
    input  logic [1:0]  din_burst,     // ARBURST
    input  logic [3:0]  din_qos,       // ARQOS
    input  logic [1:0]  din_rresp,     // RRESP (for responses)
    input  logic [$clog2(MAX_BEATS+1)-1:0] din_nbeats, // # beats in burst
    input  logic [MAX_BEATS*DATA_WIDTH-1:0] din_payload, // flattened RDATA payload
    input  logic [7:0]  din_tag,       // internal UID / row:col

    // === Read data (current head entry) ===
    output logic        dout_kind,
    output logic [7:0]  dout_id,
    output logic [31:0] dout_addr,
    output logic [7:0]  dout_len,
    output logic [2:0]  dout_size,
    output logic [1:0]  dout_burst,
    output logic [3:0]  dout_qos,
    output logic [1:0]  dout_rresp,
    output logic [$clog2(MAX_BEATS+1)-1:0] dout_nbeats,
    output logic [MAX_BEATS*DATA_WIDTH-1:0] dout_payload,
    output logic [7:0]  dout_tag,

    // === Status ===
    output logic empty,
    output logic full
);

    // ===========================================================
    // Internal type: one full burst record
    // ===========================================================
    typedef struct packed {
        logic        kind;
        logic [7:0]  id;
        logic [31:0] addr;
        logic [7:0]  len;
        logic [2:0]  size;
        logic [1:0]  burst;
        logic [3:0]  qos;
        logic [1:0]  rresp;
        logic [$clog2(MAX_BEATS+1)-1:0] nbeats;
        logic [MAX_BEATS*DATA_WIDTH-1:0] payload;
        logic [7:0]  tag;
    } burst_t;

    // ===========================================================
    // Storage and state
    // ===========================================================
    localparam int ADDR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
    localparam int CNT_W  = $clog2(DEPTH + 1);

    burst_t mem [0:DEPTH-1];
    logic [ADDR_W-1:0] wr_ptr_q, rd_ptr_q;
    logic [CNT_W-1:0]  count_q;

    // ===========================================================
    // Status flags (bitwise only)
    // ===========================================================
    always_comb begin
        empty = (count_q == '0);
        full  = (count_q == DEPTH[CNT_W-1:0]);
    end

    // ===========================================================
    // Form gated enables (bitwise)
    // ===========================================================
    logic push, pop;
    always_comb begin
        push = wr_en & (~full);
        pop  = rd_en & (~empty);
    end

    // ===========================================================
    // Output current head entry (combinational)
    // ===========================================================
    assign dout_kind     = mem[rd_ptr_q].kind;
    assign dout_id       = mem[rd_ptr_q].id;
    assign dout_addr     = mem[rd_ptr_q].addr;
    assign dout_len      = mem[rd_ptr_q].len;
    assign dout_size     = mem[rd_ptr_q].size;
    assign dout_burst    = mem[rd_ptr_q].burst;
    assign dout_qos      = mem[rd_ptr_q].qos;
    assign dout_rresp    = mem[rd_ptr_q].rresp;
    assign dout_nbeats   = mem[rd_ptr_q].nbeats;
    assign dout_payload  = mem[rd_ptr_q].payload;
    assign dout_tag      = mem[rd_ptr_q].tag;

    // ===========================================================
    // Single-cycle strobes for accepted ops
    // ===========================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            alloc_gnt <= 1'b0;
            free_ack  <= 1'b0;
        end
        else begin
            alloc_gnt <= push;
            free_ack  <= pop;
        end
    end

    // ===========================================================
    // Pointer wrap function (bitwise safe)
    // ===========================================================
    function automatic [ADDR_W-1:0] inc_wrap(input [ADDR_W-1:0] ptr);
        if (ptr == (DEPTH-1)[ADDR_W-1:0])
            inc_wrap = '0;
        else
            inc_wrap = ptr + {{(ADDR_W-1){1'b0}},1'b1};
    endfunction

    // ===========================================================
    // Sequential update (bitwise logic only)
    // ===========================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr_q <= '0;
            rd_ptr_q <= '0;
            count_q  <= '0;
        end
        else begin
            // ------------------
            // Write path (push)
            // ------------------
            if (push) begin
                mem[wr_ptr_q].kind     <= din_kind;
                mem[wr_ptr_q].id       <= din_id;
                mem[wr_ptr_q].addr     <= din_addr;
                mem[wr_ptr_q].len      <= din_len;
                mem[wr_ptr_q].size     <= din_size;
                mem[wr_ptr_q].burst    <= din_burst;
                mem[wr_ptr_q].qos      <= din_qos;
                mem[wr_ptr_q].rresp    <= din_rresp;
                mem[wr_ptr_q].nbeats   <= din_nbeats;
                mem[wr_ptr_q].payload  <= din_payload;
                mem[wr_ptr_q].tag      <= din_tag;
                wr_ptr_q <= inc_wrap(wr_ptr_q);
            end

            // ------------------
            // Read path (pop)
            // ------------------
            if (pop) begin
                rd_ptr_q <= inc_wrap(rd_ptr_q);
            end

            // ------------------
            // Count update
            // ------------------
            unique case ({push, pop})
                2'b10: count_q <= count_q + {{(CNT_W-1){1'b0}},1'b1};
                2'b01: count_q <= count_q - {{(CNT_W-1){1'b0}},1'b1};
                default: /* no change */ ;
            endcase
        end
    end

endmodule
