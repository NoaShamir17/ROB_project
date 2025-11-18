// ============================================================================
// OutcomingRequestBuffer.sv
// ----------------------------------------------------------------------------
// 8-entry FIFO for AXI AR requests on the outgoing side.
//
// ar_in  : AR from ar_id_ordering_unit (ar_if.receiver)
// ar_out : AR to AXI slave             (ar_if.sender)
//
// Behavior:
//   - When not full, ar_in.ready = 1 and we accept requests on
//       ar_in.valid & ar_in.ready (push).
//   - When not empty, ar_out.valid = 1 and we provide the oldest request.
//       On ar_out.valid & ar_out.ready we pop that request.
//
// Control uses only bitwise &, |, ~ (no &&, ||, !).
// ============================================================================

module OutcomingRequestBuffer #(
    parameter int ID_WIDTH    = 4,
    parameter int ADDR_WIDTH  = 32,
    parameter int LEN_WIDTH   = 8,
    parameter int SIZE_WIDTH  = 3,
    parameter int BURST_WIDTH = 2,
    parameter int QOS_WIDTH   = 4,
    parameter int DEPTH       = 8
)(
    input  logic clk,
    input  logic rst,          // async, active-high

    // AR from ar_id_ordering_unit
    ar_if.receiver ar_in,

    // AR toward AXI slave
    ar_if.sender   ar_out,

    // NEW: explicit full indicator
    output logic Outcoming_buffer_full
);

    // -------------------------------------------------------------
    // One AR entry record — fully parametric
    // -------------------------------------------------------------
    typedef struct packed {
        logic [ID_WIDTH-1:0]    id;
        logic [ADDR_WIDTH-1:0]  addr;
        logic [LEN_WIDTH-1:0]   len;
        logic [SIZE_WIDTH-1:0]  size;
        logic [BURST_WIDTH-1:0] burst;
        logic [QOS_WIDTH-1:0]   qos;
    } ar_entry_t;

    localparam int PTR_W = (DEPTH <= 2) ? 1 : $clog2(DEPTH);
    localparam int CNT_W = $clog2(DEPTH + 1);

    // FIFO storage
    ar_entry_t            mem     [0:DEPTH-1];
    logic [PTR_W-1:0]     wr_ptr_q;
    logic [PTR_W-1:0]     rd_ptr_q;
    logic [CNT_W-1:0]     count_q;

    // Status flags
    logic                 empty;
    logic                 full;

    // -------------------------------------------------------------
    // Empty / full
    // -------------------------------------------------------------
    always_comb begin
        empty = (count_q == '0);
        full  = (count_q == DEPTH[CNT_W-1:0]);

        // NEW: export full flag
        Outcoming_buffer_full = full;
    end

    // -------------------------------------------------------------
    // Handshake and fire signals
    // -------------------------------------------------------------
    logic push;   // accept new request from upstream
    logic pop;    // release request to downstream

    // Upstream sees ready when not full
    always_comb begin
        ar_in.ready = ~full;
    end

    // Downstream sees valid when not empty
    always_comb begin
        ar_out.valid = ~empty;
    end

    // Push/pop conditions (bitwise & only)
    always_comb begin
        push = ar_in.valid & ar_in.ready;
        pop  = ar_out.valid & ar_out.ready;
    end

    // -------------------------------------------------------------
    // Head entry → ar_out (combinational)
    // -------------------------------------------------------------
    always_comb begin
        ar_out.id    = mem[rd_ptr_q].id;
        ar_out.addr  = mem[rd_ptr_q].addr;
        ar_out.len   = mem[rd_ptr_q].len;
        ar_out.size  = mem[rd_ptr_q].size;
        ar_out.burst = mem[rd_ptr_q].burst;
        ar_out.qos   = mem[rd_ptr_q].qos;
    end

    // -------------------------------------------------------------
    // Sequential update: write, read, count, pointers
    // -------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_ptr_q <= '0;
            rd_ptr_q <= '0;
            count_q  <= '0;
        end
        else begin

            // --------------------
            // WRITE side (push)
            // --------------------
            if (push) begin
                mem[wr_ptr_q].id    <= ar_in.id;
                mem[wr_ptr_q].addr  <= ar_in.addr;
                mem[wr_ptr_q].len   <= ar_in.len;
                mem[wr_ptr_q].size  <= ar_in.size;
                mem[wr_ptr_q].burst <= ar_in.burst;
                mem[wr_ptr_q].qos   <= ar_in.qos;

                // wr_ptr wrap-around
                if (wr_ptr_q == (DEPTH-1)[PTR_W-1:0])
                    wr_ptr_q <= '0;
                else
                    wr_ptr_q <= wr_ptr_q + {{(PTR_W-1){1'b0}}, 1'b1};
            end

            // --------------------
            // READ side (pop)
            // --------------------
            if (pop) begin
                // rd_ptr wrap-around
                if (rd_ptr_q == (DEPTH-1)[PTR_W-1:0])
                    rd_ptr_q <= '0;
                else
                    rd_ptr_q <= rd_ptr_q + {{(PTR_W-1){1'b0}}, 1'b1};
            end

            // --------------------
            // count_q update
            // --------------------
            if (push & (~pop)) begin
                count_q <= count_q + {{(CNT_W-1){1'b0}}, 1'b1};
            end
            else if ((~push) & pop) begin
                count_q <= count_q - {{(CNT_W-1){1'b0}}, 1'b1};
            end
        end
    end

endmodule
