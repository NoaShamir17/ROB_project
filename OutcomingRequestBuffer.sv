`include "fifo.sv"
`include "ar_if.sv"

module OutcomingRequestBuffer #(
    parameter ID_WIDTH    = 4,    // AXI ID width
    parameter ADDR_WIDTH  = 32,   // Memory address width
    parameter LEN_WIDTH   = 8,    // Burst length width
    parameter TAG_WIDTH   = 4,    // Internal tag width
    parameter FIFO_DEPTH  = 16    // FIFO depth for buffering outgoing AR requests
)(
    input  logic clk,             // Clock
    input  logic rst,             // Synchronous reset (active high)

    ar_if.receiver in_if,         // Input from internal logic (ROB, remapper, etc.)
    ar_if.sender   out_if         // Output to AXI slave (external)
);

    // Struct for bundling an AXI Read request into FIFO
    typedef struct packed {
        logic [ID_WIDTH-1:0]   id;     // AXI transaction ID
        logic [ADDR_WIDTH-1:0] addr;   // Memory address
        logic [LEN_WIDTH-1:0]  len;    // Burst length
        logic [2:0]            size;   // Beat size
        logic [1:0]            burst;  // Burst type (INCR, FIXED, etc.)
        logic [3:0]            qos;    // Quality of service
        logic [TAG_WIDTH-1:0]  tagid;  // Internal tag for tracking
    } ar_req_t;

    // Internal FIFO wiring
    ar_req_t fifo_in, fifo_out;        // FIFO input/output data
    logic fifo_full, fifo_empty;       // FIFO status
    logic push_req, pop_req;           // Control signals

    // Group incoming signals into FIFO struct
    assign fifo_in = '{
        id:    in_if.id,
        addr:  in_if.addr,
        len:   in_if.len,
        size:  in_if.size,
        burst: in_if.burst,
        qos:   in_if.qos,
        tagid: in_if.tagid
    };

    // Push: when internal logic sends a valid transaction and FIFO has space
    assign push_req = in_if.valid && !fifo_full;

    // Pop: when FIFO has data and output is allowed to send
    assign pop_req = !fifo_empty && (!out_if.valid || out_if.ready);

    // Tell internal logic it can send a request
    assign in_if.ready = !fifo_full;

    // FIFO instantiation for buffering outgoing requests
    fifo #(
        .DATA_WIDTH($bits(ar_req_t)),  // Total width of packed request
        .DEPTH(FIFO_DEPTH)
    ) fifo_inst (
        .clk(clk),
        .rst(rst),
        .wr_en(push_req),              // Push from internal logic
        .rd_en(pop_req),               // Pop to external AXI slave
        .din(fifo_in),
        .dout(fifo_out),
        .empty(fifo_empty),
        .full(fifo_full)
    );

    // Output request logic
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // Clear valid on reset
            out_if.valid <= 0;
        end
        else if (pop_req) begin
            // Drive new request from FIFO to AXI slave
            out_if.valid <= 1;
            out_if.id    <= fifo_out.id;
            out_if.addr  <= fifo_out.addr;
            out_if.len   <= fifo_out.len;
            out_if.size  <= fifo_out.size;
            out_if.burst <= fifo_out.burst;
            out_if.qos   <= fifo_out.qos;
            out_if.tagid <= fifo_out.tagid;
        end
        else if (out_if.ready && out_if.valid) begin
            // After successful transmission, drop valid
            out_if.valid <= 0;
        end
    end

endmodule
