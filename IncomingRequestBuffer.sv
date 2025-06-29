`include "fifo.sv"
`include "ar_if.sv"

module IncomingRequestBuffer #(
    parameter ID_WIDTH   = 4,    // AXI ID width
    parameter ADDR_WIDTH = 32,   // Memory address width
    parameter LEN_WIDTH  = 8,    // AXI burst length width
    parameter TAG_WIDTH  = 4,    // Internal tag width
    parameter FIFO_DEPTH = 16    // FIFO depth for buffering requests
)(
    input  logic clk,            // Clock
    input  logic rst,            // Synchronous active-high reset

    ar_if.receiver in_if,        // Incoming read address request from AXI master
    ar_if.sender   out_if        // Outgoing to internal logic (e.g., tag remapping unit)
);

    // Struct to bundle incoming AR fields into one packed value
    typedef struct packed {
        logic [ID_WIDTH-1:0]   id;     // AXI transaction ID
        logic [ADDR_WIDTH-1:0] addr;   // Read address
        logic [LEN_WIDTH-1:0]  len;    // Number of beats in burst
        logic [2:0]            size;   // Beat size (bytes per transfer)
        logic [1:0]            burst;  // Burst type (INCR, FIXED)
        logic [3:0]            qos;    // QoS hint
        logic [TAG_WIDTH-1:0]  tagid;  // Internal tracking tag
    } ar_req_t;

    // FIFO control wires
    ar_req_t fifo_in, fifo_out;         // Data into and out of FIFO
    logic fifo_empty, fifo_full;        // FIFO status signals
    logic push_req, pop_req;            // Control signals for write/read

    // Bundle the incoming AXI signals into FIFO input struct
    assign fifo_in = '{
        id:    in_if.id,      // Transaction ID from master
        addr:  in_if.addr,    // Address from master
        len:   in_if.len,     // Burst length
        size:  in_if.size,    // Beat size
        burst: in_if.burst,   // Burst type
        qos:   in_if.qos,     // QoS info
        tagid: in_if.tagid    // Internal tag from tag allocator
    };

    // Push to FIFO only when:
    // - Master sends a valid request
    // - FIFO is not full
    assign push_req = in_if.valid && !fifo_full;

    // Tell master we are ready when FIFO is not full
    assign in_if.ready = !fifo_full;

    // Pop from FIFO only when:
    // - FIFO has something
    // - Output not currently asserting valid (no back-pressure)
    assign pop_req = !fifo_empty && (!out_if.valid || out_if.ready);

    // FIFO instantiation
    fifo #(
        .DATA_WIDTH($bits(ar_req_t)),   // Struct width (bit-packed)
        .DEPTH(FIFO_DEPTH)              // Configurable FIFO depth
    ) fifo_inst (
        .clk(clk),
        .rst(rst),
        .wr_en(push_req),               // Write enable when input is valid
        .rd_en(pop_req),                // Read enable when downstream is ready
        .din(fifo_in),                  // Input data
        .dout(fifo_out),                // Output data
        .empty(fifo_empty),
        .full(fifo_full)
    );

    // Output logic to send read request to tag remapping stage
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // On reset, clear valid to avoid spurious transaction
            out_if.valid <= 0;
        end
        else if (pop_req) begin
            // New data available and output is ready — load it into output
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
            // Output stage has accepted our data — clear valid
            out_if.valid <= 0;
        end
    end

endmodule
