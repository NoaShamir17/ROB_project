`include "fifo.sv"
`include "axi_ar_if.sv"

module OutcomingRequestBuffer #(
    parameter ID_WIDTH = 4,
    parameter ADDR_WIDTH = 32,
    parameter LEN_WIDTH = 8,
    parameter TAG_WIDTH = 4,
    parameter FIFO_DEPTH = 16
)(
    input  logic clk,               // Clock signal
    input  logic rst,               // Reset signal

    axi_ar_if.slave in_if,          // Input interface from ROB (AXI master)
    axi_ar_if.master out_if         // Output interface to AXI slave
);

    // Define a bundled struct for AXI Read requests (to be sent in FIFO) - maybe should be changed in the future
    typedef struct packed {
        logic [ID_WIDTH-1:0]   id;
        logic [ADDR_WIDTH-1:0] addr;
        logic [LEN_WIDTH-1:0]  len;
        logic [2:0]            size;
        logic [1:0]            burst;
        logic [3:0]            qos;
        logic [TAG_WIDTH-1:0]  tagid;
    } ar_req_t;

    // Internal variables: request going into FIFO, request coming out of FIFO
    ar_req_t fifo_in, fifo_out;
    logic fifo_empty, fifo_full;
    logic push_req, pop_req;

    // Aggregate incoming signals from in_if into a struct
    assign fifo_in = '{
        id:    in_if.id,
        addr:  in_if.addr,
        len:   in_if.len,
        size:  in_if.size,
        burst: in_if.burst,
        qos:   in_if.qos,
        tagid: in_if.tagid
    };

    // Push only if input is valid and FIFO has room
    assign push_req = in_if.valid && !fifo_full;

    // Pop only if FIFO is not empty AND output interface is not asserting valid AND ready
    assign pop_req  = !fifo_empty && !out_if.valid && out_if.ready;

    // Tell the master it can send more requests
    assign in_if.ready = !fifo_full;

    // Instantiate FIFO with struct width
    fifo #(
        .DATA_WIDTH($bits(ar_req_t)),  // Pass struct width as a flat integer
        .DEPTH(FIFO_DEPTH)
    ) fifo_inst (
        .clk(clk),
        .rst(rst),
        .wr_en(push_req),
        .rd_en(pop_req),
        .din(fifo_in),
        .dout(fifo_out),
        .empty(fifo_empty),
        .full(fifo_full)
    );

    // output logic
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset output valid
            out_if.valid <= 0;
        end else if (pop_req) begin
            // Send out a request from FIFO
            out_if.valid <= 1;
            out_if.id    <= fifo_out.id;
            out_if.addr  <= fifo_out.addr;
            out_if.len   <= fifo_out.len;
            out_if.size  <= fifo_out.size;
            out_if.burst <= fifo_out.burst;
            out_if.qos   <= fifo_out.qos;
            out_if.tagid <= fifo_out.tagid;
        end else if (out_if.ready) begin
            // Once accepted, drop valid
            out_if.valid <= 0;
        end
    end

endmodule
