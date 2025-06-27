`include "fifo.sv"
`include "ar_if.sv"

module IncomingRequestBuffer #(
    parameter ID_WIDTH   = 4,
    parameter ADDR_WIDTH = 32,
    parameter LEN_WIDTH  = 8,
    parameter TAG_WIDTH  = 4,
    parameter FIFO_DEPTH = 16
)(
    input  logic clk,
    input  logic rst,

    // Interface from AXI master (external)
    ar_if.slave  in_if,

    // Interface to internal ID Remapping unit
    ar_if.master out_if
);

    // Define a struct to bundle the read request fields
    typedef struct packed {
        logic [ID_WIDTH-1:0]   id;
        logic [ADDR_WIDTH-1:0] addr;
        logic [LEN_WIDTH-1:0]  len;
        logic [2:0]            size;
        logic [1:0]            burst;
        logic [3:0]            qos;
        logic [TAG_WIDTH-1:0]  tagid;
    } ar_req_t;

    // Internal wires to connect to FIFO
    ar_req_t fifo_in, fifo_out;
    logic fifo_full, fifo_empty;
    logic push_req, pop_req;

    // Aggregate inputs from in_if into the FIFO input struct
    assign fifo_in = '{
        id:    in_if.id,
        addr:  in_if.addr,
        len:   in_if.len,
        size:  in_if.size,
        burst: in_if.burst,
        qos:   in_if.qos,
        tagid: in_if.tagid
    };

    // Push when valid and FIFO not full
    assign push_req = in_if.valid && !fifo_full;
    assign in_if.ready = !fifo_full;

    // Pop when FIFO not empty and downstream module is ready
    assign pop_req = !fifo_empty && !out_if.valid;

    // Instantiate FIFO
    fifo #(
        .DATA_WIDTH($bits(ar_req_t)),
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

    // Drive output to the ID Remapping unit
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            out_if.valid <= 0;
        end else if (pop_req) begin
            out_if.valid <= 1;
            out_if.id    <= fifo_out.id;
            out_if.addr  <= fifo_out.addr;
            out_if.len   <= fifo_out.len;
            out_if.size  <= fifo_out.size;
            out_if.burst <= fifo_out.burst;
            out_if.qos   <= fifo_out.qos;
            out_if.tagid <= fifo_out.tagid;
        end else if (out_if.ready) begin
            out_if.valid <= 0;
        end
    end

endmodule
