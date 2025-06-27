`include "fifo.sv"
`include "r_if.sv"

module OutcomingResponseBuffer #(
    parameter ID_WIDTH   = 4,
    parameter DATA_WIDTH = 64,
    parameter RESP_WIDTH = 2,
    parameter TAG_WIDTH  = 4,
    parameter FIFO_DEPTH = 16
)(
    input  logic clk,     // Clock signal
    input  logic rst,     // Reset signal

    r_if.slave  in_if,    // Input from ROB (internal logic)
    r_if.master out_if    // Output to AXI master (external)
);

    // Packed response structure for buffering
    typedef struct packed {
        logic [ID_WIDTH-1:0]    id;
        logic [DATA_WIDTH-1:0]  data;
        logic [RESP_WIDTH-1:0]  resp;
        logic                   last;
        logic [TAG_WIDTH-1:0]   tagid;
    } r_resp_t;

    // Internal wires for FIFO logic
    r_resp_t fifo_in, fifo_out;
    logic fifo_empty, fifo_full;
    logic push_resp, pop_resp;

    // Bundle signals from input interface
    assign fifo_in = '{
        id:    in_if.id,
        data:  in_if.data,
        resp:  in_if.resp,
        last:  in_if.last,
        tagid: in_if.tagid
    };

    // Write to FIFO when input is valid and FIFO is not full
    assign push_resp = in_if.valid && !fifo_full;
    assign in_if.ready = !fifo_full;

    // Read from FIFO when not empty and out_if is ready
    assign pop_resp = !fifo_empty && !out_if.valid && out_if.ready;

    // Instantiate FIFO for buffering outgoing responses
    fifo #(
        .DATA_WIDTH($bits(r_resp_t)),
        .DEPTH(FIFO_DEPTH)
    ) fifo_inst (
        .clk(clk),
        .rst(rst),
        .wr_en(push_resp),
        .rd_en(pop_resp),
        .din(fifo_in),
        .dout(fifo_out),
        .empty(fifo_empty),
        .full(fifo_full)
    );

    // Drive output to AXI master interface
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            out_if.valid <= 0;
        end else if (pop_resp) begin
            out_if.valid <= 1;
            out_if.id    <= fifo_out.id;
            out_if.data  <= fifo_out.data;
            out_if.resp  <= fifo_out.resp;
            out_if.last  <= fifo_out.last;
            out_if.tagid <= fifo_out.tagid;
        end else if (out_if.ready) begin
            out_if.valid <= 0;
        end
    end

endmodule
