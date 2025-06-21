`include "fifo.sv"
`include "r_if.sv"

module IncomingResponseBuffer #(
    parameter ID_WIDTH   = 4,
    parameter DATA_WIDTH = 64,
    parameter RESP_WIDTH = 2,
    parameter TAG_WIDTH  = 4,
    parameter FIFO_DEPTH = 16
)(
    input  logic clk,
    input  logic rst,

    r_if.slave  in_if,   // Input from AXI slave (R channel)
    r_if.master out_if   // Output to internal ROB path
);

    // Struct for packed response
    typedef struct packed {
        logic [ID_WIDTH-1:0]    id;
        logic [DATA_WIDTH-1:0]  data;
        logic [RESP_WIDTH-1:0]  resp;
        logic                   last;
        logic [TAG_WIDTH-1:0]   tagid;
    } r_resp_t;

    r_resp_t fifo_in, fifo_out;
    logic fifo_full, fifo_empty, push_rsp, pop_rsp;

    // Aggregate slave response into struct
    assign fifo_in = '{
        id:    in_if.id,
        data:  in_if.data,
        resp:  in_if.resp,
        last:  in_if.last,
        tagid: in_if.tagid
    };

    // Accept response only if not full
    assign push_rsp = in_if.valid && !fifo_full;
    assign in_if.ready = !fifo_full;

    // Allow output if not empty and out_if not currently valid
    assign pop_rsp = !fifo_empty && !out_if.valid;

    // FIFO to buffer validated responses
    fifo #(
        .DATA_WIDTH($bits(r_resp_t)),
        .DEPTH(FIFO_DEPTH)
    ) fifo_inst (
        .clk(clk),
        .rst(rst),
        .wr_en(push_rsp),
        .rd_en(pop_rsp),
        .din(fifo_in),
        .dout(fifo_out),
        .empty(fifo_empty),
        .full(fifo_full)
    );

    // Drive output toward ROB
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            out_if.valid <= 0;
        end else if (pop_rsp) begin
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
