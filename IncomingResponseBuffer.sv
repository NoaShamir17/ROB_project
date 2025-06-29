`include "fifo.sv"
`include "r_if.sv"

module IncomingResponseBuffer #(
    parameter ID_WIDTH   = 4,    // Width of AXI ID field
    parameter DATA_WIDTH = 64,   // Width of AXI data field
    parameter RESP_WIDTH = 2,    // Width of AXI response field
    parameter TAG_WIDTH  = 4,    // Internal tag ID width (not part of AXI)
    parameter FIFO_DEPTH = 16    // FIFO depth for buffering R responses
)(
    input  logic clk,            // Clock
    input  logic rst,            // Active-high synchronous reset

    r_if.receiver in_if,         // AXI slave sends the R channel response (we receive it)
    r_if.sender  out_if          // We forward response to the core/ROB (we send it)
);

    // Struct to hold one R-channel beat
    typedef struct packed {
        logic [ID_WIDTH-1:0]    id;
        logic [DATA_WIDTH-1:0]  data;
        logic [RESP_WIDTH-1:0]  resp;
        logic                   last;
        logic [TAG_WIDTH-1:0]   tagid;
    } r_resp_t;

    // Internal FIFO wiring
    r_resp_t fifo_in, fifo_out;
    logic fifo_empty, fifo_full;
    logic push_rsp, pop_rsp;

    // Pack incoming fields from AXI slave into the struct
    assign fifo_in = '{
        id:    in_if.id,
        data:  in_if.data,
        resp:  in_if.resp,
        last:  in_if.last,
        tagid: in_if.tagid
    };

    // Push into FIFO when data is valid and there's room
    assign push_rsp = in_if.valid && !fifo_full;

    // Tell the AXI slave (sender) that we're ready if FIFO has room
    assign in_if.ready = !fifo_full;

    // Pop from FIFO when:
    // - FIFO is not empty
    // - We're not holding a response already or receiver accepted it
    assign pop_rsp = !fifo_empty && (!out_if.valid || out_if.ready);

    // FIFO instance to buffer responses
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

    // Drive output toward core/ROB (we're acting like the sender now)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            out_if.valid <= 0;
        end
        // When we're popping, present a valid response
        else if (pop_rsp) begin
            out_if.valid <= 1;
            out_if.id    <= fifo_out.id;
            out_if.data  <= fifo_out.data;
            out_if.resp  <= fifo_out.resp;
            out_if.last  <= fifo_out.last;
            out_if.tagid <= fifo_out.tagid;
        end
        // When the receiver has accepted the data
        else if (out_if.ready && out_if.valid) begin
            out_if.valid <= 0;
        end
    end

endmodule
