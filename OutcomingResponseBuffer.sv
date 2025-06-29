`include "fifo.sv"
`include "r_if.sv"

module OutcomingResponseBuffer #(
    parameter ID_WIDTH   = 4,    // Width of AXI transaction ID
    parameter DATA_WIDTH = 64,   // Width of AXI data signal
    parameter RESP_WIDTH = 2,    // Width of AXI response signal
    parameter TAG_WIDTH  = 4,    // Internal tag for matching/tracking
    parameter FIFO_DEPTH = 16    // Depth of the FIFO for buffering
)(
    input  logic clk,            // Clock input
    input  logic rst,            // Active-high reset

    r_if.receiver in_if,         // Input from internal logic (like ROB) that receives R channel
    r_if.sender   out_if         // Output toward AXI master (external), sends R channel
);

    // Define a packed structure for storing R responses in the FIFO
    typedef struct packed {
        logic [ID_WIDTH-1:0]    id;     // ID of transaction
        logic [DATA_WIDTH-1:0]  data;   // Data to send
        logic [RESP_WIDTH-1:0]  resp;   // Response type
        logic                   last;   // Whether this is the last beat
        logic [TAG_WIDTH-1:0]   tagid;  // Internal tag
    } r_resp_t;

    // Internal signals for FIFO logic
    r_resp_t fifo_in, fifo_out;         // Data going into and out of FIFO
    logic fifo_empty, fifo_full;        // FIFO status flags
    logic push_rsp, pop_rsp;            // Control signals for pushing/popping

    // Pack incoming signals into a struct to write into the FIFO
    assign fifo_in = '{
        id:    in_if.id,      // Take transaction ID from receiver
        data:  in_if.data,    // Take data
        resp:  in_if.resp,    // Take response
        last:  in_if.last,    // Take last signal
        tagid: in_if.tagid    // Take internal tag
    };

    // Push into FIFO only when:
    // - input is valid (new data is ready)
    // - FIFO is not full (we have space)
    assign push_rsp = in_if.valid && !fifo_full;

    // Signal readiness to sender only if FIFO is not full
    assign in_if.ready = !fifo_full;

    // Pop from FIFO only when:
    // - FIFO is not empty (we have something to send)
    // - The output is not already holding a valid value, or it is ready to accept a new one
    assign pop_rsp = !fifo_empty && (!out_if.valid || out_if.ready);

    // FIFO instantiation
    fifo #(
        .DATA_WIDTH($bits(r_resp_t)),   // Width is equal to the packed struct
        .DEPTH(FIFO_DEPTH)              // FIFO depth as configured
    ) fifo_inst (
        .clk(clk),                      // Clock
        .rst(rst),                      // Reset
        .wr_en(push_rsp),              // Write enable
        .rd_en(pop_rsp),               // Read enable
        .din(fifo_in),                 // Data into FIFO
        .dout(fifo_out),               // Data out from FIFO
        .empty(fifo_empty),            // FIFO is empty?
        .full(fifo_full)               // FIFO is full?
    );

    // Drive the outgoing interface (toward AXI master)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            out_if.valid <= 0;  // Reset: nothing is valid yet
        end
        else if (pop_rsp) begin
            // Pop is enabled — meaning receiver wants data and we have something
            out_if.valid <= 1;             // Assert valid
            out_if.id    <= fifo_out.id;   // Output ID
            out_if.data  <= fifo_out.data; // Output data
            out_if.resp  <= fifo_out.resp; // Output response
            out_if.last  <= fifo_out.last; // Output last
            out_if.tagid <= fifo_out.tagid;// Output tag
        end
        else if (out_if.ready && out_if.valid) begin
            // Once output receiver accepts the data, deassert valid
            out_if.valid <= 0;
        end
    end

endmodule
