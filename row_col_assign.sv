module row_col_assign #(
    parameter ID_WIDTH        = 4,    // AXI ID width
    parameter ADDR_WIDTH      = 32,   // Memory address width
    parameter LEN_WIDTH       = 8,    // AXI burst length width
    parameter TAG_WIDTH       = 4,    // Internal tag width
    parameter FIFO_DEPTH      = 16,   // FIFO depth for buffering requests
    parameter MAX_OUTSTANDING = 16    // Max outstanding transactions supported
)(
    input  logic clk,            // Clock
    input  logic rst,            // Synchronous active-high reset
    input logic [ID_WIDTH-1:0] in_id,

    output logic [ID_WIDTH-1:0] out_id
    output logic [$clog2(MAX_OUTSTANDING)-1:0] unique_id;

    );

    
    typedef struct packed {
        logic used;
        logic [ID_WIDTH-1:0] id;
        logic available;
        //logic release;
    } row_variables_t;

    row_variables_t row_variables [0:MAX_OUTSTANDING-1];
    genvar i;
    generate
        for (i = 0; i < MAX_OUTSTANDING; i = i + 1) begin : row_init
            always_ff @(posedge clk) begin
                if (rst) begin
                    row_variables[i].used <= 1'b0;
                    row_variables[i].id <= '0;
                    row_variables[i].available <= 1'b1;
                end
            end
        end
    endgenerate



    
endmodule