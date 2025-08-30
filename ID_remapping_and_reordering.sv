module ID_remapping_and_reordering #(
    parameter ID_WIDTH        = 4,    // AXI ID width
    parameter ADDR_WIDTH      = 32,   // Memory address width
    parameter LEN_WIDTH       = 8,    // AXI burst length width
    parameter TAG_WIDTH       = 4,    // Internal tag width
    parameter FIFO_DEPTH      = 16,   // FIFO depth for buffering requests
    parameter MAX_OUTSTANDING = 16    // Max outstanding transactions supported
)(
    input  logic clk,            // Clock
    input  logic rst,            // Synchronous active-high reset

    ar_if.receiver ar_in_if,        // Incoming read address request from AXI master
    ar_if.sender   ar_out_if        // Outgoing to internal logic (e.g., tag remapping unit)
    r_if.sender   r_out_if      // Outgoing to AXI slave (after ID remapping)
    r_if.receiver r_in_if    // Incoming read data from AXI slave
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

    
    logic push_req, pop_req;            // Control signals for write/read
    logic matrix_full;
    logic [MAX_OUTSTANDING-1:0] used_rows;
    




endmodule