// ============================================================================
// top_module
// ----------------------------------------------------------------------------
// Description:
//   Connects the AXI Read Address/Data channels to the reordering logic,
//   utilizing FIFOs to decouple flow stages and an allocator for unique IDs.
//
// Component Flow:
// 1. AXI Master (axi_ar_in) -> incoming_request_buffer
// 2. incoming_request_buffer -> ar_ordering_unit (gets UID from allocator)
// 3. ar_ordering_unit -> outgoing_request_buffer
// 4. outgoing_request_buffer -> AXI Slave (axi_ar_out)
//
// 5. AXI Slave (axi_r_in) -> incoming_response_buffer
// 6. incoming_response_buffer -> r_ordering_unit + response_memory
// 7. r_ordering_unit -> outgoing_response_buffer
// 8. outgoing_response_buffer -> AXI Master (axi_r_out)
//
// Allocator is shared by ar_ordering_unit (alloc) and r_ordering_unit (free).
// AR and R roads are separated; only the allocator maps orig_id <-> UID.
// ============================================================================

module top #(
    parameter int ID_WIDTH        = 32,
    parameter int DATA_WIDTH      = 64,
    parameter int RESP_WIDTH      = 2,
    parameter int TAG_WIDTH       = 4,   // currently unused; kept for TB compat
    parameter int ADDR_WIDTH      = 32,
    parameter int MAX_OUTSTANDING = 16,
    parameter int MAX_LEN         = 8,

    // FIFO depths – match your buffer modules' DEPTH param
    parameter int REQ_FIFO_DEPTH  = 8,
    parameter int RESP_FIFO_DEPTH = 8
)(
    input  logic clk,
    input  logic rst,   // active-high

    // external AXI interfaces (interface-typed ports)
    ar_if.receiver axi_ar_in,    // AXI master -> incoming_request_buffer
    ar_if.sender   axi_ar_out,   // outgoing_request_buffer -> AXI slave

    r_if.sender    axi_r_out,    // outgoing_response_buffer -> AXI master
    r_if.receiver  axi_r_in      // AXI slave -> incoming_response_buffer
);

    // ------------------------------------------------------------------------
    // Local width parameters – must match ar_if / r_if definitions
    // ------------------------------------------------------------------------
    localparam int LEN_WIDTH   = 8;
    localparam int SIZE_WIDTH  = 3;
    localparam int BURST_WIDTH = 2;
    localparam int QOS_WIDTH   = 4;

    // for blocks that want conceptual rows/cols
    localparam int NUM_ROWS = MAX_OUTSTANDING;
    localparam int NUM_COLS = MAX_OUTSTANDING;

    // ------------------------------------------------------------------------
    // Internal interface instances
    // ------------------------------------------------------------------------

    // AR: master -> incoming_request_buffer -> ar_ordering_unit -> outgoing_request_buffer
    ar_if #(
        .ID_WIDTH    (ID_WIDTH),
        .ADDR_WIDTH  (ADDR_WIDTH),
        .LEN_WIDTH   (LEN_WIDTH),
        .SIZE_WIDTH  (SIZE_WIDTH),
        .BURST_WIDTH (BURST_WIDTH),
        .QOS_WIDTH   (QOS_WIDTH)
    ) ar_to_order_if ();   // incoming_request_buffer -> ar_ordering_unit

    ar_if #(
        .ID_WIDTH    (ID_WIDTH),
        .ADDR_WIDTH  (ADDR_WIDTH),
        .LEN_WIDTH   (LEN_WIDTH),
        .SIZE_WIDTH  (SIZE_WIDTH),
        .BURST_WIDTH (BURST_WIDTH),
        .QOS_WIDTH   (QOS_WIDTH)
    ) ar_from_order_if (); // ar_ordering_unit -> outgoing_request_buffer

    // R: incoming_response_buffer -> r_ordering_unit -> outgoing_response_buffer
    r_if #(
        .ID_WIDTH   (ID_WIDTH),      // carries UID (same width as allocator.unique_id)
        .DATA_WIDTH (DATA_WIDTH),
        .RESP_WIDTH (RESP_WIDTH)
    ) r_to_order_if ();    // incoming_response_buffer -> r_ordering_unit

    r_if #(
        .ID_WIDTH   (ID_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .RESP_WIDTH (RESP_WIDTH)
    ) r_from_order_if ();  // r_ordering_unit -> outgoing_response_buffer

    // Between r_ordering_unit and response_memory
    r_if #(
        .ID_WIDTH   (ID_WIDTH),      // UID width
        .DATA_WIDTH (DATA_WIDTH),
        .RESP_WIDTH (RESP_WIDTH)
    ) rm_store_if ();      // r_ordering_unit.r_store  -> response_memory.r_in

    r_if #(
        .ID_WIDTH   (ID_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .RESP_WIDTH (RESP_WIDTH)
    ) rm_release_if ();    // response_memory.r_out    -> r_ordering_unit.r_release

    // ------------------------------------------------------------------------
    // Allocator / tag-map signals
    // ------------------------------------------------------------------------
    logic                alloc_req_ar;        // from ar_ordering_unit
    logic                alloc_gnt;
    logic [ID_WIDTH-1:0] alloc_in_id;
    logic [ID_WIDTH-1:0] unique_id;
    logic                id_matrix_full;

    logic                allocator_free_req;  // from r_ordering_unit
    logic [ID_WIDTH-1:0] uid_to_restore;
    logic [ID_WIDTH-1:0] restored_id;

    // ------------------------------------------------------------------------
    // response_memory control signals (from r_ordering_unit)
    // ------------------------------------------------------------------------
    //logic [ID_WIDTH-1:0] rm_uid_to_alloc;
    //logic                rm_alloc_req;
    //logic [ID_WIDTH-1:0] rm_uid_to_free;
    //logic                rm_free_req;
    logic                rm_free_ack;

    // r_ordering_unit’s bookkeeping UID (not used by response_memory directly)
    logic [ID_WIDTH-1:0] rm_release_uid;

    // ------------------------------------------------------------------------
    // Allocator: maps original ID -> internal UID and back
    // ------------------------------------------------------------------------
    allocator #(
        .ID_WIDTH        (ID_WIDTH),
        .MAX_OUTSTANDING (MAX_OUTSTANDING)
        // NUM_ROWS / NUM_COLS use allocator defaults
    ) u_allocator (
        .clk              (clk),
        .rst              (rst),

        // alloc path (AR side)
        .alloc_req        (alloc_req_ar),
        .in_orig_id       (alloc_in_id),
        .alloc_gnt        (alloc_gnt),
        .unique_id        (unique_id),
        .id_matrix_full   (id_matrix_full),

        // free path (R side)
        .free_req         (allocator_free_req),
        .unique_id_to_free(uid_to_restore),
        .restored_id      (restored_id)
        // .free_ack unused
    );

    // ------------------------------------------------------------------------
    // AR path
    // ------------------------------------------------------------------------

    // incoming_request_buffer: AXI master -> ar_ordering_unit
    incoming_request_buffer #(
        .ID_WIDTH    (ID_WIDTH),
        .ADDR_WIDTH  (ADDR_WIDTH),
        .LEN_WIDTH   (LEN_WIDTH),
        .SIZE_WIDTH  (SIZE_WIDTH),
        .BURST_WIDTH (BURST_WIDTH),
        .QOS_WIDTH   (QOS_WIDTH),
        .DEPTH       (REQ_FIFO_DEPTH)
    ) u_incoming_request_buffer (
        .clk   (clk),
        .rst   (rst),
        .ar_in (axi_ar_in),        // external AR in
        .ar_out(ar_to_order_if)    // to ar_ordering_unit
    );

    // ar_ordering_unit: replaces original ID with UID from allocator
    ar_ordering_unit #(
        .ID_WIDTH    (ID_WIDTH),
        .ADDR_WIDTH  (ADDR_WIDTH),
        .LEN_WIDTH   (LEN_WIDTH),
        .SIZE_WIDTH  (SIZE_WIDTH),
        .BURST_WIDTH (BURST_WIDTH),
        .QOS_WIDTH   (QOS_WIDTH)
    ) u_ar_ordering_unit (
        .clk          (clk),
        .rst          (rst),

        .ar_in        (ar_to_order_if),
        .ar_out       (ar_from_order_if),

        // allocator handshake
        .alloc_req    (alloc_req_ar),
        .alloc_gnt    (alloc_gnt),
        .alloc_in_id  (alloc_in_id),
        .unique_id    (unique_id),
        .tag_map_full (id_matrix_full)
    );

    // outgoing_request_buffer: ar_ordering_unit -> AXI slave
    outgoing_request_buffer #(
        .ID_WIDTH    (ID_WIDTH),
        .ADDR_WIDTH  (ADDR_WIDTH),
        .LEN_WIDTH   (LEN_WIDTH),
        .SIZE_WIDTH  (SIZE_WIDTH),
        .BURST_WIDTH (BURST_WIDTH),
        .QOS_WIDTH   (QOS_WIDTH),
        .DEPTH       (REQ_FIFO_DEPTH)
    ) u_outgoing_request_buffer (
        .clk   (clk),
        .rst   (rst),
        .ar_in (ar_from_order_if),
        .ar_out(axi_ar_out)
    );

    // ------------------------------------------------------------------------
    // R path
    // ------------------------------------------------------------------------

    // incoming_response_buffer: AXI slave -> r_ordering_unit
    incoming_response_buffer #(
        .ID_WIDTH    (ID_WIDTH),     // UID width on R channel
        .DATA_WIDTH  (DATA_WIDTH),
        .RESP_WIDTH  (RESP_WIDTH),
        .DEPTH       (RESP_FIFO_DEPTH)
    ) u_incoming_response_buffer (
        .clk   (clk),
        .rst   (rst),
        .r_in  (axi_r_in),
        .r_out (r_to_order_if)
    );



    // response_memory: per-UID storage of response beats
    response_memory #(
        .NUM_UIDS   (MAX_OUTSTANDING),
        .MAX_BEATS  (MAX_LEN),
        .ID_WIDTH   (ID_WIDTH),      // UID width
        .DATA_WIDTH (DATA_WIDTH),
        .RESP_WIDTH (RESP_WIDTH)
    ) u_response_memory (
        .clk         (clk),
        .rst_n       (~rst),

        // write side from r_ordering_unit
        .r_in        (rm_store_if),

        // read side toward r_ordering_unit
        .r_out       (rm_release_if),


        // Control: pop one beat for some UID
        .uid_to_free (uid_to_restore)


        // Pop handshake (combinational: valid & ready)
        //.free_ack    (rm_free_ack)
    );





    // r_ordering_unit: enforces per-original-ID ordering and interacts with
    // allocator (for freeing UIDs) and response_memory (via r_store/r_release).
    r_ordering_unit #(
        .ID_WIDTH        (ID_WIDTH),
        .MAX_OUTSTANDING (MAX_OUTSTANDING),
        .NUM_ROWS        (NUM_ROWS),
        .NUM_COLS        (NUM_COLS),
        .MAX_LEN         (MAX_LEN)
    ) u_r_ordering_unit (
        .clk              (clk),
        .rst              (rst),

        // from incoming_response_buffer (R from fabric)
        .r_in             (r_to_order_if),

        // to outgoing_response_buffer (R to AXI master)
        .r_out            (r_from_order_if),

        // allocator: free UID when its response burst is fully drained
        .allocator_free_req(allocator_free_req),
        .uid_to_restore   (uid_to_restore),
        .restored_id      (restored_id),

        // response_memory data paths
        .r_store          (rm_store_if),      // writes beats into response_memory
        .rm_release_uid   (rm_release_uid),   // internal bookkeeping; not used by RM
        .r_release        (rm_release_if)    // reads beats from response_memory

    );

    // outgoing_response_buffer: r_ordering_unit -> AXI master
    outgoing_response_buffer #(
        .ID_WIDTH    (ID_WIDTH),
        .DATA_WIDTH  (DATA_WIDTH),
        .RESP_WIDTH  (RESP_WIDTH),
        .DEPTH       (RESP_FIFO_DEPTH)
    ) u_outgoing_response_buffer (
        .clk   (clk),
        .rst   (rst),
        .r_in  (r_from_order_if),
        .r_out (axi_r_out)
    );

endmodule
