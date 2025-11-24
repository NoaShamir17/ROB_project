// ============================================================================
// top_module
// ----------------------------------------------------------------------------
// Description:
//   Connects the AXI Read Address/Data channels to the reordering logic,
//   utilizing FIFOs to decouple the flow stages and an allocator for unique IDs.
//
// Component Flow:
// 1. AXI Master (axi_ar_in) -> incoming_request_buffer
// 2. incoming_request_buffer -> ar_ordering_unit (gets UID from allocator)
// 3. ar_ordering_unit -> outgoing_request_buffer
// 4. outgoing_request_buffer -> AXI Slave (axi_ar_out)
//
// 5. AXI Slave (axi_r_in) -> incoming_response_buffer
// 6. incoming_response_buffer -> r_ordering_unit (stores in response_memory/WM)
// 7. r_ordering_unit -> outgoing_response_buffer (releases ordered data)
// 8. outgoing_response_buffer -> AXI Master (axi_r_out)
//
// Allocator (allocator_tag_map) is shared by ar_ordering_unit (alloc) and
// r_ordering_unit (free + restored ID lookup).

//TODO: match all r_if and ar_if
//TODO: implement everything marked as TODO in the code below
//TODO: count how many cycles it takes for a request to go from axi_ar_in to axi_ar_out
//TODO: count how many cycles it takes for a response to go from axi_r_in to axi_r_out
//TODO: check if r_ordering_unit needs FSM like ar_ordering_unit
//TODO: add the "last" signal to response_memory and every where it's missing and check what happens with multi-beat requests/responses
// ============================================================================
module top #(
    parameter int ID_WIDTH          = 4,
    parameter int DATA_WIDTH        = 64,
    parameter int RESP_WIDTH        = 2,
    parameter int TAG_WIDTH         = 4,
    parameter int ADDR_WIDTH        = 32,
    parameter int MAX_OUTSTANDING   = 16,
    parameter int MAX_LEN           = 8
)(
    input  logic clk,
    input  logic rst,

    // external AXI interfaces (these are interface-typed ports)
    ar_if.receiver axi_ar_in,    // AXI master -> incoming request buffer
    ar_if.sender   axi_ar_out,   // outgoing request -> AXI slave

    r_if.sender    axi_r_out,    // outgoing response -> AXI master
    r_if.receiver  axi_r_in      // AXI slave -> incoming response buffer
);

    // derived widths
    localparam int NUM_ROWS = MAX_OUTSTANDING;
    localparam int NUM_COLS = MAX_OUTSTANDING;
    localparam int ROW_W = $clog2(NUM_ROWS);
    localparam int COL_W = $clog2(NUM_COLS);
    localparam int UID_W = ROW_W + COL_W;

    // ---------------------------
    // Internal interface instances
    // ---------------------------
    // ic_req: AXI AR after incoming_request_buffer -> input to ar_ordering_unit
    ar_if #(
        .ID_WIDTH(ID_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .LEN_WIDTH($clog2(MAX_LEN)) // TODO: confirm ar_if.len width param name/size
    ) ic_req_if ();

    // oc_req: output of ar_ordering_unit -> outgoing_request_buffer -> axi_ar_out
    ar_if #(
        .ID_WIDTH(ID_WIDTH),        // TODO: ar_ordering_unit will write UID into id field: confirm width expectations (may need UID_W)
        .ADDR_WIDTH(ADDR_WIDTH),
        .LEN_WIDTH($clog2(MAX_LEN))
    ) oc_req_if ();

    // ic_resp: AXI R from slave -> incoming_response_buffer -> input to r_ordering_unit
    r_if #(
        .ID_WIDTH(UID_W),           // TODO: this must hold UID coming from AR path; if your r_if uses original ID width, change accordingly
        .DATA_WIDTH(DATA_WIDTH),
        .RESP_WIDTH(RESP_WIDTH),
        .TAG_WIDTH(TAG_WIDTH)
    ) ic_resp_if ();

    // oc_resp: output of r_ordering_unit -> outgoing_response_buffer -> axi_r_out
    r_if #(
        .ID_WIDTH(ID_WIDTH),       // final outgoing response should have original master ID
        .DATA_WIDTH(DATA_WIDTH),
        .RESP_WIDTH(RESP_WIDTH),
        .TAG_WIDTH(TAG_WIDTH)
    ) oc_resp_if ();

    // ---------------------------
    // Allocator wires (shared)
    // ---------------------------
    logic                 alloc_req;
    logic [ID_WIDTH-1:0]  alloc_in_id;
    logic                 alloc_gnt;
    logic [UID_W-1:0]     alloc_unique_id;
    logic                 id_matrix_full;  // produced by allocator

    logic                 free_req;
    logic [UID_W-1:0]     free_unique_id;
    logic [ID_WIDTH-1:0]  restored_id;
    logic                 free_ack;        // optional

    // ---------------------------
    // response_memory wires
    // ---------------------------
    // exact response_memory ports are tool-dependent; expose request/release control
    logic                 wm_in_valid;
    logic                 wm_in_ready;   // TODO: connect if response_memory exposes
    logic [UID_W-1:0]     wm_in_uid;
    logic [DATA_WIDTH-1:0] wm_in_data;
    logic [RESP_WIDTH-1:0] wm_in_resp;
    logic                 wm_in_last;
    logic [ID_WIDTH-1:0]  wm_in_orig_id;
    logic [TAG_WIDTH-1:0] wm_in_tagid;

    logic                 wm_alloc_req;
    logic [UID_W-1:0]     wm_alloc_uid;
    logic                 wm_alloc_gnt;
    logic [DATA_WIDTH-1:0] wm_out_data;
    logic [RESP_WIDTH-1:0] wm_out_resp;
    logic                 wm_out_last;
    logic [ID_WIDTH-1:0]  wm_out_orig_id;
    logic [TAG_WIDTH-1:0] wm_out_tagid;
    logic                 wm_full;

    // -------------------------------------------------------------------------
    // Instantiate the modules requested (connect interfaces where sensible).
    // NOTE: exact port names / directions for these modules may differ in your
    // repo. I use typical port names below. Wherever a port name/width is not
    // known I added a //TODO: port unused/missing comment.
    // -------------------------------------------------------------------------

    // allocator
    allocator #(
        .ID_WIDTH(ID_WIDTH),
        .MAX_OUTSTANDING(MAX_OUTSTANDING)
    ) u_allocator (
        .clk               (clk),
        .rst               (rst),
        .alloc_req         (alloc_req),
        .in_orig_id        (alloc_in_id),
        .alloc_gnt         (alloc_gnt),
        .unique_id         (alloc_unique_id),
        .id_matrix_full    (id_matrix_full),
        .free_req          (free_req),
        .unique_id_to_free (free_unique_id),
        .restored_id       (restored_id)
        // .free_ack       (free_ack) // TODO: port unused/missing if allocator supports free_ack
    );

    // incoming_request_buffer: connect external axi_ar_in -> ic_req_if
    incoming_request_buffer #(/* TODO: add params if module is parameterized */) u_incoming_request_buffer (
        .clk     (clk),
        .rst     (rst),
        .in_if   (axi_ar_in),   // TODO: if module expects modport type name different, update
        .out_if  (ic_req_if)
    );

    // outgoing_request_buffer: connect oc_req_if -> external axi_ar_out
    outgoing_request_buffer #(/* TODO: params */) u_outcoming_request_buffer (
        .clk     (clk),
        .rst     (rst),
        .in_if   (oc_req_if),
        .out_if  (axi_ar_out)
    );

    // ar_ordering_unit: get UIDs from allocator; s_ar = ic_req_if, m_ar = oc_req_if
    ar_ordering_unit #(
        .ID_WIDTH(ID_WIDTH),
        .MAX_OUTSTANDING(MAX_OUTSTANDING)
        // TODO: add other params (UID width etc.) if module defines them
    ) u_ar_id_ordering_unit (
        .clk         (clk),
        .rst         (rst),
        .s_ar        (ic_req_if),   // receiver modport expected by unit
        .m_ar        (oc_req_if),   // sender modport expected by unit

        // allocator handshake (names must match unit's ports)
        .alloc_req   (alloc_req),       // TODO: port name check
        .alloc_in_id (alloc_in_id),     // TODO: port name check
        .alloc_gnt   (alloc_gnt),       // TODO: port name check
        .unique_id   (alloc_unique_id)  // TODO: port name check
    );

    // incoming_response_buffer: connect external axi_r_in -> ic_resp_if
    incoming_response_buffer #(/* TODO: params */) u_incoming_response_buffer (
        .clk     (clk),
        .rst     (rst),
        .in_if   (axi_r_in),
        .out_if  (ic_resp_if)
    );

    // outgoing_response_buffer: connect oc_resp_if -> external axi_r_out
    outgoing_response_buffer #(/* TODO: params */) u_outcoming_response_buffer (
        .clk     (clk),
        .rst     (rst),
        .in_if   (oc_resp_if),
        .out_if  (axi_r_out)
    );

    // response_memory: store partial/multi-beat responses (ports vary across your file)
    response_memory #(
        .NUM_ROWS(NUM_ROWS),
        .NUM_COLS(NUM_COLS),
        .MAX_REQ(MAX_OUTSTANDING),
        .DATA_WIDTH(DATA_WIDTH),
        .RESP_WIDTH(RESP_WIDTH),
        .ID_WIDTH(ID_WIDTH),
        .TAG_WIDTH(TAG_WIDTH)
    ) u_response_memory (
        .clk         (clk),
        .rst         (rst),

        // write path: driven by r_ordering_unit's "store" handshake
        .in_valid    (wm_in_valid),    // TODO: connect to r_ordering_unit store-valid
        .in_ready    (wm_in_ready),    // TODO: check if response_memory exposes in_ready
        .in_uid      (wm_in_uid),      // UID
        .in_data     (wm_in_data),
        .in_resp     (wm_in_resp),
        .in_last     (wm_in_last),
        .in_orig_id  (wm_in_orig_id),
        .in_tagid    (wm_in_tagid),

        // release (alloc) request from r_ordering_unit
        .alloc_req   (wm_alloc_req),   // r_ordering_unit asks park to release next ordered resp
        .alloc_uid   (wm_alloc_uid),   // requested UID to release
        .alloc_gnt   (wm_alloc_gnt),
        .out_data    (wm_out_data),
        .out_resp    (wm_out_resp),
        .out_last    (wm_out_last),
        .out_orig_id (wm_out_orig_id),
        .out_tagid   (wm_out_tagid),

        .free_req    (free_req),        // free request from r_ordering_unit -> allocator
        .id_to_release (free_unique_id),
        .full        (wm_full)
        // .free_ack  () // TODO: if response_memory exposes free_ack, connect or mark unused
    );

    // r_ordering_unit: read from ic_resp_if, write to response_memory and/or oc_resp_if
    r_ordering_unit #(
        .ID_WIDTH(ID_WIDTH),
        .MAX_OUTSTANDING(MAX_OUTSTANDING),
        .NUM_ROWS(NUM_ROWS),
        .NUM_COLS(NUM_COLS)
        // TODO: add UID width param if unit defines it
    ) u_r_ordering_unit (
        .clk          (clk),
        .rst          (rst),

        // response input (from ic_resp_if)
        .r_in         (ic_resp_if),

        // response output (to oc_resp_if)
        .r_out        (oc_resp_if),

        // response-park handshake (store releases)
        .wm_write_en  (wm_in_valid),    // TODO: exact port names may differ
        .wm_write_uid (wm_in_uid),
        .wm_write_data_id   (wm_in_orig_id), // TODO: many modules expose structured ports; adapt as needed
        .wm_write_data_data (wm_in_data),
        .wm_write_data_resp (wm_in_resp),
        .wm_write_data_last (wm_in_last),
        .wm_write_data_tag  (wm_in_tagid),

        .wm_release_en (wm_alloc_req),
        .wm_release_uid (wm_alloc_uid),
        .wm_release_gnt (wm_alloc_gnt), // TODO: may be output from response_memory instead

        // allocator free request (when a UID is finished and can be freed)
        .free_req     (free_req),
        .free_uid     (free_unique_id),

        // restored id from allocator (if r_ordering_unit needs lookups)
        .restored_id  (restored_id) // TODO: ensure r_ordering_unit has this input
    );

    // connect response_memory release outputs into oc_resp_if (if not done inside r_ordering_unit)
    // Common patterns:
    // - r_ordering_unit asks park to release and then forwards park's out_* into its r_out -> oc_resp_if
    // If your r_ordering_unit does NOT forward park outputs, you need to add assigns here.
    // Example (uncomment/adapt if needed):
    // assign oc_resp_if.id   = wm_out_orig_id;
    // assign oc_resp_if.data = wm_out_data;
    // assign oc_resp_if.resp = wm_out_resp;
    // assign oc_resp_if.last = wm_out_last;
    // assign oc_resp_if.tagid= wm_out_tagid;
    // // and drive oc_resp_if.valid from wm_alloc_gnt / r_ordering_unit release handshake

endmodule
