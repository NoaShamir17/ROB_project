// ============================================================================
// response_park  — Parking-lot for whole read RESPONSES (indexed by UNIQUE {row,col})
// ----------------------------------------------------------------------------
// PURPOSE
//   When an R-response cannot be delivered to the master yet (AXI per-ID ordering),
//   we temporarily park it here. Later, when your ordering logic decides a specific
//   UID ({row,col}) is now legal to send, it requests *that* UID from the park.
//
// KEY IDEAS
//   • One *whole* response per UID slot (single item, not a per-UID FIFO).
//   • Capacity = MAX_REQ-1 (e.g., 15 when NUM_ROWS*NUM_COLS=16). This leaves
//     one headroom slot in the system.
//   • Content-addressable by UID: you explicitly select which UID to hand out,
//     using (alloc_req, alloc_uid). The park does not choose ordering.
//   • A 'checked_out' bit keeps a handed-out response reserved until you say
//     free_req for its UID (so you can hold/use the output for multiple cycles).
//
// INTERFACE SUMMARY
//   ENQUEUE (park a response):
//     - in_valid, in_ready, in_uid, in_data/resp/orig_id/tagid
//     - Backpressures if pool is full OR that uid-slot is occupied.
//
//   ALLOCATE (hand out a specific parked response):
//     - alloc_req + alloc_uid    -> alloc_gnt + out_* for that UID (1-cycle pulse)
//     - Does NOT clear the slot; marks it 'checked_out' until free_req.
//
//   FREE (release a slot after consumer is done with it):
//     - free_req + id_to_release -> free_ack (1-cycle pulse), slot is reusable.
//
// NOTES
//   • If you later need multi-beat per UID, swap each slot to a small FIFO.
//   • If you want output ready/valid, wrap out_* with a skid and gate alloc_gnt.
// ============================================================================

module response_park #(
  // Geometry: total number of distinct internal IDs (UIDs) = NUM_ROWS*NUM_COLS
  parameter int NUM_ROWS    = 4,
  parameter int NUM_COLS    = 4,

  // Total UID count (e.g., 16). Pool capacity is MAX_REQ-1 (e.g., 15).
  parameter int MAX_REQ     = NUM_ROWS*NUM_COLS,

  // Payload widths for a "whole response"
  parameter int DATA_WIDTH  = 256, // as requested: max 256 bits per response
  parameter int RESP_WIDTH  = 2,
  parameter int ID_WIDTH    = 4,   // ORIGINAL RID width (optional to store here)
  parameter int TAG_WIDTH   = 4
)(
  input  logic clk,
  input  logic rst,

  // =================== ENQUEUE: park a whole response ======================
  input  logic                                        in_valid,
  output logic                                        in_ready,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] in_uid,
  input  logic [DATA_WIDTH-1:0]                       in_data,
  input  logic [RESP_WIDTH-1:0]                       in_resp,
  input  logic [ID_WIDTH-1:0]                         in_orig_id,
  input  logic [TAG_WIDTH-1:0]                        in_tagid,

  // =================== ALLOCATE: hand out a specific UID ===================
  input  logic                                        alloc_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] alloc_uid,

  // =================== FREE: release a slot after use ======================
  input  logic                                        free_req,
  input  logic [$clog2(NUM_ROWS)+$clog2(NUM_COLS)-1:0] id_to_release,

  // =================== OUTPUT: the handed-out response =====================
  output logic                                        alloc_gnt,
  output logic [DATA_WIDTH-1:0]                       out_data,
  output logic [RESP_WIDTH-1:0]                       out_resp,
  output logic [ID_WIDTH-1:0]                         out_orig_id,
  output logic [TAG_WIDTH-1:0]                        out_tagid,

  output logic                                        free_ack
);
  localparam int UID_W = $clog2(NUM_ROWS)+$clog2(NUM_COLS);

  logic                    v_q   [MAX_REQ],  v_d   [MAX_REQ];
  logic                    co_q  [MAX_REQ],  co_d  [MAX_REQ];
  logic [DATA_WIDTH-1:0]   data_q[MAX_REQ],  data_d[MAX_REQ];
  logic [RESP_WIDTH-1:0]   resp_q[MAX_REQ],  resp_d[MAX_REQ];
  logic [ID_WIDTH-1:0]     oid_q [MAX_REQ],  oid_d [MAX_REQ];
  logic [TAG_WIDTH-1:0]    tag_q [MAX_REQ],  tag_d [MAX_REQ];

  logic [$clog2(MAX_REQ):0] used_cnt_q, used_cnt_d;

  wire park_full  = (used_cnt_q == MAX_REQ-1);
  wire park_empty = (used_cnt_q == '0);

  assign in_ready = !park_full && !v_q[in_uid];

  always_comb begin
    alloc_gnt = 1'b0;
    free_ack  = 1'b0;

    out_data    = '0;
    out_resp    = '0;
    out_orig_id = '0;
    out_tagid   = '0;

    used_cnt_d = used_cnt_q;

    for (int i = 0; i < MAX_REQ; i++) begin
      v_d  [i] = v_q  [i];
      co_d [i] = co_q [i];
      data_d[i] = data_q[i];
      resp_d[i] = resp_q[i];
      oid_d [i] = oid_q [i];
      tag_d [i] = tag_q [i];
    end

    if (in_valid && in_ready) begin
      v_d   [in_uid] = 1'b1;
      co_d  [in_uid] = 1'b0;
      data_d[in_uid] = in_data;
      resp_d[in_uid] = in_resp;
      oid_d [in_uid] = in_orig_id;
      tag_d [in_uid] = in_tagid;
      used_cnt_d     = used_cnt_q + 1;
    end

    if (alloc_req && v_q[alloc_uid]) begin
      alloc_gnt    = 1'b1;
      out_data     = data_q[alloc_uid];
      out_resp     = resp_q[alloc_uid];
      out_orig_id  = oid_q [alloc_uid];
      out_tagid    = tag_q [alloc_uid];
      co_d[alloc_uid] = 1'b1;
    end

    if (free_req && (v_q[id_to_release] || co_q[id_to_release])) begin
      free_ack = 1'b1;
      if (!park_empty && v_q[id_to_release])
        used_cnt_d = used_cnt_q - 1;
      v_d [id_to_release] = 1'b0;
      co_d[id_to_release] = 1'b0;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      used_cnt_q <= '0;
      for (int i = 0; i < MAX_REQ; i++) begin
        v_q [i] <= 1'b0;
        co_q[i] <= 1'b0;
        data_q[i] <= '0;
        resp_q[i] <= '0;
        oid_q [i] <= '0;
        tag_q [i] <= '0;
      end
    end else begin
      used_cnt_q <= used_cnt_d;
      for (int i = 0; i < MAX_REQ; i++) begin
        v_q [i] <= v_d [i];
        co_q[i] <= co_d[i];
        data_q[i] <= data_d[i];
        resp_q[i] <= resp_d[i];
        oid_q [i] <= oid_d [i];
        tag_q [i] <= tag_d [i];
      end
    end
  end

endmodule
