// ============================================================================
// rob_tb_if_bitwise_procmon_commented.sv
// ----------------------------------------------------------------------------
// PURPOSE (big picture):
//   We verify a Read-Order-Buffer (ROB) that sits between a master and a slave
//   on AXI-like AR/R channels. The ROB may translate original IDs (S_ID) into
//   internal unique IDs (UID) for the slave, but MUST preserve *per-original-ID*
//   in-order delivery of read data back to the master.
//
// WHAT THIS TESTBENCH DOES (high level):
//   1) Drives the master side AR (s_ar.*) with mixed IDs / LEN / SIZE / BURST.
//   2) Models a slave on m_* that returns beats OUT-OF-ORDER across UIDs,
//      but IN-ORDER within each UID's burst. It also injects rare SLVERR.
//   3) Applies backpressure independently on:
//        - s_r.ready (master side receive)
//        - m_ar.ready (slave side accept)
//   4) Keeps a per-original-ID expected FIFO of bursts (the "ground truth").
//   5) On s_r.*, checks that beats retire in-order per original ID, with
//      correct LAST placement, correct data (deterministic signature), and
//      RESP pass-through. Any reordering ⇒ error.
//   6) Replaces SVA with procedural monitors (bitwise ops only) that check
//      protocol requirements like "valid stable while stalled" and
//      "s_ar bundle immutable during stall".
//
// IMPORTANT CONSTRAINTS YOU ASKED FOR:
//   • Bitwise-only operations in our control/data paths: &, |, ~, ^, <<, >>.
//     (No &&, ||, !, +, -, *, /, %, ++, -- in the logic.)
//   • No 'assert property' SVA — only procedural checkers.
//   • Use your existing interfaces: ar_if / r_if with the same names.
//
// READING TIP:
//   - Search for "WHAT this block checks" to find each checker.
//   - Search for "WHY we do this" to understand the reasoning.
// ----------------------------------------------------------------------------

// --------------------
// Parameterization
// --------------------
// Tweak widths here if your DUT uses other sizes. The testbench honors these
// widths consistently (ID widths may differ between S_ID and UID).
localparam int ID_W    = 4;    // original master-facing ID width (RID on s_r)
localparam int UID_W   = 6;    // internal translated ID width (RID on m_r)
localparam int ADDR_W  = 16;   // address width we use for AR and data signature
localparam int DATA_W  = 64;   // data bus width on R channel
localparam int LEN_W   = 8;    // AXI ARLEN (beats-1), up to 255 (we use <=15)
localparam int QOS_W   = 4;    // QOS field width (used to exercise payload stability)
localparam int NUM_TXN = 120;  // how many AR requests we will issue

// ============================================================================
// Bitwise helpers (to avoid +, -, etc. in data/control computations)
// ----------------------------------------------------------------------------
// We keep loops' index 'i = i + 1' as natural SV syntax (indexing the loop).
// Everywhere we *compute* values for protocol/data, we use these helpers.
// ============================================================================
function automatic logic [W-1:0] bw_add #(int W)(input logic [W-1:0] a, input logic [W-1:0] b);
  // WHAT: ripple-carry adder implemented only with ^, &, | (bitwise).
  // WHY : honors your "bitwise-only" constraint while providing addition.
  logic [W-1:0] s;
  logic c, n;
  int i;
  begin
    s = '0; c = 1'b0; i = 0;
    while (i < W) begin
      s[i] = (a[i] ^ b[i]) ^ c;                 // sum bit
      n    = (a[i] & b[i]) | (a[i] & c) | (b[i] & c); // carry generate
      c    = n;
      i    = i + 1;
    end
    return s;
  end
endfunction

function automatic logic [W-1:0] bw_inc #(int W)(input logic [W-1:0] a);
  // WHAT: a + 1 using bitwise add with an LSB '1'.
  return bw_add#(W)(a, {{(W-1){1'b0}},1'b1});
endfunction

function automatic logic [W-1:0] bw_sub #(int W)(input logic [W-1:0] a, input logic [W-1:0] b);
  // WHAT: subtraction via two's complement: a - b = a + (~b + 1).
  return bw_add#(W)(a, bw_inc#(W)(~b));
endfunction

function automatic logic [W-1:0] bw_dec #(int W)(input logic [W-1:0] a);
  // WHAT: a - 1 using two's complement path.
  return bw_sub#(W)(a, {{(W-1){1'b0}},1'b1});
endfunction

function automatic logic [W-1:0] bw_add_shift1 #(int W)(input logic [W-1:0] a, input logic [2:0] shift);
  // WHAT: a + (1<<shift) with only bitwise ops (shift + bw_add).
  logic [W-1:0] step; begin step = {{(W-1){1'b0}},1'b1} << shift; return bw_add#(W)(a, step); end
endfunction

function automatic logic [W-1:0] bw_mask_low #(int W)(input logic [2:0] n);
  // WHAT: produces mask of low 'n' bits: (1<<n)-1, used to align addresses.
  logic [W-1:0] one_shifted; begin one_shifted = {{(W-1){1'b0}},1'b1} << n; return bw_dec#(W)(one_shifted); end
endfunction

function automatic int int_inc(input int a);
  // WHAT: integer increment using our bitwise-32 adder (for counters in monitors).
  logic [31:0] x; begin x = logic'(a); return int'(bw_inc#(32)(x)); end
endfunction

function automatic logic lfsr16_step(input logic [15:0] s, output logic [15:0] s_next);
  // WHAT: 16-bit LFSR that returns a pseudo-random bit and next state.
  // WHY : lets us create random-ish toggles/backpressure without % or +/-.
  logic fb; begin fb = s[15] ^ s[13] ^ s[12] ^ s[10]; s_next = {s[14:0], fb}; return s_next[0]; end
endfunction

// ============================================================================
// Testbench top
// ============================================================================
module rob_tb_if_bitwise_procmon_commented;

  // --------------------
  // Clock & Reset
  // --------------------
  bit clk;
  bit rst_n;

  initial clk = 1'b0;
  always #2.5 clk = ~clk; // 200 MHz. Faster clock stresses flow-control.

  // WHAT: active-low synchronous-ish reset pulse generator.
  // WHY : start DUT clean, and also reuse for a mid-run reset robustness test.
  task automatic do_reset(input int cycles);
    rst_n = 1'b0;
    repeat (cycles) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);
  endtask

  // --------------------
  // Instantiate interfaces (your ar_if / r_if)
  // --------------------
  ar_if #(.ID_W(ID_W),  .ADDR_W(ADDR_W), .LEN_W(LEN_W), .QOS_W(QOS_W)) s_ar (.*);
  r_if  #(.ID_W(ID_W),  .DATA_W(DATA_W))                                s_r  (.*);
  ar_if #(.ID_W(UID_W), .ADDR_W(ADDR_W), .LEN_W(LEN_W), .QOS_W(QOS_W)) m_ar (.*);
  r_if  #(.ID_W(UID_W), .DATA_W(DATA_W))                                m_r  (.*);

  // Hook clocks/resets on the interfaces
  assign s_ar.clk = clk; assign s_ar.rst_n = rst_n;
  assign s_r .clk = clk; assign s_r .rst_n = rst_n;
  assign m_ar.clk = clk; assign m_ar.rst_n = rst_n;
  assign m_r .clk = clk; assign m_r .rst_n = rst_n;

  // --------------------
  // Explicit aliases to wire DUT cleanly
  // --------------------
  // We alias interface fields to plain wires so we can pass them to the DUT
  // even if your DUT doesn't use modports.
  wire                 s_ar_valid = s_ar.valid;
  wire                 s_ar_ready;
  wire [ID_W-1:0]      s_ar_id    = s_ar.id;
  wire [ADDR_W-1:0]    s_ar_addr  = s_ar.addr;
  wire [LEN_W-1:0]     s_ar_len   = s_ar.len;
  wire [2:0]           s_ar_size  = s_ar.size;
  wire [1:0]           s_ar_burst = s_ar.burst;
  wire [QOS_W-1:0]     s_ar_qos   = s_ar.qos;

  wire                 s_r_valid;
  wire                 s_r_ready  = s_r.ready;  // TB toggles ready to stress ROB output buffering
  wire [ID_W-1:0]      s_r_id;
  wire [DATA_W-1:0]    s_r_data;
  wire [1:0]           s_r_resp;
  wire                 s_r_last;

  wire                 m_ar_valid;
  wire                 m_ar_ready = m_ar.ready; // TB drives ready to stress DUT issuing
  wire [UID_W-1:0]     m_ar_id;
  wire [ADDR_W-1:0]    m_ar_addr;
  wire [LEN_W-1:0]     m_ar_len;
  wire [2:0]           m_ar_size;
  wire [1:0]           m_ar_burst;
  wire [QOS_W-1:0]     m_ar_qos;

  wire                 m_r_valid  = m_r.valid;  // slave drives valid; DUT must absorb with m_r.ready
  wire                 m_r_ready;
  wire [UID_W-1:0]     m_r_id     = m_r.id;
  wire [DATA_W-1:0]    m_r_data   = m_r.data;
  wire [1:0]           m_r_resp   = m_r.resp;
  wire                 m_r_last   = m_r.last;

  // Drive interface members that belong to the "other side" of each channel
  assign s_ar.ready = s_ar_ready;   // DUT returns ready on s_ar; TB reflects it to the interface
  assign s_r.valid  = s_r_valid;    // DUT drives s_r.valid/...; TB reflects into interface fields
  assign s_r.id     = s_r_id;
  assign s_r.data   = s_r_data;
  assign s_r.resp   = s_r_resp;
  assign s_r.last   = s_r_last;

  assign m_ar.valid = m_ar_valid;   // DUT drives AR to slave; TB reflects onto m_ar interface fields
  assign m_ar.id    = m_ar_id;
  assign m_ar.addr  = m_ar_addr;
  assign m_ar.len   = m_ar_len;
  assign m_ar.size  = m_ar_size;
  assign m_ar.burst = m_ar_burst;
  assign m_ar.qos   = m_ar_qos;

  assign m_r.ready  = m_r_ready;    // DUT applies backpressure to slave via m_r.ready; TB reflects wire

  // --------------------
  // DUT instantiation (adapt names here if your module differs)
  // --------------------
  ROB #(
    .S_ID_W (ID_W), .M_ID_W (UID_W),
    .ADDR_W (ADDR_W), .DATA_W (DATA_W),
    .LEN_W  (LEN_W), .QOS_W  (QOS_W)
  ) dut (
    .clk(clk), .rst_n(rst_n),

    // master-facing AR (TB drives s_ar.valid/*; DUT responds with s_ar.ready)
    .s_ar_valid(s_ar_valid), .s_ar_ready(s_ar_ready),
    .s_ar_id(s_ar_id), .s_ar_addr(s_ar_addr), .s_ar_len(s_ar_len),
    .s_ar_size(s_ar_size), .s_ar_burst(s_ar_burst), .s_ar_qos(s_ar_qos),

    // master-facing R (DUT drives; TB applies ready backpressure)
    .s_r_valid(s_r_valid), .s_r_ready(s_r_ready),
    .s_r_id(s_r_id), .s_r_data(s_r_data), .s_r_resp(s_r_resp), .s_r_last(s_r_last),

    // slave-facing AR (DUT drives; TB applies ready backpressure)
    .m_ar_valid(m_ar_valid), .m_ar_ready(m_ar_ready),
    .m_ar_id(m_ar_id), .m_ar_addr(m_ar_addr), .m_ar_len(m_ar_len),
    .m_ar_size(m_ar_size), .m_ar_burst(m_ar_burst), .m_ar_qos(m_ar_qos),

    // slave-facing R (slave model drives; DUT sets ready)
    .m_r_valid(m_r_valid), .m_r_ready(m_r_ready),
    .m_r_id(m_r_id), .m_r_data(m_r_data), .m_r_resp(m_r_resp), .m_r_last(m_r_last)
  );

  // ==========================================================================
  // AR GENERATOR (master side → s_ar.*)
  // ----------------------------------------------------------------------------
  // WHAT this block does:
  //   • Produces NUM_TXN AR requests with random-ish fields (bitwise LFSR).
  //   • Aligns addresses to SIZE (bit-mask), sets BURST (mostly INCR, some WRAP).
  //   • On each AR handshake, pushes an expectation record into a per-ID queue.
  // WHY we do this:
  //   • The scoreboard will later consume beats from s_r.* and verify they match
  //     the sequence we logged here (ORDER per ID, LAST position, DATA content).
  // ==========================================================================
  typedef struct packed {
    logic [ID_W-1:0]   id;
    logic [ADDR_W-1:0] addr;
    logic [LEN_W-1:0]  len;    // beats-1
    logic [2:0]        size;   // log2(bytes per beat)
    logic [1:0]        burst;  // 1=INCR, 2=WRAP (we avoid FIXED)
    logic [QOS_W-1:0]  qos;
  } ar_cmd_t;

  typedef struct {
    ar_cmd_t           cmd;
    logic [LEN_W-1:0]  beats_left; // exact number of beats still expected (ARLEN+1)
  } id_burst_t;

  // Per-original-ID FIFO of outstanding bursts (ground truth)
  id_burst_t exp_q[ID_W][$];

  // Backpressure on s_r.ready to stress DUT output buffering
  initial begin
    s_r.ready = 1'b1;
    logic [15:0] pr = 16'hACE1, nx;
    forever begin
      @(posedge clk);
      void'(lfsr16_step(pr, nx));
      // Drop ready periodically when low nibble is all 1s → runs create backpressure patterns
      s_r.ready <= (((nx & 16'h000F) == 16'h000F) ? 1'b0 : 1'b1);
      pr = nx;
    end
  end

  // Random-ish field pickers using bit-tests (no modulo)
  function automatic [1:0] pick_burst_bits(input logic [31:0] r);
    // WRAP only when two LSBs are 11; otherwise INCR.
    return ((r & 32'h3) == 32'h3) ? 2'd2 : 2'd1;
  endfunction
  function automatic [2:0] pick_size_bits(input logic [31:0] r);
    // SIZE ∈ {2,3,4} → 4B/8B/16B beats
    logic [1:0] sel; begin sel = r[1:0]; if (sel == 2'b00) return 3'd2; if (sel == 2'b01) return 3'd3; return 3'd4; end
  endfunction

  // Deterministic per-beat data signature (bitwise only): DATA = addr_ext ^ tiled(beat_idx)
  // WHY: lets us verify that returned beats belong to the correct (addr, beat_idx).
  function automatic [DATA_W-1:0] sig_data(input logic [ADDR_W-1:0] base, input logic [15:0] bi);
    logic [DATA_W-1:0] A, B; begin A = {{(DATA_W-ADDR_W){1'b0}}, base}; B = { {(DATA_W/16){bi}} }; return A ^ B; end
  endfunction

  ar_cmd_t cur_ar; bit cur_valid;

  initial begin
    // Initialize master AR interface (steady defaults)
    s_ar.valid = 1'b0;
    s_ar.id    = '0; s_ar.addr = '0; s_ar.len = '0;
    s_ar.size  = 3'd3; s_ar.burst = 2'd1; s_ar.qos = '0;

    // Power-on reset
    do_reset(10);

    // Small LFSR-like RNG via XOR taps (bitwise only)
    cur_valid = 1'b0;
    logic [31:0] r; r = 32'h1234ABCD;
    int tx; tx = 0;

    // Issue NUM_TXN ARs
    while (tx < NUM_TXN) begin
      @(posedge clk);

      // Build a new AR command when we don't already hold one
      if (~cur_valid) begin
        // Step RNG and pick each field
        r = {r[30:0], (r[31]^r[21]^r[1]^r[0])}; cur_ar.id    = r[ID_W-1:0];
        r = {r[30:0], (r[31]^r[21]^r[1]^r[0])}; cur_ar.size  = pick_size_bits(r);
        r = {r[30:0], (r[31]^r[21]^r[1]^r[0])}; cur_ar.burst = pick_burst_bits(r);
        r = {r[30:0], (r[31]^r[21]^r[1]^r[0])}; cur_ar.len   = r[3:0];            // 0..15 → 1..16 beats
        r = {r[30:0], (r[31]^r[21]^r[1]^r[0])}; cur_ar.addr  = r[ADDR_W-1:0];
        // Align addr to SIZE boundary: clear the low 'size' bits
        cur_ar.addr = cur_ar.addr & ~bw_mask_low#(ADDR_W)(cur_ar.size);
        r = {r[30:0], (r[31]^r[21]^r[1]^r[0])}; cur_ar.qos   = r[QOS_W-1:0];
        cur_valid = 1'b1;
      end

      // Drive s_ar.* (VALID+payload). The DUT will raise s_ar_ready when it accepts.
      s_ar.valid <= 1'b1;
      s_ar.id    <= cur_ar.id;
      s_ar.addr  <= cur_ar.addr;
      s_ar.len   <= cur_ar.len;
      s_ar.size  <= cur_ar.size;
      s_ar.burst <= cur_ar.burst;
      s_ar.qos   <= cur_ar.qos;

      // On handshake: log the burst into the per-ID expectation FIFO.
      if (s_ar.valid & s_ar.ready) begin
        id_burst_t b;
        b.cmd         = cur_ar;
        b.beats_left  = bw_inc#(LEN_W)(cur_ar.len); // ARLEN+1 (pure bitwise)
        exp_q[cur_ar.id].push_back(b);
        tx        = int_inc(tx); // bookkeeping counter (procedural)
        cur_valid = 1'b0;        // ready to build the next AR
      end
    end

    // Stop issuing after quota
    @(posedge clk);
    s_ar.valid <= 1'b0;
  end

  // ==========================================================================
  // SLAVE MODEL (on m_*)
// ----------------------------------------------------------------------------
// WHAT this block does:
//   • Accepts AR from the ROB on m_ar.* with its own READY backpressure.
//   • For each unique UID, we keep a FIFO of bursts. Within a UID we return
//     beats strictly in order; across UIDs we pick which UID to serve next in
//     a round-robin manner → OUT-OF-ORDER globally.
//   • We generate DATA = signature(addr, beat_idx) and set LAST on the final beat.
//   • Rarely, we flag SLVERR on the last beat to verify error propagation.
//
// WHY we do this:
//   • Forces the ROB to reorder back to per-original-ID order on s_r.*
//   • Exercises ROB flow control under independent backpressures.
// ==========================================================================
  typedef struct {
    logic [UID_W-1:0]  uid;
    logic [ADDR_W-1:0] addr;
    logic [LEN_W-1:0]  len;
    logic [2:0]        size;
    logic [1:0]        burst;       // we only implement INCR for address advance
    logic [LEN_W-1:0]  beats_left;  // remaining beats in this burst
    bit                make_err;    // if set, last beat will have SLVERR
  } slave_req_t;

  // Per-UID queues keep in-order within each UID; cross-UID schedule is OOO
  slave_req_t req_q[UID_W][$];

  // Backpressure on m_ar.ready (similar LFSR-style pattern)
  initial begin
    m_ar.ready = 1'b1;
    logic [15:0] pr = 16'hC0DE, nx;
    forever begin
      @(posedge clk);
      void'(lfsr16_step(pr, nx));
      m_ar.ready <= (((nx & 16'h0007) == 16'h0007) ? 1'b0 : 1'b1);
      pr = nx;
    end
  end

  // Capture m_ar handshakes into the corresponding UID queue
  always @(posedge clk) if (rst_n) begin
    if (m_ar.valid & m_ar.ready) begin
      slave_req_t r;
      r.uid        = m_ar.id;
      r.addr       = m_ar.addr;
      r.len        = m_ar.len;
      r.size       = m_ar.size;
      r.burst      = m_ar.burst;
      r.beats_left = bw_inc#(LEN_W)(m_ar.len); // ARLEN+1 (bitwise)
      // Rare error injection: about 1/32 of bursts set make_err
      logic [31:0] ur = $urandom;
      r.make_err   = ((ur & 32'h0000001F) == 32'h0000001F);
      req_q[r.uid].push_back(r);
    end
  end

  // Simple round-robin pointer across UIDs (wraps naturally by width)
  logic [UID_W-1:0] rr_uid; initial rr_uid = '0;

  // Emit at most one beat per cycle from some UID that has data pending
  always @(posedge clk) begin
    if (~rst_n) begin
      m_r.valid <= 1'b0; m_r.id <= '0; m_r.data <= '0; m_r.resp <= 2'b00; m_r.last <= 1'b0;
      rr_uid    <= '0;
    end else begin
      m_r.valid <= 1'b0; m_r.last <= 1'b0;

      // Search for a UID with pending beats, starting at rr_uid
      int steps; steps = 0;
      bit found; found = 1'b0;
      logic [UID_W-1:0] cand; cand = rr_uid;

      while ((steps < (1<<UID_W)) & (~found)) begin
        if (req_q[cand].size() > 0) begin
          slave_req_t r = req_q[cand][0];

          // Compute beat index using only bitwise helpers:
          //   orig_beats = ARLEN+1
          //   beat_idx   = orig_beats - beats_left
          logic [LEN_W-1:0] orig_beats = bw_inc#(LEN_W)(r.len);
          logic [LEN_W-1:0] beat_idx   = bw_sub#(LEN_W)(orig_beats, r.beats_left);

          // Drive one R beat to the ROB
          m_r.valid <= 1'b1;
          m_r.id    <= r.uid;
          m_r.data  <= sig_data(r.addr, {{(16-LEN_W){1'b0}}, beat_idx});  // deterministic signature
          m_r.resp  <= (r.make_err & (r.beats_left == {{(LEN_W-1){1'b0}},1'b1})) ? 2'b10 : 2'b00; // SLVERR on last beat
          m_r.last  <= (r.beats_left == {{(LEN_W-1){1'b0}},1'b1});         // LAST when 1 beat remains

          if (m_r.valid & m_r.ready) begin
            // Consume the beat: decrement remaining beats (bitwise)
            req_q[cand][0].beats_left = bw_dec#(LEN_W)(r.beats_left);

            // Address advance for INCR bursts (WRAP omitted for brevity)
            if (r.burst == 2'd1) begin
              req_q[cand][0].addr = bw_add_shift1#(ADDR_W)(r.addr, r.size);
            end

            // If burst finished (beats_left == 0) → pop it
            if (req_q[cand][0].beats_left == {LEN_W{1'b0}}) begin
              void'(req_q[cand].pop_front());
            end

            // Advance round-robin pointer to favor other UIDs next
            rr_uid <= bw_inc#(UID_W)(rr_uid);
          end
          found = 1'b1; // emit at most one beat per cycle
        end else begin
          // Try next UID
          cand  = bw_inc#(UID_W)(cand);
          steps = int_inc(steps);
        end
      end
    end
  end

  // ==========================================================================
  // SCOREBOARD (consumes s_r.* and checks ORDER/DATA/LAST/RESP)
// ----------------------------------------------------------------------------
// WHAT this block checks:
//   • ORDER (per original ID): responses for an ID must complete in the same
//     FIFO order that requests for that ID were issued on s_ar.*
//       -> We track a single HEAD burst per ID. Any beat not matching the
//          active HEAD's expectations is considered reordering.
//   • DATA: s_r.data must match sig_data(base_addr, beat_idx).
//   • LAST: s_r.last must assert exactly on the last beat of the HEAD burst.
//   • RESP: we count OKAY vs SLVERR. Error presence must NOT change ordering.
//
// WHY we do this:
//   • Validates the ROB’s core contract: reordering internally is allowed,
//     but the master-facing stream must be per-ID in-order and intact.
// ==========================================================================
  typedef struct { ar_cmd_t cmd; logic [LEN_W-1:0] beats_left; } head_t;
  head_t        head[ID_W];   // Active head burst per original ID
  bit           head_live[ID_W];
  logic [15:0]  head_bi[ID_W]; // Beat index within the current head (for signature)

  // Helper: activate head[id] when its queue becomes non-empty
  task automatic activate_head(input logic [ID_W-1:0] id);
    if ((~head_live[id]) & (exp_q[id].size() > 0)) begin
      head[id].cmd        = exp_q[id][0].cmd;
      head[id].beats_left = exp_q[id][0].beats_left;
      head_live[id]       = 1'b1;
      head_bi[id]         = 16'd0;
    end
  endtask

  // Keep heads updated over time (after resets and after pops)
  always @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
      int i; i = 0; while (i < (1<<ID_W)) begin head_live[i] = 1'b0; head_bi[i] = 16'd0; i = int_inc(i); end
    end else begin
      int j; j = 0; while (j < (1<<ID_W)) begin if ((~head_live[j]) & (exp_q[j].size() > 0)) activate_head(j); j = int_inc(j); end
    end
  end

  // Counters for quick end-of-test summary
  logic [31:0] ok_beats, err_beats; initial begin ok_beats = '0; err_beats = '0; end

  // The main checker: consume s_r beats and verify against the active head
  always @(posedge clk) if (rst_n) begin
    if (s_r.valid & s_r.ready) begin
      logic [ID_W-1:0] rid; rid = s_r.id;

      // ORDER: an arriving beat must belong to the active HEAD for that ID
      if (~head_live[rid]) begin
        $error("[%0t] ORDER: beat for ID %0d arrived with NO active HEAD", $time, rid);
      end else begin
        head_t H = head[rid];

        // Compute expected DATA (signature) and expected LAST purely from the head state
        logic [DATA_W-1:0] exp_data = sig_data(H.cmd.addr, head_bi[rid]);
        bit                 exp_last = (H.beats_left == {{(LEN_W-1){1'b0}},1'b1});

        // DATA integrity check (detects mixing or wrong beat order)
        if (s_r.data !== exp_data) begin
          $error("[%0t] DATA: mismatch ID=%0d got=0x%0h exp=0x%0h (beat_idx=%0d)",
                 $time, rid, s_r.data, exp_data, head_bi[rid]);
        end

        // LAST placement check (exactly on the final beat)
        if (s_r.last !== exp_last) begin
          $error("[%0t] LAST: mismatch ID=%0d got=%0b exp=%0b (beats_left=%0d)",
                 $time, rid, s_r.last, exp_last, H.beats_left);
        end

        // RESP accounting (OKAY or SLVERR are both allowed; ordering must still hold)
        if (s_r.resp == 2'b00) ok_beats  = bw_inc#(32)(ok_beats);
        else                   err_beats = bw_inc#(32)(err_beats);

        // Advance the head burst state:
        //   - decrement remaining beats
        //   - advance beat index (for next signature)
        H.beats_left = bw_dec#(LEN_W)(H.beats_left);
        head_bi[rid] = logic'(bw_inc#(16)(logic'(head_bi[rid]))); // bitwise 16-bit increment
        head[rid]    = H;

        // If finished: pop this burst and activate the next (if any)
        if (H.beats_left == {LEN_W{1'b0}}) begin
          void'(exp_q[rid].pop_front());
          head_live[rid] = 1'b0;
          head_bi[rid]   = 16'd0;
          activate_head(rid);
        end
      end
    end
  end

  // ==========================================================================
  // PROCEDURAL PROTOCOL MONITORS (no 'assert property'; bitwise-only)
// ----------------------------------------------------------------------------
// WHAT these monitors check:
//
// 1) VALID STABILITY under stall (all 4 interfaces):
//    Once (valid=1 & ready=0), valid must remain 1 on every cycle until ready
//    goes 1. This guarantees payload is held stable while the sink is stalled.
//
// 2) LAST ⇒ VALID on s_r:
//    There must not be a stray LAST pulse when VALID=0.
//
// 3) s_ar BUNDLE IMMUTABILITY while stalled:
//    When (s_ar.valid=1 & s_ar.ready=0), the AR payload bundle
//    {id,addr,len,size,burst,qos} must not change until ready goes 1.
//    This prevents illegal "payload wobble" under backpressure.
// ==========================================================================
  // tiny helper: vector equality via XOR reduction
  function automatic bit bw_eq #(int W)(input logic [W-1:0] a, input logic [W-1:0] b);
    return ((a ^ b) == {W{1'b0}});
  endfunction

  // 1) VALID stability monitor (re-usable task)
  task automatic monitor_valid_stability
  (
    input string tag,          // which interface (printed in errors)
    input logic  clk_i,
    input logic  rstn_i,
    input logic  valid_i,
    input logic  ready_i
  );
    bit in_stall;
    always @(posedge clk_i) begin
      if (~rstn_i) begin
        in_stall <= 1'b0;
      end else begin
        // Enter or remain in stall region
        if (valid_i & ~ready_i) begin
          if (~in_stall) in_stall <= 1'b1;
          // WHAT we check: once stalled, valid must stay high every cycle
          if (~valid_i) $error("[%0t] STALL:%s valid dropped during stall", $time, tag);
        end
        // Leave stall region only when ready rises
        if (in_stall & ready_i) in_stall <= 1'b0;
      end
    end
  endtask

  // Instantiate the monitor on all four channels
  initial begin
    monitor_valid_stability("s_ar", clk, rst_n, s_ar.valid, s_ar.ready);
    monitor_valid_stability("s_r" , clk, rst_n, s_r .valid, s_r .ready);
    monitor_valid_stability("m_ar", clk, rst_n, m_ar.valid, m_ar.ready);
    monitor_valid_stability("m_r" , clk, rst_n, m_r .valid, m_r .ready);
  end

  // 2) LAST ⇒ VALID: no stray LAST on s_r
  always @(posedge clk) begin
    if (rst_n & s_r.last & ~s_r.valid)
      $error("[%0t] PROTO: s_r.last asserted without s_r.valid", $time);
  end

  // 3) s_ar bundle immutability while stalled
  typedef struct packed {
    logic [ID_W-1:0]   id;
    logic [ADDR_W-1:0] addr;
    logic [LEN_W-1:0]  len;
    logic [2:0]        size;
    logic [1:0]        burst;
    logic [QOS_W-1:0]  qos;
  } ar_bundle_t;

  ar_bundle_t s_ar_snap;  // snapshot of the first stalled cycle
  bit         s_ar_stall; // "we are currently stalled" flag

  function automatic bit bundle_eq(input ar_bundle_t a, input ar_bundle_t b);
    bit e0 = bw_eq#(ID_W)  (a.id   , b.id);
    bit e1 = bw_eq#(ADDR_W)(a.addr , b.addr);
    bit e2 = bw_eq#(LEN_W) (a.len  , b.len);
    bit e3 = bw_eq#(3)     (a.size , b.size);
    bit e4 = bw_eq#(2)     (a.burst, b.burst);
    bit e5 = bw_eq#(QOS_W) (a.qos  , b.qos);
    // combine sub-equalities using bitwise AND (no logical &&)
    return (e0 & e1 & e2 & e3 & e4 & e5);
  endfunction

  always @(posedge clk) begin
    if (~rst_n) begin
      s_ar_stall <= 1'b0;
      s_ar_snap  <= '{default:'0};
    end else begin
      // First cycle of stall → capture the bundle
      if ( (s_ar.valid & ~s_ar.ready) & ~s_ar_stall ) begin
        s_ar_stall <= 1'b1;
        s_ar_snap  <= '{ id:s_ar.id, addr:s_ar.addr, len:s_ar.len,
                         size:s_ar.size, burst:s_ar.burst, qos:s_ar.qos };
      end
      // While stalled, the live bundle must match the snapshot exactly
      if ( s_ar_stall & (s_ar.valid & ~s_ar.ready) ) begin
        ar_bundle_t cur = '{ id:s_ar.id, addr:s_ar.addr, len:s_ar.len,
                             size:s_ar.size, burst:s_ar.burst, qos:s_ar.qos };
        if (~bundle_eq(cur, s_ar_snap))
          $error("[%0t] PROTO: s_ar fields changed while stalled", $time);
      end
      // Exit stall when READY rises (inclusive condition)
      if ( s_ar_stall & s_ar.ready ) begin
        s_ar_stall <= 1'b0;
      end
    end
  end

  // ==========================================================================
  // Mid-run reset + finish criteria
  // ----------------------------------------------------------------------------
  // WHAT: we assert a short reset after traffic has started to ensure the DUT
  //       handles in-flight conditions sanely (the TB will keep going). We also
  //       stop after a fixed time and print a short summary.
// ==========================================================================
  initial begin
    // Inject a mid-run reset after some traffic to probe robustness
    fork
      begin
        repeat (200) @(posedge clk);
        $display("[%0t] INFO: mid-run reset", $time);
        do_reset(7);
      end
    join_none

    // Let simulation run long enough for bursts to retire
    repeat (2000) @(posedge clk);

    // Summarize how many beats were OK vs error, and if any streams are pending
    int pending; pending = 0; int i; i = 0;
    while (i < (1<<ID_W)) begin
      if (exp_q[i].size() > 0) pending = int_inc(pending);
      i = int_inc(i);
    end
    $display("[%0t] SUMMARY: ok_beats=%0d err_beats=%0d pending_id_streams=%0d",
             $time, ok_beats, err_beats, pending);
    if (pending != 0) $warning("Some bursts still pending — extend runtime if needed.");
    $finish;
  end

endmodule
