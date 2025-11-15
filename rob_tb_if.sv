// ============================================================================
// ROB Read-Channel Testbench (with full project monitors)
//   * ENGLISH-ONLY
//   * ONLY BITWISE ops in control conditions (&, |, ~). No &&, ||, !.
//   * Monitors print exactly what the SLAVE gets/sends and what the MASTER gets/sends.
//   * Heavily commented for clarity.
// ============================================================================


module rob_tb_if;

  // ==========================================================================
  // 1) CLOCK & RESET
  // ==========================================================================
  logic clk;
  logic rst_n;

  initial begin
    clk = 1'b1;
    forever #5 clk = ~clk; // 100MHz
  end

  initial begin
    rst_n = 1'b0;
    #20;
    rst_n = 1'b1;
  end

  // ==========================================================================
  // 2) INTERFACES: upstream (master-facing) and downstream (slave-facing)
  // ==========================================================================
  ar_if s_ar();  // Master -> ROB  (AR)
  r_if  s_r ();  // ROB    -> Master (R)

  ar_if m_ar();  // ROB    -> Slave  (AR)
  r_if  m_r ();  // Slave  -> ROB    (R)

  // ==========================================================================
  // 3) DUT INSTANTIATION
  // ==========================================================================
  top dut (
    .clk   (clk),
    .rst_n (rst_n),
    .s_ar  (s_ar),
    .s_r   (s_r),
    .m_ar  (m_ar),
    .m_r   (m_r)
  );

  // ==========================================================================
  // 4) DEFAULT SIGNALS (idle)
  // ==========================================================================
  initial begin
    // Upstream AR (master -> ROB)
    s_ar.valid = 1'b0;
    s_ar.id    = '0;
    s_ar.addr  = '0;
    s_ar.len   = '0;
    s_ar.size  = 3'd3;
    s_ar.burst = 2'b01;
    s_ar.qos   = '0;

    // Upstream R (ROB -> master)
    s_r.ready  = 1'b1;     // Master always ready in this TB

    // Downstream AR (ROB -> slave)
    m_ar.ready = 1'b1;     // Slave always ready for AR in this TB

    // Downstream R (slave -> ROB)
    m_r.valid  = 1'b0;
    m_r.id     = '0;
    m_r.data   = '0;
    m_r.resp   = 2'b00;
    m_r.last   = 1'b0;
  end

  // ==========================================================================
  // 5) STATE: capture downstream ARs to learn internal UIDs
  // ==========================================================================
  typedef struct packed {
    logic [31:0]  addr;
    logic [7:0]   len;     // beats-1
    logic [2:0]   size;
    logic [1:0]   burst;
    logic [3:0]   qos;
    logic [15:0]  uid;     // internal m_ar.id
  } req_t;

  req_t inflight_reqs[$];
  int   data_seed = 32'h1111;

  typedef struct packed {
    logic [7:0] rid;
    logic       last;
  } exp_beat_t;

  exp_beat_t expected_seq[$];

  // ==========================================================================
  // 6) EXPECTATION QUEUE (used by checker on s_r)
  // ==========================================================================
  task automatic expect_beat(input logic [7:0] rid_expected,
                             input logic       last_expected);
    exp_beat_t e;
    e.rid  = rid_expected;
    e.last = last_expected;
    expected_seq.push_back(e);
  endtask

  // ==========================================================================
  // 7) UPSTREAM STIMULUS: send AR into ROB (master side)
  //    - Drives s_ar.* and waits for (s_ar.valid & s_ar.ready) == 1
  // ==========================================================================
  task automatic send_ar_to_rob(
      input logic [7:0]  arid,
      input logic [31:0] araddr,
      input logic [7:0]  arlen,     // beats-1
      input logic [2:0]  arsize,
      input logic [1:0]  arburst,
      input logic [3:0]  arqos
  );
    @(posedge clk);
    s_ar.id    <= arid;
    s_ar.addr  <= araddr;
    s_ar.len   <= arlen;
    s_ar.size  <= arsize;
    s_ar.burst <= arburst;
    s_ar.qos   <= arqos;
    s_ar.valid <= 1'b1;

    // Wait for AR handshake upstream (master -> ROB)
    while (((s_ar.valid & s_ar.ready) == 1'b0)) @(posedge clk);

    // One extra cycle to complete transfer, then drop valid
    @(posedge clk);
    s_ar.valid <= 1'b0;
  endtask

  // ==========================================================================
  // 8) DOWNSTREAM SPY: capture a single AR (ROB -> slave) -> learn UID
  // ==========================================================================
  task automatic monitor_downstream_ar_once();
    while (((m_ar.valid & m_ar.ready) == 1'b0)) @(posedge clk);
    req_t r;
    r.addr   = m_ar.addr;
    r.len    = m_ar.len;
    r.size   = m_ar.size;
    r.burst  = m_ar.burst;
    r.qos    = m_ar.qos;
    r.uid    = m_ar.id;    // internal UID assigned by ROB
    inflight_reqs.push_back(r);
  endtask

  // ==========================================================================
  // 9) SLAVE MODEL: send R responses (slave -> ROB) for a given UID
  // ==========================================================================
  task automatic slave_send_response(
      input logic [15:0] uid,
      input int          beats,
      input int          initial_delay_cycles,
      input int          gap_cycles_between_beats
  );
    int i;
    repeat (initial_delay_cycles) @(posedge clk);

    for (i = 0; i < beats; i = i + 1) begin
      data_seed = data_seed + 1;

      m_r.data  <= data_seed;
      m_r.id    <= uid;
      m_r.resp  <= 2'b00;
      m_r.last  <= (i == (beats - 1));
      m_r.valid <= 1'b1;

      while (((m_r.valid & m_r.ready) == 1'b0)) @(posedge clk);

      @(posedge clk);
      m_r.valid <= 1'b0;

      repeat (gap_cycles_between_beats) @(posedge clk);
    end
  endtask

  // ==========================================================================
  // 10) CHECKER: monitor master returns (ROB -> master) vs expectations
  // ==========================================================================
  task automatic monitor_and_check_master_returns(input int total_beats);
    int got;
    got = 0;

    while (got < total_beats) begin
      while ((s_r.valid == 1'b0)) @(posedge clk);
      if ((s_r.ready == 1'b0)) begin
        $display("[%0t] WARN: s_r.valid=1 while s_r.ready=0, waiting...", $time);
      end
      while (((s_r.valid & s_r.ready) == 1'b0)) @(posedge clk);

      if (expected_seq.size() == 0) begin
        $error("[%0t] Unexpected beat: no expectations left (RID=%0d last=%0b data=0x%08x)",
               $time, s_r.id, s_r.last, s_r.data);
      end
      else begin
        exp_beat_t e;
        e = expected_seq.pop_front();

        if ((s_r.id == e.rid) == 1'b0) begin
          $error("[%0t] RID mismatch: expected %0d, got %0d (data=0x%08x last=%0b)",
                 $time, e.rid, s_r.id, s_r.data, s_r.last);
        end
        if ((s_r.last == e.last) == 1'b0) begin
          $error("[%0t] LAST mismatch: expected %0b, got %0b (RID=%0d data=0x%08x)",
                 $time, e.last, s_r.last, s_r.id, s_r.data);
        end
      end

      got = got + 1;
      @(posedge clk);
    end
  endtask

  // ==========================================================================
  // 11) CONTINUOUS MONITORS (bitwise-only wait conditions)
  //     These four loops provide a full audit trail of the entire datapath:
  //       * What MASTER sends (upstream AR)
  //       * What SLAVE gets   (downstream AR)
  //       * What SLAVE sends  (downstream R)
  //       * What MASTER gets  (upstream R)
  // ==========================================================================
  // MASTER -> ROB (AR) : what the MASTER sends
  initial begin : MON_UP_AR
    // Wait for reset to complete
    while ((rst_n == 1'b0)) @(posedge clk);
    forever begin
      while (((s_ar.valid & s_ar.ready) == 1'b0)) @(posedge clk);
      $display("[%0t] MASTER->ROB AR  : ID=%0d ADDR=0x%08x LEN=%0d SIZE=%0d BURST=0x%0x QOS=0x%0x",
               $time, s_ar.id, s_ar.addr, s_ar.len, s_ar.size, s_ar.burst, s_ar.qos);
      @(posedge clk);
    end
  end

  // ROB -> SLAVE (AR) : what the SLAVE gets
  initial begin : MON_DOWN_AR
    while ((rst_n == 1'b0)) @(posedge clk);
    forever begin
      while (((m_ar.valid & m_ar.ready) == 1'b0)) @(posedge clk);
      $display("[%0t] ROB->SLAVE AR   : UID=%0d ADDR=0x%08x LEN=%0d SIZE=%0d BURST=0x%0x QOS=0x%0x",
               $time, m_ar.id, m_ar.addr, m_ar.len, m_ar.size, m_ar.burst, m_ar.qos);
      @(posedge clk);
    end
  end

  // SLAVE -> ROB (R)  : what the SLAVE sends
  initial begin : MON_DOWN_R
    while ((rst_n == 1'b0)) @(posedge clk);
    forever begin
      while (((m_r.valid & m_r.ready) == 1'b0)) @(posedge clk);
      $display("[%0t] SLAVE->ROB R    : UID=%0d DATA=0x%08x LAST=%0b RESP=0x%0x",
               $time, m_r.id, m_r.data, m_r.last, m_r.resp);
      @(posedge clk);
    end
  end

  // ROB -> MASTER (R) : what the MASTER gets
  initial begin : MON_UP_R
    while ((rst_n == 1'b0)) @(posedge clk);
    forever begin
      while (((s_r.valid & s_r.ready) == 1'b0)) @(posedge clk);
      $display("[%0t] ROB->MASTER R   : RID=%0d DATA=0x%08x LAST=%0b RESP=0x%0x",
               $time, s_r.id, s_r.data, s_r.last, s_r.resp);
      @(posedge clk);
    end
  end

  // ==========================================================================
  // 12) SCENARIOS (same as before; ordering + inter-ID behavior)
  // ==========================================================================
  initial begin : run_scenarios
    while ((rst_n == 1'b0)) @(posedge clk);
    repeat (2) @(posedge clk);

    // ----- Scenario A: two reads with the same original ID=3 (each 4 beats)
    int beatsA1;
    int beatsA2;
    beatsA1 = 4;
    beatsA2 = 4;

    // master sends two ARs (ID=3)
    send_ar_to_rob(8'd3, 32'h1000_0000, beatsA1 - 1, 3'd3, 2'b01, 4'd0);
    send_ar_to_rob(8'd3, 32'h1000_1000, beatsA2 - 1, 3'd3, 2'b01, 4'd0);

    // capture two downstream ARs to learn UIDs
    fork
      monitor_downstream_ar_once(); // inflight_reqs[0]
      monitor_downstream_ar_once(); // inflight_reqs[1]
    join

    // expected upstream sequence (per-ID order: first burst then second)
    for (int i = 0; i < beatsA1; i = i + 1) expect_beat(8'd3, (i == (beatsA1 - 1)));
    for (int j = 0; j < beatsA2; j = j + 1) expect_beat(8'd3, (j == (beatsA2 - 1)));

    // slave returns out-of-order: second first, then first
    fork
      slave_send_response(inflight_reqs[1].uid, beatsA2, 2, 0);
      slave_send_response(inflight_reqs[0].uid, beatsA1,10, 0);
      monitor_and_check_master_returns(beatsA1 + beatsA2);
    join

    inflight_reqs.delete();

    // ----- Scenario B: different IDs (ID=3:4 beats, ID=5:2 beats)
    send_ar_to_rob(8'd3, 32'h2000_0000, 8'd3, 3'd3, 2'b01, 4'd1); // 4 beats
    send_ar_to_rob(8'd5, 32'h2000_1000, 8'd1, 3'd3, 2'b01, 4'd1); // 2 beats

    fork
      monitor_downstream_ar_once();
      monitor_downstream_ar_once();
    join

    // expectations: ID=5 beats first (2), then ID=3 (4)
    expect_beat(8'd5, 1'b0);
    expect_beat(8'd5, 1'b1);
    for (int k = 0; k < 4; k = k + 1) expect_beat(8'd3, (k == 3));

    // pick UIDs by address (two entries known)
    logic [15:0] uid_id5;
    logic [15:0] uid_id3;

    if ((inflight_reqs.size() == 2)) begin
      if ((inflight_reqs[0].addr == 32'h2000_1000)) uid_id5 = inflight_reqs[0].uid;
      else                                           uid_id3 = inflight_reqs[0].uid;

      if ((inflight_reqs[1].addr == 32'h2000_1000)) uid_id5 = inflight_reqs[1].uid;
      else                                           uid_id3 = inflight_reqs[1].uid;
    end
    else begin
      $error("[%0t] Unexpected inflight size in Scenario B: %0d",
             $time, inflight_reqs.size());
    end

    fork
      slave_send_response(uid_id5, 2, 2, 0);
      slave_send_response(uid_id3, 4, 8, 0);
      monitor_and_check_master_returns(6);
    join

    $display("[%0t] All scenarios completed.", $time);
    #50 $finish;
  end

endmodule
