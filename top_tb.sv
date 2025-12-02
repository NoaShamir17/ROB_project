module top_tb;

  // ---------------------------------------------------------------------------
  // Local parameters – must match DUT + interfaces
  // ---------------------------------------------------------------------------
  localparam int ID_WIDTH        = 32;
  localparam int DATA_WIDTH      = 64;
  localparam int RESP_WIDTH      = 2;
  localparam int TAG_WIDTH       = 4;
  localparam int ADDR_WIDTH      = 32;
  localparam int MAX_OUTSTANDING = 16;
  localparam int MAX_LEN         = 8;

  // AR interface field widths
  localparam int LEN_WIDTH   = 8;
  localparam int SIZE_WIDTH  = 3;
  localparam int BURST_WIDTH = 2;
  localparam int QOS_WIDTH   = 4;

  // FIFO depths – must match top.sv params
  localparam int REQ_FIFO_DEPTH  = 8;
  localparam int RESP_FIFO_DEPTH = 8;

  // Timeout in cycles for any handshake
  localparam int TIMEOUT_CYCLES = 1000;

  // Useful constants
  localparam logic [DATA_WIDTH-1:0] DATA_GILSTOLR =
      DATA_WIDTH'(64'h4749_4C53_544F_4C52); // "GILSTOLR" in ASCII hex

  // ---------------------------------------------------------------------------
  // Scenario flag (for easier log reading)
  // ---------------------------------------------------------------------------
  int scenario_id; // 0 = idle, 1..4 = which scenario is running

  // ---------------------------------------------------------------------------
  // Clock and reset
  // ---------------------------------------------------------------------------
  logic clk;
  logic rst;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  initial begin
    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;
  end

  // ---------------------------------------------------------------------------
  // Instantiate AXI interfaces that connect to the DUT
  // ---------------------------------------------------------------------------
  ar_if #(
    .ID_WIDTH    (ID_WIDTH),
    .ADDR_WIDTH  (ADDR_WIDTH),
    .LEN_WIDTH   (LEN_WIDTH),
    .SIZE_WIDTH  (SIZE_WIDTH),
    .BURST_WIDTH (BURST_WIDTH),
    .QOS_WIDTH   (QOS_WIDTH)
  ) axi_ar_in_if ();

  ar_if #(
    .ID_WIDTH    (ID_WIDTH),
    .ADDR_WIDTH  (ADDR_WIDTH),
    .LEN_WIDTH   (LEN_WIDTH),
    .SIZE_WIDTH  (SIZE_WIDTH),
    .BURST_WIDTH (BURST_WIDTH),
    .QOS_WIDTH   (QOS_WIDTH)
  ) axi_ar_out_if ();

  r_if #(
    .ID_WIDTH   (ID_WIDTH),
    .DATA_WIDTH (DATA_WIDTH),
    .RESP_WIDTH (RESP_WIDTH)
  ) axi_r_out_if ();

  r_if #(
    .ID_WIDTH   (ID_WIDTH),
    .DATA_WIDTH (DATA_WIDTH),
    .RESP_WIDTH (RESP_WIDTH)
  ) axi_r_in_if ();

  // ---------------------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------------------
  top #(
    .ID_WIDTH        (ID_WIDTH),
    .DATA_WIDTH      (DATA_WIDTH),
    .RESP_WIDTH      (RESP_WIDTH),
    .TAG_WIDTH       (TAG_WIDTH),
    .ADDR_WIDTH      (ADDR_WIDTH),
    .MAX_OUTSTANDING (MAX_OUTSTANDING),
    .MAX_LEN         (MAX_LEN),
    .REQ_FIFO_DEPTH  (REQ_FIFO_DEPTH),
    .RESP_FIFO_DEPTH (RESP_FIFO_DEPTH)
  ) dut (
    .clk        (clk),
    .rst        (rst),
    .axi_ar_in  (axi_ar_in_if),
    .axi_ar_out (axi_ar_out_if),
    .axi_r_out  (axi_r_out_if),
    .axi_r_in   (axi_r_in_if)
  );

  // ---------------------------------------------------------------------------
  // Scoreboard for expected responses from DUT (axi_r_out_if)
  //   • expected_q holds the golden sequence the ROB must emit on R channel
  //   • sb_expect() pushes beats into expected_q
  //   • the monitor pops and checks them in order
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [ID_WIDTH-1:0]   id;
    logic [DATA_WIDTH-1:0] data;
    logic                  last;
  } expected_rsp_t;

  expected_rsp_t expected_q [0:31];
  int exp_count;
  int exp_index;

  // Scoreboard reset: clear counters
  task sb_reset;
    begin
      exp_count = 0;
      exp_index = 0;
    end
  endtask

  // Push expected beat into scoreboard queue
  task sb_expect(
    input logic [ID_WIDTH-1:0]   id,
    input logic [DATA_WIDTH-1:0] data,
    input logic                  last
  );
    begin
      expected_q[exp_count].id   = id;
      expected_q[exp_count].data = data;
      expected_q[exp_count].last = last;
      exp_count = exp_count + 1;
    end
  endtask

  // Monitor DUT -> master R channel and compare to scoreboard
  always @(posedge clk) begin
    if (rst == 1'b0) begin
      if ((axi_r_out_if.valid & axi_r_out_if.ready) == 1'b1) begin
        // R ROB->MASTER handshake succeeded
        $display("[%0t] (scenario %0d) R ROB->MASTER: id=%0d data=%h last=%0b resp=%0d",
                 $time, scenario_id,
                 axi_r_out_if.id,
                 axi_r_out_if.data,
                 axi_r_out_if.last,
                 axi_r_out_if.resp);

        if (exp_index >= exp_count) begin
          $error("[%0t] (scenario %0d) unexpected response from ROB: id=%0d data=%h last=%0b",
                 $time, scenario_id,
                 axi_r_out_if.id, axi_r_out_if.data, axi_r_out_if.last);
        end
        else begin
          if ( (axi_r_out_if.id   == expected_q[exp_index].id) &
               (axi_r_out_if.data == expected_q[exp_index].data) &
               (axi_r_out_if.last == expected_q[exp_index].last) ) begin
            $display("[%0t] (scenario %0d) SCOREBOARD: beat %0d OK (id=%0d)",
                     $time, scenario_id, exp_index, axi_r_out_if.id);
          end
          else begin
            $error("[%0t] (scenario %0d) SCOREBOARD MISMATCH at beat %0d: got id=%0d data=%h last=%0b expected id=%0d data=%h last=%0b",
                   $time, scenario_id, exp_index,
                   axi_r_out_if.id,   axi_r_out_if.data,   axi_r_out_if.last,
                   expected_q[exp_index].id,
                   expected_q[exp_index].data,
                   expected_q[exp_index].last);
          end
          exp_index = exp_index + 1;
        end
      end
    end
  end

  // Wait until all expected beats are seen or timeout
  task sb_wait_all(input string tag);
    int guard;
    begin
      guard = 0;
      while ((exp_index < exp_count) & (guard < TIMEOUT_CYCLES)) begin
        @(posedge clk);
        guard = guard + 1;
      end

      if (exp_index == exp_count) begin
        $display("[%0t] (scenario %0d) scenario %s completed ok (%0d beats)",
                 $time, scenario_id, tag, exp_count);
      end
      else begin
        $error("[%0t] (scenario %0d) TIMEOUT: ROB did not produce all expected beats in scenario %s (seen %0d / %0d)",
               $time, scenario_id, tag, exp_index, exp_count);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Track internal UIDs sent by DUT on axi_ar_out_if.id
  //   • captured_uid[n] = internal UID for nth forwarded AR
  // ---------------------------------------------------------------------------
  localparam int MAX_REQ = 16;
  logic [ID_WIDTH-1:0] captured_uid [0:MAX_REQ-1];
  int                  uid_count;

  // Reset UID capture state
  task uid_reset;
    begin
      uid_count = 0;
    end
  endtask

  // Wait for AR handshake on ROB->slave, capture UID, and log AR path
  task capture_next_uid;
    int guard;
    begin
      guard = 0;
      @(posedge clk);
      // wait for AR ROB->SLAVE handshake with timeout
      while ((axi_ar_out_if.valid & axi_ar_out_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      if ((axi_ar_out_if.valid & axi_ar_out_if.ready) == 1'b0) begin
        $error("[%0t] (scenario %0d) TIMEOUT: AR ROB->SLAVE handshake FAILED (no forwarded AR observed)",
               $time, scenario_id);
        disable capture_next_uid;
      end

      captured_uid[uid_count] = axi_ar_out_if.id;
      $display("[%0t] (scenario %0d) AR ROB->SLAVE: uid=%0d addr=%h len=%0d size=%0d burst=%0d qos=%0d",
               $time, scenario_id,
               captured_uid[uid_count],
               axi_ar_out_if.addr,
               axi_ar_out_if.len,
               axi_ar_out_if.size,
               axi_ar_out_if.burst,
               axi_ar_out_if.qos);

      uid_count = uid_count + 1;
    end
  endtask

  // ---------------------------------------------------------------------------
  // Small helper: wait for a number of idle cycles between scenarios
  // ---------------------------------------------------------------------------
  task wait_idle_cycles(input int cycles);
    int i;
    begin
      for (i = 0; i < cycles; i = i + 1) begin
        @(posedge clk);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read address driver on master side (axi_ar_in_if)
  //   • All AR fields are passed as arguments (except valid/ready)
  //   • len = number of beats (project convention, not AXI-1-based)
  //   • Handshake is robust: valid is held until a sampled valid&ready edge.
  // ---------------------------------------------------------------------------
  task send_read_req(
    input logic [ID_WIDTH-1:0]    id,
    input logic [ADDR_WIDTH-1:0]  addr,
    input logic [LEN_WIDTH-1:0]   len,
    input logic [SIZE_WIDTH-1:0]  size,
    input logic [BURST_WIDTH-1:0] burst,
    input logic [QOS_WIDTH-1:0]   qos
  );
    int guard;
    begin
      // Drive all AR fields from the task arguments
      axi_ar_in_if.id    = id;
      axi_ar_in_if.addr  = addr;
      axi_ar_in_if.len   = len;
      axi_ar_in_if.size  = size;
      axi_ar_in_if.burst = burst;
      axi_ar_in_if.qos   = qos;

      axi_ar_in_if.valid = 1'b1;

      guard = 0;
      @(posedge clk);
      // wait for AR MASTER->ROB handshake with timeout
      while ((axi_ar_in_if.valid & axi_ar_in_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      if ((axi_ar_in_if.valid & axi_ar_in_if.ready) == 1'b0) begin
        $error("[%0t] (scenario %0d) TIMEOUT: AR MASTER->ROB handshake FAILED (id=%0d addr=%h len=%0d size=%0d burst=%0d qos=%0d)",
               $time, scenario_id, id, addr, len, size, burst, qos);
        axi_ar_in_if.valid = 1'b0;
        disable send_read_req;
      end

      $display("[%0t] (scenario %0d) AR MASTER->ROB: id=%0d addr=%h len=%0d size=%0d burst=%0d qos=%0d",
               $time, scenario_id, id, addr, len, size, burst, qos);

      axi_ar_in_if.valid = 1'b0;

      // Wait for ROB->SLAVE forwarding and capture internal UID
      capture_next_uid();
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver on slave side (axi_r_in_if) – single beat
  //   • All R fields are passed as arguments (except valid/ready)
  //   • Handshake is robust: valid is held until a sampled valid&ready edge.
  // ---------------------------------------------------------------------------
  task send_single_beat_rsp(
    input logic [ID_WIDTH-1:0]   id,
    input logic [DATA_WIDTH-1:0] data,
    input logic [RESP_WIDTH-1:0] resp,
    input logic                  last
  );
    int guard;
    begin
      axi_r_in_if.id    = id;
      axi_r_in_if.data  = data;
      axi_r_in_if.resp  = resp;
      axi_r_in_if.last  = last;
      axi_r_in_if.valid = 1'b1;

      guard = 0;
      @(posedge clk);
      // wait for R SLAVE->ROB handshake with timeout
      while ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      if ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0) begin
        $error("[%0t] (scenario %0d) TIMEOUT: R SLAVE->ROB handshake FAILED (id=%0d data=%h last=%0b resp=%0d)",
               $time, scenario_id,
               id, data, last, resp);
        axi_r_in_if.valid = 1'b0;
        disable send_single_beat_rsp;
      end

      $display("[%0t] (scenario %0d) R SLAVE->ROB: id=%0d data=%h last=%0b resp=%0d",
               $time, scenario_id,
               id, data, last, resp);

      axi_r_in_if.valid = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver – multi-beat burst
  //   • beats  = number of beats sent
  //   • resp   = same response code for all beats in this burst
  //   • Handshake is robust for each beat.
  // ---------------------------------------------------------------------------
  task send_burst_rsp(
    input logic [ID_WIDTH-1:0]   id,
    input int                    beats,
    input logic [DATA_WIDTH-1:0] base_data,
    input logic [RESP_WIDTH-1:0] resp
  );
    int i;
    int guard;
    logic [DATA_WIDTH-1:0] cur_data;
    logic                  last;
    begin
      for (i = 0; i < beats; i = i + 1) begin
        cur_data = base_data + DATA_WIDTH'(i);
        last     = (i == (beats - 1));

        axi_r_in_if.id    = id;
        axi_r_in_if.data  = cur_data;
        axi_r_in_if.resp  = resp;
        axi_r_in_if.last  = last;
        axi_r_in_if.valid = 1'b1;

        guard = 0;
        @(posedge clk);
        while ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0 &
               guard < TIMEOUT_CYCLES) begin
          guard = guard + 1;
          @(posedge clk);
        end

        if ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0) begin
          $error("[%0t] (scenario %0d) TIMEOUT: R SLAVE->ROB handshake FAILED in burst (id=%0d data=%h beat=%0d last=%0b resp=%0d)",
                 $time, scenario_id,
                 id, cur_data, i, last, resp);
          axi_r_in_if.valid = 1'b0;
          disable send_burst_rsp;
        end

        $display("[%0t] (scenario %0d) R SLAVE->ROB BURST: beat %0d id=%0d data=%h last=%0b resp=%0d",
                 $time, scenario_id,
                 i, id, cur_data, last, resp);

        axi_r_in_if.valid = 1'b0;
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Initial defaults
  // ---------------------------------------------------------------------------
  initial begin
    scenario_id        = 0;

    axi_ar_in_if.valid = 1'b0;
    axi_ar_in_if.id    = {ID_WIDTH{1'b0}};
    axi_ar_in_if.addr  = {ADDR_WIDTH{1'b0}};
    axi_ar_in_if.len   = {LEN_WIDTH{1'b0}};
    axi_ar_in_if.size  = {SIZE_WIDTH{1'b0}};
    axi_ar_in_if.burst = {BURST_WIDTH{1'b0}};
    axi_ar_in_if.qos   = {QOS_WIDTH{1'b0}};

    axi_ar_out_if.ready = 1'b1;

    axi_r_out_if.ready  = 1'b1;

    axi_r_in_if.valid = 1'b0;
    axi_r_in_if.id    = {ID_WIDTH{1'b0}};
    axi_r_in_if.data  = {DATA_WIDTH{1'b0}};
    axi_r_in_if.resp  = {RESP_WIDTH{1'b0}};
    axi_r_in_if.last  = 1'b0;

    sb_reset();
    uid_reset();
  end

  // ---------------------------------------------------------------------------
  // Direct test scenarios
  //
  // Scenario 1 – single read, in-order response
  //   • One AR with id=0, len=1, size=8B, INCR, qos=0.
  //   • Fabric returns one beat for that UID with data="GILSTOLR".
  //   • Expectation: master sees exactly one beat (id=0, data="GILSTOLR", last=1).
  //
  // Scenario 2 – two different IDs, fabric returns responses swapped
  //   • AR #0: id=3, len=1, data=1111...., AR #1: id=5, len=1, data=2222....
  //   • Fabric returns UID1 first then UID0.
  //   • We mainly check that BOTH responses are seen, ID=3 and ID=5.
  //
  // Scenario 3 – single ID, 4-beat burst, in-order fabric
  //
  // Scenario 4 – response_memory stress (two 4-beat bursts, out-of-order)
  // ---------------------------------------------------------------------------
  initial begin : main_stimulus
    logic [DATA_WIDTH-1:0] base3;
    logic [DATA_WIDTH-1:0] base4_a;
    logic [DATA_WIDTH-1:0] base4_b;
    int k;

    @(posedge clk);
    while (rst == 1'b1) begin
      @(posedge clk);
    end

    // ============================
    // Scenario 1
    // ============================
    scenario_id = 1;
    $display("---- SCENARIO 1: one read, in-order response ----");
    sb_reset();
    uid_reset();

    // Tell scoreboard what we expect from ROB on R channel
    sb_expect(ID_WIDTH'(0), DATA_GILSTOLR, 1'b1);

    // Send AR with all fields explicit
    send_read_req(
      ID_WIDTH'(0),                             // id
      ADDR_WIDTH'(32'h0000_1000),              // addr
      LEN_WIDTH'(1),                           // len = 1 beat
      SIZE_WIDTH'(3),                          // size = 3'b011 (8 bytes)
      BURST_WIDTH'(1),                         // burst = INCR
      QOS_WIDTH'({QOS_WIDTH{1'b0}})            // qos = 0
    );

    // Slave returns one R beat for captured_uid[0]
    send_single_beat_rsp(
      captured_uid[0],                         // internal UID
      DATA_GILSTOLR,                           // data = "GILSTOLR"
      RESP_WIDTH'(0),                          // resp = OKAY (0)
      1'b1                                     // last = 1
    );

    // Wait until scoreboard sees all beats
    sb_wait_all("SCENARIO_1");
    wait_idle_cycles(10);

    // ============================
    // Scenario 2
    // ============================
    scenario_id = 2;
    $display("---- SCENARIO 2: two IDs, swapped fabric returns ----");
    sb_reset();
    uid_reset();

    // AR #0: id=3, len=1
    send_read_req(
      ID_WIDTH'(3),
      ADDR_WIDTH'(32'h0000_2000),
      LEN_WIDTH'(1),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // AR #1: id=5, len=1
    send_read_req(
      ID_WIDTH'(5),
      ADDR_WIDTH'(32'h0000_3000),
      LEN_WIDTH'(1),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // Expected order at master: id=3 beat, then id=5 beat
    sb_expect(ID_WIDTH'(3), DATA_WIDTH'(64'h1111_0000_0000_0001), 1'b1);
    sb_expect(ID_WIDTH'(5), DATA_WIDTH'(64'h2222_0000_0000_0002), 1'b1);

    // Fabric returns id=5 (UID1) first...
    send_single_beat_rsp(
      captured_uid[1],
      DATA_WIDTH'(64'h2222_0000_0000_0002),
      RESP_WIDTH'(0),
      1'b1
    );
    // ...then id=3 (UID0)
    send_single_beat_rsp(
      captured_uid[0],
      DATA_WIDTH'(64'h1111_0000_0000_0001),
      RESP_WIDTH'(0),
      1'b1
    );

    sb_wait_all("SCENARIO_2");
    wait_idle_cycles(10);

    // ============================
    // Scenario 3 – 4-beat burst, in-order
    // ============================
    scenario_id = 3;
    $display("---- SCENARIO 3: single ID, 4-beat burst ----");
    sb_reset();
    uid_reset();

    send_read_req(
      ID_WIDTH'(5),
      ADDR_WIDTH'(32'h0000_4000),
      LEN_WIDTH'(4),                           // 4 beats
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    base3 = DATA_WIDTH'(64'hAAA0_0000_0000_0000);

    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(5), base3 + DATA_WIDTH'(k), (k == 3));
    end

    send_burst_rsp(
      captured_uid[0],                          // UID for this AR
      4,                                        // beats
      base3,
      RESP_WIDTH'(0)
    );

    sb_wait_all("SCENARIO_3");
    wait_idle_cycles(10);

    // ============================
    // Scenario 4 – response_memory stress
    // ============================
    scenario_id = 4;
    $display("---- SCENARIO 4: response_memory stress (two 4-beat bursts, out-of-order) ----");
    sb_reset();
    uid_reset();

    // First 4-beat AR
    send_read_req(
      ID_WIDTH'(7),
      ADDR_WIDTH'(32'h0000_5000),
      LEN_WIDTH'(4),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // Second 4-beat AR (same orig ID)
    send_read_req(
      ID_WIDTH'(7),
      ADDR_WIDTH'(32'h0000_6000),
      LEN_WIDTH'(4),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    base4_a = DATA_WIDTH'(64'hA000_0000_0000_0000);
    base4_b = DATA_WIDTH'(64'hB000_0000_0000_0000);

    // Expected: first all beats of first AR, then all beats of second AR
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(7), base4_a + DATA_WIDTH'(k), (k == 3));
    end
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(7), base4_b + DATA_WIDTH'(k), (k == 3));
    end

    // Fabric returns bursts in reverse AR order
    send_burst_rsp(
      captured_uid[1],                          // second AR UID
      4,
      base4_b,
      RESP_WIDTH'(0)
    );
    send_burst_rsp(
      captured_uid[0],                          // first AR UID
      4,
      base4_a,
      RESP_WIDTH'(0)
    );

    sb_wait_all("SCENARIO_4");

    scenario_id = 0;
    $display("==== ALL SCENARIOS DONE ====");
    #20;
    $finish;
  end

endmodule
