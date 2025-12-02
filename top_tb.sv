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

  // ---------------------------------------------------------------------------
  // Clock and reset
  // ---------------------------------------------------------------------------
  logic clk;
  logic rst;

  // Global scenario flag – tells us which scenario is currently running
  int current_scenario;

  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  // Power-up reset (initial reset only)
  initial begin
    rst = 1'b1;
    current_scenario = 0;
    repeat (5) @(posedge clk);
    rst = 1'b0;
  end

  // Task: pulse reset between scenarios
  task do_reset_between_scenarios;
    begin
      rst = 1'b1;
      repeat (3) @(posedge clk);
      rst = 1'b0;
      @(posedge clk);
    end
  endtask

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

  // Task: reset scoreboard indices (no expected beats)
  task sb_reset;
    begin
      exp_count = 0;
      exp_index = 0;
    end
  endtask

  // Task: push one expected beat into the scoreboard queue
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

  // Monitor: check DUT → master R channel against the scoreboard
  always @(posedge clk) begin
    if (rst == 1'b0) begin
      if ((axi_r_out_if.valid & axi_r_out_if.ready) == 1'b1) begin
        // R ROB->MASTER handshake succeeded
        $display("[%0t] (scenario %0d) R ROB->MASTER: id=%0b data=%h last=%0b resp=%0b",
                 $time, current_scenario, axi_r_out_if.id, axi_r_out_if.data,
                 axi_r_out_if.last, axi_r_out_if.resp);

        if (exp_index >= exp_count) begin
          $error("[%0t] (scenario %0d) unexpected response from ROB: id=%0b data=%h last=%0b",
                 $time, current_scenario,
                 axi_r_out_if.id, axi_r_out_if.data, axi_r_out_if.last);
        end
        else begin
          if ( (axi_r_out_if.id   == expected_q[exp_index].id) &
               (axi_r_out_if.data == expected_q[exp_index].data) &
               (axi_r_out_if.last == expected_q[exp_index].last) ) begin
            $display("[%0t] (scenario %0d) SCOREBOARD: beat %0d OK (id=%0b)",
                     $time, current_scenario, exp_index, axi_r_out_if.id);
          end
          else begin
            $error("[%0t] (scenario %0d) SCOREBOARD MISMATCH at beat %0d: "
                   "got id=%0b data=%h last=%0b  expected id=%0b data=%h last=%0b",
                   $time, current_scenario, exp_index,
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

  // Task: wait until scoreboard consumed all beats, or timeout
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
                 $time, current_scenario, tag, exp_count);
      end
      else begin
        $error("[%0t] (scenario %0d) TIMEOUT: R ROB->MASTER did not produce all expected beats "
               "in scenario %s (seen %0d / %0d)",
               $time, current_scenario, tag, exp_index, exp_count);
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

  // Task: reset UID capture counter
  task uid_reset;
    begin
      uid_count = 0;
    end
  endtask

  // Task: wait for AR ROB->SLAVE handshake, capture UID, log AR path
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
               $time, current_scenario);
        disable capture_next_uid;
      end

      captured_uid[uid_count] = axi_ar_out_if.id;
      $display("[%0t] (scenario %0d) AR ROB->SLAVE: uid=%0b addr=%h len=%0d",
               $time, current_scenario, captured_uid[uid_count],
               axi_ar_out_if.addr, axi_ar_out_if.len);

      uid_count = uid_count + 1;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read address driver on master side (axi_ar_in_if)
  //   NOTE: here len = number of beats in this environment.
  // ---------------------------------------------------------------------------
  task send_read_req(
    input logic [ID_WIDTH-1:0]   arid,
    input logic [ADDR_WIDTH-1:0] addr,
    input logic [LEN_WIDTH-1:0]  len
  );
    int guard;
    begin
      axi_ar_in_if.id    = arid;
      axi_ar_in_if.addr  = addr;
      axi_ar_in_if.len   = len;           // len = number of beats (testbench convention)
      axi_ar_in_if.size  = 3'b011;        // 8 bytes (param in real design, here fixed for simplicity)
      axi_ar_in_if.burst = 2'b01;         // INCR
      axi_ar_in_if.qos   = {QOS_WIDTH{1'b0}};

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
        $error("[%0t] (scenario %0d) TIMEOUT: AR MASTER->ROB handshake FAILED (orig_id=%0b addr=%h len=%0d)",
               $time, current_scenario, arid, addr, len);
        axi_ar_in_if.valid = 1'b0;
        disable send_read_req;
      end

      $display("[%0t] (scenario %0d) AR MASTER->ROB: orig_id=%0b addr=%h len=%0d",
               $time, current_scenario, arid, addr, len);

      axi_ar_in_if.valid = 1'b0;

      // Wait for ROB->SLAVE forwarding and capture internal UID
      capture_next_uid();
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver on slave side (axi_r_in_if) – single beat
  //   • Drives one beat with uid, data, resp, last=1.
  // ---------------------------------------------------------------------------
  task send_single_beat_rsp(
    input logic [ID_WIDTH-1:0]   uid,
    input logic [DATA_WIDTH-1:0] data,
    input logic [RESP_WIDTH-1:0] resp
  );
    int guard;
    begin
      axi_r_in_if.id    = uid;
      axi_r_in_if.data  = data;
      axi_r_in_if.resp  = resp;
      axi_r_in_if.last  = 1'b1;
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
        $error("[%0t] (scenario %0d) HIGILSTOLLER: TIMEOUT: R SLAVE->ROB handshake FAILED (uid=%0b data=%h)",
               $time, current_scenario, uid, data);
        axi_r_in_if.valid = 1'b0;
        disable send_single_beat_rsp;
      end

      $display("[%0t] (scenario %0d) HIGILSTOLLER: R SLAVE->ROB single beat: uid=%0b data=%h last=%0b resp=%0b",
               $time, current_scenario, uid, data, axi_r_in_if.last, resp);

      axi_r_in_if.valid = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver – multi-beat burst
  //   • "beats" = number of beats you actually send
  //   • resp is constant 2'b00 for all beats.
  // ---------------------------------------------------------------------------
  task send_burst_rsp(
    input logic [ID_WIDTH-1:0]   uid,
    input int                    beats,
    input logic [DATA_WIDTH-1:0] base_data
  );
    int i;
    int guard;
    logic [DATA_WIDTH-1:0] cur_data;
    begin
      for (i = 0; i < beats; i = i + 1) begin
        cur_data = base_data + DATA_WIDTH'(i);

        axi_r_in_if.id    = uid;
        axi_r_in_if.data  = cur_data;
        axi_r_in_if.resp  = RESP_WIDTH'(2'b00);
        axi_r_in_if.last  = (i == (beats - 1));
        axi_r_in_if.valid = 1'b1;

        guard = 0;
        @(posedge clk);
        while ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0 &
               guard < TIMEOUT_CYCLES) begin
          guard = guard + 1;
          @(posedge clk);
        end

        if ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0) begin
          $error("[%0t] (scenario %0d) HIGILSTOLLER: TIMEOUT: R SLAVE->ROB handshake FAILED in burst "
                 "(uid=%0b data=%h beat=%0d)",
                 $time, current_scenario, uid, cur_data, i);
          axi_r_in_if.valid = 1'b0;
          disable send_burst_rsp;
        end

        $display("[%0t] (scenario %0d) HIGILSTOLLER: R SLAVE->ROB BURST beat %0d: uid=%0b data=%h last=%0b",
                 $time, current_scenario, i, uid, cur_data, axi_r_in_if.last);

        axi_r_in_if.valid = 1'b0;
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Initial defaults
  // ---------------------------------------------------------------------------
  initial begin
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
    current_scenario = 0;
  end

  // ---------------------------------------------------------------------------
  // Direct test scenarios
  //
  // Scenario 1 – single read, in-order response
  //   • One AR with orig_id=0, len=1
  //   • Fabric returns one beat for that UID, with data="GILSTOLR", last=1.
  //   • Expectation: master sees exactly that beat, id=0, data="GILSTOLR", last=1.
  //
  // Scenario 2 – two single-beat reads, different IDs, out-of-order from fabric
  //   • Two ARs: ID=3 then ID=5, len=1 each.
  //   • Fabric returns UID for ID=5 first, then UID for ID=3.
  //   • Expectation: ROB does not reorder different IDs, so master sees first AR
  //     (ID=3) then second AR (ID=5).
  //
  // Scenario 3 – single ID, 4-beat burst, in-order fabric
  //   • One AR with ID=7, len=4.
  //   • Fabric returns 4 beats in-order, last=1 on beat 3.
  //   • Expectation: master sees the same 4-beat burst unchanged.
  //
  // Scenario 4 – response_memory stress (two 4-beat bursts, reversed by fabric)
  //   • Two ARs with same ID=9, each len=4.
  //   • Fabric returns all 4 beats of the second burst first, then all 4 beats of
  //     the first burst.
  //   • Expectation: ROB must buffer in response_memory and emit:
  //       - first burst (ID=9) 4 beats, last=1 on beat 3
  //       - then second burst (ID=9) 4 beats, last=1 on beat 3
  //
  // Between scenarios:
  //   • We wait some idle cycles, then pulse rst (do_reset_between_scenarios),
  //     then reset scoreboard + UID capture and update current_scenario.
  // ---------------------------------------------------------------------------
  initial begin : main_stimulus
    logic [DATA_WIDTH-1:0] base3;
    logic [DATA_WIDTH-1:0] base4_a;
    logic [DATA_WIDTH-1:0] base4_b;
    int k;

    // Wait for initial reset to deassert
    @(posedge clk);
    while (rst == 1'b1) begin
      @(posedge clk);
    end

    // ============================
    // Scenario 1
    // ============================
    current_scenario = 1;
    $display("---- SCENARIO 1: one read, in-order response ----");
    sb_reset();
    uid_reset();

    // Expect one beat with ID=0 and data = "GILSTOLR"
    sb_expect(ID_WIDTH'(0), DATA_WIDTH'(64'h4749_4C53_544F_4C52), 1'b1);  // "GILSTOLR"
    // 1 beat → len = 1 (testbench convention: len = number of beats)
    send_read_req(ID_WIDTH'(0), ADDR_WIDTH'(32'h0000_1000), LEN_WIDTH'(8'd1));
    send_single_beat_rsp(captured_uid[0], DATA_WIDTH'(64'h4749_4C53_544F_4C52), RESP_WIDTH'(2'b00));
    sb_wait_all("SCENARIO_1");

    // Idle gap + reset between scenarios
    repeat (10) @(posedge clk);
    do_reset_between_scenarios();
    sb_reset();
    uid_reset();

    // ============================
    // Scenario 2
    // ============================
    current_scenario = 2;
    $display("---- SCENARIO 2: two IDs, out-of-order fabric returns ----");

    // Two separate 1-beat reads, different original IDs
    send_read_req(ID_WIDTH'(3), ADDR_WIDTH'(32'h0000_2000), LEN_WIDTH'(8'd1)); // first request, orig_id=3
    send_read_req(ID_WIDTH'(5), ADDR_WIDTH'(32'h0000_3000), LEN_WIDTH'(8'd1)); // second request, orig_id=5

    // Golden order at master: 3 then 5 (ROB keeps per-ID semantics; different IDs can be seen in issue order)
    sb_expect(ID_WIDTH'(3), DATA_WIDTH'(64'h1111_0000_0000_0001), 1'b1);
    sb_expect(ID_WIDTH'(5), DATA_WIDTH'(64'h2222_0000_0000_0002), 1'b1);

    // Fabric returns second UID first, then first UID:
    send_single_beat_rsp(captured_uid[1], DATA_WIDTH'(64'h2222_0000_0000_0002), RESP_WIDTH'(2'b00));
    send_single_beat_rsp(captured_uid[0], DATA_WIDTH'(64'h1111_0000_0000_0001), RESP_WIDTH'(2'b00));

    sb_wait_all("SCENARIO_2");

    // Idle gap + reset between scenarios
    repeat (10) @(posedge clk);
    do_reset_between_scenarios();
    sb_reset();
    uid_reset();

    // ============================
    // Scenario 3
    // ============================
    current_scenario = 3;
    $display("---- SCENARIO 3: single ID, 4-beat burst ----");

    // len = 4 (we treat len as number of beats in TB)
    send_read_req(ID_WIDTH'(7), ADDR_WIDTH'(32'h0000_4000), LEN_WIDTH'(8'd4));

    base3 = DATA_WIDTH'(64'hAAA0_0000_0000_0000);
    // Golden: 4 beats, last=1 on final beat
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(7), base3 + DATA_WIDTH'(k), (k == 3));
    end

    // Fabric returns the same pattern in-order
    send_burst_rsp(captured_uid[0], 4, base3);

    sb_wait_all("SCENARIO_3");

    // Idle gap + reset between scenarios
    repeat (10) @(posedge clk);
    do_reset_between_scenarios();
    sb_reset();
    uid_reset();

    // ============================
    // Scenario 4 – response_memory stress
    // ============================
    current_scenario = 4;
    $display("---- SCENARIO 4: response_memory stress (two 4-beat bursts, out-of-order) ----");

    // Two 4-beat bursts with same original ID=9
    send_read_req(ID_WIDTH'(9), ADDR_WIDTH'(32'h0000_5000), LEN_WIDTH'(8'd4)); // first burst
    send_read_req(ID_WIDTH'(9), ADDR_WIDTH'(32'h0000_6000), LEN_WIDTH'(8'd4)); // second burst

    base4_a = DATA_WIDTH'(64'hA000_0000_0000_0000);
    base4_b = DATA_WIDTH'(64'hB000_0000_0000_0000);

    // Golden view at master:
    //   First all beats of first transaction (base4_a), then all beats of second (base4_b)
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(9), base4_a + DATA_WIDTH'(k), (k == 3));
    end
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(9), base4_b + DATA_WIDTH'(k), (k == 3));
    end

    // Fabric returns COMPLETELY reversed order:
    //   1) all beats of second UID
    //   2) all beats of first UID
    send_burst_rsp(captured_uid[1], 4, base4_b); // second AR's UID
    send_burst_rsp(captured_uid[0], 4, base4_a); // first AR's UID

    sb_wait_all("SCENARIO_4");

    $display("==== ALL SCENARIOS DONE ====");
    #20;
    $finish;
  end

endmodule
