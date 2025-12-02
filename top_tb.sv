module top_tb;

  // ---------------------------------------------------------------------------
  // Local parameters – must match DUT + interfaces
  // ---------------------------------------------------------------------------
  // Global widths / sizes – these must be aligned with top.sv and ar_if / r_if
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

  // Timeout for any handshake (AR or R) to avoid infinite simulation
  localparam int TIMEOUT_CYCLES = 1000;

  // Useful constant: "GILSTOLR" encoded in ASCII into a 64-bit word
  localparam logic [DATA_WIDTH-1:0] DATA_GILSTOLR =
      DATA_WIDTH'(64'h4749_4C53_544F_4C52); // 'G' 'I' 'L' 'S' 'T' 'O' 'L' 'R'

  // ---------------------------------------------------------------------------
  // Scenario flag (for easier log reading)
  // ---------------------------------------------------------------------------
  // scenario_id helps us tag all prints/errors with which scenario is active.
  // 0 = idle, 1..4 = specific scenario running.
  int scenario_id;

  // ---------------------------------------------------------------------------
  // Clock and reset
  // ---------------------------------------------------------------------------
  logic clk;
  logic rst;

  // Free-running clock: 10ns period, 50% duty cycle.
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  // Synchronous reset: held high for 5 cycles, then deasserted.
  initial begin
    rst = 1'b1;
    repeat (5) @(posedge clk);
    rst = 1'b0;
  end

  // ---------------------------------------------------------------------------
  // Instantiate AXI interfaces that connect to the DUT
  // ---------------------------------------------------------------------------
  // These interfaces model the AXI AR and R channels on both master and slave
  // sides around the ROB top.
  ar_if #(
    .ID_WIDTH    (ID_WIDTH),
    .ADDR_WIDTH  (ADDR_WIDTH),
    .LEN_WIDTH   (LEN_WIDTH),
    .SIZE_WIDTH  (SIZE_WIDTH),
    .BURST_WIDTH (BURST_WIDTH),
    .QOS_WIDTH   (QOS_WIDTH)
  ) axi_ar_in_if ();   // AR from "master" (testbench) into ROB

  ar_if #(
    .ID_WIDTH    (ID_WIDTH),
    .ADDR_WIDTH  (ADDR_WIDTH),
    .LEN_WIDTH   (LEN_WIDTH),
    .SIZE_WIDTH  (SIZE_WIDTH),
    .BURST_WIDTH (BURST_WIDTH),
    .QOS_WIDTH   (QOS_WIDTH)
  ) axi_ar_out_if ();  // AR from ROB out to "slave" (fabric)

  r_if #(
    .ID_WIDTH   (ID_WIDTH),
    .DATA_WIDTH (DATA_WIDTH),
    .RESP_WIDTH (RESP_WIDTH)
  ) axi_r_out_if ();   // R from ROB to "master" (testbench)

  r_if #(
    .ID_WIDTH   (ID_WIDTH),
    .DATA_WIDTH (DATA_WIDTH),
    .RESP_WIDTH (RESP_WIDTH)
  ) axi_r_in_if ();    // R from "slave" (testbench) into ROB

  // ---------------------------------------------------------------------------
  // DUT instantiation – the ROB top module
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
  //
  // Role:
  //   • expected_q holds the "golden" sequence of R beats that the ROB must
  //     send to the master.
  //   • We fill expected_q using sb_expect() before sending the fabric R beats.
  //   • The monitor (always block) pops and compares on each R handshake.
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [ID_WIDTH-1:0]   id;
    logic [DATA_WIDTH-1:0] data;
    logic                  last;
  } expected_rsp_t;

  expected_rsp_t expected_q [0:31]; // small fixed-size queue of expected beats
  int exp_count;                    // how many entries we wrote
  int exp_index;                    // next entry index to check against DUT

  // Reset scoreboard counters to empty
  task sb_reset;
    begin
      exp_count = 0;
      exp_index = 0;
    end
  endtask

  // Push one expected beat (id, data, last) into the scoreboard queue
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

  // Monitor R channel (DUT -> master) and check against expected_q
  always @(posedge clk) begin
    if (rst == 1'b0) begin
      // Handshake: one beat is actually transferred when valid & ready == 1
      if ((axi_r_out_if.valid & axi_r_out_if.ready) == 1'b1) begin
        // Print both hex and ASCII (for DATA_GILSTOLR we see text "GILSTOLR")
        $display("[%0t] (scenario %0d) R ROB->MASTER: id=%0b data=%h (%s) last=%0b resp=%0b",
                 $time, scenario_id,
                 axi_r_out_if.id,
                 axi_r_out_if.data, axi_r_out_if.data,
                 axi_r_out_if.last,
                 axi_r_out_if.resp);

        // If scoreboard has no more expected entries, this beat is unexpected
        if (exp_index >= exp_count) begin
          $error("[%0t] (scenario %0d) unexpected response from ROB: id=%0b data=%h last=%0b",
                 $time, scenario_id,
                 axi_r_out_if.id, axi_r_out_if.data, axi_r_out_if.last);
        end
        else begin
          // Compare actual vs expected for id/data/last
          if ( (axi_r_out_if.id   == expected_q[exp_index].id) &
               (axi_r_out_if.data == expected_q[exp_index].data) &
               (axi_r_out_if.last == expected_q[exp_index].last) ) begin
            $display("[%0t] (scenario %0d) SCOREBOARD: beat %0d OK (id=%0b)",
                     $time, scenario_id, exp_index, axi_r_out_if.id);
          end
          else begin
            // Detailed mismatch report
            $error("[%0t] (scenario %0d) SCOREBOARD MISMATCH at beat %0d: got id=%0b data=%h last=%0b expected id=%0b data=%h last=%0b",
                   $time, scenario_id, exp_index,
                   axi_r_out_if.id,   axi_r_out_if.data,   axi_r_out_if.last,
                   expected_q[exp_index].id,
                   expected_q[exp_index].data,
                   expected_q[exp_index].last);
          end
          // Move to next expected entry
          exp_index = exp_index + 1;
        end
      end
    end
  end

  // Wait until all expected beats were observed on R channel or timeout
  task sb_wait_all(input string tag);
    int guard;
    begin
      guard = 0;
      // Wait while not all expected beats received, but with timeout
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
  //
  // Idea:
  //   • Our allocator / remapper maps orig_id → internal UID {row,col}.
  //   • Testbench must use that UID on R channel when sending responses.
  //   • For each AR sent, we capture the UID in captured_uid[uid_count].
  // ---------------------------------------------------------------------------
  localparam int MAX_REQ = 16;
  logic [ID_WIDTH-1:0] captured_uid [0:MAX_REQ-1];
  int                  uid_count;

  // Reset UID capture queue
  task uid_reset;
    begin
      uid_count = 0;
    end
  endtask

  // Wait until ROB forwards an AR (ROB->SLAVE), then record its UID
  task capture_next_uid;
    int guard;
    begin
      guard = 0;
      @(posedge clk);
      // Wait for AR handshake: valid&ready == 1
      while ((axi_ar_out_if.valid & axi_ar_out_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      // If we timed out, report error and exit this task
      if ((axi_ar_out_if.valid & axi_ar_out_if.ready) == 1'b0) begin
        $error("[%0t] (scenario %0d) TIMEOUT: AR ROB->SLAVE handshake FAILED (no forwarded AR observed)",
               $time, scenario_id);
        disable capture_next_uid;
      end

      // Capture internal UID from ROB->SLAVE AR channel
      captured_uid[uid_count] = axi_ar_out_if.id;
      $display("[%0t] (scenario %0d) AR ROB->SLAVE: uid=%0b addr=%h len=%0d size=%0b burst=%0b qos=%0b",
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
  // Small helper: wait some idle cycles between scenarios
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
  //
  // Usage:
  //   • Drives all AR fields (id, addr, len, size, burst, qos).
  //   • Asserts valid, waits for ready from ROB.
  //   • Holds valid high across at least one full clock period, as required.
  //   • After handshake completes, deasserts valid and then calls
  //     capture_next_uid() to record the internal UID that the ROB sends out.
  //
  // Note: In this testbench, len is interpreted as "number of beats" directly
  // (not AXI-style len+1). The rest of the design must be consistent with that.
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
      // Drive all AR fields
      axi_ar_in_if.id    = id;
      axi_ar_in_if.addr  = addr;
      axi_ar_in_if.len   = len;
      axi_ar_in_if.size  = size;
      axi_ar_in_if.burst = burst;
      axi_ar_in_if.qos   = qos;

      axi_ar_in_if.valid = 1'b1;

      guard = 0;
      @(posedge clk); // value must be stable through a full clock edge
      // Wait for AR MASTER->ROB handshake (valid & ready == 1)
      while ((axi_ar_in_if.valid & axi_ar_in_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      if ((axi_ar_in_if.valid & axi_ar_in_if.ready) == 1'b0) begin
        $error("[%0t] (scenario %0d) TIMEOUT: AR MASTER->ROB handshake FAILED (id=%0b addr=%h len=%0d size=%0b burst=%0b qos=%0b)",
               $time, scenario_id, id, addr, len, size, burst, qos);
        axi_ar_in_if.valid = 1'b0;
        disable send_read_req;
      end

      $display("[%0t] (scenario %0d) AR MASTER->ROB: id=%0b addr=%h len=%0d size=%0b burst=%0b qos=%0b",
               $time, scenario_id, id, addr, len, size, burst, qos);

      // After handshake, deassert valid on the next cycle
      axi_ar_in_if.valid = 1'b0;

      // Now wait for ROB to forward this AR and capture UID
      capture_next_uid();
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver on slave side (axi_r_in_if) – single beat
  //
  // Usage:
  //   • Called when we want to send exactly one data beat from "fabric" (slave)
  //     into the ROB.
  //   • We drive id (UID), data, resp, last, raise valid, then wait for ready.
  //   • We keep valid high for at least one full cycle around posedge clk.
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
      @(posedge clk); // ensure data/valid are visible through the clock edge
      // Wait for R SLAVE->ROB handshake
      while ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      if ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0) begin
        $error("[%0t] (scenario %0d) TIMEOUT: R SLAVE->ROB handshake FAILED (id=%0b data=%h (%s) last=%0b resp=%0b)",
               $time, scenario_id,
               id, data, data, last, resp);
        axi_r_in_if.valid = 1'b0;
        disable send_single_beat_rsp;
      end

      $display("[%0t] (scenario %0d) R SLAVE->ROB: id=%0b data=%h (%s) last=%0b resp=%0b",
               $time, scenario_id,
               id, data, data, last, resp);

      axi_r_in_if.valid = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver – multi-beat burst
  //
  // Usage:
  //   • Sends 'beats' beats of data for a given UID.
  //   • Each beat increments data by +k (cast to DATA_WIDTH).
  //   • 'last' is asserted only on the final beat.
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
        // Wait for handshake of this beat
        while ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0 &
               guard < TIMEOUT_CYCLES) begin
          guard = guard + 1;
          @(posedge clk);
        end

        if ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0) begin
          $error("[%0t] (scenario %0d) TIMEOUT: R SLAVE->ROB handshake FAILED in burst (id=%0b data=%h (%s) beat=%0d last=%0b resp=%0b)",
                 $time, scenario_id,
                 id, cur_data, cur_data, i, last, resp);
          axi_r_in_if.valid = 1'b0;
          disable send_burst_rsp;
        end

        $display("[%0t] (scenario %0d) R SLAVE->ROB BURST: beat %0d id=%0b data=%h (%s) last=%0b resp=%0b",
                 $time, scenario_id,
                 i, id, cur_data, cur_data, last, resp);

        axi_r_in_if.valid = 1'b0;
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Initial defaults for interfaces and scoreboard
  // ---------------------------------------------------------------------------
  initial begin
    scenario_id        = 0;

    // AR master defaults
    axi_ar_in_if.valid = 1'b0;
    axi_ar_in_if.id    = {ID_WIDTH{1'b0}};
    axi_ar_in_if.addr  = {ADDR_WIDTH{1'b0}};
    axi_ar_in_if.len   = {LEN_WIDTH{1'b0}};
    axi_ar_in_if.size  = {SIZE_WIDTH{1'b0}};
    axi_ar_in_if.burst = {BURST_WIDTH{1'b0}};
    axi_ar_in_if.qos   = {QOS_WIDTH{1'b0}};

    // We keep the slave side AR ready always high in TB
    axi_ar_out_if.ready = 1'b1;

    // Master R channel is always ready (backpressure-free master)
    axi_r_out_if.ready  = 1'b1;

    // R slave defaults
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
  //   • AR #0: id=3, len=1, data=1111....; AR #1: id=5, len=1, data=2222....
  //   • Fabric returns UID1 (ID=5) first, then UID0 (ID=3).
  //   • ROB is not required to reorder across different IDs, so master should
  //     see ID 5 first, then ID 3.
  //
  // Scenario 3 – single ID, 4-beat burst, in-order fabric
  //   • One AR with id=5, len=4.
  //   • Fabric returns 4 beats in-order; ROB passes them through.
  //
  // Scenario 4 – response_memory stress (two 4-beat bursts, out-of-order)
  //   • Two ARs with same id=7, each len=4.
  //   • Fabric returns *second* burst first and *first* burst second.
  //   • ROB should buffer correctly and emit all beats of first AR then all
  //     beats of second AR.
  // ---------------------------------------------------------------------------
  initial begin : main_stimulus
    logic [DATA_WIDTH-1:0] base3;
    logic [DATA_WIDTH-1:0] base4_a;
    logic [DATA_WIDTH-1:0] base4_b;
    int k;

    // Wait until reset is deasserted
    @(posedge clk);
    while (rst == 1'b1) begin
      @(posedge clk);
    end

    // ============================
    // Scenario 1 – in-order single beat
    // ============================
    scenario_id = 1;
    $display("---- SCENARIO 1: one read, in-order response ----");
    sb_reset();
    uid_reset();

    // Expected output at master side:
    //   R beat: id=0, data="GILSTOLR", last=1
    sb_expect(ID_WIDTH'(0), DATA_GILSTOLR, 1'b1);

    // Send AR with all fields specified
    send_read_req(
      ID_WIDTH'(0),                             // id
      ADDR_WIDTH'(32'h0000_1000),              // addr
      LEN_WIDTH'(1),                           // len = 1 beat
      SIZE_WIDTH'(3),                          // 3'b011 = 8 bytes
      BURST_WIDTH'(1),                         // 2'b01  = INCR
      QOS_WIDTH'({QOS_WIDTH{1'b0}})            // qos  = 0
    );

    // Slave returns one R beat for captured_uid[0] with "GILSTOLR"
    send_single_beat_rsp(
      captured_uid[0],                         // internal UID
      DATA_GILSTOLR,                           // data = "GILSTOLR"
      RESP_WIDTH'(0),                          // resp = OKAY
      1'b1                                     // last = 1
    );

    // Wait until scoreboard sees everything
    sb_wait_all("SCENARIO_1");
    wait_idle_cycles(10);

    // ============================
    // Scenario 2 – two IDs, swapped fabric returns
    // ============================
    scenario_id = 2;
    $display("---- SCENARIO 2: two IDs, swapped fabric returns (no cross-ID reordering) ----");
    sb_reset();
    uid_reset();

    // AR #0: orig_id=3, len=1
    send_read_req(
      ID_WIDTH'(3),
      ADDR_WIDTH'(32'h0000_2000),
      LEN_WIDTH'(1),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // AR #1: orig_id=5, len=1
    send_read_req(
      ID_WIDTH'(5),
      ADDR_WIDTH'(32'h0000_3000),
      LEN_WIDTH'(1),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // Fabric returns UID1 (ID=5) first, then UID0 (ID=3).
    // Since IDs are different, ROB may simply reflect that order.
    sb_expect(ID_WIDTH'(5), DATA_WIDTH'(64'h2222_0000_0000_0002), 1'b1);
    sb_expect(ID_WIDTH'(3), DATA_WIDTH'(64'h1111_0000_0000_0001), 1'b1);

    // Fabric: second AR's UID first...
    send_single_beat_rsp(
      captured_uid[1],
      DATA_WIDTH'(64'h2222_0000_0000_0002),
      RESP_WIDTH'(0),
      1'b1
    );
    // ...then first AR's UID
    send_single_beat_rsp(
      captured_uid[0],
      DATA_WIDTH'(64'h1111_0000_0000_0001),
      RESP_WIDTH'(0),
      1'b1
    );

    sb_wait_all("SCENARIO_2");
    wait_idle_cycles(10);

    // ============================
    // Scenario 3 – single ID, 4-beat burst, in-order
    // ============================
    scenario_id = 3;
    $display("---- SCENARIO 3: single ID, 4-beat burst ----");
    sb_reset();
    uid_reset();

    // One AR: id=5, 4 beats
    send_read_req(
      ID_WIDTH'(5),
      ADDR_WIDTH'(32'h0000_4000),
      LEN_WIDTH'(4),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    base3 = DATA_WIDTH'(64'hAAA0_0000_0000_0000);

    // Golden: 4 beats, last=1 only on the final beat
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(5), base3 + DATA_WIDTH'(k), (k == 3));
    end

    // Fabric returns 4 beats in-order for that UID
    send_burst_rsp(
      captured_uid[0],                          // UID for this AR
      4,                                        // beats
      base3,
      RESP_WIDTH'(0)
    );

    sb_wait_all("SCENARIO_3");
    wait_idle_cycles(10);

    // ============================
    // Scenario 4 – response_memory stress, two 4-beat bursts, reversed
    // ============================
    scenario_id = 4;
    $display("---- SCENARIO 4: response_memory stress (two 4-beat bursts, out-of-order) ----");
    sb_reset();
    uid_reset();

    // First 4-beat AR, orig_id=7
    send_read_req(
      ID_WIDTH'(7),
      ADDR_WIDTH'(32'h0000_5000),
      LEN_WIDTH'(4),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // Second 4-beat AR, same orig_id=7
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

    // Golden: all beats of first AR (base4_a) then all beats of second AR (base4_b)
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(7), base4_a + DATA_WIDTH'(k), (k == 3));
    end
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(7), base4_b + DATA_WIDTH'(k), (k == 3));
    end

    // Fabric returns bursts in reverse AR order:
    //   1) All beats of second AR's UID
    //   2) All beats of first AR's UID
    // This forces use of response_memory inside the ROB.
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
