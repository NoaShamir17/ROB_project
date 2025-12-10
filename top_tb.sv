// ============================================================================
// top_tb – Testbench for AXI Read Order Buffer (ROB)
// ============================================================================

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

  // Timeout for any handshake (AR or R) to avoid infinite simulation
  localparam int TIMEOUT_CYCLES = 1000;

  // Useful constant: "GILSTOLR" encoded in ASCII into a 64-bit word
  localparam logic [DATA_WIDTH-1:0] DATA_GILSTOLR =
      DATA_WIDTH'(64'h4749_4C53_544F_4C52); // 'G' 'I' 'L' 'S' 'T' 'O' 'L' 'R'

  // ---------------------------------------------------------------------------
  // Scenario flag (for easier log reading)
  // ---------------------------------------------------------------------------
  // scenario_id helps us tag all prints/errors with which scenario is active.
  // 0 = idle, 1..5 = specific scenario running.
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
  // Important:
  //   • We only enforce per-ID ordering.
  //   • Between different IDs, beats may interleave arbitrarily.
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [ID_WIDTH-1:0]   id;
    logic [DATA_WIDTH-1:0] data;
    logic                  last;
  } expected_rsp_t;

  expected_rsp_t expected_q   [0:255]; // enough for all beats in all scenarios
  logic          expected_done[0:255];

  int exp_count;     // how many expected beats are defined
  int beats_seen;    // how many beats have matched successfully

  // Reset scoreboard counters
  task sb_reset;
    int index_reset;
    begin
      exp_count  = 0;
      beats_seen = 0;
      for (index_reset = 0; index_reset < 256; index_reset = index_reset + 1) begin
        expected_done[index_reset] = 1'b0;
      end
    end
  endtask

  // Push one expected beat (id, data, last)
  task sb_expect(
    input logic [ID_WIDTH-1:0]   id,
    input logic [DATA_WIDTH-1:0] data,
    input logic                  last
  );
    begin
      expected_q[exp_count].id   = id;
      expected_q[exp_count].data = data;
      expected_q[exp_count].last = last;
      expected_done[exp_count]   = 1'b0;
      exp_count = exp_count + 1;
    end
  endtask

  // Monitor R channel (DUT -> master) and compare with expected_q
  // Rule: for each ID, beats must appear in the same order as we pushed them
  //       with sb_expect. Beats from different IDs can be interleaved freely.
  always @(posedge clk) begin
    if (rst == 1'b0) begin
      if ((axi_r_out_if.valid & axi_r_out_if.ready) == 1'b1) begin
        int search_index;
        int earlier_index;
        int matched_index;
        bit match_found;
        bit earlier_open;

        // Do not print R road in scenario five
        if (scenario_id != 5) begin
          $display("[%0t] (scenario %0d) R ROB->MASTER: id=%0b data=%h last=%0b resp=%0b",
                   $time, scenario_id,
                   axi_r_out_if.id,
                   axi_r_out_if.data,
                   axi_r_out_if.last,
                   axi_r_out_if.resp);
        end

        matched_index = -1;
        match_found   = 1'b0;

        // Look for a matching expected entry that:
        //   • has same (id,data,last)
        //   • is not already used
        //   • has no earlier, still-open entry with the same id
        for (search_index = 0; search_index < exp_count; search_index = search_index + 1) begin
          if (!expected_done[search_index] &&
              axi_r_out_if.id   == expected_q[search_index].id &&
              axi_r_out_if.data == expected_q[search_index].data &&
              axi_r_out_if.last == expected_q[search_index].last) begin

            earlier_open = 1'b0;
            for (earlier_index = 0; earlier_index < search_index; earlier_index = earlier_index + 1) begin
              if (!expected_done[earlier_index] &&
                  expected_q[earlier_index].id == axi_r_out_if.id) begin
                earlier_open = 1'b1;
              end
            end

            if (!earlier_open && !match_found) begin
              matched_index = search_index;
              match_found   = 1'b0 | 1'b1; // keep as single assignment bitwise style
              match_found   = 1'b1;
            end
          end
        end

        if (!match_found) begin
          // In scenario five we stay silent (no per-beat scoreboard errors),
          // in other scenarios we still flag mismatches.
          if (scenario_id != 5) begin
            $error("[%0t] (scenario %0d) SCOREBOARD ERROR: beat id=%0b data=%h last=%0b does not match any valid next beat for its ID",
                   $time, scenario_id,
                   axi_r_out_if.id,
                   axi_r_out_if.data,
                   axi_r_out_if.last);
          end
        end
        else begin
          expected_done[matched_index] = 1'b1;
          beats_seen = beats_seen + 1;

          // Do NOT print OK lines in scenario five to keep log short.
          if (scenario_id != 5) begin
            $display("[%0t] (scenario %0d) SCOREBOARD: beat %0d OK (id=%0b matched_index=%0d)",
                     $time, scenario_id, beats_seen - 1,
                     axi_r_out_if.id, matched_index);
          end
        end
      end
    end
  end

  // Wait until all expected beats were observed or timeout
  task sb_wait_all(input string tag);
    int guard_counter;
    begin
      guard_counter = 0;
      while ((beats_seen < exp_count) & (guard_counter < TIMEOUT_CYCLES)) begin
        @(posedge clk);
        guard_counter = guard_counter + 1;
      end

      if (beats_seen == exp_count) begin
        $display("[%0t] (scenario %0d) scenario %s completed ok (%0d beats)",
                 $time, scenario_id, tag, exp_count);
      end
      else begin
        $error("[%0t] (scenario %0d) TIMEOUT: ROB did not produce all expected beats in scenario %s (seen %0d / %0d)",
               $time, scenario_id, tag, beats_seen, exp_count);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Track internal UIDs sent by DUT on axi_ar_out_if.id
  // ---------------------------------------------------------------------------
  localparam int MAX_REQ = 32; // allow enough captured requests (incl. scenario five)
  logic [ID_WIDTH-1:0] captured_uid [0:MAX_REQ-1];
  int                  uid_count;

  task uid_reset;
    begin
      uid_count = 0;
    end
  endtask

  // Non-blocking AR monitor: capture UIDs whenever ROB forwards to slave
  always @(posedge clk) begin
    if (rst == 1'b0) begin
      if ((axi_ar_out_if.valid & axi_ar_out_if.ready) == 1'b1) begin
        captured_uid[uid_count] = axi_ar_out_if.id;
        $display("[%0t] (scenario %0d) AR ROB->SLAVE: uid=%0b addr=%h len=%0d size=%0b burst=%0b qos=%0b",
                 $time, scenario_id,
                 axi_ar_out_if.id,
                 axi_ar_out_if.addr,
                 axi_ar_out_if.len,
                 axi_ar_out_if.size,
                 axi_ar_out_if.burst,
                 axi_ar_out_if.qos);
        uid_count = uid_count + 1;
      end
    end
  end

  // Wait until at least "required" UIDs were observed or timeout
  task wait_for_uids(input int required);
    int guard_counter;
    begin
      guard_counter = 0;
      while ((uid_count < required) & (guard_counter < TIMEOUT_CYCLES)) begin
        @(posedge clk);
        guard_counter = guard_counter + 1;
      end
      if (uid_count < required) begin
        $error("[%0t] (scenario %0d) TIMEOUT: did not observe %0d AR ROB->SLAVE transactions (only %0d)",
               $time, scenario_id, required, uid_count);
      end
    end
  endtask

  // Small helper: idle cycles between scenarios
  task wait_idle_cycles(input int cycles);
    int cycle_index;
    begin
      for (cycle_index = 0; cycle_index < cycles; cycle_index = cycle_index + 1) begin
        @(posedge clk);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read address driver on master side (axi_ar_in_if)
  // ---------------------------------------------------------------------------
  task send_read_req(
    input logic [ID_WIDTH-1:0]    id,
    input logic [ADDR_WIDTH-1:0]  addr,
    input logic [LEN_WIDTH-1:0]   len,
    input logic [SIZE_WIDTH-1:0]  size,
    input logic [BURST_WIDTH-1:0] burst,
    input logic [QOS_WIDTH-1:0]   qos
  );
    int guard_counter;
    begin
      #1
      axi_ar_in_if.id    = id;
      axi_ar_in_if.addr  = addr;
      axi_ar_in_if.len   = len;
      axi_ar_in_if.size  = size;
      axi_ar_in_if.burst = burst;
      axi_ar_in_if.qos   = qos;

      axi_ar_in_if.valid = 1'b1;

      guard_counter = 0;
      @(posedge clk);
      while ((axi_ar_in_if.valid & axi_ar_in_if.ready) == 1'b0 &
             guard_counter < TIMEOUT_CYCLES) begin
        guard_counter = guard_counter + 1;
        @(posedge clk);
      end

      if ((axi_ar_in_if.valid & axi_ar_in_if.ready) == 1'b0) begin
        $error("[%0t] (scenario %0d) TIMEOUT: AR MASTER->ROB handshake FAILED (id=%0b addr=%h len=%0d size=%0b burst=%0b qos=%0b)",
               $time, scenario_id, id, addr, len, size, burst, qos);
        #1
        axi_ar_in_if.valid = 1'b0;
        disable send_read_req;
      end

      $display("[%0t] (scenario %0d) AR MASTER->ROB: id=%0b addr=%h len=%0d size=%0b burst=%0b qos=%0b",
               $time, scenario_id, id, addr, len, size, burst, qos);

      #1
      axi_ar_in_if.valid = 1'b0;
      // Note: UID is now captured by the AR monitor, not here.
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver on slave side (axi_r_in_if) – single beat
  // ---------------------------------------------------------------------------
  task send_single_beat_rsp(
    input logic [ID_WIDTH-1:0]   id,
    input logic [DATA_WIDTH-1:0] data,
    input logic [RESP_WIDTH-1:0] resp,
    input logic                  last
  );
    int guard_counter;
    begin
      #1
      axi_r_in_if.id    = id;
      axi_r_in_if.data  = data;
      axi_r_in_if.resp  = resp;
      axi_r_in_if.last  = last;
      axi_r_in_if.valid = 1'b1;

      guard_counter = 0;
      @(posedge clk);
      while ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0 &
             guard_counter < TIMEOUT_CYCLES) begin
        guard_counter = guard_counter + 1;
        @(posedge clk);
      end

      if ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0) begin
        $error("[%0t] (scenario %0d) TIMEOUT: R SLAVE->ROB handshake FAILED (id=%0b data=%h last=%0b resp=%0b)",
               $time, scenario_id,
               id, data, last, resp);
        #1
        axi_r_in_if.valid = 1'b0;
        disable send_single_beat_rsp;
      end

      if (scenario_id != 5) begin
        $display("[%0t] (scenario %0d) R SLAVE->ROB: id=%0b data=%h last=%0b resp=%0b",
                 $time, scenario_id,
                 id, data, last, resp);
      end

      #1
      axi_r_in_if.valid = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver – multi-beat burst
  // ---------------------------------------------------------------------------
  task send_burst_rsp(
    input logic [ID_WIDTH-1:0]   id,
    input int                    beats,
    input logic [DATA_WIDTH-1:0] base_data,
    input logic [RESP_WIDTH-1:0] resp
  );
    int beat_index;
    int guard_counter;
    logic [DATA_WIDTH-1:0] cur_data;
    logic                  last;
    begin
      #1
      for (beat_index = 0; beat_index < beats; beat_index = beat_index + 1) begin
        cur_data = base_data + DATA_WIDTH'(beat_index);
        last     = (beat_index == (beats - 1));

        axi_r_in_if.id    = id;
        axi_r_in_if.data  = cur_data;
        axi_r_in_if.resp  = resp;
        axi_r_in_if.last  = last;
        axi_r_in_if.valid = 1'b1;

        guard_counter = 0;
        @(posedge clk);
        while ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0 &
               guard_counter < TIMEOUT_CYCLES) begin
          guard_counter = guard_counter + 1;
          @(posedge clk);
        end

        if ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0) begin
          $error("[%0t] (scenario %0d) TIMEOUT: R SLAVE->ROB handshake FAILED in burst (id=%0b data=%h beat=%0d last=%0b resp=%0b)",
                 $time, scenario_id,
                 id, cur_data, beat_index, last, resp);
          #1
          axi_r_in_if.valid = 1'b0;
          disable send_burst_rsp;
        end
        
        if (scenario_id != 5) begin
          $display("[%0t] (scenario %0d) R SLAVE->ROB BURST: beat %0d id=%0b data=%h last=%0b resp=%0b",
                   $time, scenario_id,
                   beat_index, id, cur_data, last, resp);
        end

        #1
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

    // Slave AR side always ready (no backpressure on AR)
    axi_ar_out_if.ready = 1'b1;

    // Master R side always ready (no backpressure on R)
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
  // ---------------------------------------------------------------------------
  initial begin : main_stimulus
    // Base data patterns for different scenarios
    logic [DATA_WIDTH-1:0] base_three;
    logic [DATA_WIDTH-1:0] base_four_a;
    logic [DATA_WIDTH-1:0] base_four_b;

    logic [DATA_WIDTH-1:0] base_seven;
    logic [DATA_WIDTH-1:0] base_nine;
    logic [DATA_WIDTH-1:0] base_eleven;

    // Scenario five base pattern
    logic [DATA_WIDTH-1:0] base_scenario_five;
    logic [DATA_WIDTH-1:0] base_data_request_five;

    // Temporary values used in scoreboard building
    logic [DATA_WIDTH-1:0] data_val_seven;
    logic [DATA_WIDTH-1:0] data_val_nine;
    logic [DATA_WIDTH-1:0] data_val_eleven;

    // Temporary values used in fabric-response patterns
    logic [DATA_WIDTH-1:0] base_req_eleven;
    logic [DATA_WIDTH-1:0] base_req_nine;
    logic [DATA_WIDTH-1:0] base_req_seven;

    // Original id and address helpers (scenario five)
    logic [ID_WIDTH-1:0]   orig_id_scenario_five;
    logic [ID_WIDTH-1:0]   orig_id_scenario_five_b;
    logic [ADDR_WIDTH-1:0] addr_scenario_five;

    // Helpers for scenario five request loop
    logic [ID_WIDTH-1:0]   current_id_scenario_five;
    logic [ADDR_WIDTH-1:0] current_addr_scenario_five;

    int loop_index;
    int request_index;
    int row_index;
    int beat_index;
    int local_index;
    int blocked_cycles;

    int count_seven;
    int count_nine;
    int count_eleven;

    // Wait until reset is deasserted
    @(posedge clk);
    while (rst == 1'b1) begin
      @(posedge clk);
    end

    // ============================
    // Scenario one – in-order single beat
    // ----------------------------
    // AR master sends one read with original id zero and length one.
    // Slave returns one beat with that id. ROB should pass it straight through.
    // ============================
    scenario_id = 1;
    $display("---- SCENARIO 1: one read, in-order response ----");
    sb_reset();
    uid_reset();

    sb_expect(ID_WIDTH'(0), DATA_GILSTOLR, 1'b1);

    send_read_req(
      ID_WIDTH'(0),
      ADDR_WIDTH'(32'h0000_1000),
      LEN_WIDTH'(1),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // Wait until AR was forwarded and UID captured
    wait_for_uids(1);

    send_single_beat_rsp(
      captured_uid[0],
      DATA_GILSTOLR,
      RESP_WIDTH'(0),
      1'b1
    );

    sb_wait_all("SCENARIO_1");
    wait_idle_cycles(10);

    // ============================
    // Scenario two – two IDs, swapped fabric returns
    // ----------------------------
    // AR master sends two single-beat reads with ids three and five.
    // Slave returns them in reverse order by internal uid.
    // ROB must preserve ordering per original id, scoreboard checks that.
    // ============================
    scenario_id = 2;
    $display("---- SCENARIO 2: two IDs, swapped fabric returns (no cross-ID reordering) ----");
    sb_reset();
    uid_reset();

    send_read_req(
      ID_WIDTH'(3),
      ADDR_WIDTH'(32'h0000_2000),
      LEN_WIDTH'(1),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    send_read_req(
      ID_WIDTH'(5),
      ADDR_WIDTH'(32'h0000_3000),
      LEN_WIDTH'(1),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // Wait for both ARs to be forwarded
    wait_for_uids(2);

    sb_expect(ID_WIDTH'(5), DATA_WIDTH'(64'h2222_0000_0000_0002), 1'b1);
    sb_expect(ID_WIDTH'(3), DATA_WIDTH'(64'h1111_0000_0000_0001), 1'b1);

    // Fabric returns in reverse internal uid order
    send_single_beat_rsp(
      captured_uid[1],
      DATA_WIDTH'(64'h2222_0000_0000_0002),
      RESP_WIDTH'(0),
      1'b1
    );

    send_single_beat_rsp(
      captured_uid[0],
      DATA_WIDTH'(64'h1111_0000_0000_0001),
      RESP_WIDTH'(0),
      1'b1
    );

    sb_wait_all("SCENARIO_2");
    wait_idle_cycles(10);

    // ============================
    // Scenario three – single ID, four-beat burst, in-order
    // ----------------------------
    // AR master sends one burst (length four) with original id five.
    // Slave returns four beats in-order with that internal uid.
    // ROB must preserve the beat order for that id.
    // ============================
    scenario_id = 3;
    $display("---- SCENARIO 3: single ID, 4-beat burst ----");
    sb_reset();
    uid_reset();

    send_read_req(
      ID_WIDTH'(5),
      ADDR_WIDTH'(32'h0000_4000),
      LEN_WIDTH'(4),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // Wait for AR to be forwarded
    wait_for_uids(1);

    base_three = DATA_WIDTH'(64'hAAA0_0000_0000_0000);

    for (loop_index = 0; loop_index < 4; loop_index = loop_index + 1) begin
      sb_expect(ID_WIDTH'(5), base_three + DATA_WIDTH'(loop_index), (loop_index == 3));
    end

    send_burst_rsp(
      captured_uid[0],
      4,
      base_three,
      RESP_WIDTH'(0)
    );

    sb_wait_all("SCENARIO_3");
    wait_idle_cycles(10);

    // ============================
    // Scenario four – response_memory stress, two bursts reversed
    // ----------------------------
    // AR master sends two bursts (length four each) with same original id seven.
    // Slave returns second burst first, then first burst, by internal uid order.
    // ROB must reorder so that master sees all beats from first burst,
    // then all beats from second burst, for the same original id.
    // ============================
    scenario_id = 4;
    $display("---- SCENARIO 4: response_memory stress (two 4-beat bursts, out-of-order) ----");
    sb_reset();
    uid_reset();

    send_read_req(
      ID_WIDTH'(7),
      ADDR_WIDTH'(32'h0000_5000),
      LEN_WIDTH'(4),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    send_read_req(
      ID_WIDTH'(7),
      ADDR_WIDTH'(32'h0000_6000),
      LEN_WIDTH'(4),
      SIZE_WIDTH'(3),
      BURST_WIDTH'(1),
      QOS_WIDTH'({QOS_WIDTH{1'b0}})
    );

    // Wait for both ARs to be forwarded
    wait_for_uids(2);

    base_four_a = DATA_WIDTH'(64'hA000_0000_0000_0000);
    base_four_b = DATA_WIDTH'(64'hB000_0000_0000_0000);

    for (loop_index = 0; loop_index < 4; loop_index = loop_index + 1) begin
      sb_expect(ID_WIDTH'(7), base_four_a + DATA_WIDTH'(loop_index), (loop_index == 3));
    end
    for (loop_index = 0; loop_index < 4; loop_index = loop_index + 1) begin
      sb_expect(ID_WIDTH'(7), base_four_b + DATA_WIDTH'(loop_index), (loop_index == 3));
    end

    // Slave returns second burst first
    send_burst_rsp(
      captured_uid[1],
      4,
      base_four_b,
      RESP_WIDTH'(0)
    );
    // Then first burst
    send_burst_rsp(
      captured_uid[0],
      4,
      base_four_a,
      RESP_WIDTH'(0)
    );

    sb_wait_all("SCENARIO_4");
    wait_idle_cycles(10);

    // ============================
    // Scenario five – allocator MAX_OUTSTANDING limit
    // ----------------------------
    // AR master sends seventeen read requests, each with length two.
    // Original ids alternate between seven and nine.
    // Allocator can hold at most sixteen outstanding tags.
    //
    // Expectation:
    //   • All seventeen AR MASTER->ROB handshakes succeed.
    //   • While allocator is full, one request is held back internally.
    //   • As responses free tags, the seventeenth AR is forwarded later.
    //   • We send two-beat responses for all forwarded UIDs (all 17).
    //   • R path is exercised, but we print nothing on R channels here.
    // ============================
    scenario_id = 5;
    $display("---- SCENARIO 5: allocator MAX_OUTSTANDING limit (seventeen requests, two-beat bursts) ----");
    sb_reset();
    uid_reset();

    // Two different original IDs
    orig_id_scenario_five   = ID_WIDTH'(7);
    orig_id_scenario_five_b = ID_WIDTH'(9);
    addr_scenario_five      = ADDR_WIDTH'(32'h0000_7000);

    // Send seventeen read requests, each LEN=2 (two beats), alternating ids.
    for (request_index = 0; request_index < 17; request_index = request_index + 1) begin
      current_id_scenario_five   = (request_index[0] == 1'b0) ? orig_id_scenario_five : orig_id_scenario_five_b;
      current_addr_scenario_five = addr_scenario_five + ADDR_WIDTH'(request_index * 8);

      send_read_req(
        current_id_scenario_five,
        current_addr_scenario_five,
        LEN_WIDTH'(2),
        SIZE_WIDTH'(3),
        BURST_WIDTH'(1),
        QOS_WIDTH'({QOS_WIDTH{1'b0}})
      );
      // No scoreboard expectations in scenario five.
    end

    $display("---- SCENARIO 5: finished issuing seventeen read requests ----");

    // Wait until at least sixteen UIDs have been observed (allocator full point)
    wait_for_uids(16);

    // First wave of responses for the first sixteen forwarded requests
    base_scenario_five = DATA_WIDTH'(64'hC000_0000_0000_0000);

    for (request_index = 0; request_index < 16; request_index = request_index + 1) begin
      base_data_request_five = base_scenario_five + DATA_WIDTH'(request_index << 4);

      send_burst_rsp(
        captured_uid[request_index],
        2,
        base_data_request_five,
        RESP_WIDTH'(0)
      );
    end

    // Now allocator has free tags; the seventeenth request can be forwarded.
    // Wait for UID seventeen to show up.
    wait_for_uids(17);

    // Send response for the seventeenth forwarded request as well
    base_data_request_five = base_scenario_five + DATA_WIDTH'(16 << 4);

    send_burst_rsp(
      captured_uid[16],
      2,
      base_data_request_five,
      RESP_WIDTH'(0)
    );

    // Let all responses drain
    wait_idle_cycles(50);

    // -------------------------------------------------------------------------
    // End of all scenarios
    // -------------------------------------------------------------------------
    scenario_id = 0;
    $display("==== ALL SCENARIOS DONE ====");
    #20;
    $finish;
  end

endmodule
