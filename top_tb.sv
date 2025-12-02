module top_tb;

// ============================================================================
// TESTBENCH OVERVIEW
// ============================================================================
// This testbench is a *self-checking AXI environment* for the ROB top module.
// It plays two roles:
//
//   • AXI MASTER driving:
//        - axi_ar_in_if  : AR requests into the ROB
//        - axi_r_out_if  : R responses coming out of the ROB
//
//   • AXI SLAVE driving:
//        - axi_ar_out_if : AR forwarded from ROB to the “fabric”
//        - axi_r_in_if   : R responses fed into the ROB from the “fabric”
//
// So the testbench simulates:
//
//–  A CPU sending requests (AR)
//–  A memory / fabric sending responses (R)
// and checks that the ROB preserves the correct ordering semantics.
//
// The DUT gives every AR a *UID* (internal row/col tag). The testbench captures
// these UIDs and uses them to send back R data, possibly completely out-of-order,
// to verify that the ROB reorders everything correctly before output.
//
// ============================================================================
// SCOREBOARD LOGIC
// ============================================================================
// The scoreboard tracks *expected beats* that the DUT must produce on the
// master R channel (axi_r_out_if).  Two indices are used:
//
//   exp_count : number of beats pushed into the expected list
//   exp_index : how many beats have been observed & checked
//
// sb_expect() pushes one expected beat.
// The R-channel monitor pops beats in order and compares against expected_q.
// sb_wait_all() waits until all beats are received or a timeout occurs.
//
// This ensures each scenario is *generally correct* (all beats produced) and
// *specifically correct* (content matches the golden model).
//
// ============================================================================
// UID CAPTURE (AR PATH)
// ============================================================================
// Because the DUT remaps original IDs → internal UIDs, the testbench must
// record the UIDs it sees on axi_ar_out_if.id after each AR request.
//
// The flow is:
//
//   send_read_req():  Handshake AR MASTER→ROB
//                     → then call capture_next_uid()
//
//   capture_next_uid(): Wait for AR ROB→SLAVE handshake
//                       → record the UID into captured_uid[]
//
// Later, when sending R responses, we use:
//
//     send_single_beat_rsp(captured_uid[n], ...)
//     send_burst_rsp(captured_uid[n], ...)
//
// This ensures each response is tied to the correct internal UID.
//
// ============================================================================
// DRIVER TASKS (AR and R)
// ============================================================================
//
// All drivers (AR and R) follow the standard AXI handshake idiom:
//
//     1. Drive fields
//     2. Assert .valid
//     3. Wait until (valid & ready)
//     4. On success, log and deassert .valid
//     5. On timeout, raise $error
//
// AR DRIVER (send_read_req):
//   - Sends orig_id, addr, and len (treated here as “number of beats”).
//   - Waits for handshake.
//   - Captures UID on the AXI AR output.
//
// R DRIVERS (send_single_beat_rsp / send_burst_rsp):
//   - Simulate fabric returning one or many beats.
//   - You choose any order to stress the ROB’s reordering logic.
//   - The ROB must reorder beats back to the original ID ordering.
//
// ============================================================================
// SCENARIOS (WHAT EACH TEST DOES)
// ============================================================================
//
// Scenario 1:
//   • A single request, with one data beat returned in-order.
//   • Validates baseline plumbing: AR path, R path, UID mapping.
//
// Scenario 2:
//   • Two 1-beat requests with different original IDs.
//   • Fabric returns second UID first (out-of-order across IDs).
//   • Standard AXI allows out-of-order across IDs.
//   • ROB must simply pass beats according to arrival order.
//   • Verifies no unnecessary ordering enforced across different original IDs.
//
// Scenario 3:
//   • One 4-beat burst, fabric returns all 4 beats in order.
//   • Checks multi-beat constraints, last-beat marking, sequential checks.
//
// Scenario 4:
//   • Two 4-beat bursts (same original ID) returned *fully reversed*
//        → Fabric first returns *all beats of AR#2*
//        → then returns *all beats of AR#1*
//   • ROB must buffer everything in response_memory and release in logical order:
//        → First the 4 beats of AR#1
//        → Then the 4 beats of AR#2
//   • This is the main stress test for r_ordering_unit + response_park.
//
// ============================================================================
// CORE ALGORITHM OF THE ENTIRE TESTBENCH
// ============================================================================
//
// FOR EACH SCENARIO:
//     1. Reset scoreboard & UID tracking.
//     2. Send AR requests from master to ROB.
//     3. For each AR request, record the UID from ROB→SLAVE.
//     4. Feed back R beats:
//          - possibly in worst-case out-of-order patterns
//          - using the captured UIDs
//     5. Scoreboard monitors master R output and checks correctness.
//     6. sb_wait_all() confirms all expected beats were produced.
//
// If any scenario fails:
//     → ordering, buffering, UID mapping, or rm_store/req_release logic is wrong.
//
// If all scenarios pass:
//     → the ROB functions correctly in standard and stress conditions.
//
// ============================================================================
// END OF COMMENT BLOCK
// ============================================================================

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
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [ID_WIDTH-1:0]   id;
    logic [DATA_WIDTH-1:0] data;
    logic                  last;
  } expected_rsp_t;

  expected_rsp_t expected_q [0:31];
  int exp_count;
  int exp_index;

  task sb_reset;
    begin
      exp_count = 0;
      exp_index = 0;
    end
  endtask

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

  // Monitor DUT -> master R channel and compare
  always @(posedge clk) begin
    if (rst == 1'b0) begin
      if ((axi_r_out_if.valid & axi_r_out_if.ready) == 1'b1) begin
        $display("[%0t] R ROB->MASTER succeeded: id=%0b data=%h last=%0b resp=%0b",
                 $time, axi_r_out_if.id, axi_r_out_if.data,
                 axi_r_out_if.last, axi_r_out_if.resp);

        if (exp_index >= exp_count) begin
          $error("[%0t] unexpected response from ROB: id=%0b data=%h last=%0b",
                 $time, axi_r_out_if.id, axi_r_out_if.data, axi_r_out_if.last);
        end
        else begin
          if ( (axi_r_out_if.id   == expected_q[exp_index].id) &
               (axi_r_out_if.data == expected_q[exp_index].data) &
               (axi_r_out_if.last == expected_q[exp_index].last) ) begin
            $display("[%0t] SCOREBOARD: beat %0d OK (id=%0b)",
                     $time, exp_index, axi_r_out_if.id);
          end
          else begin
            $error("[%0t] SCOREBOARD MISMATCH at beat %0d: got id=%0b data=%h last=%0b expected id=%0b data=%h last=%0b",
                   $time, exp_index,
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

  task sb_wait_all(input string tag);
    int guard;
    begin
      guard = 0;
      while ((exp_index < exp_count) & (guard < TIMEOUT_CYCLES)) begin
        @(posedge clk);
        guard = guard + 1;
      end

      if (exp_index == exp_count) begin
        $display("[%0t] scenario %s completed ok (%0d beats)",
                 $time, tag, exp_count);
      end
      else begin
        $error("[%0t] TIMEOUT: R ROB->MASTER did not produce all expected beats in scenario %s (seen %0d / %0d)",
               $time, tag, exp_index, exp_count);
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Track internal UIDs sent by DUT on axi_ar_out_if.id
  // ---------------------------------------------------------------------------
  localparam int MAX_REQ = 16;
  logic [ID_WIDTH-1:0] captured_uid [0:MAX_REQ-1];
  int                  uid_count;

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
      while ((axi_ar_out_if.valid & axi_ar_out_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      if ((axi_ar_out_if.valid & axi_ar_out_if.ready) == 1'b0) begin
        $error("[%0t] TIMEOUT: AR ROB->SLAVE handshake FAILED (no forwarded AR observed)", $time);
        disable capture_next_uid;
      end

      captured_uid[uid_count] = axi_ar_out_if.id;
      $display("[%0t] AR ROB->SLAVE succeeded: uid=%0b addr=%h len=%0d",
               $time, captured_uid[uid_count],
               axi_ar_out_if.addr, axi_ar_out_if.len);

      uid_count = uid_count + 1;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read address driver on master side (axi_ar_in_if)
  //   NOTE: In this environment, "len_beats" = number of beats (non-AXI-spec).
  // ---------------------------------------------------------------------------
  task send_read_req(
    input logic [ID_WIDTH-1:0]   arid,
    input logic [ADDR_WIDTH-1:0] addr,
    input logic [LEN_WIDTH-1:0]  len_beats
  );
    int guard;
    begin
      axi_ar_in_if.id    = arid;
      axi_ar_in_if.addr  = addr;
      axi_ar_in_if.len   = len_beats; // here: ARLEN = number of beats
      axi_ar_in_if.size  = 3'b011;    // 8 bytes
      axi_ar_in_if.burst = 2'b01;     // INCR
      axi_ar_in_if.qos   = {QOS_WIDTH{1'b0}};

      axi_ar_in_if.valid = 1'b1;

      guard = 0;
      @(posedge clk);
      while ((axi_ar_in_if.valid & axi_ar_in_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      if ((axi_ar_in_if.valid & axi_ar_in_if.ready) == 1'b0) begin
        $error("[%0t] TIMEOUT: AR MASTER->ROB handshake FAILED (orig_id=%0b addr=%h len_beats=%0d)",
               $time, arid, addr, len_beats);
        axi_ar_in_if.valid = 1'b0;
        disable send_read_req;
      end

      $display("[%0t] AR MASTER->ROB succeeded: orig_id=%0b addr=%h len_beats=%0d",
               $time, arid, addr, len_beats);

      axi_ar_in_if.valid = 1'b0;

      capture_next_uid();
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver on slave side (single beat)
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
      while ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0 &
             guard < TIMEOUT_CYCLES) begin
        guard = guard + 1;
        @(posedge clk);
      end

      if ((axi_r_in_if.valid & axi_r_in_if.ready) == 1'b0) begin
        $error("[%0t] HIGILSTOLLER: TIMEOUT: R SLAVE->ROB handshake FAILED (uid=%0b data=%h)",
               $time, uid, data);
        axi_r_in_if.valid = 1'b0;
        disable send_single_beat_rsp;
      end

      $display("[%0t] HIGILSTOLLER: R SLAVE->ROB succeeded: uid=%0b data=%h last=%0b resp=%0b",
               $time, uid, data, axi_r_in_if.last, resp);

      axi_r_in_if.valid = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver – multi-beat burst
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
        cur_data = base_data + i;

        axi_r_in_if.id    = uid;
        axi_r_in_if.data  = cur_data;
        axi_r_in_if.resp  = 2'b00;
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
          $error("[%0t] HIGILSTOLLER: TIMEOUT: R SLAVE->ROB handshake FAILED in burst (uid=%0b data=%h beat=%0d)",
                 $time, uid, cur_data, i);
          axi_r_in_if.valid = 1'b0;
          disable send_burst_rsp;
        end

        $display("[%0t] HIGILSTOLLER: R SLAVE->ROB BURST beat %0d succeeded: uid=%0b data=%h last=%0b",
                 $time, i, uid, cur_data, axi_r_in_if.last);

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
  end

  // ---------------------------------------------------------------------------
  // Direct test scenarios
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

    // Scenario 1 – single read, in-order response
    $display("---- SCENARIO 1: one read, in-order response ----");
    sb_reset();
    uid_reset();

    sb_expect(ID_WIDTH'(0), 64'h4749_4C53_544F_4C52, 1'b1);  // "GILSTOLR"
    send_read_req(ID_WIDTH'(0), 32'h0000_1000, 8'd1);
    send_single_beat_rsp(captured_uid[0], 64'h4749_4C53_544F_4C52, 2'b00);
    sb_wait_all("SCENARIO_1");

    // Scenario 2 – two different IDs, fabric out-of-order across IDs
    //   • AR1: orig_id=3, 1 beat (data=1111...0001)
    //   • AR2: orig_id=5, 1 beat (data=2222...0002)
    //   • Fabric returns UID2 first, then UID1
    //   • Since cross-ID reordering is allowed, scoreboard expects:
    //       first beat: ID=5, data=2222...
    //       second beat: ID=3, data=1111...
    $display("---- SCENARIO 2: two different IDs, OoO fabric returns ----");
    sb_reset();
    uid_reset();

    send_read_req(ID_WIDTH'(3), 32'h0000_2000, 8'd1); // first request, ID=3
    send_read_req(ID_WIDTH'(5), 32'h0000_3000, 8'd1); // second request, ID=5

    // Expected order = same as fabric (since IDs differ)
    sb_expect(ID_WIDTH'(5), 64'h2222_0000_0000_0002, 1'b1);
    sb_expect(ID_WIDTH'(3), 64'h1111_0000_0000_0001, 1'b1);

    // Fabric: return second UID first, then first
    send_single_beat_rsp(captured_uid[1], 64'h2222_0000_0000_0002, 2'b00); // ID=5
    send_single_beat_rsp(captured_uid[0], 64'h1111_0000_0000_0001, 2'b00); // ID=3

    sb_wait_all("SCENARIO_2");

    // Scenario 3 – single ID, 4-beat burst
    $display("---- SCENARIO 3: single ID, 4-beat burst ----");
    sb_reset();
    uid_reset();

    send_read_req(ID_WIDTH'(5), 32'h0000_4000, 8'd4);

    base3 = 64'hAAA0_0000_0000_0000;
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(5), base3 + k, (k == 3));
    end

    send_burst_rsp(captured_uid[0], 4, base3);
    sb_wait_all("SCENARIO_3");

    // Scenario 4 – response_memory stress, two 4-beat bursts, same ID, OoO
    $display("---- SCENARIO 4: response_memory stress (two 4-beat bursts, out-of-order) ----");
    sb_reset();
    uid_reset();

    send_read_req(ID_WIDTH'(7), 32'h0000_5000, 8'd4); // first burst
    send_read_req(ID_WIDTH'(7), 32'h0000_6000, 8'd4); // second burst

    base4_a = 64'hA000_0000_0000_0000;
    base4_b = 64'hB000_0000_0000_0000;

    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(7), base4_a + k, (k == 3));
    end
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(ID_WIDTH'(7), base4_b + k, (k == 3));
    end

    send_burst_rsp(captured_uid[1], 4, base4_b); // second AR's UID
    send_burst_rsp(captured_uid[0], 4, base4_a); // first AR's UID

    sb_wait_all("SCENARIO_4");

    $display("==== ALL SCENARIOS DONE ====");
    #20;
    $finish;
  end

endmodule
