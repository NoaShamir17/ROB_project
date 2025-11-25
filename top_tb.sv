module top_tb;

  // ---------------------------------------------------------------------------
  // Local parameters – must match DUT + interfaces
  // ---------------------------------------------------------------------------
  localparam int ID_WIDTH        = 4;
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
    .MAX_LEN         (MAX_LEN)
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

  task automatic sb_reset;
    begin
      exp_count = 0;
      exp_index = 0;
    end
  endtask

  task automatic sb_expect(
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
        // R ROB->MASTER handshake succeeded
        $display("[%0t] R ROB->MASTER succeeded: id=%0d data=%h last=%0b resp=%0b",
                 $time, axi_r_out_if.id, axi_r_out_if.data,
                 axi_r_out_if.last, axi_r_out_if.resp);

        if (exp_index >= exp_count) begin
          $error("[%0t] unexpected response from ROB: id=%0d data=%h last=%0b",
                 $time, axi_r_out_if.id, axi_r_out_if.data, axi_r_out_if.last);
        end
        else begin
          if (axi_r_out_if.id == expected_q[exp_index].id &
              axi_r_out_if.data == expected_q[exp_index].data &
              axi_r_out_if.last == expected_q[exp_index].last) begin
            $display("[%0t] SCOREBOARD: beat %0d OK (id=%0d)",
                     $time, exp_index, axi_r_out_if.id);
          end
          else begin
            $error("[%0t] SCOREBOARD MISMATCH at beat %0d: got id=%0d data=%h last=%0b expected id=%0d data=%h last=%0b",
                   $time, exp_index,
                   axi_r_out_if.id, axi_r_out_if.data, axi_r_out_if.last,
                   expected_q[exp_index].id,
                   expected_q[exp_index].data,
                   expected_q[exp_index].last);
          end
          exp_index = exp_index + 1;
        end
      end
    end
  end

  task automatic sb_wait_all(input string tag);
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

  task automatic uid_reset;
    begin
      uid_count = 0;
    end
  endtask

  // Wait for AR handshake on ROB->slave, capture UID, and log AR path
  task automatic capture_next_uid;
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
        $error("[%0t] TIMEOUT: AR ROB->SLAVE handshake FAILED (no forwarded AR observed)", $time);
        disable capture_next_uid;
      end

      captured_uid[uid_count] = axi_ar_out_if.id;
      $display("[%0t] AR ROB->SLAVE succeeded: uid[%0d]=%0d addr=%h len=%0d",
               $time, uid_count, captured_uid[uid_count],
               axi_ar_out_if.addr, axi_ar_out_if.len);

      uid_count = uid_count + 1;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read address driver on master side (axi_ar_in_if)
  // ---------------------------------------------------------------------------
  task automatic send_read_req(
    input logic [ID_WIDTH-1:0]   arid,
    input logic [ADDR_WIDTH-1:0] addr,
    input logic [LEN_WIDTH-1:0]  len
  );
    int guard;
    begin
      axi_ar_in_if.id    = arid;
      axi_ar_in_if.addr  = addr;
      axi_ar_in_if.len   = len;
      axi_ar_in_if.size  = 3'b011;   // 8 bytes
      axi_ar_in_if.burst = 2'b01;    // INCR
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
        $error("[%0t] TIMEOUT: AR MASTER->ROB handshake FAILED (orig_id=%0d addr=%h len=%0d)",
               $time, arid, addr, len);
        axi_ar_in_if.valid = 1'b0;
        disable send_read_req;
      end

      $display("[%0t] AR MASTER->ROB succeeded: orig_id=%0d addr=%h len=%0d",
               $time, arid, addr, len);

      axi_ar_in_if.valid = 1'b0;

      // Wait for ROB->SLAVE forwarding and capture internal UID
      capture_next_uid();
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver on slave side (axi_r_in_if) – single beat
  // ---------------------------------------------------------------------------
  task automatic send_single_beat_rsp(
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
        $error("[%0t] TIMEOUT: R SLAVE->ROB handshake FAILED (uid=%0d data=%h)", $time, uid, data);
        axi_r_in_if.valid = 1'b0;
        disable send_single_beat_rsp;
      end

      $display("[%0t] R SLAVE->ROB succeeded: uid=%0d data=%h last=%0b resp=%0b",
               $time, uid, data, axi_r_in_if.last, resp);

      axi_r_in_if.valid = 1'b0;
    end
  endtask

  // ---------------------------------------------------------------------------
  // AXI read data driver – multi-beat burst
  // ---------------------------------------------------------------------------
  task automatic send_burst_rsp(
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
          $error("[%0t] TIMEOUT: R SLAVE->ROB handshake FAILED in burst (uid=%0d data=%h beat=%0d)", $time, uid, cur_data, i);
          axi_r_in_if.valid = 1'b0;
          disable send_burst_rsp;
        end

        $display("[%0t] R SLAVE->ROB BURST beat %0d succeeded: uid=%0d data=%h last=%0b",
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
    int k;

    @(posedge clk);
    while (rst == 1'b1) begin
      @(posedge clk);
    end

    // Scenario 1: single read, in-order response
    $display("---- SCENARIO 1: one read, in-order response ----");
    sb_reset();
    uid_reset();

    send_read_req(4'd0, 32'h0000_1000, {LEN_WIDTH{1'b0}});
    sb_expect(4'd0, 64'h0000_0000_DEAD_BEEF, 1'b1);
    send_single_beat_rsp(captured_uid[0], 64'h0000_0000_DEAD_BEEF, 2'b00);
    sb_wait_all("SCENARIO_1");

    // Scenario 2: same ID, out-of-order fabric returns
    $display("---- SCENARIO 2: same ID, out-of-order fabric returns ----");
    sb_reset();
    uid_reset();

    send_read_req(4'd3, 32'h0000_2000, {LEN_WIDTH{1'b0}});
    send_read_req(4'd3, 32'h0000_3000, {LEN_WIDTH{1'b0}});

    sb_expect(4'd3, 64'h1111_0000_0000_0001, 1'b1);
    sb_expect(4'd3, 64'h2222_0000_0000_0002, 1'b1);

    // fabric returns second UID first, then first UID
    send_single_beat_rsp(captured_uid[1], 64'h2222_0000_0000_0002, 2'b00);
    send_single_beat_rsp(captured_uid[0], 64'h1111_0000_0000_0001, 2'b00);

    sb_wait_all("SCENARIO_2");

    // Scenario 3: single ID, multi-beat burst (4 beats)
    $display("---- SCENARIO 3: burst of 4 beats for one ID ----");
    sb_reset();
    uid_reset();

    // len = 3 → 4 beats (AXI: beats = len + 1)
    send_read_req(4'd5, 32'h0000_4000, 8'd3);

    base3 = 64'hAAA0_0000_0000_0000;
    for (k = 0; k < 4; k = k + 1) begin
      sb_expect(4'd5, base3 + k, (k == 3));
    end

    send_burst_rsp(captured_uid[0], 4, base3);

    sb_wait_all("SCENARIO_3");

    $display("==== ALL SCENARIOS DONE ====");
    #20;
    $finish;
  end

endmodule
