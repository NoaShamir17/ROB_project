module row_col_assign #(
    // ---------------- PARAMS: only ID_WIDTH is actually used below.
    // If you don’t plan to use ADDR/LEN/TAG/FIFO here, drop them later.
    parameter ID_WIDTH        = 4,    // AXI ID width (orig ID width)
    parameter ADDR_WIDTH      = 32,   // (unused here)
    parameter LEN_WIDTH       = 8,    // (unused here)
    parameter TAG_WIDTH       = 4,    // (unused here)
    parameter FIFO_DEPTH      = 16,   // (unused here)
    parameter MAX_OUTSTANDING = 16    // Number of slots to manage (N)
)(
    input  logic clk,            // Clock
    input  logic rst,            // Synchronous active-high reset

    // ---------------- INPUTS:
    // 'in_id' is the ORIGINAL request ID you want to map to a unique slot.
    input logic [ID_WIDTH-1:0] in_id,

    // ---------------- OUTPUTS (INTENT):
    // 'out_id'  — what you want to drive downstream (often you just echo in_id).
    // 'unique_id' — slot index in [0..MAX_OUTSTANDING-1] that you “allocate” for in_id.
    //
    // **** NOTE: There is a SYNTAX ISSUE here in the original code: ****
    // You are missing a comma after 'out_id' and a semicolon after 'unique_id' in the port list.
    // Do not fix now per your request — just be aware it won’t compile until you add them.
    output logic [2 * $clog2(MAX_OUTSTANDING) - 1:0] unique_id,
    output logic valid_id
    );

    // ---------------- GOAL OF THIS MODULE (high-level):
    // Maintain a small table of up to MAX_OUTSTANDING “rows/slots”.
    // For an incoming 'in_id':
    //   1) If 'in_id' already has a slot, return that slot (unique_id = existing index).
    //   2) Else, find the first available slot, mark it used, bind it to 'in_id',
    //      and return that index.
    // You will ALSO need a way to FREE a slot when the transaction retires
    // (e.g., a 'free_valid' + 'free_idx' input). That free path is currently missing.

    // ---------------- PER-SLOT STATE:
    // 'used'       : slot has been used at least once (debug/telemetry)
    // 'id'         : the orig ID that currently owns this slot (valid only if allocated)
    // 'available'  : 1 if free to allocate now, 0 if allocated
    //
    // NOTE: You probably want a single 'allocated' bit instead of 'available' (inverted meaning),
    // but it’s fine — just be consistent: available=1 means you can take it.
    typedef struct packed {
        logic used;
        logic [ID_WIDTH-1:0] id;
        logic [$clog2(MAX_OUTSTANDING)-1:0] available;
        //logic release;   // <- you could use a 'release' pulse per slot if you want a direct free, but a global free_if is cleaner.
    } row_variables_t;

    // ---------------- THE TABLE: array of per-slot records, size MAX_OUTSTANDING.
    row_variables_t row_variables [0:MAX_OUTSTANDING-1];

logic [MAX_OUTSTANDING-1:0] available_row;
/////////////////////////////////////////////////////////////////////////////////
// Main clocked block: runs on every rising edge of clk
always_ff @(posedge clk) begin
  if (rst) begin
    // ---------------- RESET BEHAVIOR ----------------
    // Clear outputs so nothing is valid after reset
    valid_id  <= 1'b0;
    unique_id <= '0;

    // Clear the entire slot table on reset
    for (int j = 0; j < MAX_OUTSTANDING; j++) begin
      row_variables[j].used      <= 1'b0; // no slot marked as used
      row_variables[j].id        <= '0;   // clear stored ID
      row_variables[j].available <= '0;   // convention: '0 means FREE in your code
    end
  end else begin
    // ---------------- NORMAL OPERATION ----------------
    // Give default assignments at the start of each cycle.
    // This ensures valid_id doesn’t “stick” at 1 when no hit occurs.
    valid_id  <= 1'b0;

    // Loop over all slots in the table on this clock edge
    for (int j = 0; j < MAX_OUTSTANDING; j++) begin
      // If we haven’t already found a match, and this slot is used,
      // and the stored ID equals the incoming ID...
      if (row_variables[j].used & (row_variables[j].id == in_id)) begin
        // Build the unique_id:
        //   - Upper bits are the index 'j' of the slot that matched
        //   - Lower bits are row_variables[j].available (in your convention)
        unique_id <= {
          j[$clog2(MAX_OUTSTANDING)-1:0],
          row_variables[j].available[$clog2(MAX_OUTSTANDING)-1:0]
        };

        // Mark the output as valid, since we found a hit
        valid_id <= 1'b1;
        row_variables[j].available <= (row_variables[j].available + 1)%MAX_OUTSTANDING; // keep available as is

      end
      else begin
        available_row[j] = &row_variables[j].used[j-1:0] & ~row_variables[j].used[j]; // find first available row
      end
    end
  end
end


    // ---------------- WHAT'S MISSING (to make this work):
    //
    // 1) A way to REQUEST and ACKNOWLEDGE allocation, e.g.:
    //      input  logic req_valid;
    //      output logic req_ready;
    //      output logic grant_valid;   // one-cycle pulse when unique_id/out_id are valid
    //    Without a handshake, you don’t know when to search/commit a slot.
    //
    // 2) A FREE path, e.g.:
    //      input logic free_valid;
    //      input logic [$clog2(MAX_OUTSTANDING)-1:0] free_idx;
    //    On free_valid, set row_variables[free_idx].available <= 1, and optionally clear id.
    //
    // 3) A COMBINATIONAL SEARCH block:
    //    - Scan all slots for:
    //        a) existing_idx where available==0 and id==in_id (reuse)
    //        b) free_idx where available==1 (first free)
    //    - Decide which index to use this cycle.
    //
    // 4) A SEQUENTIAL UPDATE block:
    //    On "allocate fire":
    //      - if existing found: do nothing to the table (still allocated)
    //      - else if free found: available<=0, used<=1, id<=in_id
    //    On "free fire":
    //      - available<=1 (and optionally clear id)
    //
    // 5) Driving outputs:
    //      out_id    <= in_id;              // usually just echo (or map)
    //      unique_id <= decided_index;      // from existing or free slot
    //    Both are typically registered on the allocation fire cycle.
    //
    // 6) Minor details:
    //    - Index width: use localparam IDX_W = $clog2(MAX_OUTSTANDING).
    //    - Ensure only one allocate happens per cycle (or add arbitration if multi-source).
    //    - Decide policy if no free slot exists (stall req_ready or return an error).
    //
    // 7) Assertions (sim-only):
    //    - Don’t allocate when no free slot exists.
    //    - Free index must be currently allocated.
    //    - Stability under backpressure if you add valid/ready.

endmodule




    
endmodule
