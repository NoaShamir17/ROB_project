module row_col_assign #(
    parameter ID_WIDTH        = 4,    // AXI ID width (orig ID width)
    parameter MAX_OUTSTANDING = 16    // Number of slots to manage (N)
)(
    input  logic clk,            // Clock
    input  logic rst,            // Synchronous active-high reset
    input  logic [ID_WIDTH-1:0] in_id,

    output logic [2 * $clog2(MAX_OUTSTANDING) - 1:0] unique_id,
    output logic valid_id
    );

    logic all_used;

    typedef struct packed {
        logic used;     // 1 = slot is allocated, 0 = free
        logic [ID_WIDTH-1:0] id; // stored original master ID for this row
        logic [$clog2(MAX_OUTSTANDING)-1:0] available_col; // next available column in this row
    } row_variables_t;

    // ---------------- THE TABLE: array of per-slot records, size MAX_OUTSTANDING.
    row_variables_t row_variables [0:MAX_OUTSTANDING-1];

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
      row_variables[j].available_col <= '0;   // convention: '0 means FREE in your code
    end
  end else begin
    // ---------------- NORMAL OPERATION ----------------
    valid_id  <= 1'b0;

    // Loop over all slots in the table on this clock edge
    for (int j = 0; j < MAX_OUTSTANDING; j++) begin
      // If we haven’t already found a match, and this slot is used,
      // and the stored ID equals the incoming ID...
      if (row_variables[j].used & (row_variables[j].id == in_id)) begin
        // Build the unique_id:
        //   - Upper bits are the index 'j' of the slot that matched
        //   - Lower bits are row_variables[j].available_col_col (in your convention)
        unique_id <= {
          j[$clog2(MAX_OUTSTANDING)-1:0],
          row_variables[j].available_col[$clog2(MAX_OUTSTANDING)-1:0]
        };

        // Mark the output as valid, since we found a hit
        valid_id <= 1'b1;
        row_variables[j].available_col <= (row_variables[j].available_col + 1)%MAX_OUTSTANDING; // keep available_col as is

      end
      else begin
        //this doesnt work!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        if (&row_variables[j-1:0].used & ~row_variables[j].used) begin // if row is the first one available_col
          unique_id <= {
            j[$clog2(MAX_OUTSTANDING)-1:0],
            row_variables[j].available_col[$clog2(MAX_OUTSTANDING)-1:0]
          };
          valid_id <= 1'b1;
          row_variables[j].used      <= 1'b1; // mark as used
          row_variables[j].id        <= in_id; // store incoming id
          row_variables[j].available_col <= (row_variables[j].available_col + 1)%MAX_OUTSTANDING; // increment available_col
        end
      end
    end
  end
end






endmodule




    

