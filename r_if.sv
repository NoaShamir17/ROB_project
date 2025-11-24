// AXI Read Data Channel interface with sender/receiver modports
interface r_if #(
    parameter ID_WIDTH    = 4,    // width of the transaction ID
    parameter DATA_WIDTH  = 64,   // width of the data bus
    parameter RESP_WIDTH  = 2    // width of the response field (usually 2 bits)
);

    logic                  valid;              // high means this beat of data is valid
    logic [ID_WIDTH-1:0]   id;                 // transaction ID for this data beat
    logic [DATA_WIDTH-1:0] data;               // actual data payload being transferred
    logic [RESP_WIDTH-1:0] resp;               // response code (OKAY, SLVERR, etc.)
    logic                  last;               // high means this is the last beat in a burst
    logic                  ready;              // receiver sets this high when it can accept data

    // Sender side (slave)— this module outputs the response data
    modport sender (
        output valid,   // tells receiver that data is valid
        output id,      // sends the transaction ID
        output data,    // sends the actual data
        output resp,    // sends the response status
        output last,    // tells whether this is the last beat
        input  ready    // listens to whether the receiver is ready
    );

    // Receiver side (master)— this module accepts the data beat
    modport receiver (
        input  valid,   // sees if sender is offering data
        input  id,      // reads the transaction ID
        input  data,    // reads the data value
        input  resp,    // reads the response type
        input  last,    // reads whether it's the last beat
        output ready    // tells sender it's ready to accept data
    );

endinterface
