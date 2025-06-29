// AXI Read Address Interface with sender/receiver modports
interface ar_if #(
    parameter ID_WIDTH    = 4,    // width of the AXI transaction ID field
    parameter ADDR_WIDTH  = 32,   // width of the memory address field
    parameter LEN_WIDTH   = 8,    // width of the burst length field
    parameter TAG_WIDTH   = 4     // internal tag width used by our design
);

    logic                  valid;    // gets asserted by the sender to say that the address/control info is valid
    logic [ID_WIDTH-1:0]   id;       // transaction ID for tracking requests/responses
    logic [ADDR_WIDTH-1:0] addr;     // the memory address where we want to read from
    logic [LEN_WIDTH-1:0]  len;      // how many beats (transfers) in this burst
    logic [2:0]            size;     // size of each beat (like 3'b011 = 8 bytes per beat)
    logic [1:0]            burst;    // tells how the address should behave across beats (like increment or fixed)
    logic [3:0]            qos;      // quality of service (optional, usually for arbitration)
    logic [TAG_WIDTH-1:0]  tagid;    // an internal tag we use to help track or reorder inside the chip
    logic                  ready;    // receiver sets this high when it can accept a new transaction

    // modport for the sender side (master), this drives all the signals
    modport sender (
        output valid,   // sender tells us it's driving a new request
        output id,      // transaction ID it is assigning
        output addr,    // memory address of the read
        output len,     // how many beats
        output size,    // beat size (in bytes)
        output burst,   // burst type (like INCR or FIXED)
        output qos,     // QoS level (used for arbitration if needed)
        output tagid,   // internal tag for tracking in our design
        input  ready    // sender listens to this to know if it can send
    );

    // modport for the receiver side (slave), this side accepts the values
    modport receiver (
        input  valid,   // sees that the sender is offering a request
        input  id,      // reads the transaction ID
        input  addr,    // gets the address to read from
        input  len,     // gets the number of beats
        input  size,    // gets the beat size
        input  burst,   // gets the burst type
        input  qos,     // gets the QoS hint
        input  tagid,   // gets the internal tag
        output ready    // receiver tells sender it's ready to accept
    );

endinterface
