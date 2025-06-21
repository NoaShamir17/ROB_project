
interface ar_if #(
    parameter ID_WIDTH    = 4,
    parameter ADDR_WIDTH  = 32,
    parameter LEN_WIDTH   = 8,
    parameter TAG_WIDTH   = 4
);

    logic                  valid;
    logic [ID_WIDTH-1:0]   id;
    logic [ADDR_WIDTH-1:0] addr;
    logic [LEN_WIDTH-1:0]  len;
    logic [2:0]            size;
    logic [1:0]            burst;
    logic [3:0]            qos;
    logic [TAG_WIDTH-1:0]  tagid;
    logic                  ready;

    modport slave (
        input  valid,
        input  id,
        input  addr,
        input  len,
        input  size,
        input  burst,
        input  qos,
        input  tagid,
        output ready
    );

    modport master (
        output valid,
        output id,
        output addr,
        output len,
        output size,
        output burst,
        output qos,
        output tagid,
        input  ready
    );

endinterface
