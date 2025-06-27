interface r_if #(
    parameter ID_WIDTH    = 4,
    parameter DATA_WIDTH  = 64,
    parameter RESP_WIDTH  = 2,
    parameter TAG_WIDTH   = 4
);

    logic                  valid;
    logic [ID_WIDTH-1:0]   id;
    logic [DATA_WIDTH-1:0] data;
    logic [RESP_WIDTH-1:0] resp;
    logic                  last;
    logic [TAG_WIDTH-1:0]  tagid;
    logic                  ready;

    modport slave (
        input  valid,
        input  id,
        input  data,
        input  resp,
        input  last,
        input  tagid,
        output ready
    );

    modport master (
        output valid,
        output id,
        output data,
        output resp,
        output last,
        output tagid,
        input  ready
    );

endinterface
