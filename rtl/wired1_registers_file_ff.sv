`include "wired0_defines.svh"

module wired_registers_file_ff #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,
    parameter bit NEED_RESET = 0,
    parameter logic[DATA_WIDTH-1:0] RESET_VAL = '0

    // DO NOT MODIFY
    parameter type T = logic[DATA_WIDTH - 1 : 0],
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    `_WIRED_GENERAL_DEFINE,
    input    [ADDR_DEPTH-1:0] waddr_i,
    input                        we_i,
    input  T                  wdata_i,

    output T [DEPTH-1:0] regfiles_o
);

    reg [DEPTH-1:0][DATA_WIDTH-1:0] regfiles_q;
    assign regfiles_o = regfiles_q;

    always_ff @(posedge clk) begin
        if(NEED_RESET && ~rst_n) begin
            regfiles_q <= 'RESET_VAL;
        end else if(we_i) begin
            regfiles_q[waddr_i] <= wdata_i;
        end
    end

endmodule
