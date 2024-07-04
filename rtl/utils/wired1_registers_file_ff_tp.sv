`include "wired0_defines.svh"

module wired_registers_file_ff_tp #(
    parameter int unsigned DATA_WIDTH = 32,
    parameter int unsigned DEPTH = 32,
    parameter bit NEED_RESET = 0,
    parameter logic[DEPTH-1:0][DATA_WIDTH-1:0] RESET_VAL = '0,

    // DO NOT MODIFY
    parameter type T = logic[DATA_WIDTH - 1 : 0],
    parameter int unsigned ADDR_DEPTH   = (DEPTH > 1) ? $clog2(DEPTH) : 1
)(
    `_WIRED_GENERAL_DEFINE,
    input    [1:0][ADDR_DEPTH-1:0] waddr_i,
    input    [1:0]                    we_i,
    input  T [1:0]                 wdata_i,

    output T [DEPTH-1:0] regfiles_o
);

    reg [DEPTH-1:0][DATA_WIDTH-1:0] regfiles_q;
    assign regfiles_o = regfiles_q;

    for(genvar i = 0 ; i < DEPTH ; i += 1) begin
        always_ff @(posedge clk) begin
            if(NEED_RESET && ~rst_n) begin
                regfiles_q[i] <= RESET_VAL[i];
            end else if(we_i[1] && waddr_i[1] == i[ADDR_DEPTH-1:0]) begin
                regfiles_q[i] <= wdata_i[1];
            end else if(we_i[0] && waddr_i[0] == i[ADDR_DEPTH-1:0]) begin
                regfiles_q[i] <= wdata_i[0];
            end
        end
    end

endmodule
